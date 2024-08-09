
/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 * CITE IT IF YOU COPY IT.
 */

#include "cpu/o3/pbtb.hh" // Technically cpu.hh includes this already?

#include "base/trace.hh"
#include "cpu/o3/cpu.hh"
#include "debug/Decode.hh" //JV TODO DEBUG TEMP
#include "debug/PBTB.hh"
#include "debug/PBTBVerbose.hh"

namespace gem5
{

namespace o3
{

const char* PrecomputedBTB::BranchTypeCodes[] = {"N/A", "TKN", "LOP",
                                                 "BIT", "B_C"};
const char* PrecomputedBTB::BranchTypeStrs[] = { "NoBranch", "Taken", "Loop",
                                                 "ShiftBit", "ShiftBit_Clear"};


// TODO: make this return an std::string?
const std::string debugPrintBottomBits(uint64_t bits, int n) {
    char debug_bit_buff[65];

    debug_bit_buff[64] = '\0'; //null terminate just to be safe
    n = std::min(n, 64);

    int i;
    for (i = 0; i < n; i++) {
        int bit_offset = n - i - 1; //1st digit is at (bits >> (n-1))
        int bit = (bits >> bit_offset) & 1;
        debug_bit_buff[i] = bit ? '1':'0';
    }
    debug_bit_buff[i] = '\0';
    return std::string { debug_bit_buff };
}

// ==============================================================
//
//                        JV PRECOMPUTED BTB
//
// ==============================================================
//

PrecomputedBTB::PrecomputedBTB(CPU *_cpu) : cpu(_cpu)
{}

std::string PrecomputedBTB::name() const {
    return cpu->name() + ".pbtb";
}


void PrecomputedBTB::debugDump(int regstart, int regstop) {
    const int which_map = 0; // NOTE: change this manually when testing
                             // If you need to see the other map
    struct pbtb_map *mapsToPrint[] = {&map_fetch, &map_final};
    const char *mapNames[] = { "F", "D" } ;
    struct pbtb_map *curr_map = mapsToPrint[which_map];

    //DPRINTF(PBTB, "PBTB (%d CPs, fst=%d, lst=%d): {\n",
    //        numActiveCheckpoints(), first_checkp, last_checkp);
    DPRINTF(PBTBVerbose, "PBTB (%s): {\n", mapNames[which_map]);

    regstart = std::max(0, regstart);
    regstop = std::min(NUM_REGS, regstop);
    int numtoprint = regstop - regstart;

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstart > 0) { DPRINTF(PBTBVerbose, "  ...\n"); }

    for (int reg = regstart; reg < regstop; reg++) {
        auto branchCode = BranchTypeCodes[curr_map->cond_type[reg]];

        DPRINTF(PBTBVerbose, "  %2i: SRC=%8lx TGT=%8lx | %s: %ld %ld\n", reg,
                (uint64_t)curr_map->source[reg],
                (uint64_t)curr_map->target[reg],
                branchCode,
                curr_map->cond_val[reg],
                curr_map->cond_aux_val[reg]);
    }

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstop < NUM_REGS)
        { DPRINTF(PBTBVerbose, "  ...\n"); }

    // I guess also handle the edge case where we omit them all
    if (numtoprint == 0) { DPRINTF(PBTBVerbose,"  ...\n"); }

    DPRINTF(PBTBVerbose, "}\n");
}

void PrecomputedBTB::debugDump() { debugDump(0, NUM_REGS); }


// =============== Per-Map functions (PRIVATE)

void PrecomputedBTB::m_setSource(struct pbtb_map *pmap,
                                 int breg, Addr source_addr) {
    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31
    pmap->source[breg] = source_addr;
    pmap->version[breg]++; //TODO: set this to seqNum instead

    // setSource also resets it to an invalid branch
    pmap->cond_type[breg]    = NoBranch;
    pmap->cond_val[breg]     = 0;
    pmap->cond_aux_val[breg] = 0;
}
void PrecomputedBTB::m_setTarget(struct pbtb_map *pmap,
                                 int breg, Addr target_addr) {
    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31
    pmap->target[breg] = target_addr;
    pmap->version[breg]++; //TODO: set this to seqNum instead

    // setTarget also resets it to a taken branch
    pmap->cond_type[breg]    = Taken;
    pmap->cond_val[breg]     = 0;
    pmap->cond_aux_val[breg] = 0;
}

// Sets the condition for a breg in a pmap
// val and n are treated differently depending on conditionType:
// - Taken or NotTaken ignore it
// - LoopTaken will be taken (val) times, then NT once, then repeat
// - ShiftBit takes a bitstring in val, and enqueues the bottom n bits of it
//
void PrecomputedBTB::m_setCondition(struct pbtb_map *pmap,
                                  int breg,
                                  BranchType conditionType,
                                  uint64_t val,
                                  int64_t n) {

    //LATER TODO: we shouldn't actually need checkpoints on bmovs, only on
    // branches (whether real or false positives)
    // However, we can't make that work until we have branches stall in decode
    // when bmovs are in flight
    //squashYoungerCheckpoints(seqNum);

    // need to squash all ops older than seqnum (or == seqnum, if the bmov
    // itself was mispredicted as a branch)
    //TODO: make a new checkpoint every time we set a condition
    //makeNewCheckpoint(seqNum);

    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31
    switch (conditionType) {
        case NoBranch:
        case Taken:
            pmap->cond_type[breg]    = conditionType;
            pmap->cond_val[breg]     = 0;
            pmap->cond_aux_val[breg] = 0;
            pmap->version[breg]++; //TODO: set this to seqNum instead
            break;
        case LoopN:
            pmap->cond_type[breg]    = conditionType;
            pmap->cond_val[breg]     = val;
            pmap->cond_aux_val[breg] = 0;
            pmap->version[breg]++; //TODO: set this to seqNum instead
            break;
        case ShiftBit:
            if (pmap->cond_type[breg] != ShiftBit) {
                // When first switching to BIT mode,
                // reset the bit fifo
                pmap->cond_val[breg]     = 0;
                pmap->cond_aux_val[breg] = 0;

                // Also, version should only increment on first
                // change (so we can produce and consume simultaneously
                //         as long as enough bits are in the queue)
                pmap->version[breg]++; //TODO: set this to seqNum instead
            }
            pmap->cond_type[breg]    = ShiftBit;

            //if (conditionType == ShiftBit_Clear) { //clear fifo
            //    pmap->cond_val[breg]     = 0; // bits
            //    pmap->cond_aux_val[breg] = 0; // num_bits
            //}

            // TODO: is there any reason to shift in 0 bits?
            // Maybe as a way to clear it?
            assert(n > 0 && n <= 64); //PBTB shiftbits must be in [1-64]

            {
            // mask out bottom n bits (I should probably use a macro for this)
            uint64_t bottom_n_bits = n >= 64 ? val : val & ((1L<<n)-1);

            // Bits shifted out at LSB, shifted in at bit N,
            // where N is current size of array
            // e.g. if curr_size == e.g. 3 bits, next bit comes in at bit 3

            pmap->cond_val[breg] |= bottom_n_bits <<
                                pmap->cond_aux_val[breg];
            pmap->cond_aux_val[breg] += n; // num_bits+=n

            uint64_t bits = pmap->cond_val[breg];
            uint64_t numbits = pmap->cond_aux_val[breg];
            DPRINTF(PBTB, "PBTB (X): bmovc_bit b%d: data{%s}(%d)\n",
                    breg, debugPrintBottomBits(bits,numbits).c_str(), numbits);

            }

            // make sure we don't touch the sign bit (just to be safe)
            // TODO: we should probably allow 64, crash at 65?
            // TODO: either way, this should trigger an exception I think
            assert(pmap->cond_aux_val[breg] <= 63);
            break;
        case ShiftBit_Clear:
        default:
            panic("Unrecognized conditiontype in PBTB");
            break;
    }
}

/**
    * Queries the given pmap for pcAddr,
    * returns the appropriate resultType
    * if a matching breg IS found, sets p_breg_out and p_version_out
    * if a taken branch is found, sets p_targetAddr_out
    * @param pcAddr The pc of branches to look for (source PC)
    * @param p_breg_out If a breg match is found, will be returned here
    * @param p_version_out If a breg match is found, version will be here
    * @param p_targetAddr_out If match is a taken branch, tgt addr will be here
    * @return Returns PBTBResultType depending on the kind of match
    */
PrecomputedBTB::PBTBResultType PrecomputedBTB::m_queryPC(
            struct pbtb_map *pmap, Addr pcAddr,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out) {

    // Queries pbtbp for pcAddr
    // if no match, returns PR_NoMatch
    // if match found:
    //     sets p_breg_out, p_version_out
    //     if taken:
    //        returns PR_Taken, sets p_targetAddr_out
    //     else:
    //        returns PR_NotTaken or PR_Exhausted

    int breg = -1; // If found, match will go here
    BranchType brType = NoBranch; // If match found, type will go here
    // Note: bregs in the map can be set to NoBranch, but we should never
    //       match against those (i.e. they're invalid, so this works fine)
    PBTBResultType resType;

    const char *debugWhichMap = "?";
    if (pmap == &map_fetch) { debugWhichMap = "F"; }
    if (pmap == &map_final) { debugWhichMap = "D"; }

    // ==== Loop to find first breg matching the source addr
    for (int i = 0; i < NUM_REGS; i++) {
        if (pmap->source[i] == pcAddr) {
            //Found a match!
            breg = i;
            brType = pmap->cond_type[i];
            break;
        }
    }

    // NOTE: NoBranch shouldn't be reported as a match
    if (brType == NoBranch) {
        // Only print this in verbose mode, there's gonna be a lot of these
        // ( ok actually turns out there's WAY TOO MANY OF THESE)
        //DPRINTF(PBTBVerbose, "PBTB QUERIED: (%s): PC:0x%x, MISS\n",
        //        debugWhichMap, pcAddr);
        *p_breg_out = -1;
        return PR_NoMatch;
    }

    // ==== Check based on branch type whether or not it's taken
    // NOTE: breg might be -1, only guaranteed for breg to be valid
    //       if type != NoBranch
    if (brType == Taken) {
        resType = PR_Taken;

        DPRINTF(PBTB, "PBTB (%s): HIT b%d: TKN 0x%x -> 0x%x\n",
                debugWhichMap, breg, pcAddr, pmap->target[breg]);

    } else if (brType == LoopN) {
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: LOOP{%d} (%s) 0x%x -?> 0x%x\n",
                debugWhichMap, breg, pmap->cond_val[breg],
                pmap->cond_val[breg]>0 ? "T" : "NT",
                pcAddr, pmap->target[breg]);

        //CONSUMED A LOOP ITERATION (debug output?)

        if (pmap->cond_val[breg] > 0) {
            resType = PR_Taken;
            pmap->cond_val[breg]--;
        } else if (pmap->cond_val[breg] == 0) {
            resType = PR_NotTaken;
            pmap->cond_val[breg] = -1;
        } else {
            // LOOP EXHAUSTED: Will be handled in decode validate/finalize
            resType = PR_Exhaust;
            DPRINTF(PBTB, "PBTB (F): Loop at -1: Exhausted\n");
            //printf("PBTB: Loop at -1: ERROR\n");
        }

    } else if (brType == ShiftBit) {
        uint64_t bits = pmap->cond_val[breg];
        uint64_t numbits = pmap->cond_aux_val[breg];
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: BIT{%s} (%s) 0x%x -?> 0x%x\n",
                debugWhichMap, breg,
                debugPrintBottomBits(bits,numbits).c_str(),
                (numbits>0 && bits&1) ? "T" : "NT",
                pcAddr, pmap->target[breg]);

        //CONSUMED A BIT
        //DPRINTF(Decode, "PBTB (F): \\-> Consumed a bit\n");

        // cond_val holds bits, cond_aux_val is number of bits valid
        if (pmap->cond_aux_val[breg] > 0) { //if num_bits > 0
            int bit = pmap->cond_val[breg] & 0x1;
            resType = bit ? PR_Taken : PR_NotTaken;
            pmap->cond_val[breg] >>= 1; // shift out bottom bit
            pmap->cond_aux_val[breg]--; // bits-- (don't underflow!)
        } else { // no bits, exhause (will be handled in decode:finalize)
            resType = PR_Exhaust;
            //printf("PBTB: ShiftBit out of bits: Exhausted\n");
            DPRINTF(PBTB, "PBTB (%s): ShiftBit out of bits: Exhausted\n",
                            debugWhichMap);
        }
    } else {
        DPRINTF(PBTB, "PBTB: Unrecognized branch type %d\n", brType);
        panic("PBTB got query for unrecognized branch type");
        // return PR_NoMatch; // Alternatively, return no match?
    }

    // ==== Prediction made: return to caller
    // Return next-fetched PC through pc arg

    assert(breg >= 0); // breg guaranteed to be valid,
                       // otherwise we'd have returned earlier
                       //
    *p_breg_out = breg;
    *p_version_out = pmap->version[breg];
    if (resType == PR_Taken) { *p_targetAddr_out = pmap->target[breg]; }
    return resType;
}

// =============== PUBLIC Modification functions
// (these use the per-map functions but touch multiple maps)


// == Each of these should correspond to one bmov instruction
void PrecomputedBTB::setSource(int breg, Addr source_addr) {
    m_setSource(&map_fetch, breg, source_addr);
    m_setSource(&map_final, breg, source_addr);
}
void PrecomputedBTB::setTarget(int breg, Addr target_addr) {
    m_setTarget(&map_fetch, breg, target_addr);
    m_setTarget(&map_final, breg, target_addr);
}
void PrecomputedBTB::setCondition(int breg, BranchType conditionType,
                    uint64_t val, int64_t n) {
    m_setCondition(&map_fetch, breg, conditionType, val, n);
    m_setCondition(&map_final, breg, conditionType, val, n);
}

// Just an alias for the previous one
// We only use n for the ShiftBit branch type, so can omit it otherwise
void PrecomputedBTB::setCondition(int breg, BranchType conditionType,
                    uint64_t val) {
    m_setCondition(&map_fetch, breg, conditionType, val, 0);
    m_setCondition(&map_final, breg, conditionType, val, 0);
}


void PrecomputedBTB::squashFinalizeToFetch() {
    DPRINTF(PBTB, "PBTB: Overwriting PBTB from finalize\n");
    // TODO: do both???
    DPRINTF(Decode, "PBTB: Overwriting PBTB from finalize\n");
    //TODO: assert that they're equal barring loop counts / bit counts
    map_fetch = map_final;
}


/**
    * Performs a prediction for the given pc
    * Sets pc to target if taken, else inst->advancePC(pc), to better match
    * interface from branchpredictor
    * @param inst The branch instruction (used for advancePC)
    * @param pc The predicted PC is passed back through this parameter.
    * @param p_breg_out breg is passed back here, or -1 if no match
    * @param p_version_out version for breg is passed back here
    * @return Returns PBTBResultType for T, NT, Exhausted, and NoMatch
    */
PrecomputedBTB::PBTBResultType PrecomputedBTB::queryFromFetch(
            const StaticInstPtr inst, PCStateBase &pc,
            int *p_breg_out, uint64_t *p_version_out) {

    Addr tgt = pc.instAddr(); //will be overwritten if taken
                              //
    PBTBResultType res = m_queryPC(&map_fetch, pc.instAddr(),
            p_breg_out, p_version_out, &tgt);

    // TODO: I'm not sure how to deal with PCStateBases: this tempAddr
    // thing seems to work, so I'm sticking with it
    auto tempAddr = GenericISA::SimplePCState<4>();
    // TODO: everywhere else uses a unique_ptr<PCState>, is there a reason for
    // that? or is a bare object fine?
    // Old code: auto target=std::make_unique<GenericISA::SimplePCState<4>>();

    // ==== Prediction made: return to caller
    // Return next-fetched PC through pc arg
    if (res == PR_Taken){
        tempAddr.set(tgt);
        set(pc, tempAddr);
    } else { //Else: no match, OR match found but not taken
        inst->advancePC(pc);
    }

    return res;
}


//Passthrough, queries the finalize/decode version of the map
//NOTE: if not taken, will use inst to instead advance targetAddr_out
//to nextPc
PrecomputedBTB::PBTBResultType PrecomputedBTB::queryFromDecode(
            const StaticInstPtr inst,  Addr pcAddr,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out) {

    PBTBResultType res = m_queryPC(&map_final, pcAddr,
            p_breg_out, p_version_out, p_targetAddr_out);

    if (res != PR_Taken) {
        // TODO: This is a horrible pile of hacks but I don't want to switch
        // everything to use PCStates
        auto tempAddr = GenericISA::SimplePCState<4>();
        tempAddr.set(pcAddr);
        inst->advancePC(tempAddr);
        *p_targetAddr_out = tempAddr.instAddr();
    }

    return res;
}

//????
//bool PrecomputedBTB::finalizePB() {
//}


// ==============================================================

} // namespace o3
} // namespace gem5
