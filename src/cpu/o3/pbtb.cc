
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
#include "cpu/inst_seq.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
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
//                        JV Bmov Tracker
//
// ==============================================================

void BmovTracker::reset() {
    lastCommittedInst   = 0;

    for (int i = 0; i < PrecomputedBTB::NUM_REGS; i++) {
        lastExecBmov[i] = 0;
        lastDecBmov[i] = 0;
        lastDecNonBitBmov[i] = 0;
    }

    lastDecAny      = 0;
    lastDecPb = 0;
    lastDecMutPb = 0;
}

// ======== tracking functions ( should be called from Decode)

void BmovTracker::recordDecodeInst(ThreadID tid, DynInstConstPtr inst) {
    // DPRINTF(Decode, "[tid:%i] [sn:%llu] BmovTracker: recordDecode\n",
    //     tid, inst->seqNum);

    // sanity check: decode should be in-order
    assert(inst->seqNum > lastDecAny);
    lastDecAny = inst->seqNum;

    if (inst->isBmov()) {
        const int breg = inst->destRegIdx(0);
        const bool nonbit = !inst->isBitBmov();

        DPRINTF(Decode, "[tid:%i] [sn:%llu] BmovTracker: decoded "
                "(%s) breg=%d %s\n",
            tid, inst->seqNum,
            inst->staticInst->disassemble(
                inst->pcState().instAddr()),
            breg,
            nonbit ? "" : ", bit-bmov");

        // sanity check, we should never be re-decoding an earlier bmov?
        // (even if we squash, seq nums should increase)
        assert(inst->seqNum > lastDecBmov[breg]);
        assert(breg >= 0 && breg < PrecomputedBTB::NUM_REGS);
        lastDecBmov[breg] = inst->seqNum;

        if (nonbit) {
            assert(inst->seqNum > lastDecNonBitBmov[breg]);
            lastDecNonBitBmov[breg] = inst->seqNum;
        }
    }
}

void BmovTracker::recordExecBmovFromIew(ThreadID tid,
        InstSeqNum bmovSeq, int breg ) {
    DPRINTF(Decode, "[tid:%i] BmovTracker: bmov b%d done "
        " from iew: [sn%d]\n", tid, breg, bmovSeq);

    // NOTE: these should always be increasing, since bmovs for the
    // same branch reg are executed serially
    assert( bmovSeq > lastExecBmov[breg] );
    assert(breg >= 0 && breg < PrecomputedBTB::NUM_REGS);
    lastExecBmov[breg] = bmovSeq;
}

void BmovTracker::recordCommit(ThreadID tid,
         InstSeqNum instSeqNum ) {
    DPRINTF(Decode, "[tid:%d] [sn:%llu] BmovTracker: recordCommit "
        "NOT IMPLEMENTED\n", tid, instSeqNum);

    //assert(newNum >= lastDoneFromCommit);
    //lastDoneFromCommit = newNum;
}

// Note: this doesn't count squashes the Decode itself generates,
// only squashed from ahead of it, e.g. from IEW or commit
void BmovTracker::recordSquashFromAhead(ThreadID tid,
         InstSeqNum squashNum) {
    // everything with seq>squashNum is gone
    DPRINTF(Decode, "[tid:%d] [sn:%llu] BmovTracker: recordingSquash\n",
        tid, squashNum);

    //TODO: this is unfinished

    for (int bi = 0; bi < PrecomputedBTB::NUM_REGS; bi++) {
        // young insts (high seqnum) are squashed away

        // If we don't have any insts, don't worry about it
        if (lastDecBmov[bi] == 0) { continue; }

        DPRINTF(Decode, "[tid:%i] BmovTracker: Squash@%d "
                " b%d: lastDecode=%d, lastDone=%d.\n",
                tid, squashNum,
                bi, lastDecBmov[bi], lastExecBmov[bi]);

        // If all our insts are old (low seqnum), they're not affected
        if (lastDecBmov[bi] <= squashNum) { continue; }


        // Else, we have some insts that got squashed!
        // If all of them were already executed, we might be ok
        if (lastDecBmov[bi] <= lastExecBmov[bi]) { continue; }

        // ELSE: we've squashed partially-executed bmovs, we can't
        // handle that
        panic("PBTB ERROR: squash from commit squashed unfinished bmovs");
    }
}

        // ===== OLD BMOV TRACKING:
        // //TODO JV PBTB: also track squash nums?
        // InstSeqNum squashSeqNum = fromCommit->commitInfo[tid].doneSeqNum;
        // // everything with seq>squashSeqNum is gone

        // DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastDecode=%d, "
        //         "squash@%d, lastCommited=%d."
        //         " Moving lastDecoded up to squash\n",
        //         tid, lastDecodedInst, squashSeqNum, lastDoneFromCommit);

        // // We've squashed some in-flight instructions, so we no longer
        // // need to wait for lastDecodedInst, just the last non-squashed inst
        // assert(squashSeqNum >= lastDoneFromCommit); // sanity check?

        // if (lastDecodedInst > squashSeqNum) {
        //     DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastDecode=%d, "
        //             "squash@%d, lastCommited=%d."
        //             " Moving lastDecoded up to squash\n",
        //             tid, lastDecodedInst,
        //             squashSeqNum, lastDoneFromCommit);
        //     lastDecodedInst = squashSeqNum;
        //     //TODO: once we track specifically the last bmov, it'll be
        //     // a little trickier to do

        // }
        // if (lastDecodedBmov > squashSeqNum) {
        //     DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastBmov=%d, "
        //             "squash@%d, lastCommited=%d."
        //             " Moving lastBmov up to squash\n",
        //             tid, lastDecodedBmov,
        //             squashSeqNum, lastDoneFromCommit);
        //     lastDecodedBmov = squashSeqNum;
        // }
        // //(fromCommit->commitInfo[tid].doneSeqNum, tid);
        // //lastSquashFromCommit = newSquashNum;

// ============ Query Functions
// if is pb or predicted-as-pb, need to stall for finalize
bool BmovTracker::instNeedsToStall(ThreadID tid,
        DynInstConstPtr inst) const {

    if (inst->isSquashed())
        { return false; }

    if (!inst->isPb() && inst->readPredBTBReg() < 0)
        { return false; }


    if (!inst->isPb()) {
        panic("BmovTracker::instNeedsToStall: NOT IMPLEMENTED FOR NON-PB OPs "
              " THAT WERE MISPREDICTED AS PBS");
        // TODO: need to reason about which breg to checkk
        //TODO: TRICKY QUESTION HERE: if predBreg != actBreg,
        //      do we stall for both?
        //int predBreg = inst->readPredBTBReg();
        //int actBreg = -1;
        //if (inst->isPb() ) {
        //    actBreg = inst->staticInst->srcRegIdx(0);
        //}
    }

    int breg = inst->staticInst->srcRegIdx(0);
    assert(breg >= 0 && breg < PrecomputedBTB::NUM_REGS);


    DPRINTF(Decode,"[tid:X] BmovTracker::instNeedsToStall [sn:%d] breg=%d, "
            "lastDecAny=%d, "
            "lastBmov[b%d]=%d, lastNBBmov[b%d]=%d, lastExecBmov[b%d]=%d\n",
            inst->seqNum,
            breg, lastDecAny,
            breg, lastDecBmov[breg],
            breg, lastDecNonBitBmov[breg],
            breg, lastExecBmov[breg]);

    // For the given breg: make sure all bmovs have executed
    if (lastExecBmov[breg] >= lastDecBmov[breg]) {
        return false;

    // Else: some bmovs in flight, but they are all bit-type,
    //       so we might be safe to read under the in-flight bmovs
    } else if (lastExecBmov[breg] >= lastDecNonBitBmov[breg]) {
        // Breg must ALREADY be bit-type, else the first bit-type bmov
        //   will clear the reg, so reading-under would be incorrect.
        // Breg must also not be exhausted, i.e. there must be at least
        //   one bit buffered for us to read.

        // Stall if either condition fails
        return ! (this->pbtb->isBregBitTypeAndReady(breg));
    } else {
        return true; // unexecuted bmovs, stall
    }

}

// ==============================================================
//
//                        JV PRECOMPUTED BTB
//
// ==============================================================
//

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
            panic("Unrecognized condition-type in PBTB");
            break;
    }
}

int PrecomputedBTB::m_findMatchingBreg(const struct pbtb_map *pmap,
                                       Addr pcAddr) const {
    // Check each breg, return first valid one matching PC
    int breg = -1;
    for (int i = 0; i < PrecomputedBTB::NUM_REGS; i++) {
        if (pmap->source[i] == pcAddr && pmap->cond_type[i] != NoBranch) {
            breg = i;
            break;
        }
    }
    return breg;
}

PrecomputedBTB::PBTBResultType PrecomputedBTB::m_queryPC(
            const struct pbtb_map *pmap, Addr pcAddr,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out
) const {

    // Queries pbtbp for pcAddr
    // if no match, returns PR_NoMatch
    // if match found:
    //     sets p_breg_out, p_version_out
    //     if taken:
    //        returns PR_Taken, sets p_targetAddr_out
    //     else:
    //        returns PR_NotTaken or PR_Exhausted

    // Note: bregs in the map can be set to NoBranch, but we should never
    //       match against those (i.e. they're invalid, so this works fine)
    PBTBResultType resType;

    const char *debugWhichMap = "?";
    if (pmap == &map_fetch) { debugWhichMap = "F"; }
    if (pmap == &map_final) { debugWhichMap = "D"; }

    // ==== Find matching breg, early return
    int breg = m_findMatchingBreg(pmap, pcAddr);
    if (breg < 0) {
        // Only print this in verbose mode, there's gonna be a lot of these
        // ( ok actually turns out there's WAY TOO MANY OF THESE)
        //DPRINTF(PBTBVerbose, "PBTB QUERIED: (%s): PC:0x%x, MISS\n",
        //        debugWhichMap, pcAddr);
        *p_breg_out = -1;
        return PR_NoMatch;
    }
    assert(breg >= 0 && breg < PrecomputedBTB::NUM_REGS);

    BranchType brType = pmap->cond_type[breg];
    assert(brType != NoBranch);

    // ==== Check based on branch type whether or not it's taken
    if (brType == Taken) {
        resType = PR_Taken;

        DPRINTF(PBTB, "PBTB (%s): HIT b%d: TKN 0x%x -> 0x%x\n",
                debugWhichMap, breg, pcAddr, pmap->target[breg]);

    } else if (brType == LoopN) {
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: LOOP{%d} (%s) 0x%x -?> 0x%x\n",
                debugWhichMap, breg, pmap->cond_val[breg],
                pmap->cond_val[breg]>0 ? "T" : "NT",
                pcAddr, pmap->target[breg]);

        if (     pmap->cond_val[breg] > 0)  { resType = PR_Taken; }
        else if (pmap->cond_val[breg] == 0) { resType = PR_NotTaken; }
        else { // LOOP EXHAUSTED: Will be handled in decode validate/finalize
            resType = PR_Exhaust;
            DPRINTF(PBTB, "PBTB (F): Loop at -1: Exhausted\n");
        }

    } else if (brType == ShiftBit) {
        uint64_t bits = pmap->cond_val[breg];
        uint64_t numbits = pmap->cond_aux_val[breg];
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: BIT{%s} (%s) 0x%x -?> 0x%x\n",
                debugWhichMap, breg,
                debugPrintBottomBits(bits,numbits).c_str(),
                (numbits>0 && bits&1) ? "T" : "NT",
                pcAddr, pmap->target[breg]);

        // cond_val holds bits, cond_aux_val is number of bits valid
        if (numbits > 0) {
            resType = bits & 0x1 ? PR_Taken : PR_NotTaken;
        } else { // no bits, exhaust (will be handled in decode:finalize)
            resType = PR_Exhaust;
            DPRINTF(PBTB, "PBTB (%s): ShiftBit out of bits: Exhausted\n",
                            debugWhichMap);
        }
    } else {
        panic("PBTB got query for unrecognized branch type");
        // return PR_NoMatch; // Alternatively, return no match?
    }

    // ==== Prediction made: return info to caller
    *p_breg_out = breg; // we asserted breg valid earlier
    *p_version_out = pmap->version[breg];
    if (resType == PR_Taken) { *p_targetAddr_out = pmap->target[breg]; }
    return resType;
}

void PrecomputedBTB::m_consumeIter(struct pbtb_map *pmap, int breg) {
    if (breg == -1) { return; }
    assert(breg >= 0 && breg < PrecomputedBTB::NUM_REGS);

    PrecomputedBTB::BranchType brType = pmap->cond_type[breg];

    if (brType == NoBranch) { // (nothing here to consume from)
    } else if (brType == Taken) { // (infinite iterations)
    } else if (brType == LoopN) {
        if (pmap->cond_val[breg] >= 0) { // if not exhausted
            pmap->cond_val[breg]--; // iterations--
        }

    } else if (brType == ShiftBit) {
        if (pmap->cond_aux_val[breg] > 0) { // if not exhausted
            pmap->cond_val[breg] >>= 1; // shift bits down 1
            pmap->cond_aux_val[breg]--; // num_bits--
        }

    } else {
        panic("PBTB consumeIter on unrecognized branch type");
    }
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
    DPRINTF(PBTB, "PBTB: Overwriting fetch PBTB from finalize\n");
    // TODO: logging to both debug flags for now??
    DPRINTF(Decode, "PBTB: Overwriting fetch PBTB from finalize\n");
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
            const StaticInstPtr inst, PCStateBase &pc_inout,
            int *p_breg_out, uint64_t *p_version_out) {

    Addr tgt = pc_inout.instAddr(); //will be overwritten if taken

    PBTBResultType res = m_queryPC(&map_fetch, pc_inout.instAddr(),
            p_breg_out, p_version_out, &tgt);

    m_consumeIter(&map_fetch, *p_breg_out);

    // TODO: I'm not sure how to deal with PCStateBases: this tempAddr
    // thing seems to work, so I'm sticking with it
    auto tempAddr = GenericISA::SimplePCState<4>();
    // TODO: everywhere else uses a unique_ptr<PCState>, is there a reason for
    // that? or is a bare object fine?
    // Old code: auto target=std::make_unique<GenericISA::SimplePCState<4>>();

    // ==== Prediction made: return to caller
    // Return next-fetched PC through pc_inout arg
    if (res == PR_Taken){
        tempAddr.set(tgt);
        set(pc_inout, tempAddr);
    } else { //Else: no match, OR match found but not taken
        inst->advancePC(pc_inout);
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

    // If the breg we query could be modified by hitting it with a pb, we need
    // to make sure we save the breg's state
    // TODO: if map_final.cond_type[breg] == LoopN || ShiftBit { ... }
    //savePrevState(test_breg, );
    m_consumeIter(&map_final, *p_breg_out);

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

// ==============================================================
// Checks if breg is ready to finalize a bit-type branch
// TODO: this is very specific to what's needed for
//       BmovTracker::instNeedsToStall, might be useful later to break it up
//       into a `isBregReady` and a `getBregType`
bool PrecomputedBTB::isBregBitTypeAndReady(int breg) const {
    // breg must already be bit-type, and must not be exhausted
    if (map_final.cond_type[breg] == PrecomputedBTB::BranchType::ShiftBit
        && map_final.cond_aux_val[breg] > 0) {
        return true;
    }
    return false;
}

// ==============================================================

} // namespace o3
} // namespace gem5
