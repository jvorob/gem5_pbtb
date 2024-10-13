/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 * CITE IT IF YOU COPY IT.
 */

#include "cpu/o3/pbtb/pbtb_map.hh"

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "debug/PBTB.hh"
#include "debug/PBTBVerbose.hh"

namespace gem5
{

namespace o3
{

const char* PBTBMap::BranchTypeCodes[] = {"N/A", "TKN", "LOP",
                                                 "BIT", "B_C"};
const char* PBTBMap::BranchTypeStrs[] = { "NoBranch", "Taken", "Loop",
                                                 "ShiftBit", "ShiftBit_Clear"};


void PBTBMap::debugDump(int regstart, int regstop) {
    DPRINTF(PBTBVerbose, "PBTB (%s): {\n", dbgId);

    regstart = std::max(0, regstart);
    regstop = std::min(NUM_REGS, regstop);
    int numtoprint = regstop - regstart;

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstart > 0) { DPRINTF(PBTBVerbose, "  ...\n"); }

    for (int reg = regstart; reg < regstop; reg++) {
        auto branchCode = BranchTypeToShortCode( cond_type[reg]);

        DPRINTF(PBTBVerbose, "  %2i: SRC=%8lx TGT=%8lx | %s: %ld %ld\n", reg,
                (uint64_t)source[reg],
                (uint64_t)target[reg],
                branchCode,
                cond_val[reg],
                cond_aux_val[reg]);
    }

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstop < NUM_REGS)
        { DPRINTF(PBTBVerbose, "  ...\n"); }

    // I guess also handle the edge case where we omit them all
    if (numtoprint == 0) { DPRINTF(PBTBVerbose,"  ...\n"); }

    DPRINTF(PBTBVerbose, "}\n");
}
void PBTBMap::debugDump() { debugDump(0, NUM_REGS); }

// =============== Single-breg accessors
//using PBTBMap::breg_data;
// Gets/sets a single bregs value?
struct PBTBMap::breg_data PBTBMap::breg_get( int breg)
{
    assert(breg >= 0 && breg < NUM_REGS);
    struct breg_data retval =
    {
        .version      = version      [breg],
        .source       = source       [breg],
        .target       = target       [breg],
        .cond_type    = cond_type    [breg],
        .cond_val     = cond_val     [breg],
        .cond_aux_val = cond_aux_val [breg],
    };
    return retval;
}

void PBTBMap::breg_set(int breg, const struct breg_data *data) {
    assert(breg >= 0 && breg < PBTBMap::NUM_REGS);
    version     [breg] = data->version;
    source      [breg] = data->source;
    target      [breg] = data->target;
    cond_type   [breg] = data->cond_type;
    cond_val    [breg] = data->cond_val;
    cond_aux_val[breg] = data->cond_aux_val;
}


// =============== Per-Map functions (PRIVATE)

PBTBMap::undo_action PBTBMap::setSource(
                                 int breg, Addr source_addr) {
    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31

    undo_action retval = { // return previous value as undo
        .breg = breg,
        .undone_ver = version[breg],
        .done_ver = version[breg]+1,
        .type = utype::U_FULL,
        .as_overwrite = breg_get(breg),
    };

    source[breg] = source_addr;
    version[breg]++; //TODO: set this to seqNum instead

    // setSource also resets it to an invalid branch
    cond_type[breg]  = BranchType::NoBranch;
    cond_val[breg]     = 0;
    cond_aux_val[breg] = 0;

    return retval;
}
PBTBMap::undo_action PBTBMap::setTarget(
                                 int breg, Addr target_addr) {
    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31
    undo_action retval = { // return previous value as undo action
        .breg = breg,
        .undone_ver = version[breg],
        .done_ver = version[breg]+1,
        .type = utype::U_FULL,
        .as_overwrite = breg_get(breg),
    };

    target[breg] = target_addr;
    version[breg]++; //TODO: set this to seqNum instead

    // setTarget also resets it to a taken branch
    cond_type[breg]    = BranchType::Taken;
    cond_val[breg]     = 0;
    cond_aux_val[breg] = 0;

    return retval;
}

// Sets the condition for a breg in a pmap
// val and n are treated differently depending on conditionType:
// - Taken or NotTaken ignore it
// - LoopTaken will be taken (val) times, then NT once, then repeat
// - ShiftBit takes a bitstring in val, and enqueues the bottom n bits of it
PBTBMap::undo_action PBTBMap::setCondition(
                                  int breg,
                                  BranchType conditionType,
                                  uint64_t val,
                                  int64_t n) {

    // Save prev value for undo-action
    // (if we only do a partial edit, we'll save something else)
    undo_action ret_undo = { // return previous value as undo action
        .breg = breg,
        .undone_ver = version[breg],
        .done_ver = version[breg]+1,
        .type = utype::U_FULL,
        .as_overwrite = breg_get(breg),
    };

    assert(breg >= 0 && breg < NUM_REGS); //breg should be 0-31
    switch (conditionType) {
        case BranchType::NoBranch:
        case BranchType::Taken:
            cond_type[breg]    = conditionType;
            cond_val[breg]     = 0;
            cond_aux_val[breg] = 0;
            version[breg]++; //TODO: set this to seqNum instead
            break;
        case BranchType::LoopN:
            cond_type[breg]    = conditionType;
            cond_val[breg]     = val;
            cond_aux_val[breg] = 0;
            version[breg]++; //TODO: set this to seqNum instead
            break;
        case BranchType::ShiftBit:
            if (cond_type[breg] != BranchType::ShiftBit) {
                // When first switching to BIT mode,
                // reset the bit fifo
                cond_val[breg]     = 0;
                cond_aux_val[breg] = 0;

                // Also, version should only increment on first
                // change (so we can produce and consume simultaneously
                //         as long as enough bits are in the queue)
                version[breg]++; //TODO: set this to seqNum instead
            } else {
                // if we're only appending, return a partial undo action
                // so we can re-order it with pb consumes
                ret_undo.type = utype::U_UNPUSH_BITS;
                ret_undo.done_ver = ret_undo.undone_ver; // ver doesn't change
                ret_undo.as_pushed_bits = 1;
            }
            cond_type[breg]    = BranchType::ShiftBit;

            //if (conditionType == ShiftBit_Clear) { //clear fifo
            //    cond_val[breg]     = 0; // bits
            //    cond_aux_val[breg] = 0; // num_bits
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

            cond_val[breg] |= bottom_n_bits <<
                                cond_aux_val[breg];
            cond_aux_val[breg] += n; // num_bits+=n

            uint64_t bits = cond_val[breg];
            uint64_t numbits = cond_aux_val[breg];
            DPRINTF(PBTB, "PBTB (X): bmovc_bit b%d: data{%s}(%d)\n",
                    breg, debugPrintBottomBits(bits,numbits).c_str(), numbits);

            }

            // make sure we don't touch the sign bit (just to be safe)
            // TODO: we should probably allow 64, crash at 65?
            // TODO: either way, this should trigger an exception I think
            assert(cond_aux_val[breg] <= 63);
            break;
        case BranchType::ShiftBit_Clear:
        default:
            panic("Unrecognized condition-type in PBTB");
            break;
    }

    return ret_undo;
}

int PBTBMap::findMatchingBreg(Addr pcAddr) const {
    // Check each breg, return first valid one matching PC
    int breg = -1;
    for (int i = 0; i < PBTBMap::NUM_REGS; i++) {
        if (source[i] == pcAddr && cond_type[i] != BranchType::NoBranch) {
            breg = i;
            break;
        }
    }
    return breg;
}

PBTBMap::PBTBResultType PBTBMap::queryPC( Addr pcAddr,
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

    // ==== Find matching breg, early return
    int breg = findMatchingBreg(pcAddr);
    if (breg < 0) {
        // Only print this in verbose mode, there's gonna be a lot of these
        // ( ok actually turns out there's WAY TOO MANY OF THESE)
        //DPRINTF(PBTBVerbose, "PBTB QUERIED: (%s): PC:0x%x, MISS\n",
        //        dbgId, pcAddr);
        *p_breg_out = -1;
        return PR_NoMatch;
    }
    assert(breg >= 0 && breg < PBTBMap::NUM_REGS);

    BranchType brType = cond_type[breg];
    assert(brType != BranchType::NoBranch);

    // ==== Check based on branch type whether or not it's taken
    if (brType == BranchType::Taken) {
        resType = PR_Taken;

        DPRINTF(PBTB, "PBTB (%s): HIT b%d: TKN 0x%x -> 0x%x\n",
                dbgId, breg, pcAddr, target[breg]);

    } else if (brType == BranchType::LoopN) {
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: LOOP{%d} (%s) 0x%x -?> 0x%x\n",
                dbgId, breg, cond_val[breg],
                cond_val[breg]>0 ? "T" : "NT",
                pcAddr, target[breg]);

        if (     cond_val[breg] > 0)  { resType = PR_Taken; }
        else if (cond_val[breg] == 0) { resType = PR_NotTaken; }
        else { // LOOP EXHAUSTED: Will be handled in decode validate/finalize
            resType = PR_Exhaust;
            DPRINTF(PBTB, "PBTB (F): Loop at -1: Exhausted\n");
        }

    } else if (brType == BranchType::ShiftBit) {
        uint64_t bits = cond_val[breg];
        uint64_t numbits = cond_aux_val[breg];
        DPRINTF(PBTB, "PBTB (%s): HIT b%d: BIT{%s} (%s) 0x%x -?> 0x%x\n",
                dbgId, breg,
                debugPrintBottomBits(bits,numbits).c_str(),
                (numbits>0 && bits&1) ? "T" : "NT",
                pcAddr, target[breg]);

        // cond_val holds bits, cond_aux_val is number of bits valid
        if (numbits > 0) {
            resType = bits & 0x1 ? PR_Taken : PR_NotTaken;
        } else { // no bits, exhaust (will be handled in decode:finalize)
            resType = PR_Exhaust;
            DPRINTF(PBTB, "PBTB (%s): ShiftBit out of bits: Exhausted\n",
                            dbgId);
        }
    } else {
        panic("PBTB got query for unrecognized branch type");
        // return PR_NoMatch; // Alternatively, return no match?
    }

    // ==== Prediction made: return info to caller
    *p_breg_out = breg; // we asserted breg valid earlier
    *p_version_out = version[breg];
    if (resType == PR_Taken) { *p_targetAddr_out = target[breg]; }
    return resType;
}

PBTBMap::undo_action PBTBMap::consumeIter(int breg)
{
    // Default our undo action to no change
    undo_action ret_undo = {
        .breg = breg,
        .undone_ver = version[breg],
        .done_ver = version[breg],  // consuming doesn't change version
        .type = utype::U_NONE,
        .as_pushed_bits = 0, // shut up the type checker
    } ;

    if (breg == -1) { return ret_undo; }
    assert(breg >= 0 && breg < PBTBMap::NUM_REGS);

    PBTBMap::BranchType brType = cond_type[breg];

    if (brType == BranchType::NoBranch) { // (nothing here to consume from)
    } else if (brType == BranchType::Taken) { // (infinite iterations)
    } else if (brType == BranchType::LoopN) {
        if (cond_val[breg] >= 0) { // if not exhausted
            // we could do this fancier, but for now just do a full undo
            ret_undo.type = utype::U_FULL;
            ret_undo.as_overwrite = breg_get(breg);

            cond_val[breg]--; // iterations--
        }

    } else if (brType == BranchType::ShiftBit) {
        if (cond_aux_val[breg] > 0) { // if not exhausted
            // pb is consuming a bit: undo action is unconsuming it
            ret_undo.type = utype::U_UNPOP_BITS;
            bool bit = cond_val[breg] & 1;
            ret_undo.as_consumed_bits = BitVec64{bit}; // only 1 bit?

            cond_val[breg] >>= 1; // shift bits down 1
            cond_aux_val[breg]--; // num_bits--
        }

    } else {
        panic("PBTB consumeIter on unrecognized branch type");
    }

    return ret_undo;
}


} // namespace o3
} // namespace gem5
