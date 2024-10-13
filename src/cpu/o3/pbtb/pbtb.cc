/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 * CITE IT IF YOU COPY IT.
 */

#include "cpu/o3/pbtb/pbtb.hh" // Technically cpu.hh includes this already?

#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "debug/Decode.hh"
#include "debug/PBTB.hh"
#include "debug/PBTBVerbose.hh"

namespace gem5
{

namespace o3
{

// TODO: make this return an std::string?
const std::string debugPrintBottomBits(
        uint64_t bits, int n, bool lsb_first)
{
    char debug_bit_buff[65];

    debug_bit_buff[64] = '\0'; //null terminate just to be safe
    n = std::min(n, 64);

    int i;
    for (i = 0; i < n; i++) {
        // MSB is at (bits >> (n-1))
        // LSB is at 0
        int bit_offset = lsb_first ? i : n - i - 1;
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

    for (int i = 0; i < PBTB::NUM_REGS; i++) {
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
        assert(breg >= 0 && breg < PBTB::NUM_REGS);
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
    assert(breg >= 0 && breg < PBTB::NUM_REGS);
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

    for (int bi = 0; bi < PBTB::NUM_REGS; bi++) {
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
    assert(breg >= 0 && breg < PBTB::NUM_REGS);


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

std::string PBTB::name() const {
    return cpu->name() + ".pbtb";
}


// TODO: we used to do all the printing from here, but not PBTBMap handles it
// should this be removed?
void PBTB::debugDump(int regstart, int regstop) {
    const int which_map = 0; // NOTE: change this manually when testing
                             // If you need to see the other map
    PBTBMap *mapsToPrint[] = {&map_fetch, &map_final};
    PBTBMap *curr_map = mapsToPrint[which_map];

    curr_map->debugDump();
}

void PBTB::debugDump() { debugDump(0, NUM_REGS); }


void PBTB::savePrevState(int breg, InstSeqNum seqnum,
                                   undo_action undo) {
    struct undo_entry undo_entry =
    {
        .seqnum = seqnum,
        .action = undo,
    };

    //TODO: don't print undo stuff for now
    DPRINTF(PBTB, "Current undo stack:\n");
    for (const auto &entry : undo_stack) {
        DPRINTF(PBTB, "- %s\n", undoEntryToString(entry));
    };

    DPRINTF(PBTB, "saving to undo stack: %s\n",
        undoEntryToString(undo_entry));


    // actions should arrive in order
    //assert(undo_stack.empty() || undo_stack.back().seqnum < seqnum);
    // TODO TEMP: we need to allow out-of-order if theyre in diff bregs,
    //            or same-breg but push/pop (same version)
    if (!( undo_stack.empty() || undo_stack.back().seqnum < seqnum )) {
        DPRINTF(PBTB, "WARNING: out-of-order undo-entries, NOT FINISHED\n");
    }
    undo_stack.push_back(undo_entry);
}

std::string PBTBMap::bdataToString(const struct breg_data &bdata) {
    auto brType = bdata.cond_type;

    if (brType == BranchType::NoBranch) {
        return csprintf("NOBRANCH");
    } else if (brType == BranchType::Taken) {
        return csprintf("TKN 0x%x -> 0x%x", bdata.source, bdata.target);

    } else if (brType == BranchType::LoopN) {
        const char* outcome = bdata.cond_val >0 ? "T" :
                              bdata.cond_val==0 ? "NT":
                                                  "EX";
        return csprintf("LOOP{%d} (%s) 0x%x -?> 0x%x",
                bdata.cond_val, outcome,
                bdata.source, bdata.target);

    } else if (brType == BranchType::ShiftBit) {
        uint64_t bits = bdata.cond_val;
        uint64_t numbits = bdata.cond_aux_val;
        const char* outcome = numbits == 0 ? "EX" : (bits & 0x1 ? "T": "NT");
        return csprintf("BIT{%s} (%s) 0x%x -?> 0x%x",
                debugPrintBottomBits(bits,numbits).c_str(),
                outcome,
                bdata.source, bdata.target);

    } else {
        panic("bdata got unrecognized branch type");
    }
}

// undoes back to and including squashingSeqNum
void PBTB::unwindSquash(InstSeqNum squashingSeqNum) {
    DPRINTF(PBTB, "PBTB: Unwinding to inst [sn:%d]\n", squashingSeqNum);
    while (!undo_stack.empty()) {
        struct undo_entry curr = undo_stack.back();
        if (curr.seqnum >= squashingSeqNum) {
            DPRINTF(PBTB, "PBTB: undoing PBTB op [sn:%d], b%d\n",
                squashingSeqNum, curr.action.breg);
            // TODO:
            panic("breg undo not implemented");
            //breg_set(&map_final, curr.breg, &curr.prev_state);
            undo_stack.pop_back();
        }
    }
}


// =============== PUBLIC Modification functions
// (these use the per-map functions but touch multiple maps)


// == Each of these should correspond to one bmov instruction
void PBTB::setSource(int breg, InstSeqNum seqnum, Addr source_addr) {
    map_fetch.setSource(breg, source_addr);
    auto undo = map_final.setSource(breg, source_addr);
    savePrevState(breg, seqnum, undo);
}
void PBTB::setTarget(int breg, InstSeqNum seqnum, Addr target_addr) {
    //savePrevState(breg, seqnum);
    map_fetch.setTarget(breg, target_addr);
    auto undo = map_final.setTarget(breg, target_addr);
    savePrevState(breg, seqnum, undo);
}
void PBTB::setCondition(int breg, InstSeqNum seqnum,
                    BranchType conditionType, uint64_t val, int64_t n) {
    //savePrevState(breg, seqnum);
    map_fetch.setCondition(breg, conditionType, val, n);
    auto undo = map_final.setCondition(breg, conditionType, val, n);
    savePrevState(breg, seqnum, undo);
}

// Just an alias for the previous one
// We only use n for the ShiftBit branch type, so can omit it otherwise
void PBTB::setCondition(int breg, InstSeqNum seqnum,
                    BranchType conditionType, uint64_t val) {
    setCondition(breg, seqnum, conditionType, val, 0);
}


void PBTB::squashFinalizeToFetch() {
    DPRINTF(PBTB, "PBTB: Overwriting fetch PBTB from finalize\n");
    // TODO: logging to both debug flags for now??
    DPRINTF(Decode, "PBTB: Overwriting fetch PBTB from finalize\n");
    //TODO: assert that they're equal barring loop counts / bit counts
    map_fetch.setFrom(map_final);
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
PBTB::PBTBResultType PBTB::queryFromFetch(
            const StaticInstPtr inst, PCStateBase &pc_inout,
            int *p_breg_out, uint64_t *p_version_out) {

    Addr tgt = pc_inout.instAddr(); //will be overwritten if taken

    PBTBResultType res = map_fetch.queryPC(pc_inout.instAddr(),
            p_breg_out, p_version_out, &tgt);

    map_fetch.consumeIter(*p_breg_out);

    // TODO: I'm not sure how to deal with PCStateBases: this tempAddr
    // thing seems to work, so I'm sticking with it
    auto tempAddr = GenericISA::SimplePCState<4>();
    // TODO: everywhere else uses a unique_ptr<PCState>, is there a reason for
    // that? or is a bare object fine?
    // Old code: auto target=std::make_unique<GenericISA::SimplePCState<4>>();

    // ==== Prediction made: return to caller
    // Return next-fetched PC through pc_inout arg
    if (res == PBTBResultType::PR_Taken){
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
PBTB::PBTBResultType PBTB::queryFromDecode(
            const StaticInstPtr inst,  Addr pcAddr, InstSeqNum seqnum,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out) {



    PBTBResultType res = map_final.queryPC( pcAddr,
            p_breg_out, p_version_out, p_targetAddr_out);

    // If the breg we query could be modified by hitting it with a pb, we need
    // to make sure we save the breg's state

    undo_action undo = map_final.consumeIter(*p_breg_out);
    if (undo.type != utype::U_NONE) {
        // Undo type will only be non-None if we hit a loop or bit-branch
        savePrevState(*p_breg_out, seqnum, undo);
    }

    if (res != PBTBResultType::PR_Taken) {
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
bool PBTB::isBregBitTypeAndReady(int breg) const {
    // breg must already be bit-type, and must not be exhausted
    if (map_final.cond_type[breg] == BranchType::ShiftBit
        && map_final.cond_aux_val[breg] > 0) {
        return true;
    }
    return false;
}

//
//bool PBTBMap::pbCouldModifyState(int breg) const
//{
//    if (breg < 0) { return false; }
//    assert(breg >= 0 && breg < PBTB::NUM_REGS);
//
//    // TODO: technically we don't care if branch is exhausted?
//    //       but that shouldn't matter since that only happens in exceptions,
//    switch ( map_final.cond_type[breg] ) {
//        case PBTB::BranchType::LoopN:
//        case PBTB::BranchType::ShiftBit:
//            return true;
//        case PBTB::BranchType::NoBranch:
//        case PBTB::BranchType::Taken:
//            return false;
//        case PBTB::BranchType::ShiftBit_Clear:
//            panic("ShiftBit_Clear is not a valid branchtype");
//    }
//    panic("Oops, unexpected switch val");
//}
//
// ==============================================================

} // namespace o3
} // namespace gem5
