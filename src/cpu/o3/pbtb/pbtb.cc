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

    allFlyingBmovs.clear();
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

        DPRINTF(Decode, "[tid:%i] BmovTracker: decoded [sn:%llu]"
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


        // ==== NEW: keeping track of the numbering might not be sufficient,
        // also push into vec of in-flight bmovs
        struct tracker_entry ent;
        ent.seqNum = inst->seqNum;
        ent.isBmov = true;
        ent.isBitBmov = inst->isBitBmov();
        ent.isPb = false; // in case we decide to also hold onto pbs?
        ent.breg = breg;
        ent.isExecuted = false;
        ent.isSquashed = false;
        allFlyingBmovs.push_back(ent);

        //TOOD: TEMP
        debugDump();
    }
}

void BmovTracker::recordExecBmovFromIew(ThreadID tid,
        InstSeqNum bmovSeq, int breg ) {
    DPRINTF(Decode, "[tid:%i] BmovTracker: bmov executed [sn:%d]: b%d"
        " from iew\n", tid, bmovSeq, breg);

    // NOTE: these should always be increasing, since bmovs for the
    // same branch reg are executed serially
    assert( bmovSeq > lastExecBmov[breg] );
    assert(breg >= 0 && breg < PBTB::NUM_REGS);
    lastExecBmov[breg] = bmovSeq;


    // === Also update the info in our new list of in-flight bmovs
    // NOTE: recordExec will be called once per breg, since we can have
    //        multiple bmovs execute in a cycle
    //       However, there should only every be one bmov per breg per cycle
    //       for now, so we don't have to worry about that.

    auto theBmov = std::find_if(
        allFlyingBmovs.begin(), allFlyingBmovs.end(),
        [bmovSeq](const auto& entry) { return entry.seqNum == bmovSeq; });


    if (theBmov == allFlyingBmovs.end()) {
        DPRINTF(Decode, "[tid:%i] BmovTracker ERROR: no bmov matching"
            " seqNum:%d", bmovSeq);
        debugDump();
        panic("BUG IN BMOVTRACKER");
    }

    theBmov->isExecuted = true;

    //TOOD: TEMP
    debugDump();
}

void BmovTracker::recordCommit(ThreadID tid,
         InstSeqNum commitSeqNum ) {
    //DPRINTF(Decode, "[tid:%d] [sn:%llu] BmovTracker: recordCommit "
    //    "NOT IMPLEMENTED\n", tid, instSeqNum);

    assert(commitSeqNum >= lastCommittedInst);
    lastCommittedInst = commitSeqNum;

    // === Also update the info in our new list of in-flight bmovs

    // everything with seqnum <= commitSeqNum is now committed
    DPRINTF(Decode, "[tid:%d] BmovTracker: recordCommit "
        "for [sn:%d]\n", tid, commitSeqNum);

    auto it = allFlyingBmovs.begin();

    while (it != allFlyingBmovs.end() && it->seqNum <= commitSeqNum) {
        DPRINTF(Decode, "[tid:%d] - bmov [sn:%d] marked committed\n",
            tid, it->seqNum);
        it++;
    }

    // iterator should now be at first non-committed inst
    assert(it == allFlyingBmovs.end() || it->seqNum > commitSeqNum);

    // Delete all the elements up the iterator
    allFlyingBmovs.erase(allFlyingBmovs.begin(), it);

    //TOOD: TEMP
    debugDump();
}

// Note: this doesn't count squashes the Decode itself generates,
// only squashed from ahead of it, e.g. from IEW or commit
void BmovTracker::recordSquashFromAhead(ThreadID tid,
         InstSeqNum squashNum) {
    // everything with seq>squashNum is gone
    DPRINTF(Decode, "[tid:%d] [sn:%llu] BmovTracker: recordingSquash\n",
        tid, squashNum);


    // === New Tracking:

    for (auto& entry: allFlyingBmovs) {
        // young insts (high seqnum) are squashed away
        // i.e. anythinge >= squashNum

        if (entry.seqNum > squashNum) {
        // NOTE: it looks like squashNum is not actually squashed, only
        // everythign > than it. (via looking at load-store queue?)
            DPRINTF(Decode, "[tid:%i] - squashed [sn:%d], b%d\n",
                    tid, entry.seqNum, entry.breg);
            entry.isSquashed = true;
        }
    }
    //TOOD: TEMP
    debugDump();

    // update numbers to account for this latest squash
    //  - we need to reset lastDecBmov, lastDecNonBitBmov, lastExecBmov,
    //    for each of the 32 bregs

    for (int i = 0; i < NUM_PBTB_REGS; i++) {
        lastDecBmov[i] = lastCommittedInst;
        lastDecNonBitBmov[i] = lastCommittedInst;
        lastExecBmov[i] = lastCommittedInst;
    }
    lastDecAny = lastCommittedInst; // this one isnt strictly necessary?

    // walk the list, find the new mins
    // we want the last (i.e. latest) for each of these, so highest seqnum
    for (auto& entry: allFlyingBmovs) {
        // For each non-squashed entry: if it is later than the prev latest,
        // update that counter
        if (entry.isSquashed)
            { continue; }
        int breg = entry.breg;
        int seq = entry.seqNum;

        if (seq > lastDecBmov[breg])
            { lastDecBmov[breg] = seq;}

        if (!entry.isBitBmov && seq > lastDecNonBitBmov[breg])
            { lastDecNonBitBmov[breg] = seq;}

        if (entry.isExecuted && seq > lastExecBmov[breg])
            { lastExecBmov[breg] = seq;}

        if (seq > lastDecAny)
            { lastDecAny = seq; }
    }

    //// VERY VERBOSE: TEMP FOR DEBUGGING
    //for (int breg = 0; breg < NUM_PBTB_REGS; breg++) {
    //DPRINTF(Decode,"[tid:X] BmovTracker: stats: "
    //        "lastDecAny=%d, "
    //        "lastBmov[b%d]=%d, lastNBBmov[b%d]=%d, lastExecBmov[b%d]=%d\n",
    //        lastDecAny,
    //        breg, lastDecBmov[breg],
    //        breg, lastDecNonBitBmov[breg],
    //        breg, lastExecBmov[breg]);
    //}

    /* ===== === OLD: TODO: this was  unfinished

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
    */
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


void BmovTracker::debugDump() {
    DPRINTF(Decode, "===== DUMPING BmovTracker: (Bit/Exec/Sqsh) ====\n");
    for (const auto& entry : allFlyingBmovs) {
        DPRINTF(Decode, "- [sn:%d], b%d, (%c%c%c)\n",
            entry.seqNum, entry.breg,
            //entry.isBmov?'T':'F',
            entry.isBitBmov?'B':'.',
            entry.isExecuted?'X':'.',
            entry.isSquashed?'S':'.');
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


// TODO: we used to do all the printing from here, but now PBTBMap handles it
// should this be removed?
void PBTB::debugDump(int regstart, int regstop) {
    const int which_map = 0; // NOTE: change this manually when testing
                             // If you need to see the other map
    PBTBMap *mapsToPrint[] = {&map_fetch, &map_final};
    PBTBMap *curr_map = mapsToPrint[which_map];

    curr_map->debugDump();
}

void PBTB::debugDump() { debugDump(0, NUM_REGS); }


// ==== Predictor code

// reset all to weak not-taken
void PBTB::clear_predictor() {
    for (int i = 0; i < NUM_REGS; i++) { predictor_ctrs[i] = 1; }
}

void PBTB::write_predictor(int breg, bool taken) {
    // updating 2-bit saturating counter
    int c = predictor_ctrs[breg];
    c += taken ? 1 : -1;
    if (c < 0) { c = 0; }
    if (c > 3) { c = 3; }
    predictor_ctrs[breg] = c;
}

bool PBTB::query_predictor(int breg) {
    // 0 or 1 is not taken, 2 or 3 is taken
    return predictor_ctrs[breg] >= 2;
}

// ==== Undo Code


// Prints out undo stack for the given breg
void PBTB::debugDumpUndo(int breg) {
    //TODO: don't print undo stuff for now
    DPRINTF(PBTBVerbose, " ==== PBTB: Undo stack for b%d (%d actions)\n",
        breg, undo_stacks[breg].size());
    for (const auto &entry : undo_stacks[breg]) {
        DPRINTF(PBTBVerbose, "- %s\n", undoEntryToString(entry));
    };

}

// Prints out all non-empty stacks
void PBTB::debugDumpAllUndo() {
    int num_nonempty = 0;
    for (int breg = 0; breg < NUM_PBTB_REGS; breg++) {
        num_nonempty++;
        if (undo_stacks[breg].size() > 0) {
            debugDumpUndo(breg);
        }
    }

    if (num_nonempty == 0) {
        DPRINTF(PBTBVerbose, "==== PBTB: all undo stacks empty\n");
    }
}

void PBTB::savePrevState(int breg, InstSeqNum seqnum,
                                   undo_action undo) {
    struct undo_entry NEW =
    {
        .seqnum = seqnum,
        .action = undo,
    };
    assert(undo.breg == breg);

    //TODO: temp debug
    debugDumpUndo(breg);

    DPRINTF(PBTB, "saving to undo stack for breg %d: NEW=%s\n",
        breg, undoEntryToString(NEW));


    // Now: The seqnum of the undo-entry we're adding might be lower than the
    // one on top of stack e.g:
    //      TOS:    pb (sn:7, applied in Decode/finalize)
    //      adding: bmov (sn:4, executed in IEW).
    // This can happen because a pb can operate on precomputed bits,
    // so can occur without stalling for the
    // bmov, even though the bmov came first in code order / seqnum
    //
    // However, we want our undo stacks to be in seqnum order, so that we can
    // safely undo to a specific seqnum when squashing.
    //
    // To keep our stacks in seqnum order, we will insert new undo entries
    // BELOW THE TOP OF THE STACK (TOS), but only if they are reorderable, i.e.
    // pbs consuming bits from a SHIFTBIT breg, or bmovs pushing bits to
    // a SHIFTBIT breg (NOTE: NOT bmovs initializing a SHIFTBIT, since that
    // clears the breg). We make sure they're safely reorderable
    // - NEW is undoing a bit-append bmov (type UNPUSH_BITS) and TOS is a pb
    //      (TYPE UNPOP_BITS). We can only safely reorder a pb and bmov, since
    //      that preserves the relative orderings of pbs and bmovs.
    //      Also, we should never have a bmov executing too early, so don't
    //      need to check NEW=pb, TOS=bmov.
    // - The version number is the same between TOS/NEW, and done/undone_ver
    //     ( this makes sure we're consuming/pushing to THE SAME VERSION of a
    //        given breg, not just that the breg happened to be reused )
    // - NEW is from an older (smaller) seqnum than TOS (i.e. it should have
    //       happened earlier in code order)

    // If we don't meet the above criteria, then NEW.seqnum must be >TOS.seqnum

    // Aim to insert at end, but if we find the seqnums are not increasing,
    // walk backwards until we find a seqnum < than us
    auto rit = undo_stacks[breg].rbegin();
    while (rit != undo_stacks[breg].rend())  {
        auto NOS = *rit; // next on stack

        // when we insert, we can insert at insert(rit.base()), which
        // will insert ABOVE the current element

        //          begin                 end
        //  (NULL)? 1      2      3       (NULL?)
        //  rend                  rbegin
        //                                rbegin.base(), // inserts before null
        //                        // or equivalently, AFTER *rbegin()==3

        if (NOS.seqnum < seqnum) { // in order, all good
            break;
        } else {
            // NOS is a more recent inst, we need to insert NEW below it
            DPRINTF(PBTB, "UNDO REORDER: putting new [sn:%d] under [sn:%d]\n",
                NEW.seqnum, NOS.seqnum);

            // Check if it's safe to reorder
            assert(NOS.action.type == utype::U_UNPOP_BITS); // NOS is early pb
            assert(NEW.action.type == utype::U_UNPUSH_BITS); // curr is bmov
            assert(NOS.seqnum > NEW.seqnum); // curr is earlier in code order

            auto ver = NEW.action.done_ver; // all versions should
            assert(NEW.action.done_ver == ver);      // match up (bitqueue ops)
            assert(NEW.action.undone_ver == ver);    // don't increment version
            assert(NOS.action.done_ver == ver);
            assert(NOS.action.undone_ver == ver);


            // If wer'e her, it's safe to reorder, advance by one
            rit++;
        }
    }
    // rit should now be the place to insert at
    // (e.g. if array empty, rit == rend(), so insert at rend.base() == end())
    // (e.g. if just pushing to top of undo stack, rit == rbegin(),
    //       so rit.base() == end(), so will insert at end)
    // (e.g. if we walked to some element X and then broke out, *rit is X,
    //       so rit.base() is after X, so we will insert after X)


    undo_stacks[breg].insert(rit.base(), NEW);
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

// undoes back to (but not including!) squashingSeqNum
void PBTB::unwindSquash(InstSeqNum squashingSeqNum) {
    DPRINTF(PBTB, "PBTB: Squashed, unwinding to before inst [sn:%d]\n",
        squashingSeqNum);

    for (int breg = 0; breg < NUM_REGS; breg++) {
        auto& curr_stack = undo_stacks[breg];

        while (!curr_stack.empty()
            && curr_stack.back().seqnum > squashingSeqNum) {

            DPRINTF(PBTB, "PBTB: (b%d) undoing PBTB op [sn:%d], b%d\n",
                breg, curr_stack.back().seqnum,
                curr_stack.back().action.breg);

            // Do the undo?
            map_final.apply_undo(curr_stack.back().action);
            curr_stack.pop_back();
        }
    }
}


// ==============================================================
//
//                PBTB: PUBLIC Access Functions
//
// ==============================================================
// (these use the per-map functions but can touch multiple maps)


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
    * @param p_exhaust_out PREDICTOR HACK, this will be true if branch was exh.
    * @return Returns PBTBResultType for T, NT, Exhausted, and NoMatch
    * // NOTE: no longer returns Exhaust-type: now will set the exhaust flag
    * //       instead
    */
PBTB::PBTBResultType PBTB::queryFromFetch(
            const StaticInstPtr inst, PCStateBase &pc_inout,
            int *p_breg_out, uint64_t *p_version_out, bool *p_exhaust_out) {

    Addr tgt = pc_inout.instAddr(); //will be overwritten if taken

    PBTBResultType res = map_fetch.queryPC(pc_inout.instAddr(),
            p_breg_out, p_version_out, &tgt);

    map_fetch.consumeIter(*p_breg_out);

    // TODO: I'm not sure how to deal with PCStateBases: this tempAddr
    // thing seems to work, so I'm sticking with it
    //auto tempAddr = GenericISA::SimplePCState<4>();
    //auto tempAddr = pc_inout.as<GenericISA::PCStateWithNext>();
    // TODO: everywhere else uses a unique_ptr<PCState>, is there a reason for
    // that? or is a bare object fine?
    // Old code: auto target=std::make_unique<GenericISA::SimplePCState<4>>();


    // If we matched an exhausted breg, we might fallback to our predictor
    if (res == PBTBResultType::PR_Exhaust) {
        *p_exhaust_out = true;

        switch (PBTB_PREDICTOR_CONF) {
            case PBTB_pred_conf_t::PBTB_Pred_None:
                // If we've disabled the predictor, do nothing,
                // just return PR_Exhaust
                break;
            case PBTB_pred_conf_t::PBTB_Pred_2bit:
                // If we want to use the builtin 2-bit predictor, do that
                // explicitly here, and return the predicted addr / result type
                {
                    bool pred = query_predictor(*p_breg_out);
                    res = pred ? PBTBResultType::PR_Taken :
                                PBTBResultType::PR_NotTaken;
                    if (pred) { tgt = map_fetch.target[*p_breg_out]; }
                }
                break;
            case PBTB_pred_conf_t::PBTB_Pred_vanilla:
                // I guess we also do nothing here, since we let
                // fetch handle this? (again just return exhaust)
                // (god this is an ugly hack)
                break;
            default:
                panic("unimplemented PBTB predictor option");
        }
    } else {
        *p_exhaust_out = false;
    }

    // ==== Prediction made: return to caller
    // Return next-fetched PC through pc_inout arg
    if (res == PBTBResultType::PR_Taken){

        // We need to set the next pc (normally a branch inst does this when
        //   it executes, in the generated isa code)
        pc_inout.as<GenericISA::PCStateWithNext>().npc(tgt);

        // We also need to advancePC so that we actually go to that next pc?
        // normally this would happen in IEW when it hits squashDueToBranch(),
        // so that the branch target is saved in .instAddr(), not in .npc()
        inst->advancePC(pc_inout);

    } else { //Else: no match, OR match found but not taken
        // pc + 4 (probably)
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


    // update the predictor from the pb we just verified (if we got an answer)
    if (res == PBTBResultType::PR_Taken ||
        res == PBTBResultType::PR_NotTaken) {

        bool taken = (res == PBTBResultType::PR_Taken);
        write_predictor(*p_breg_out, taken);
        DPRINTF(PBTB, "PBTB: [sn:%llu] got pb b%d (%s) Updating predictor to "
            "%d/3\n",
            seqnum, *p_breg_out, taken ? "T" : "NT",
            predictor_ctrs[*p_breg_out]);
    }


    if (res != PBTBResultType::PR_Taken) {
        // TODO: This is a horrible pile of hacks but I don't want to switch
        // everything to use PCStates.

        // ALSO NOTE: creating a generic PCState was the cause of a lot of
        // bugs (PCstates track their instruction type/width, i.e. RV64-
        // specific stuff), so I switched to using the existing pc_inout
        // and setting its .npc (see queryFromFetch).
        // However, it shouldn't matter here because we're only returning the
        // address, and we're just using the PCState for a PC+4 or PC+2 that
        // satisfies the type-checker.
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
