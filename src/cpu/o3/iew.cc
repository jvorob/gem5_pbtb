/*
 * Copyright (c) 2010-2013, 2018-2019 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// @todo: Fix the instantaneous communication among all the stages within
// iew.  There's a clear delay between issue and execute, yet backwards
// communication happens simultaneously.

#include "cpu/o3/iew.hh"

#include <queue>

#include "cpu/checker/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/fu_pool.hh"
#include "cpu/o3/limits.hh"
#include "cpu/timebuf.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/IEW.hh"
#include "debug/O3PipeView.hh"
#include "debug/PBTB.hh"
#include "params/BaseO3CPU.hh"

namespace gem5
{

namespace o3
{

IEW::IEW(CPU *_cpu, const BaseO3CPUParams &params)
    : issueToExecQueue(params.backComSize, params.forwardComSize),
      cpu(_cpu),
      instQueue(_cpu, this, params),
      ldstQueue(_cpu, this, params),
      fuPool(params.fuPool),
      commitToIEWDelay(params.commitToIEWDelay),
      renameToIEWDelay(params.renameToIEWDelay),
      issueToExecuteDelay(params.issueToExecuteDelay),
      dispatchWidth(params.dispatchWidth),
      issueWidth(params.issueWidth),
      wbNumInst(0),
      wbCycle(0),
      wbWidth(params.wbWidth),
      numThreads(params.numThreads),
      iewStats(cpu)
{
    if (dispatchWidth > MaxWidth)
        fatal("dispatchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             dispatchWidth, static_cast<int>(MaxWidth));
    if (issueWidth > MaxWidth)
        fatal("issueWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             issueWidth, static_cast<int>(MaxWidth));
    if (wbWidth > MaxWidth)
        fatal("wbWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             wbWidth, static_cast<int>(MaxWidth));

    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    // Setup wire to read instructions coming from issue.
    fromIssue = issueToExecQueue.getWire(-issueToExecuteDelay);

    // Instruction queue needs the queue between issue and execute.
    instQueue.setIssueToExecuteQueue(&issueToExecQueue);

    for (ThreadID tid = 0; tid < MaxThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    skidBufferMax = (renameToIEWDelay + 1) * params.renameWidth;

    // JV PBTB
    cpu->pbtb.tracker.reset();
}

std::string
IEW::name() const
{
    return cpu->name() + ".iew";
}

void
IEW::regProbePoints()
{
    ppDispatch = new ProbePointArg<DynInstPtr>(
            cpu->getProbeManager(), "Dispatch");
    ppMispredict = new ProbePointArg<DynInstPtr>(
            cpu->getProbeManager(), "Mispredict");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction starts to execute.
     */
    ppExecute = new ProbePointArg<DynInstPtr>(
            cpu->getProbeManager(), "Execute");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction execution completes and it is marked ready to commit.
     */
    ppToCommit = new ProbePointArg<DynInstPtr>(
            cpu->getProbeManager(), "ToCommit");
}

IEW::IEWStats::IEWStats(CPU *cpu)
    : statistics::Group(cpu, "iew"),
    ADD_STAT(idleCycles, statistics::units::Cycle::get(),
             "Number of cycles IEW is idle"),
    ADD_STAT(squashCycles, statistics::units::Cycle::get(),
             "Number of cycles IEW is squashing"),
    ADD_STAT(blockCycles, statistics::units::Cycle::get(),
             "Number of cycles IEW is blocking"),
    ADD_STAT(unblockCycles, statistics::units::Cycle::get(),
             "Number of cycles IEW is unblocking"),
    ADD_STAT(dispatchedInsts, statistics::units::Count::get(),
             "Number of instructions dispatched to IQ"),
    ADD_STAT(dispSquashedInsts, statistics::units::Count::get(),
             "Number of squashed instructions skipped by dispatch"),
    ADD_STAT(dispLoadInsts, statistics::units::Count::get(),
             "Number of dispatched load instructions"),
    ADD_STAT(dispStoreInsts, statistics::units::Count::get(),
             "Number of dispatched store instructions"),
    ADD_STAT(dispNonSpecInsts, statistics::units::Count::get(),
             "Number of dispatched non-speculative instructions"),
    ADD_STAT(iqFullEvents, statistics::units::Count::get(),
             "Number of times the IQ has become full, causing a stall"),
    ADD_STAT(lsqFullEvents, statistics::units::Count::get(),
             "Number of times the LSQ has become full, causing a stall"),
    ADD_STAT(memOrderViolationEvents, statistics::units::Count::get(),
             "Number of memory order violations"),
    ADD_STAT(predictedTakenIncorrect, statistics::units::Count::get(),
             "Number of branches that were predicted taken incorrectly"),
    ADD_STAT(predictedNotTakenIncorrect, statistics::units::Count::get(),
             "Number of branches that were predicted not taken incorrectly"),
    ADD_STAT(branchMispredicts, statistics::units::Count::get(),
             "Number of branch mispredicts detected at execute",
             predictedTakenIncorrect + predictedNotTakenIncorrect),
    executedInstStats(cpu),
    ADD_STAT(instsToCommit, statistics::units::Count::get(),
             "Cumulative count of insts sent to commit"),
    ADD_STAT(writebackCount, statistics::units::Count::get(),
             "Cumulative count of insts written-back"),
    ADD_STAT(producerInst, statistics::units::Count::get(),
             "Number of instructions producing a value"),
    ADD_STAT(consumerInst, statistics::units::Count::get(),
             "Number of instructions consuming a value"),
    ADD_STAT(wbRate, statistics::units::Rate<
                statistics::units::Count, statistics::units::Cycle>::get(),
             "Insts written-back per cycle"),
    ADD_STAT(wbFanout, statistics::units::Rate<
                statistics::units::Count, statistics::units::Count>::get(),
             "Average fanout of values written-back"),

    /* ============ JV: PBTB Stats ========== */
    ADD_STAT(pbtbFinalizedBmovs, statistics::units::Count::get(),
            "Number of bmov insts that pass the finalize point"),
    ADD_STAT(pbtbFinalizedPbs, statistics::units::Count::get(),
            "Number of pb insts that pass the finalize point"),
    ADD_STAT(pbtbFinalizedPbsThatBlocked, statistics::units::Count::get(),
            "Number of finalized pbs that had to stall at least a cycle "
            "for in-flight bmovs"),
    ADD_STAT(pbtbSquashes, statistics::units::Count::get(),
            "Number of pbtb squashes (due to a pb 'mispredict' at fetch)"),
    ADD_STAT(pbtbBlockedCycles, statistics::units::Count::get(),
            "Number of cycles spent blocked on a pb waiting for a bmov"),
    ADD_STAT(pbtbFinalizedImaginedPbs, statistics::units::Count::get(),
            "Number of finalized non-pb insts that had predicted as pbs "
            "(FinalizedPbs+ImaginedPbs should sum to the 6 FinalStates)"),

    // =============== PBTB: Finalize Outcomes: =============
    ADD_STAT(pbtbFinalState_ReadyCorr, statistics::units::Count::get(),
            "Number of pbs that had fetched with up-to-date pbtb info, "
            "and therefore came out correct in finalize"),
    ADD_STAT(pbtbFinalState_ReadyMisp, statistics::units::Count::get(),
            "Number of pbs that fetched with up-to-date pbtb info, but mis"
            "predicted (SHOULD BE 0? Only happen when fetch-pbtb desyncs)"),
    ADD_STAT(pbtbFinalState_ExhaustedCorr, statistics::units::Count::get(),
            "Number of pbs that were fetched as exhausted, but outcome was "
            "predicted correctly"),
    ADD_STAT(pbtbFinalState_ExhaustedMisp, statistics::units::Count::get(),
            "Number of pbs that were fetched as exhausted, but the branch "
            "predictor mispredicted"),
    ADD_STAT(pbtbFinalState_WrongVersionCorr,
            statistics::units::Count::get(),
            "Number of pbs that fetched with an outdated pbtb entry, "
            "but that coincidentally had the right outcome. (SHOULD BE "
            "CLOSE TO 0? Almost always lead to desync on subsequent pbs?)"),
    ADD_STAT(pbtbFinalState_WrongVersionMisp,
            statistics::units::Count::get(),
            "Number of pbs that fetched with an outdated pbtb entry, "
            "and were squashed as a result.")
{
    instsToCommit
        .init(cpu->numThreads)
        .flags(statistics::total);

    writebackCount
        .init(cpu->numThreads)
        .flags(statistics::total);

    producerInst
        .init(cpu->numThreads)
        .flags(statistics::total);

    consumerInst
        .init(cpu->numThreads)
        .flags(statistics::total);

    wbRate
        .flags(statistics::total);
    wbRate = writebackCount / cpu->baseStats.numCycles;

    wbFanout
        .flags(statistics::total);
    wbFanout = producerInst / consumerInst;
}

IEW::IEWStats::ExecutedInstStats::ExecutedInstStats(CPU *cpu)
    : statistics::Group(cpu),
    ADD_STAT(numSquashedInsts, statistics::units::Count::get(),
             "Number of squashed instructions skipped in execute"),
    ADD_STAT(numSwp, statistics::units::Count::get(),
             "Number of swp insts executed")
{
    numSwp
        .init(cpu->numThreads)
        .flags(statistics::total);
}

void
IEW::startupStage()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        toRename->iewInfo[tid].usedIQ = true;
        toRename->iewInfo[tid].freeIQEntries =
            instQueue.numFreeEntries(tid);

        toRename->iewInfo[tid].usedLSQ = true;
        toRename->iewInfo[tid].freeLQEntries =
            ldstQueue.numFreeLoadEntries(tid);
        toRename->iewInfo[tid].freeSQEntries =
            ldstQueue.numFreeStoreEntries(tid);
    }

    // Initialize the checker's dcache port here
    if (cpu->checker) {
        cpu->checker->setDcachePort(&ldstQueue.getDataPort());
    }

    cpu->activateStage(CPU::IEWIdx);

    // JV PBTB
    cpu->pbtb.tracker.reset();
}

void
IEW::clearStates(ThreadID tid)
{
    toRename->iewInfo[tid].usedIQ = true;
    toRename->iewInfo[tid].freeIQEntries =
        instQueue.numFreeEntries(tid);

    toRename->iewInfo[tid].usedLSQ = true;
    toRename->iewInfo[tid].freeLQEntries = ldstQueue.numFreeLoadEntries(tid);
    toRename->iewInfo[tid].freeSQEntries = ldstQueue.numFreeStoreEntries(tid);
}

void
IEW::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from commit.
    fromCommit = timeBuffer->getWire(-commitToIEWDelay);

    // Setup wire to write information back to previous stages.
    toRename = timeBuffer->getWire(0);
    toDecode = timeBuffer->getWire(0); // JV PBTB
    toFetch = timeBuffer->getWire(0);

    // Instruction queue also needs main time buffer.
    instQueue.setTimeBuffer(tb_ptr);
}

void
IEW::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to read information from rename queue.
    fromRename = renameQueue->getWire(-renameToIEWDelay);
}

void
IEW::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    iewQueue = iq_ptr;

    // Setup wire to write instructions to commit.
    toCommit = iewQueue->getWire(0);
}

void
IEW::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;

    ldstQueue.setActiveThreads(at_ptr);
    instQueue.setActiveThreads(at_ptr);
}

void
IEW::setScoreboard(Scoreboard *sb_ptr)
{
    scoreboard = sb_ptr;
}

bool
IEW::isDrained() const
{
    bool drained = ldstQueue.isDrained() && instQueue.isDrained();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!insts[tid].empty()) {
            DPRINTF(Drain, "%i: Insts not empty.\n", tid);
            drained = false;
        }
        if (!skidBuffer[tid].empty()) {
            DPRINTF(Drain, "%i: Skid buffer not empty.\n", tid);
            drained = false;
        }
        drained = drained && dispatchStatus[tid] == Running;
    }

    // Also check the FU pool as instructions are "stored" in FU
    // completion events until they are done and not accounted for
    // above
    if (drained && !fuPool->isDrained()) {
        DPRINTF(Drain, "FU pool still busy.\n");
        drained = false;
    }

    return drained;
}

void
IEW::drainSanityCheck() const
{
    assert(isDrained());

    instQueue.drainSanityCheck();
    ldstQueue.drainSanityCheck();
}

void
IEW::takeOverFrom()
{
    // Reset all state.
    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    instQueue.takeOverFrom();
    ldstQueue.takeOverFrom();
    fuPool->takeOverFrom();

    startupStage();
    cpu->activityThisCycle();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    for (int i = 0; i < issueToExecQueue.getSize(); ++i) {
        issueToExecQueue.advance();
    }
}

void
IEW::squash(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] Squashing all instructions.\n", tid);

    // ================ JV PBTB =======================
    // everything with seq>squashNum is gone
    InstSeqNum squashNum = fromCommit->commitInfo[tid].doneSeqNum;
    //JV PBTB: record committed inst and/or sanity check to make sure
    //         we haven't squashed in an illegal way
    cpu->pbtb.tracker.recordSquashFromAhead(tid, squashNum);
    cpu->pbtb.unwindSquash(squashNum); // roll back main pbtb

    // once main pbtb is rewound, also update fetch pbtb?
    cpu->pbtb.squashFinalizeToFetch();
    // ================ END PBTB

    // Tell the IQ to start squashing.
    instQueue.squash(tid);

    // Tell the LDSTQ to start squashing.
    ldstQueue.squash(fromCommit->commitInfo[tid].doneSeqNum, tid);
    updatedQueues = true;

    // Clear the skid buffer in case it has any data in it.
    DPRINTF(IEW,
            "Removing skidbuffer instructions until "
            "[sn:%llu] [tid:%i]\n",
            fromCommit->commitInfo[tid].doneSeqNum, tid);

    while (!skidBuffer[tid].empty()) {
        if (skidBuffer[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (skidBuffer[tid].front()->isStore() ||
            skidBuffer[tid].front()->isAtomic()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        skidBuffer[tid].pop();
    }

    emptyRenameInsts(tid);


}

void
IEW::squashDueToBranch(const DynInstPtr& inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] [sn:%llu] Squashing from a specific instruction,"
            " PC: %s "
            "\n", tid, inst->seqNum, inst->pcState() );

    //TODO JV: We should never hit this case, since we no longer squash
    // from branches, by the time a pb passes decode it should be verified
    // as correct
    panic("JV PBTB Should never squash due to branch in IEW!\n");

    if (!toCommit->squash[tid] ||
            inst->seqNum < toCommit->squashedSeqNum[tid]) {
        toCommit->squash[tid] = true;
        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->branchTaken[tid] = inst->pcState().branching();

        set(toCommit->pc[tid], inst->pcState());
        inst->staticInst->advancePC(*toCommit->pc[tid]);

        toCommit->mispredictInst[tid] = inst;
        toCommit->includeSquashInst[tid] = false;

        wroteToTimeBuffer = true;
    }

}

void
IEW::squashDueToPBTB(const DynInstPtr& inst, ThreadID tid)
{
    // Target should have been updated when we queried the PBTB?
    DPRINTF(IEW, "[tid:%i] [sn:%llu] Squashing from pbtb mispredict at "
            "dispatch (correct target: 0x%x).\n",
            tid, inst->seqNum,
            inst->readPredTarg().instAddr());

    // ================ JV PBTB:  NOTIFY COMMIT OF SQUASH
    // Original squash behavior on a branch mispredict is to tell commit
    // about the squash, and it will echo it back to us in 2 cycles.

    // NEW BEHAVIOR: We still need to tell commit about the squash
    //   to clear out the ROB (instructions unfortunately get put in ROB
    //   as soon as they leave rename), but we want it to not echo that
    //   squash to anyone

    // Instead: we tell commit about the squash, but make it a weak squash.
    // Weak squash is overridden by any actual squash in the same cycle.
    // We also change the logic for the frontend stages so they
    // handle commit squashes first, and weak squashes only if no commit squash

    if (!toCommit->squash[tid] ||
        inst->seqNum < toCommit->squashedSeqNum[tid]) {

        // NOTE: Originally, this allowed us to override a squash from
        // a younger branch in the same cycle, but with PBTB that
        // should never happen (PBTB squashes happen in dispatch and in-order)
        // (any other squashes should be happening in exec, and be older insts)
        if (toCommit->squash[tid]) {
            assert(!(inst->seqNum < toCommit->squashedSeqNum[tid]));
        }

        //toCommit->squash[tid] = true;
        toCommit->weakSquash[tid] = true; // JV: ONLY WEAKLY SQUASH
        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->branchTaken[tid] = inst->readPredTaken()
                                  || inst->isUncondCtrl();
        // OLD:  toCommit->branchTaken[tid] = inst->pcState().branching();

        // TODO JV PBTB: originally this used advancePC to get the target
        // from the branch instructions,
        // However, now the pbs dont actually have a target
        // Instead, finalizing a pb updates its inst->predTarg
        set(toCommit->pc[tid], inst->readPredTarg());

        // OLD:
        //set(toCommit->pc[tid], inst->pcState());
        //inst->staticInst->advancePC(*toCommit->pc[tid]);

        toCommit->mispredictInst[tid] = inst;
        toCommit->includeSquashInst[tid] = false;

        wroteToTimeBuffer = true;
    }

    // ================ JV PBTB: NOTIFY FRONTEND
    // Fetch will use this to update predictor/refetch
    // Fetch, Decode, Rename should squash

    // TODO: Send back mispredict information.
    toFetch->iewInfo[tid].branchMispredict = true;
    toFetch->iewInfo[tid].predIncorrect = true;
    toFetch->iewInfo[tid].mispredictInst = inst;
    toFetch->iewInfo[tid].squash = true;
    toFetch->iewInfo[tid].doneSeqNum = inst->seqNum;

    // TODO JV PBTB: originally this used the branchTarget from the branch
    // However, now the pbs dont actually have a target
    // Instead, we update the inst->predTarg
    set(toFetch->iewInfo[tid].nextPC, inst->readPredTarg());
    //OLD: set(toFetch->iewInfo[tid].nextPC, *inst->branchTarget());

    // ALSO on squash:
    toFetch->iewInfo[tid].branchTaken = inst->readPredTaken()
                                     || inst->isUncondCtrl();
    toFetch->iewInfo[tid].squashInst = inst;

    // PBTB: Reset the fetch stage's PBTB to the finalize one
    cpu->pbtb.squashFinalizeToFetch();

    wroteToTimeBuffer = true;


    // =========  SQUASH OUR OWN INSTS:

    //TODO: how do we instruct preceding stages to clear themselves?


    // We DONT want to clear the inst-queue or ldst queue,
    // those are after dispatch

    // Clear the skid buffer in case it has any data in it.
    DPRINTF(IEW,
            "Removing skidbuffer instructions until "
            "[sn:%llu] [tid:%i]\n",
            fromCommit->commitInfo[tid].doneSeqNum, tid);

    // == NOTE: I'm setting insts to squashed here instead of waiting
    // for ROB to do it, which I THINK is less correct? but should
    // make sure they don't go off and start doing stuff while we're
    // waiting for commit to get back to us?
    // Will see if this comes back to bite me
    while (!skidBuffer[tid].empty()) {
        if (skidBuffer[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (skidBuffer[tid].front()->isStore() ||
            skidBuffer[tid].front()->isAtomic()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        skidBuffer[tid].front()->setSquashed();
        skidBuffer[tid].pop();
        wroteToTimeBuffer = true;
    }

    // Clear out self->insts[tid]
    emptyRenameInsts(tid);

    // === JV PBTB AAAA This is cargo cult programming but I just want it
    // to work: let's do some of the stuff the decode does when manually
    // squashing:

    // Might have to tell rename to unblock.
    if (dispatchStatus[tid] == Blocked ||
        dispatchStatus[tid] == Unblocking) {
        toRename->iewUnblock[tid] = 1;
    }
    // Set status to squashing.
    dispatchStatus[tid] = Squashing;

    // === NOTE:
    // For the frontend squash, Fetch will call cpu->removeInstsNotInROB
    // For the squashed instructions in the dispatch queue in IEW, commit will
    // squash them as the ROB cycles through, and will then do a corresponing
    // cpu>removeInsts[something] on them
}



void
IEW::squashDueToMemOrder(const DynInstPtr& inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] Memory violation, squashing violator and younger "
            "insts, PC: %s [sn:%llu].\n", tid, inst->pcState(), inst->seqNum);

    // //TODO JV: PBTB is a WIP in for handling squashes: let's allow this

    // Need to include inst->seqNum in the following comparison to cover the
    // corner case when a branch misprediction and a memory violation for the
    // same instruction (e.g. load PC) are detected in the same cycle.  In this
    // case the memory violator should take precedence over the branch
    // misprediction because it requires the violator itself to be included in
    // the squash.
    if (!toCommit->squash[tid] ||
            inst->seqNum <= toCommit->squashedSeqNum[tid]) {
        toCommit->squash[tid] = true;

        toCommit->squashedSeqNum[tid] = inst->seqNum;
        set(toCommit->pc[tid], inst->pcState());
        toCommit->mispredictInst[tid] = NULL;

        // Must include the memory violator in the squash.
        toCommit->includeSquashInst[tid] = true;

        wroteToTimeBuffer = true;
    }
}

void
IEW::block(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] Blocking.\n", tid);

    if (dispatchStatus[tid] != Blocked &&
        dispatchStatus[tid] != Unblocking) {
        toRename->iewBlock[tid] = true;
        wroteToTimeBuffer = true;
    }

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    dispatchStatus[tid] = Blocked;
}

void
IEW::unblock(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] Reading instructions out of the skid "
            "buffer %u.\n",tid, tid);

    // If the skid bufffer is empty, signal back to previous stages to unblock.
    // Also switch status to running.
    if (skidBuffer[tid].empty()) {
        toRename->iewUnblock[tid] = true;
        wroteToTimeBuffer = true;
        DPRINTF(IEW, "[tid:%i] Done unblocking.\n",tid);
        dispatchStatus[tid] = Running;
    }
}

void
IEW::wakeDependents(const DynInstPtr& inst)
{
    instQueue.wakeDependents(inst);
}

void
IEW::rescheduleMemInst(const DynInstPtr& inst)
{
    instQueue.rescheduleMemInst(inst);
}

void
IEW::replayMemInst(const DynInstPtr& inst)
{
    instQueue.replayMemInst(inst);
}

void
IEW::blockMemInst(const DynInstPtr& inst)
{
    instQueue.blockMemInst(inst);
}

void
IEW::cacheUnblocked()
{
    instQueue.cacheUnblocked();
}

void
IEW::instToCommit(const DynInstPtr& inst)
{
    // This function should not be called after writebackInsts in a
    // single cycle.  That will cause problems with an instruction
    // being added to the queue to commit without being processed by
    // writebackInsts prior to being sent to commit.

    // First check the time slot that this instruction will write
    // to.  If there are free write ports at the time, then go ahead
    // and write the instruction to that time.  If there are not,
    // keep looking back to see where's the first time there's a
    // free slot.
    while ((*iewQueue)[wbCycle].insts[wbNumInst]) {
        ++wbNumInst;
        if (wbNumInst == wbWidth) {
            ++wbCycle;
            wbNumInst = 0;
        }
    }

    DPRINTF(IEW, "Current wb cycle: %i, width: %i, numInst: %i\nwbActual:%i\n",
            wbCycle, wbWidth, wbNumInst, wbCycle * wbWidth + wbNumInst);
    // Add finished instruction to queue to commit.
    (*iewQueue)[wbCycle].insts[wbNumInst] = inst;
    (*iewQueue)[wbCycle].size++;
}

void
IEW::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        DPRINTF(IEW,"[tid:%i] Inserting [sn:%lli] PC:%s into "
                "dispatch skidBuffer %i\n",tid, inst->seqNum,
                inst->pcState(),tid);

        skidBuffer[tid].push(inst);
    }

    assert(skidBuffer[tid].size() <= skidBufferMax &&
           "Skidbuffer Exceeded Max Size");
}

int
IEW::skidCount()
{
    int max=0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned thread_count = skidBuffer[tid].size();
        if (max < thread_count)
            max = thread_count;
    }

    return max;
}

bool
IEW::skidsEmpty()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

void
IEW::updateStatus()
{
    bool any_unblocking = false;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (dispatchStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // If there are no ready instructions waiting to be scheduled by the IQ,
    // and there's no stores waiting to write back, and dispatch is not
    // unblocking, then there is no internal activity for the IEW stage.
    instQueue.iqIOStats.intInstQueueReads++;
    if (_status == Active && !instQueue.hasReadyInsts() &&
        !ldstQueue.willWB() && !any_unblocking) {
        DPRINTF(IEW, "IEW switching to idle\n");

        deactivateStage();

        _status = Inactive;
    } else if (_status == Inactive && (instQueue.hasReadyInsts() ||
                                       ldstQueue.willWB() ||
                                       any_unblocking)) {
        // Otherwise there is internal activity.  Set to active.
        DPRINTF(IEW, "IEW switching to active\n");

        activateStage();

        _status = Active;
    }
}

bool
IEW::checkStall(ThreadID tid)
{
    bool ret_val(false);

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(IEW,"[tid:%i] Stall from Commit stage detected.\n",tid);
        ret_val = true;
    } else if (instQueue.isFull(tid)) {
        DPRINTF(IEW,"[tid:%i] Stall: IQ  is full.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

void
IEW::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if there is.
    // If status was Blocked
    //     if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.



    // =================== JV PBTB =======================
    // DEBUG the from-commit signals
    //DPRINTF(Decode, "[tid:%i] fromcommit DEBUGGING: "
    //                "squash:%c doneSeqNum=%d\n",
    //                tid, fromCommit->commitInfo[tid].squash ? 'T':'F',
    //                    fromCommit->commitInfo[tid].doneSeqNum);

    // If we committed this cycle then doneSeqNum will be > 0
    if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
        !fromCommit->commitInfo[tid].squash) {
        cpu->pbtb.tracker.recordCommit(tid,
                fromCommit->commitInfo[tid].doneSeqNum);
    }

    // =======================================================

    if (fromCommit->commitInfo[tid].squash) {
        // everything with seq>squashNum is gone
        InstSeqNum squashNum = fromCommit->commitInfo[tid].doneSeqNum;
        DPRINTF(IEW, "[tid:%i] Squashing instructions due to squash "
                "from commit [sn:%lld] .\n", tid, squashNum);
        squash(tid); //JV PBTB: updated to also unwind PBTB and tracker

        if (dispatchStatus[tid] == Blocked ||
            dispatchStatus[tid] == Unblocking) {
            toRename->iewUnblock[tid] = true;
            wroteToTimeBuffer = true;
        }

        dispatchStatus[tid] = Squashing;
        fetchRedirect[tid] = false;
        return;
    }

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(IEW, "[tid:%i] ROB is still squashing.\n", tid);

        dispatchStatus[tid] = Squashing;
        emptyRenameInsts(tid);
        wroteToTimeBuffer = true;
    }

    if (checkStall(tid)) {
        block(tid);
        dispatchStatus[tid] = Blocked;
        return;
    }

    // ===== JV PBTB:
    // if next inst is a placeholder branch,
    // we have to stall until it's ready to finalize (all bmovs have executed)
    bool reading_from_skid = (dispatchStatus[tid] == Blocked
                           || dispatchStatus[tid] == Unblocking);
    const std::queue<DynInstPtr>
        &insts_to_dispatch = reading_from_skid? skidBuffer[tid] : insts[tid];

    if (insts_to_dispatch.size() > 0) {
        DynInstPtr inst = insts_to_dispatch.front();

        // WARNING: This check is duplicated in dispatchInsts()
        // MAKE SURE BOTH HAVE THE SAME LOGIC

        if (cpu->pbtb.tracker.instNeedsToStall(tid, inst)) {
            DPRINTF(IEW,"[tid:%i] Stalling for pb [sn:%d] finalize "
                           "(in checkSignalsAndUpdate)\n",
                    tid, inst->seqNum);
            ++iewStats.pbtbBlockedCycles;
            inst->setPredBTBDidBlock(true); // we may set this multiple times,
            // (also in decodeInsts), but we'll be sure we marked them all

            block(tid);
            dispatchStatus[tid] = Blocked;
            return;
        }
    }
    // ========== END PBTB



    if (dispatchStatus[tid] == Blocked) {
        // Status from previous cycle was blocked, but there are no more stall
        // conditions.  Switch over to unblocking.
        DPRINTF(IEW, "[tid:%i] Done blocking, switching to unblocking.\n",
                tid);

        dispatchStatus[tid] = Unblocking;

        unblock(tid);

        return;
    }

    if (dispatchStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        DPRINTF(IEW, "[tid:%i] Done squashing, switching to running.\n",
                tid);

        dispatchStatus[tid] = Running;

        return;
    }
}

void
IEW::sortInsts()
{
    int insts_from_rename = fromRename->size;
#ifdef GEM5_DEBUG
    for (ThreadID tid = 0; tid < numThreads; tid++)
        assert(insts[tid].empty());
#endif
    for (int i = 0; i < insts_from_rename; ++i) {
        insts[fromRename->insts[i]->threadNumber].push(fromRename->insts[i]);
    }
}

void
IEW::emptyRenameInsts(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i] Removing incoming rename instructions\n", tid);

    while (!insts[tid].empty()) {

        if (insts[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (insts[tid].front()->isStore() ||
            insts[tid].front()->isAtomic()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        insts[tid].front()->setSquashed();
        insts[tid].pop();
    }
}

void
IEW::wakeCPU()
{
    cpu->wakeCPU();
}

void
IEW::activityThisCycle()
{
    DPRINTF(Activity, "Activity this cycle.\n");
    cpu->activityThisCycle();
}

void
IEW::activateStage()
{
    DPRINTF(Activity, "Activating stage.\n");
    cpu->activateStage(CPU::IEWIdx);
}

void
IEW::deactivateStage()
{
    DPRINTF(Activity, "Deactivating stage.\n");
    cpu->deactivateStage(CPU::IEWIdx);
}

void
IEW::dispatch(ThreadID tid)
{
    // If status is Running or idle,
    //     call dispatchInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from rename
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (dispatchStatus[tid] == Blocked) {
        ++iewStats.blockCycles;

    } else if (dispatchStatus[tid] == Squashing) {
        ++iewStats.squashCycles;
    }

    // Dispatch should try to dispatch as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (dispatchStatus[tid] == Running ||
        dispatchStatus[tid] == Idle) {
        DPRINTF(IEW, "[tid:%i] Not blocked, so attempting to run "
                "dispatch.\n", tid);

        dispatchInsts(tid);
    } else if (dispatchStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        dispatchInsts(tid);

        ++iewStats.unblockCycles;

        if (fromRename->size != 0) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        unblock(tid);
    }
}

void
IEW::dispatchInsts(ThreadID tid)
{
    // Obtain instructions from skid buffer if unblocking, or queue from rename
    // otherwise.
    std::queue<DynInstPtr> &insts_to_dispatch =
        dispatchStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    int insts_to_add = insts_to_dispatch.size();

    DynInstPtr inst;
    bool add_to_iq = false;
    int dis_num_inst = 0;

    // Loop through the instructions, putting them in the instruction
    // queue.
    for ( ; dis_num_inst < insts_to_add &&
              dis_num_inst < dispatchWidth;
          ++dis_num_inst)
    {
        inst = insts_to_dispatch.front();

        if (dispatchStatus[tid] == Unblocking) {
            DPRINTF(IEW, "[tid:%i] Issue: Examining instruction from skid "
                    "buffer\n", tid);
        }

        // Make sure there's a valid instruction there.
        assert(inst);

        DPRINTF(IEW, "[tid:%i] Issue: Adding PC %s [sn:%lli] [tid:%i] to "
                "IQ.\n",
                tid, inst->pcState(), inst->seqNum, inst->threadNumber);

        // Be sure to mark these instructions as ready so that the
        // commit stage can go ahead and execute them, and mark
        // them as issued so the IQ doesn't reprocess them.

        // Check for squashed instructions.
        if (inst->isSquashed()) {
            DPRINTF(IEW, "[tid:%i] Issue: Squashed instruction encountered, "
                    "not adding to IQ.\n", tid);

            ++iewStats.dispSquashedInsts;

            insts_to_dispatch.pop();

            //Tell Rename That An Instruction has been processed
            if (inst->isLoad()) {
                toRename->iewInfo[tid].dispatchedToLQ++;
            }
            if (inst->isStore() || inst->isAtomic()) {
                toRename->iewInfo[tid].dispatchedToSQ++;
            }

            toRename->iewInfo[tid].dispatched++;

            continue;
        }

        // Check for full conditions.
        if (instQueue.isFull(tid)) {
            DPRINTF(IEW, "[tid:%i] Issue: IQ has become full.\n", tid);

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            ++iewStats.iqFullEvents;
            break;
        }


        // If it's a pb or it was predicted as a pb, need to
        // make sure we're ready to finalize it
        // WARNING: This is duplicated in Decode::checkSignalsAndUpdate(...)
        // MAKE SURE BOTH HAVE THE SAME LOGIC
        if (cpu->pbtb.tracker.instNeedsToStall(tid, inst)) {
            DPRINTF(IEW,"[tid:%i] Stalling for pb [sn:%d] finalize "
                            " (in dispatchInsts)\n",
                    tid, inst->seqNum);

            ++iewStats.pbtbBlockedCycles;
            inst->setPredBTBDidBlock(true); // we also set this in
            // checkSignalsAndUpdate, since it may have blocked there
            // but be ready by the time we get here?

            //if (inst->seqNum == 0) {
            //    DPRINTF(IEW,"[tid:%i] Stalling inst %d, readPredBTBreg:%d, "
            //                "isControl:%d, isBmov:%d, isSquashed:%d\n",
            //        inst->seqNum, inst->readPredBTBReg(),
            //        inst->isControl(), inst->isBmov(), inst->isSquashed());
            //    panic("Inst with seqnum 0 should never happen?");
            //}

            // call function to start blocking
            block(tid);
            break;
        }

        // ==================== HANDLE PBTB FINALIZE ===================
        // Note: all insts do this check, since even non-pbs
        //       can mispredict as pbs.

        // we know it's ready to finalize
        // we know it's not squashed
        bool pbtbMispredicted = resolvePBTBAndCheckMispredict(tid, inst);
        // This also updates all stats related to finalized insts,
        // mispredicts, pbs. This also updates the branch target
        // so that squashing propagates it correctly to fetch

        // WE WILL CHECK THE RESULT OF THE MISPREDICT AND SQUASH AT END
        // OF DISPATCH LOOP

        // Check LSQ if inst is LD/ST
        if ((inst->isAtomic() && ldstQueue.sqFull(tid)) ||
            (inst->isLoad() && ldstQueue.lqFull(tid)) ||
            (inst->isStore() && ldstQueue.sqFull(tid))) {
            DPRINTF(IEW, "[tid:%i] Issue: %s has become full.\n",tid,
                    inst->isLoad() ? "LQ" : "SQ");

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            ++iewStats.lsqFullEvents;
            break;
        }

        // hardware transactional memory
        // CPU needs to track transactional state in program order.
        const int numHtmStarts = ldstQueue.numHtmStarts(tid);
        const int numHtmStops = ldstQueue.numHtmStops(tid);
        const int htmDepth = numHtmStarts - numHtmStops;

        if (htmDepth > 0) {
            inst->setHtmTransactionalState(ldstQueue.getLatestHtmUid(tid),
                                            htmDepth);
        } else {
            inst->clearHtmTransactionalState();
        }


        // Otherwise issue the instruction just fine.
        if (inst->isAtomic()) {
            DPRINTF(IEW, "[tid:%i] Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            ldstQueue.insertStore(inst);

            ++iewStats.dispStoreInsts;

            // AMOs need to be set as "canCommit()"
            // so that commit can process them when they reach the
            // head of commit.
            inst->setCanCommit();
            instQueue.insertNonSpec(inst);
            add_to_iq = false;

            ++iewStats.dispNonSpecInsts;

            toRename->iewInfo[tid].dispatchedToSQ++;
        } else if (inst->isLoad()) {
            DPRINTF(IEW, "[tid:%i] Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            // Reserve a spot in the load store queue for this
            // memory access.
            ldstQueue.insertLoad(inst);

            ++iewStats.dispLoadInsts;

            add_to_iq = true;

            toRename->iewInfo[tid].dispatchedToLQ++;
        } else if (inst->isStore()) {
            DPRINTF(IEW, "[tid:%i] Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            ldstQueue.insertStore(inst);

            ++iewStats.dispStoreInsts;

            if (inst->isStoreConditional()) {
                // Store conditionals need to be set as "canCommit()"
                // so that commit can process them when they reach the
                // head of commit.
                // @todo: This is somewhat specific to Alpha.
                inst->setCanCommit();
                instQueue.insertNonSpec(inst);
                add_to_iq = false;

                ++iewStats.dispNonSpecInsts;
            } else {
                add_to_iq = true;
            }

            toRename->iewInfo[tid].dispatchedToSQ++;
        } else if (inst->isReadBarrier() || inst->isWriteBarrier()) {
            // Same as non-speculative stores.
            inst->setCanCommit();
            instQueue.insertBarrier(inst);
            add_to_iq = false;
        } else if (inst->isNop()) {
            DPRINTF(IEW, "[tid:%i] Issue: Nop instruction encountered, "
                    "skipping.\n", tid);

            inst->setIssued();
            inst->setExecuted();
            inst->setCanCommit();

            instQueue.recordProducer(inst);

            cpu->executeStats[tid]->numNop++;

            add_to_iq = false;
        } else {
            assert(!inst->isExecuted());
            add_to_iq = true;
        }

        if (add_to_iq && inst->isNonSpeculative()) {
            DPRINTF(IEW, "[tid:%i] Issue: Nonspeculative instruction "
                    "encountered, skipping.\n", tid);

            // Same as non-speculative stores.
            inst->setCanCommit();

            // Specifically insert it as nonspeculative.
            instQueue.insertNonSpec(inst);

            ++iewStats.dispNonSpecInsts;

            add_to_iq = false;
        }

        // If the instruction queue is not full, then add the
        // instruction.
        if (add_to_iq) {
            instQueue.insert(inst);
        }

        insts_to_dispatch.pop();

        toRename->iewInfo[tid].dispatched++;

        ++iewStats.dispatchedInsts;

#if TRACING_ON
        inst->dispatchTick = curTick() - inst->fetchTick;
#endif
        ppDispatch->notify(inst);

        // ==== JV PBTB:
        // NOW: if this instruction mispredicted, everything after it is
        // wrong, so now that we've dispatched it, we should squash
        // everything behind us
        if (pbtbMispredicted) {
            // squash overwrites fetch-pbtb to correct state (from finalize),
            // squashes all remaining insts
            // and updates fetch's branch predicotr
            squashDueToPBTB(inst, inst->threadNumber);
            break;
        }

    }

    if (!insts_to_dispatch.empty()) {
        DPRINTF(IEW,"[tid:%i] Issue: Bandwidth Full. Blocking.\n", tid);
        block(tid);
        toRename->iewUnblock[tid] = false;
    }

    if (dispatchStatus[tid] == Idle && dis_num_inst) {
        dispatchStatus[tid] = Running;

        updatedQueues = true;
    }

    dis_num_inst = 0;
}

void
IEW::printAvailableInsts()
{
    int inst = 0;

    std::cout << "Available Instructions: ";

    while (fromIssue->insts[inst]) {

        if (inst%3==0) std::cout << "\n\t";

        std::cout << "PC: " << fromIssue->insts[inst]->pcState()
             << " TN: " << fromIssue->insts[inst]->threadNumber
             << " SN: " << fromIssue->insts[inst]->seqNum << " | ";

        inst++;

    }

    std::cout << "\n";
}


// JV: UGLY HACK:
// I need a 64-bit int hash for debugging in executeInsts, so I'm sticking
// it here. I'm so sorry
static uint64_t jv_int64hash(uint64_t x) {
    // hash stolen from https://nullprogram.com/blog/2018/07/31/
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9U;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebU;
    x ^= x >> 31;
    return x;
}

void
IEW::executeInsts()
{
    wbNumInst = 0;
    wbCycle = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        fetchRedirect[tid] = false;
    }

    // Uncomment this if you want to see all available instructions.
    // @todo This doesn't actually work anymore, we should fix it.
//    printAvailableInsts();

    // Execute/writeback any instructions that are available.
    int insts_to_execute = fromIssue->size;
    int inst_num = 0;
    for (; inst_num < insts_to_execute;
          ++inst_num) {

        DPRINTF(IEW, "Execute: Executing instructions from IQ.\n");

        DynInstPtr inst = instQueue.getInstToExecute();

        DPRINTF(IEW, "Execute: Processing PC %s, [tid:%i] [sn:%llu].\n",
                inst->pcState(), inst->threadNumber,inst->seqNum);

        // Notify potential listeners that this instruction has started
        // executing
        ppExecute->notify(inst);

        // Check if the instruction is squashed; if so then skip it
        if (inst->isSquashed() ||
            (inst->isBmov() && inst->isSquashedInIQ())) {
                // JV PBTB: Make sure we don't execute a bmov that's just been
                // squashed in the IQ, or else it will get sent to
                // pbtb AFTER being undone (or after it would have been
                // undone? which we don't want)
                // Specifically, it seemed to be an issue where an inst
                // was issued, but wouldn't get isSquashed until the ROB
                // got around to it in several cycles, so the squash signal
                // got to Decode/the PBTB first, the undo was applied, and
                // THEN the (should-have-been-squashed) bmov executed in IEW,
                // big problem

                // NOTE: this might be a problem since we toss the inst
                // before it's marked as squashed by ROBSquashing?
                // but it should be ok since I think dispatch and WB are
                // stalled during squashing, even if exe is not
            DPRINTF(IEW, "Execute: Instruction was squashed. PC: %s, [tid:%i]"
                         " [sn:%llu]\n", inst->pcState(), inst->threadNumber,
                         inst->seqNum);

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();

            ++iewStats.executedInstStats.numSquashedInsts;

            continue;
        }

        Fault fault = NoFault;

        // Execute instruction.
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef()) {
            DPRINTF(IEW, "Execute: Calculating address for memory "
                    "reference.\n");

            // Tell the LDSTQ to execute this instruction (if it is a load).
            if (inst->isAtomic()) {
                // AMOs are treated like store requests
                fault = ldstQueue.executeStore(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "store.\n");
                    instQueue.deferMemInst(inst);
                    continue;
                }
            } else if (inst->isLoad()) {
                // Loads will mark themselves as executed, and their writeback
                // event adds the instruction to the queue to commit
                fault = ldstQueue.executeLoad(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "load.\n");
                    instQueue.deferMemInst(inst);
                    continue;
                }

                if (inst->isDataPrefetch() || inst->isInstPrefetch()) {
                    inst->fault = NoFault;
                }
            } else if (inst->isStore()) {
                fault = ldstQueue.executeStore(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "store.\n");
                    instQueue.deferMemInst(inst);
                    continue;
                }

                // If the store had a fault then it may not have a mem req
                if (fault != NoFault || !inst->readPredicate() ||
                        !inst->isStoreConditional()) {
                    // If the instruction faulted, then we need to send it
                    // along to commit without the instruction completing.
                    // Send this instruction to commit, also make sure iew
                    // stage realizes there is activity.
                    inst->setExecuted();
                    instToCommit(inst);
                    activityThisCycle();
                }

                // Store conditionals will mark themselves as
                // executed, and their writeback event will add the
                // instruction to the queue to commit.
            } else {
                panic("Unexpected memory type!\n");
            }

        } else {
            // If the instruction has already faulted, then skip executing it.
            // Such case can happen when it faulted during ITLB translation.
            // If we execute the instruction (even if it's a nop) the fault
            // will be replaced and we will lose it.
            if (inst->getFault() == NoFault) {
                inst->execute();
                if (!inst->readPredicate())
                    inst->forwardOldRegs();
            }

            inst->setExecuted();

            instToCommit(inst);
        }

        updateExeInstStats(inst);

        // Check if branch prediction was correct, if not then we need
        // to tell commit to squash in flight instructions.  Only
        // handle this if there hasn't already been something that
        // redirects fetch in this group of instructions.

        // This probably needs to prioritize the redirects if a different
        // scheduler is used.  Currently the scheduler schedules the oldest
        // instruction first, so the branch resolution order will be correct.
        ThreadID tid = inst->threadNumber;

        if (!fetchRedirect[tid] ||
            !toCommit->squash[tid] ||
            toCommit->squashedSeqNum[tid] > inst->seqNum) {

            // JV BMOV
            // If a bmov gets here, we need to squash everything
            // behind it in case precomputed branches were added/removed
            if (inst->isBmov()) {
                const int breg = inst->destRegIdx(0);

                DPRINTF(PBTB, "[tid:%i] [sn:%llu] Executing BMOV: "
                        "breg=%d. (%s)\n",
                    tid, inst->seqNum, breg,
                    inst->staticInst->disassemble(
                        inst->pcState().instAddr()));

                assert(!inst->isSquashedInIQ()); // JV NOTE: this was
                // an issue before, but now it shouldn't happen since we
                // explicitly skip bmovs that are squashedInIQ

                // NOTE: we should only be able to execute one bmov
                //       per breg per cycle.?

                // NEW: With finalize in IEW:
                // We can just record it as soon as we exec, not
                // have to send it back in time
                cpu->pbtb.tracker.recordExecBmovFromIew(
                        tid, inst->seqNum, breg);

                // Signal to decode-PBTB of this execution (OBSOLETE?)
                toDecode->iewInfo->lastExecBmovSeqNum[breg] = inst->seqNum;

                //TODO Originally this was to get bmovs working, but now
                //we want to only squash in decode due to finalize
                //squashDueToBranch(inst, tid);

                //TODO JV: agh idk what this does
                //ppMispredict->notify(inst);

                //These aren't quite correct anymore
                //if (inst->readPredTaken()) {
                //    iewStats.predictedTakenIncorrect++;
                //} else {
                //    iewStats.predictedNotTakenIncorrect++;
            }
            /* TODO JV TEMP: Don't squash on branches, let PBTP handle it

            // Prevent testing for misprediction on load instructions,
            // that have not been executed.
            bool loadNotExecuted = !inst->isExecuted() && inst->isLoad();

            if (inst->mispredicted() && !loadNotExecuted) {
                fetchRedirect[tid] = true;

                DPRINTF(IEW, "[tid:%i] [sn:%llu] Execute: "
                        "Branch mispredict detected.\n",
                        tid, inst->seqNum);
                DPRINTF(IEW, "[tid:%i] [sn:%llu] "
                        "Predicted target was PC: %s\n",
                        tid, inst->seqNum, inst->readPredTarg());
                DPRINTF(IEW, "[tid:%i] [sn:%llu] Execute: "
                        "Redirecting fetch to PC: %s\n",
                        tid, inst->seqNum, inst->pcState());
                // If incorrect, then signal the ROB that it must be squashed.
                squashDueToBranch(inst, tid);

                ppMispredict->notify(inst);

                if (inst->readPredTaken()) {
                    iewStats.predictedTakenIncorrect++;
                } else {
                    iewStats.predictedNotTakenIncorrect++;
                }
            } else  */
            if (ldstQueue.violation(tid)) {
                assert(inst->isMemRef());
                // If there was an ordering violation, then get the
                // DynInst that caused the violation.  Note that this
                // clears the violation signal.
                DynInstPtr violator;
                violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation. Violator PC: %s "
                        "[sn:%lli], inst PC: %s [sn:%lli]. Addr is: %#x.\n",
                        violator->pcState(), violator->seqNum,
                        inst->pcState(), inst->seqNum, inst->physEffAddr);

                fetchRedirect[tid] = true;

                // Tell the instruction queue that a violation has occured.
                instQueue.violation(inst, violator);

                // Squash.
                squashDueToMemOrder(violator, tid);

                ++iewStats.memOrderViolationEvents;

            // ======== JV DEBUG: TRIGGER A MANUAL SQUASH SOMETIMES
            } else if (false) { // swap the comments to enable squashing
            //} else if (jv_int64hash(inst->seqNum) % 11 == 0) {
                // (to verify squash logic is correct)
                //
                // Note: I want to randomize (via hash) which seqnums get
                // squashed, so we don't end up in perpetual loops of
                // accidentally squashing the first inst each time

                // NOTE ON SQUASH NUMBERING:
                //   squashDueToMemOrder is a special case that re-execs
                //   the squashed inst. Normally all other squashes take the
                //   squashSeqNum as NOT re-executed, so all squashes go
                //   up to > squashSeqNum, not >=
                //   (memOrder does a -1 to get the >= behavor, so let's print
                //    with the -1 here for consistency)
                DPRINTF(PBTB, "JV DEBUG !!!!: IEW Triggering manual squash"
                            " up to inst [sn:%d]\n", inst->seqNum-1);
                DPRINTF(IEW, "JV DEBUG !!!!: IEW Triggering manual squash"
                            " up to inst [sn:%d]\n", inst->seqNum-1);
                squashDueToMemOrder(inst, tid);
            }

        } else {
            // Reset any state associated with redirects that will not
            // be used.
            if (ldstQueue.violation(tid)) {
                assert(inst->isMemRef());

                DynInstPtr violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation.  Violator PC: "
                        "%s, inst PC: %s.  Addr is: %#x.\n",
                        violator->pcState(), inst->pcState(),
                        inst->physEffAddr);
                DPRINTF(IEW, "Violation will not be handled because "
                        "already squashing\n");

                ++iewStats.memOrderViolationEvents;
            }
        }
    }

    // Update and record activity if we processed any instructions.
    if (inst_num) {
        if (exeStatus == Idle) {
            exeStatus = Running;
        }

        updatedQueues = true;

        cpu->activityThisCycle();
    }

    // Need to reset this in case a writeback event needs to write into the
    // iew queue.  That way the writeback event will write into the correct
    // spot in the queue.
    wbNumInst = 0;

}

void
IEW::writebackInsts()
{
    // Loop through the head of the time buffer and wake any
    // dependents.  These instructions are about to write back.  Also
    // mark scoreboard that this instruction is finally complete.
    // Either have IEW have direct access to scoreboard, or have this
    // as part of backwards communication.
    for (int inst_num = 0; inst_num < wbWidth &&
             toCommit->insts[inst_num]; inst_num++) {
        DynInstPtr inst = toCommit->insts[inst_num];
        ThreadID tid = inst->threadNumber;

        DPRINTF(IEW, "Sending instructions to commit, [sn:%lli] PC %s.\n",
                inst->seqNum, inst->pcState());

        iewStats.instsToCommit[tid]++;
        // Notify potential listeners that execution is complete for this
        // instruction.
        ppToCommit->notify(inst);

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Strictly ordered loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the strictly ordered load.
        if (!inst->isSquashed() && inst->isExecuted() &&
                inst->getFault() == NoFault) {
            int dependents = instQueue.wakeDependents(inst);

            for (int i = 0; i < inst->numDestRegs(); i++) {
                // Mark register as ready if not pinned
                if (inst->renamedDestIdx(i)->
                        getNumPinnedWritesToComplete() == 0) {
                    DPRINTF(IEW,"Setting Destination Register %i (%s)\n",
                            inst->renamedDestIdx(i)->index(),
                            inst->renamedDestIdx(i)->className());
                    scoreboard->setReg(inst->renamedDestIdx(i));
                }
            }

            if (dependents) {
                iewStats.producerInst[tid]++;
                iewStats.consumerInst[tid]+= dependents;
            }
            iewStats.writebackCount[tid]++;
        }
    }
}

void
IEW::tick()
{
    wbNumInst = 0;
    wbCycle = 0;

    wroteToTimeBuffer = false;
    updatedQueues = false;

    ldstQueue.tick();

    sortInsts();

    // Free function units marked as being freed this cycle.
    fuPool->processFreeUnits();

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    // Check stall and squash signals, dispatch any instructions.
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(IEW,"Issue: Processing [tid:%i]\n",tid);

        checkSignalsAndUpdate(tid);
        dispatch(tid);
    }

    if (exeStatus != Squashing) {
        executeInsts();

        writebackInsts();

        // Have the instruction queue try to schedule any ready instructions.
        // (In actuality, this scheduling is for instructions that will
        // be executed next cycle.)
        instQueue.scheduleReadyInsts();

        // Also should advance its own time buffers if the stage ran.
        // Not the best place for it, but this works (hopefully).
        issueToExecQueue.advance();
    }

    bool broadcast_free_entries = false;

    if (updatedQueues || exeStatus == Running || updateLSQNextCycle) {
        exeStatus = Idle;
        updateLSQNextCycle = false;

        broadcast_free_entries = true;
    }

    // Writeback any stores using any leftover bandwidth.
    ldstQueue.writebackStores();

    // Check the committed load/store signals to see if there's a load
    // or store to commit.  Also check if it's being told to execute a
    // nonspeculative instruction.
    // This is pretty inefficient...

    threads = activeThreads->begin();
    while (threads != end) {
        ThreadID tid = (*threads++);

        DPRINTF(IEW,"Processing [tid:%i]\n",tid);

        // Update structures based on instructions committed.
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            !fromCommit->commitInfo[tid].robSquashing) {

            ldstQueue.commitStores(fromCommit->commitInfo[tid].doneSeqNum,tid);

            ldstQueue.commitLoads(fromCommit->commitInfo[tid].doneSeqNum,tid);

            updateLSQNextCycle = true;
            instQueue.commit(fromCommit->commitInfo[tid].doneSeqNum,tid);
        }

        if (fromCommit->commitInfo[tid].nonSpecSeqNum != 0) {

            //DPRINTF(IEW,"NonspecInst from thread %i",tid);
            if (fromCommit->commitInfo[tid].strictlyOrdered) {
                instQueue.replayMemInst(
                    fromCommit->commitInfo[tid].strictlyOrderedLoad);
                fromCommit->commitInfo[tid].strictlyOrderedLoad->setAtCommit();
            } else {
                instQueue.scheduleNonSpec(
                    fromCommit->commitInfo[tid].nonSpecSeqNum);
            }
        }

        if (broadcast_free_entries) {
            toFetch->iewInfo[tid].iqCount =
                instQueue.getCount(tid);
            toFetch->iewInfo[tid].ldstqCount =
                ldstQueue.getCount(tid);

            toRename->iewInfo[tid].usedIQ = true;
            toRename->iewInfo[tid].freeIQEntries =
                instQueue.numFreeEntries(tid);
            toRename->iewInfo[tid].usedLSQ = true;

            toRename->iewInfo[tid].freeLQEntries =
                ldstQueue.numFreeLoadEntries(tid);
            toRename->iewInfo[tid].freeSQEntries =
                ldstQueue.numFreeStoreEntries(tid);

            wroteToTimeBuffer = true;
        }

        DPRINTF(IEW, "[tid:%i], Dispatch dispatched %i instructions.\n",
                tid, toRename->iewInfo[tid].dispatched);
    }

    DPRINTF(IEW, "IQ has %i free entries (Can schedule: %i).  "
            "LQ has %i free entries. SQ has %i free entries.\n",
            instQueue.numFreeEntries(), instQueue.hasReadyInsts(),
            ldstQueue.numFreeLoadEntries(), ldstQueue.numFreeStoreEntries());

    updateStatus();

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }
}

void
IEW::updateExeInstStats(const DynInstPtr& inst)
{
    ThreadID tid = inst->threadNumber;

    cpu->executeStats[tid]->numInsts++;

#if TRACING_ON
    if (debug::O3PipeView) {
        inst->completeTick = curTick() - inst->fetchTick;
    }
#endif

    //
    //  Control operations
    //
    if (inst->isControl()) {
        cpu->executeStats[tid]->numBranches++;
    }

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        cpu->executeStats[tid]->numMemRefs++;

        if (inst->isLoad()) {
            cpu->executeStats[tid]->numLoadInsts++;
        }
    }
}

void
IEW::checkMisprediction(const DynInstPtr& inst)
{
    ThreadID tid = inst->threadNumber;

    if (!fetchRedirect[tid] ||
        !toCommit->squash[tid] ||
        toCommit->squashedSeqNum[tid] > inst->seqNum) {

        if (inst->mispredicted()) {
            fetchRedirect[tid] = true;

            DPRINTF(IEW, "[tid:%i] [sn:%llu] Execute: "
                    "Branch mispredict detected.\n",
                    tid, inst->seqNum);
            DPRINTF(IEW, "[tid:%i] [sn:%llu] Predicted target was PC: %s\n",
                    tid, inst->seqNum, inst->readPredTarg());
            DPRINTF(IEW, "[tid:%i] [sn:%llu] Execute: "
                    "Redirecting fetch to PC: %s\n",
                    tid, inst->seqNum, inst->pcState());
            // If incorrect, then signal the ROB that it must be squashed.
            squashDueToBranch(inst, tid);

            if (inst->readPredTaken()) {
                iewStats.predictedTakenIncorrect++;
            } else {
                iewStats.predictedNotTakenIncorrect++;
            }
        }
    }
}

/**
 * NOTE: actually all insts need to go through this, since they might have been
 * mispredicted as pbs
 *
 * Given a (possibly-pb) inst that's ready to finalize, determine whether it
 * predicted correctly, then update predictions, stats, logging.
 * Returns true if it mispredicted and needs to squash
 */
bool
IEW::resolvePBTBAndCheckMispredict(int tid, const DynInstPtr &inst) {
    // Stalling needs to happen
    assert(!cpu->pbtb.tracker.instNeedsToStall(tid, inst));

    // Original code had this after the squash checks,
    // if we want to make this handle squashed insts that will
    // take some thinking to make sure it doesn't break anything
    assert(!inst->isSquashed());

    // PBTB: update bmov stats (we'll get pbtbFinalizedPbs down below)
    if (inst->isBmov()){ ++iewStats.pbtbFinalizedBmovs; }

    // ==== Count all insts as decoded for the dependency tracker?
    cpu->pbtb.tracker.recordDecodeInst(tid, inst);

    // ===== PBTB: requery the finalize pbtb

    PBTB::PBTBResultType res;
    int      d_breg  = -1;
    uint64_t d_version = 0;
    bool     d_exhausted;
    bool     d_taken;
    Addr     d_targAddr = 0;

    // Note: the address is an in-out param, so make sure to clone
    //        the pc state if we don't want to clobber it when querying
    std::unique_ptr<PCStateBase> d_targPC(inst->pcState().clone());

    res = cpu->pbtb.queryFromDispatch(inst->staticInst,
                                    *d_targPC,
                                    inst->seqNum,
                                    &d_breg, &d_version);

    d_targAddr = d_targPC->instAddr();
    d_exhausted = (res == PBTB::PBTBResultType::PR_Exhaust);
    d_taken = (res == PBTB::PBTBResultType::PR_Taken);

    // ===== Also pull up the original predictions from fetch to compare
    int      f_breg      = inst->readPredBTBReg();
    uint64_t f_version   = inst->readPredBTBVersion();
    bool     f_exhausted = inst->readPredBTBExhausted();
    bool     f_taken     = inst->readPredTaken();
    Addr     f_targAddr  = inst->readPredTarg().instAddr();

    // We now have all relevant data

    // ====== Check if the code itself is malformed (and panic)

    if (d_exhausted) { // Incorrect code: missed a bmov somewhere
        panic("PBTB: hit exhausted breg in finalize (in dispatch).\n"
                "This likely means that your binary is invalid, executing "
                "too many pbs without enough corresponding "
                "bmov_c insts to supply them");
        //TODO: this should eventually throw a BMOV exception
    }

    // finalize-PBTB sees a branch where it shouldn't be, or sees no branch
    // where there should be one
    // (This isn't a prediction error, it means the binary is incorrect,
    //  so we have to just panic out for now)
    if ((inst->isPb() && d_breg <  0) ||
        (!inst->isPb() && d_breg >= 0)) {
        DPRINTF(IEW, "ERROR: [sn:%d] inst->isPb:%c, d_breg=%d\n",
                inst->seqNum, inst->isPb()?'T':'F', d_breg);

        //NOTE: debugDump only prints with PBTBVerbose, so enable it
        //      manually here (we're going to panic anyway so it's fine)
        debug::findFlag(std::string("PBTBVerbose"))->enable();
        cpu->pbtb.debugDump() ;
        panic("PBTB doesn't match placeholder"
                " branch in finalize (in dispatch)\n");
        //TODO: change this to check the pb inst itself and verify that
        //      the breg matches d_breg?
        //TODO: this should eventually throw a BMOV exception
    }


    // ====== Next phase: Determine whether fetch mispredicted and how


    bool wrong_outcome = false; // did we go to the right next pc?

    bool mispred = false; // are we going to squash?
    char mispredReason[256] = "";

    // NOTE:
    // If we fetched from an exhausted pb, should we allow that to pass
    // or always squash it? If we're using any kind of exhaust-predictor,
    // we have to allow exhausted pbs to pass to get any benefit from it.
    // However, there may also be benefit to allowing exhausted pbs in the
    // no-predictor pbtb, since treating exhausted pbs as not-taken can
    // also coincidentally be correct.
    const bool FORBID_EXHAUST_FETCH =
        (cpu->pbtb.CONF_PREDICTOR == PBTB_pred_conf_t::PBTB_Pred_None);


    // Keep track of nextPC outcome separately from whether we will squash.
    if (d_targAddr != f_targAddr || d_taken != f_taken) {
        wrong_outcome = true;
        snprintf(mispredReason, sizeof(mispredReason),
                "fetch went to wrong pc: f->0x%lx, d->0x%lx",
                f_targAddr, d_targAddr);
    }


    // Now: figure it out what the situation of the branch was between
    //      fetch and now.

    if (d_breg < 0 && f_breg < 0) {
        // do nothing, no pb in prediction or in actuality

    } else if (d_breg != f_breg || d_version != f_version) {
        // Case 1: Fetch has reaaaally out of date info, either predicting
        //     an unrelated branch / wrong version, or missing it entirely

        if (f_breg >= 0 && d_breg < 0) {
            // Fetch imagined a pb where there should be none
            iewStats.pbtbFinalizedImaginedPbs++;
        }


        // NOTE: also, if fetch hit an old version or a wrong breg,
        //   it's likely to soon desync, but might also be coincidentally
        //   correct. Should we also squash in that case even if the
        //   prediction was right? (config determines it)
        if (cpu->pbtb.CONF_FORBID_VERSION_MISMATCH || wrong_outcome) {
            iewStats.pbtbFinalState_WrongVersionMisp++;
            mispred = true;
            snprintf(mispredReason, sizeof(mispredReason),
                    "breg/vers mismatch: f@b%d-v%ld, d@b%d-v%ld",
                    f_breg, f_version, d_breg, d_version);
        } else {
            iewStats.pbtbFinalState_WrongVersionCorr++;
        }

    } else if (f_exhausted) {
        // Case 2: Fetch read from an exhausted breg. It should now
        //   be down to the predictor

        if (FORBID_EXHAUST_FETCH || wrong_outcome) {
            iewStats.pbtbFinalState_ExhaustedMisp++;
            mispred = true;
        } else {
            iewStats.pbtbFinalState_ExhaustedCorr++;
        }

        if (FORBID_EXHAUST_FETCH) {
            snprintf(mispredReason, sizeof(mispredReason),
                    "exhausted-fetch FORBIDDEN: f@b%d-EXH", f_breg);
        } else if (wrong_outcome) {
            snprintf(mispredReason, sizeof(mispredReason),
                    "exhausted-fetch mispred: f@b%d-EXH", f_breg);
        }


    } else {
        // Case 3: there is a branch, all versions match up, it was ready
        //   at fetch time.
        assert(d_breg == f_breg);
        assert(d_version == f_version);
        assert(!f_exhausted);

        if (wrong_outcome) {
            // This should only happen if the fetch-pbtb desynced
            // for some reason, which SHOULD only happen if there's
            // a bug in the pbtb code, or if we allow version mismatches
            // to pass without squashing
            iewStats.pbtbFinalState_ReadyMisp++;
            mispred = true;
            snprintf(mispredReason, sizeof(mispredReason),
                    "ready breg DESYNCED?? f@b%d-v%ld, d@b%d-v%ld",
                    f_breg, f_version, d_breg, d_version);
        } else {
            iewStats.pbtbFinalState_ReadyCorr++;
        }
    }

    // ================================================
    // Condition determined, now print some final debug info
    // and update stats

    //TODO TEMP DEBUG
    // DPRINTF(IEW, "JV PBTB: FINALIZING: DEBUG INFO: @pc0x%x\n"
    //             "------------------d: b%dv%ld: %s%s, ->0x%x\n"
    //             "------------------f: b%dv%ld: %s%s, ->0x%x\n",
    //         inst->pcState().instAddr(),
    //         d_breg, d_version,
    //         d_exhausted ? "EXH ":"", d_taken?"T":"NT",
    //         d_targAddr,
    //         f_breg, f_version,
    //         f_exhausted ? "EXH ":"", f_taken?"T":"NT",
    //         f_targAddr);

    // Record any pbs we found and whether or not they mispredicted
    if (d_breg >= 0) {
        DPRINTF(IEW, "JV PBTB: Finalized a branch (%s)"
                        " at pc0x%x->0x%x [sn:%d], b%dv%d\n",
                d_taken ? "T":"NT",
                inst->pcState().instAddr(), d_targAddr,
                inst->seqNum,
                d_breg, d_version);

        ++iewStats.pbtbFinalizedPbs;
        if (inst->readPredBTBDidBlock()) {
            ++iewStats.pbtbFinalizedPbsThatBlocked;
        }

        if (!mispred) { // We correctly took a pb
            assert(d_taken == f_taken);
            assert(!d_taken || (d_targAddr == f_targAddr));

            //TODO: Original code had:
            //     ++stats.branchResolved;
        } else {
            // we mispredcted in one of many complicated ways

            // ORIGINAL CODE HAD THIS, but now there's cases where
            // we mispredict and there's wasnt a pb, so this
            // doesn't work?

            //++stats.branchMispred;

            //// Might want to set some sort of boolean and just do
            //// a check at the end
            //squash(inst, inst->threadNumber);

            //DPRINTF(IEW,
            //    "[tid:%i] [sn:%llu] "
            //    "Updating predictions: Wrong predicted target: "
            //    "%s PredPC: %s\n",
            //    tid, inst->seqNum, inst->readPredTarg(), *target);
            ////The micro pc after an instruction level branch
            ////should be 0
            //inst->setPredTarg(*target);
            //break;
        }
    }


    if (mispred) {
        DPRINTF(IEW,
                "[tid:%i] [sn:%llu] "
                "JV: PBTB dispatch caught mispredict: %s"
                ". misp target:0x%x."
                " corr target:0x%x\n",
                tid, inst->seqNum, mispredReason,
                f_targAddr, d_targAddr);

        //++iewStats.controlMispred;  //TODO: TODO TODO
        ++iewStats.pbtbSquashes; // JV PBTB

        // Store finalized branch outcome as prediction on inst. This
        // isn't quite the right thing, but we need it in squash() to send
        // back the corrected branch outcome to fetch's predictor
        inst->setPredTarg(*d_targPC);
        inst->setPredTaken(d_taken);


        // === JV: WE MISPREDICTED!
        // squash(inst, inst->threadNumber);
        // caller needs to squash to::
        // - overwrite fetch-pbtb to correct state (from finalize),
        // - squash all incorrectly fetched insts,
        // - updates fetch's branch predicotr
        return true;


        //break;
    }

    // All good!
    return false;
}



} // namespace o3
} // namespace gem5
