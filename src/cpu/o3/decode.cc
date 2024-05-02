/*
 * Copyright (c) 2012, 2014 ARM Limited
 * All rights reserved
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

#include "cpu/o3/decode.hh"

#include "arch/generic/pcstate.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Activity.hh"
#include "debug/Decode.hh"
#include "debug/O3PipeView.hh"
#include "params/BaseO3CPU.hh"
#include "sim/full_system.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::list;

namespace gem5
{

namespace o3
{

Decode::Decode(CPU *_cpu, const BaseO3CPUParams &params)
    : cpu(_cpu),
      renameToDecodeDelay(params.renameToDecodeDelay),
      iewToDecodeDelay(params.iewToDecodeDelay),
      commitToDecodeDelay(params.commitToDecodeDelay),
      fetchToDecodeDelay(params.fetchToDecodeDelay),
      decodeWidth(params.decodeWidth),
      numThreads(params.numThreads),
      stats(_cpu)
{
    if (decodeWidth > MaxWidth)
        fatal("decodeWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             decodeWidth, static_cast<int>(MaxWidth));

    // @todo: Make into a parameter
    skidBufferMax = (fetchToDecodeDelay + 1) *  params.decodeWidth;
    for (int tid = 0; tid < MaxThreads; tid++) {
        stalls[tid] = {false};
        decodeStatus[tid] = Idle;
        bdelayDoneSeqNum[tid] = 0;
        squashInst[tid] = nullptr;
        squashAfterDelaySlot[tid] = 0;
    }
}

void
Decode::startupStage()
{
    resetStage();
}

void
Decode::clearStates(ThreadID tid)
{
    decodeStatus[tid] = Idle;
    stalls[tid].rename = false;
}

void
Decode::resetStage()
{
    _status = Inactive;

    // Setup status, make sure stall signals are clear.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        decodeStatus[tid] = Idle;

        stalls[tid].rename = false;
    }


    // TODO JV: Is this necessary? is this the right place for it?
    lastDecodedInst      = 0;
    lastDecodedBmov      = 0;
    lastDoneFromCommit   = 0;
    //lastSquashFromCommit = 0;
}

std::string
Decode::name() const
{
    return cpu->name() + ".decode";
}

Decode::DecodeStats::DecodeStats(CPU *cpu)
    : statistics::Group(cpu, "decode"),
      ADD_STAT(idleCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is idle"),
      ADD_STAT(blockedCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is blocked"),
      ADD_STAT(runCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is running"),
      ADD_STAT(unblockCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is unblocking"),
      ADD_STAT(squashCycles, statistics::units::Cycle::get(),
               "Number of cycles decode is squashing"),
      ADD_STAT(branchResolved, statistics::units::Count::get(),
               "Number of times decode resolved a branch"),
      ADD_STAT(branchMispred, statistics::units::Count::get(),
               "Number of times decode detected a branch misprediction"),
      ADD_STAT(controlMispred, statistics::units::Count::get(),
               "Number of times decode detected an instruction incorrectly "
               "predicted as a control"),
      ADD_STAT(decodedInsts, statistics::units::Count::get(),
               "Number of instructions handled by decode"),
      ADD_STAT(squashedInsts, statistics::units::Count::get(),
               "Number of squashed instructions handled by decode")
{
    idleCycles.prereq(idleCycles);
    blockedCycles.prereq(blockedCycles);
    runCycles.prereq(runCycles);
    unblockCycles.prereq(unblockCycles);
    squashCycles.prereq(squashCycles);
    branchResolved.prereq(branchResolved);
    branchMispred.prereq(branchMispred);
    controlMispred.prereq(controlMispred);
    decodedInsts.prereq(decodedInsts);
    squashedInsts.prereq(squashedInsts);
}

void
Decode::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to write information back to fetch.
    toFetch = timeBuffer->getWire(0);

    // Create wires to get information from proper places in time buffer.
    fromRename = timeBuffer->getWire(-renameToDecodeDelay);
    fromIEW = timeBuffer->getWire(-iewToDecodeDelay);
    fromCommit = timeBuffer->getWire(-commitToDecodeDelay);
}

void
Decode::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    decodeQueue = dq_ptr;

    // Setup wire to write information to proper place in decode queue.
    toRename = decodeQueue->getWire(0);
}

void
Decode::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    fetchQueue = fq_ptr;

    // Setup wire to read information from fetch queue.
    fromFetch = fetchQueue->getWire(-fetchToDecodeDelay);
}

void
Decode::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
Decode::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        assert(insts[tid].empty());
        assert(skidBuffer[tid].empty());
    }
}

bool
Decode::isDrained() const
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        if (!insts[tid].empty() || !skidBuffer[tid].empty() ||
                (decodeStatus[tid] != Running && decodeStatus[tid] != Idle))
            return false;
    }
    return true;
}

bool
Decode::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].rename) {
        DPRINTF(Decode,"[tid:%i] Stall fom Rename stage detected.\n", tid);
        ret_val = true;
    }


    // ===== TODO JV: PBTB:
    // if next inst is a placeholder branch,
    // we have to stall until it's ready to finalize (all bmovs have executed)
    // For now we're going to stall much more coarsely just to get it working

    bool readingFromSkid = (decodeStatus[tid] == Blocked
                         || decodeStatus[tid] == Unblocking);
    const std::deque<DynInstPtr>
        &insts_to_decode = readingFromSkid ?  skidBuffer[tid] : insts[tid];

    if (insts_to_decode.size() > 0) {
        DynInstPtr inst = insts_to_decode.front();

        if (    (inst->isDirectCtrl()
                    || inst->isBmov()) // TODO JV TEMP: Bmov dependencies hack
             && !isPBReadyToFinalize(inst))
        {
            DPRINTF(Decode,"[tid:%i] Stalling for pb finalize "
                           "(in checkStall)\n", tid);
            ret_val = true;
        }
    }




    return ret_val;
}

bool
Decode::fetchInstsValid()
{
    return fromFetch->size > 0;
}

// =============== JV PBTB FUNCS

bool Decode::isPBReadyToFinalize(DynInstPtr inst) const {
    DPRINTF(Decode,"[tid:X] isPBReadyToFinalize lastDecoded=%d, "
            "lastBmov=%d, lastCommitted=%d\n",
            lastDecodedInst, lastDecodedBmov, lastDoneFromCommit);

    //if (lastDoneFromCommit >= lastDecodedInst) {
    if (lastDoneFromCommit >= lastDecodedBmov) {
        return true;
    }

    return false;
    //TODO v1: assert isControl, check if last decoded op has committed
    //TODO v2: track last executed bmov? check stalls? ???
}
// =============== END JV PBTB FUNCS

bool
Decode::block(ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] Blocking.\n", tid);

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    // If the decode status is blocked or unblocking then decode has not yet
    // signalled fetch to unblock. In that case, there is no need to tell
    // fetch to block.
    if (decodeStatus[tid] != Blocked) {
        // Set the status to Blocked.
        decodeStatus[tid] = Blocked;

        if (toFetch->decodeUnblock[tid]) {
            toFetch->decodeUnblock[tid] = false;
        } else {
            toFetch->decodeBlock[tid] = true;
            wroteToTimeBuffer = true;
        }

        return true;
    }

    return false;
}

bool
Decode::unblock(ThreadID tid)
{
    // Decode is done unblocking only if the skid buffer is empty.
    if (skidBuffer[tid].empty()) {
        DPRINTF(Decode, "[tid:%i] Done unblocking.\n", tid);
        toFetch->decodeUnblock[tid] = true;
        wroteToTimeBuffer = true;

        decodeStatus[tid] = Running;
        return true;
    }

    DPRINTF(Decode, "[tid:%i] Currently unblocking.\n", tid);

    return false;
}

void
Decode::squash(const DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] [sn:%llu] Squashing due to incorrect branch "
            "prediction detected at decode (correct target: 0x%x).\n",
            tid, inst->seqNum,
            inst->readPredTarg().instAddr());

    // Send back mispredict information.
    toFetch->decodeInfo[tid].branchMispredict = true;
    toFetch->decodeInfo[tid].predIncorrect = true;
    toFetch->decodeInfo[tid].mispredictInst = inst;
    toFetch->decodeInfo[tid].squash = true;
    toFetch->decodeInfo[tid].doneSeqNum = inst->seqNum;

    // TODO JV PBTB: originally this used the branchTarget from the branch
    // However, now the pbs dont actually have a target
    // Instead, we update the inst->predTarg
    set(toFetch->decodeInfo[tid].nextPC, inst->readPredTarg());
    //OLD: set(toFetch->decodeInfo[tid].nextPC, *inst->branchTarget());

    // Looking at inst->pcState().branching()
    // may yield unexpected results if the branch
    // was predicted taken but aliased in the BTB
    // with a branch jumping to the next instruction (mistarget)
    // Using PCState::branching()  will send execution on the
    // fallthrough and this will not be caught at execution (since
    // branch was correctly predicted taken)
    toFetch->decodeInfo[tid].branchTaken = inst->readPredTaken() ||
                                           inst->isUncondCtrl();

    toFetch->decodeInfo[tid].squashInst = inst;

    InstSeqNum squash_seq_num = inst->seqNum;

    // Might have to tell fetch to unblock.
    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
        toFetch->decodeUnblock[tid] = 1;
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid &&
            fromFetch->insts[i]->seqNum > squash_seq_num) {
            fromFetch->insts[i]->setSquashed();
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop_front();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop_front();
    }


    // PBTB: Reset the fetch stage's PBTB to the finalize one
    cpu->PBTB.squashFinalizeToFetch();

    // Squash instructions up until this one
    cpu->removeInstsUntil(squash_seq_num, tid);
}

unsigned
Decode::squash(ThreadID tid)
{
    DPRINTF(Decode, "[tid:%i] Squashing.\n",tid);
    // (squash from commit and/or other stages)

    if (decodeStatus[tid] == Blocked ||
        decodeStatus[tid] == Unblocking) {
        if (FullSystem) {
            toFetch->decodeUnblock[tid] = 1;
        } else {
            // In syscall emulation, we can have both a block and a squash due
            // to a syscall in the same cycle.  This would cause both signals
            // to be high.  This shouldn't happen in full system.
            // @todo: Determine if this still happens.
            if (toFetch->decodeBlock[tid])
                toFetch->decodeBlock[tid] = 0;
            else
                toFetch->decodeUnblock[tid] = 1;
        }
    }

    // Set status to squashing.
    decodeStatus[tid] = Squashing;

    // Go through incoming instructions from fetch and squash them.
    unsigned squash_count = 0;

    for (int i=0; i<fromFetch->size; i++) {
        if (fromFetch->insts[i]->threadNumber == tid) {
            fromFetch->insts[i]->setSquashed();
            squash_count++;
        }
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    while (!insts[tid].empty()) {
        insts[tid].pop_front();
    }

    while (!skidBuffer[tid].empty()) {
        skidBuffer[tid].pop_front();
    }

    // PBTB: Reset the fetch stage's PBTB to the finalize one
    cpu->PBTB.squashFinalizeToFetch();

    return squash_count;
}

void
Decode::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop_front();

        assert(tid == inst->threadNumber);

        skidBuffer[tid].push_back(inst);

        DPRINTF(Decode, "Inserting [tid:%d][sn:%lli] PC: %s into decode "
                "skidBuffer %i\n", inst->threadNumber, inst->seqNum,
                inst->pcState(), skidBuffer[tid].size());
    }

    // @todo: Eventually need to enforce this by not letting a thread
    // fetch past its skidbuffer
    assert(skidBuffer[tid].size() <= skidBufferMax);
}

bool
Decode::skidsEmpty()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

void
Decode::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (decodeStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Decode will have activity if it's unblocking.
    if (any_unblocking) {
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(CPU::DecodeIdx);
        }
    } else {
        // If it's not unblocking, then decode will not have any internal
        // activity.  Switch it to inactive.
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(CPU::DecodeIdx);
        }
    }
}

void
Decode::sortInsts()
{
    int insts_from_fetch = fromFetch->size;
    for (int i = 0; i < insts_from_fetch; ++i) {
        insts[fromFetch->insts[i]->threadNumber].push_back(
            fromFetch->insts[i]);
    }
}

void
Decode::readStallSignals(ThreadID tid)
{
    if (fromRename->renameBlock[tid]) {
        stalls[tid].rename = true;
    }

    if (fromRename->renameUnblock[tid]) {
        assert(stalls[tid].rename);
        stalls[tid].rename = false;
    }
}

bool
Decode::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is.
    // Check stall signals, block if necessary.
    // If status was blocked
    //     Check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    // Update the per thread stall statuses.
    readStallSignals(tid);

    // DEBUG the from-commit signals
    //DPRINTF(Decode, "[tid:%i] fromcommit DEBUGGING: "
    //                "squash:%c doneSeqNum=%d\n",
    //                tid, fromCommit->commitInfo[tid].squash ? 'T':'F',
    //                    fromCommit->commitInfo[tid].doneSeqNum);



    // ==== TODO JV PBTB: keep track of committed info
    //                    for pb tracking
    // If we committed this cycle then doneSeqNum will be > 0
    if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
        !fromCommit->commitInfo[tid].squash) {

        InstSeqNum newNum = fromCommit->commitInfo[tid].doneSeqNum;
        DPRINTF(Decode, "[tid:%i] BMOV Tracking: new done inst"
                " from commit: [sn%d]\n", tid, newNum);
        assert(newNum >= lastDoneFromCommit);
        lastDoneFromCommit = newNum;
    }

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Decode, "[tid:%i] Squashing instructions due to squash "
                "from commit.\n", tid);


        //TODO JV PBTB: also track squash nums?
        InstSeqNum squashSeqNum = fromCommit->commitInfo[tid].doneSeqNum;
        // everything with seq>squashSeqNum is gone

        DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastDecode=%d, "
                "squash@%d, lastCommited=%d."
                " Moving lastDecoded up to squash\n",
                tid, lastDecodedInst, squashSeqNum, lastDoneFromCommit);

        // We've squashed some in-flight instructions, so we no longer
        // need to wait for lastDecodedInst, just the last non-squashed inst
        assert(squashSeqNum >= lastDoneFromCommit); // sanity check?

        if (lastDecodedInst > squashSeqNum) {
            DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastDecode=%d, "
                    "squash@%d, lastCommited=%d."
                    " Moving lastDecoded up to squash\n",
                    tid, lastDecodedInst,
                    squashSeqNum, lastDoneFromCommit);
            lastDecodedInst = squashSeqNum;
            //TODO: once we track specifically the last bmov, it'll be
            // a little trickier to do

        }
        if (lastDecodedBmov > squashSeqNum) {
            DPRINTF(Decode, "[tid:%i] BMOV Tracking: lastBmov=%d, "
                    "squash@%d, lastCommited=%d."
                    " Moving lastBmov up to squash\n",
                    tid, lastDecodedBmov,
                    squashSeqNum, lastDoneFromCommit);
            lastDecodedBmov = squashSeqNum;
        }
        //(fromCommit->commitInfo[tid].doneSeqNum, tid);
        //lastSquashFromCommit = newSquashNum;

        squash(tid);

        return true;
    }

    if (checkStall(tid)) {
        return block(tid);
    }

    if (decodeStatus[tid] == Blocked) {
        DPRINTF(Decode, "[tid:%i] Done blocking, switching to unblocking.\n",
                tid);

        decodeStatus[tid] = Unblocking;

        unblock(tid);

        return true;
    }

    if (decodeStatus[tid] == Squashing) {
        // Switch status to running if decode isn't being told to block or
        // squash this cycle.
        DPRINTF(Decode, "[tid:%i] Done squashing, switching to running.\n",
                tid);

        decodeStatus[tid] = Running;

        return false;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause decode to change its status.  Decode remains the same as before.
    return false;
}

void
Decode::tick()
{
    wroteToTimeBuffer = false;

    bool status_change = false;

    toRenameIndex = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    sortInsts();

    //Check stall and squash signals.
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(Decode,"Processing [tid:%i]\n",tid);
        status_change =  checkSignalsAndUpdate(tid) || status_change;

        decode(status_change, tid);
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");

        cpu->activityThisCycle();
    }
}

void
Decode::decode(bool &status_change, ThreadID tid)
{
    // If status is Running or idle,
    //     call decodeInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from fetch
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (decodeStatus[tid] == Blocked) {
        ++stats.blockedCycles;
    } else if (decodeStatus[tid] == Squashing) {
        ++stats.squashCycles;
    }

    // Decode should try to decode as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (decodeStatus[tid] == Running ||
        decodeStatus[tid] == Idle) {
        DPRINTF(Decode, "[tid:%i] Not blocked, so attempting to run "
                "stage.\n",tid);

        decodeInsts(tid);
    } else if (decodeStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        decodeInsts(tid);

        if (fetchInstsValid()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        status_change = unblock(tid) || status_change;
    }
}

void
Decode::decodeInsts(ThreadID tid)
{
    // Instructions can come either from the skid buffer or the list of
    // instructions coming from fetch, depending on decode's status.
    int insts_available = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();

    if (insts_available == 0) {
        DPRINTF(Decode, "[tid:%i] Nothing to do, breaking out"
                " early.\n",tid);
        // Should I change the status to idle?
        ++stats.idleCycles;
        return;
    } else if (decodeStatus[tid] == Unblocking) {
        DPRINTF(Decode, "[tid:%i] Unblocking, removing insts from skid "
                "buffer.\n",tid);
        ++stats.unblockCycles;
    } else if (decodeStatus[tid] == Running) {
        ++stats.runCycles;
    }

    std::deque<DynInstPtr>
        &insts_to_decode = decodeStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    DPRINTF(Decode, "[tid:%i] Sending instruction to rename.\n",tid);

    while (insts_available > 0 && toRenameIndex < decodeWidth) {
        assert(!insts_to_decode.empty());

        DynInstPtr inst = std::move(insts_to_decode.front());

        insts_to_decode.pop_front();

        DPRINTF(Decode, "[tid:%i] Processing instruction [sn:%lli] with "
                "PC %s\n", tid, inst->seqNum, inst->pcState());

        if (inst->isSquashed()) {
            DPRINTF(Decode, "[tid:%i] Instruction %i with PC %s is "
                    "squashed, skipping.\n",
                    tid, inst->seqNum, inst->pcState());

            ++stats.squashedInsts;

            --insts_available;

            continue;
        }

        // TODO JV: change this to isPB
        // TODO JV: pb AND not squashed (checked above)
        // If it's a pb or it was predicted as a pb, need to
        // make sure we're ready to finalize it
        if ((inst->readPredBTBReg() >= 0 || inst->isControl()
                    || inst->isBmov()) //TODO JV TEMP: Bmov dependency hack
                && !isPBReadyToFinalize(inst))
        {
            DPRINTF(Decode,"[tid:%i] Stalling for pb finalize\n", tid);
            //DPRINTF(Decode, "JV PBTB: Blocking branch in finalize"
            //                " (in decode)!\n");

            insts_to_decode.push_front(inst);
            // push_back has to happen first so block()
            // can shift remaining insts to the skidbuffer correctly
            block(tid);
            break;
        }

        // Also check if instructions have no source registers.  Mark
        // them as ready to issue at any time.  Not sure if this check
        // should exist here or at a later stage; however it doesn't matter
        // too much for function correctness.
        if (inst->numSrcRegs() == 0) {
            inst->setCanIssue();
        }

        // This current instruction is valid, so add it into the decode
        // queue.  The next instruction may not be valid, so check to
        // see if branches were predicted correctly.
        toRename->insts[toRenameIndex] = inst;


        // ==== TODO JV PBTB: count this as decoded
        // TODO: make this only count bmovs
        // Note: we know it's not squashed at this point)
        assert(inst->seqNum > lastDecodedInst);
        lastDecodedInst = inst->seqNum;
        if (inst->isBmov()) {
            assert(inst->seqNum > lastDecodedBmov);
            lastDecodedBmov = inst->seqNum;
        }

        ++(toRename->size);
        ++toRenameIndex;
        ++stats.decodedInsts;
        --insts_available;

#if TRACING_ON
        if (debug::O3PipeView) {
            inst->decodeTick = curTick() - inst->fetchTick;
        }
#endif

        // ===== PBTB: requery the finalize pbtb

        PrecomputedBTB::PBTBResultType res;
        int      d_breg  = -1;
        uint64_t d_version = 0;
        bool     d_exhausted;
        bool     d_taken;
        Addr     d_targAddr = 0; //TODO
        //TODO: do this properly
        res = cpu->PBTB.queryFromDecode(inst->staticInst,
                                        inst->pcState().instAddr(),
                                        &d_breg, &d_version, &d_targAddr);
        d_exhausted = (res == PrecomputedBTB::PR_Exhaust);
        d_taken = (res == PrecomputedBTB::PR_Taken);

        // ===== Also pull up the original predictions from fetch to compare
        int      f_breg      = inst->readPredBTBReg();
        uint64_t f_version   = inst->readPredBTBVersion();
        bool     f_exhausted = inst->readPredBTBExhausted();
        bool     f_taken     = inst->readPredTaken();
        Addr     f_targAddr  = inst->readPredTarg().instAddr();

        //We now have all our variables

        if (d_exhausted) { // Incorrect code: missed a bmov somewhere
            panic("PBTB exhausted in finalize (in decode)\n");
            //TODO: this should eventually throw a BMOV exception
        }

        // PBTB sees a branch where it shouldn't be, or sees no branch
        // where there should be one (Incorrect code)
        if ((inst->isControl()  && d_breg<0) ||
           (!inst->isControl() && d_breg >= 0)) {
            DPRINTF(Decode, "ERROR: inst->isControl:%c, d_breg=%d\n",
                    inst->isControl()?'T':'F', d_breg);

            panic("PBTB doesn't match placeholder"
                  " branch in finalize (in decode)\n");
            //TODO: change this to check pb and verify matching breg
            //TODO: this should eventually throw a BMOV exception
        }


        bool mispred = false;
        char mispredReason[256] = "";

        // Fetch predicted a branch, but there's no branch here now
        if (d_breg < 0 && f_breg >= 0) {
            mispred = true;
            snprintf(mispredReason, sizeof(mispredReason),
                "fetch predicted a branch where there was none: f@b%d-vi%ld",
                f_breg, f_version);
        } else if (d_breg >= 0) {
            // There is a branch here:

            if (f_breg != d_breg) {
                mispred = true;
                snprintf(mispredReason, sizeof(mispredReason),
                        "fetch had wrong breg: f@b%d, d@b%d",
                        f_breg, d_breg);
            } else if (f_exhausted) {
                mispred = true;
                snprintf(mispredReason, sizeof(mispredReason),
                        "fetch read from exhausted breg: f@b%d-EXH",
                        f_breg);
            } else if (f_version != d_version) {
                mispred = true;
                snprintf(mispredReason, sizeof(mispredReason),
                        "version mismatch: f@b%d-v%ld, d@b%d-v%ld",
                        f_breg, f_version, d_breg, d_version);
            }

            DPRINTF(Decode, "JV PBTB: Finalized a branch (%s)"
                            " at pc0x%x->0x%x [sn:%d], b%dv%d\n",
                    d_taken ? "T":"NT",
                    inst->pcState().instAddr(), d_targAddr,
                    inst->seqNum,
                    d_breg, d_version);
            //TODO TEMP DEBUG
            // DPRINTF(Decode, "JV PBTB: FINALIZING: DEBUG INFO: @pc0x%x\n"
            //             "------------------d: b%dv%ld: %s%s, ->0x%x\n"
            //             "------------------f: b%dv%ld: %s%s, ->0x%x\n",
            //         inst->pcState().instAddr(),
            //         d_breg, d_version,
            //         d_exhausted ? "EXH ":"", d_taken?"T":"NT",
            //         d_targAddr,
            //         f_breg, f_version,
            //         f_exhausted ? "EXH ":"", f_taken?"T":"NT",
            //         f_targAddr);

            if (!mispred) {
                // At this point, finalize sees a branch, and fetch
                // seems to have matched it
                // Make sure that the actual outcome agrees
                //DPRINTF(Decode,
                //        "JV PBTB: Finalized a branch at pc0x%x->0x%x\n",
                //        inst->pcState().instAddr(), d_targAddr);
                assert(d_taken == f_taken);
                assert(!d_taken || (d_targAddr == f_targAddr));
                //TODO: should this be only if taken?
                //     ++stats.branchResolved;
            } else {
                //++stats.branchMispred;

                //// Might want to set some sort of boolean and just do
                //// a check at the end
                //squash(inst, inst->threadNumber);

                //DPRINTF(Decode,
                //        "[tid:%i] [sn:%llu] "
                //        "Updating predictions: Wrong predicted target: %s \
                //        PredPC: %s\n",
                //        tid, inst->seqNum, inst->readPredTarg(), *target);
                ////The micro pc after an instruction level branch should be 0
                //inst->setPredTarg(*target);
                //break;
            }

        }

        if (mispred) {
            DPRINTF(Decode,
                    "[tid:%i] [sn:%llu] "
                    "JV: PBTB decode caught mispredict: %s"
                    ". misp target:0x%x."
                    " corr target:0x%x\n",
                    tid, inst->seqNum, mispredReason,
                    f_targAddr, d_targAddr);
            ++stats.controlMispred;


            //Update to correct target
            auto tempAddr = GenericISA::SimplePCState<4>();
            tempAddr.set(d_targAddr);
            inst->setPredTarg(tempAddr);

            // squash overwrites pbtb to correct state (from finalize)
            squash(inst, inst->threadNumber);

            break;
        }


        // TODO JV TEMP: DON'T CHECK BRANCHES, LET PBTB DRIVE IT
        // // Ensure that if it was predicted as a branch, it really is a
        // // branch.
        // if (inst->readPredTaken() && !inst->isControl()) {
        //     panic("Instruction predicted as a branch!");

        //     ++stats.controlMispred;

        //     // Might want to set some sort of boolean and just do
        //     // a check at the end
        //     squash(inst, inst->threadNumber);

        //     break;
        // }

        // // OLD JV DEBUG INFO
        // if (inst->readPredTaken()) {
        //     DPRINTF(Decode,
        //             "[tid:%i] [sn:%llu] "
        //             "JV: PBTB predicted a branch to: %s\n",
        //             //(op target: %s\n)",
        //             tid, inst->seqNum, inst->readPredTarg());
        //             //, *(inst->branchTarget()));
        // }

        //TODO JV TEMP: Don't check branches! Default to predicted target??
        // // Go ahead and compute any PC-relative branches.
        // // This includes direct unconditional control and
        // // direct conditional control that is predicted taken.
        // if (inst->isDirectCtrl() &&
        //    (inst->isUncondCtrl() || inst->readPredTaken()))
        // {
        //     ++stats.branchResolved;

        //     std::unique_ptr<PCStateBase> target = inst->branchTarget();
        //     if (*target != inst->readPredTarg()) {
        //         ++stats.branchMispred;

        //         // Might want to set some sort of boolean and just do
        //         // a check at the end
        //         squash(inst, inst->threadNumber);

        //         DPRINTF(Decode,
        //                 "[tid:%i] [sn:%llu] "
        //                 "Updating predictions: Wrong predicted target: %s \
        //                 PredPC: %s\n",
        //                 tid, inst->seqNum, inst->readPredTarg(), *target);
        //         //The micro pc after an instruction level branch should be 0
        //         inst->setPredTarg(*target);
        //         break;
        //     }
        // }
    }

    // If we didn't process all instructions, then we will need to block
    // and put all those instructions into the skid buffer.
    if (!insts_to_decode.empty()) {
        block(tid);
    }

    // Record that decode has written to the time buffer for activity
    // tracking.
    if (toRenameIndex) {
        wroteToTimeBuffer = true;
    }
}

} // namespace o3
} // namespace gem5
