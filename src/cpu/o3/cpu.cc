/*
 * Copyright (c) 2011-2012, 2014, 2016, 2017, 2019-2020 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2011 Regents of the University of California
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

#include "cpu/o3/cpu.hh"

#include "cpu/activity.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "cpu/o3/thread_context.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/O3CPU.hh"
#include "debug/Quiesce.hh"
#include "enums/MemoryMode.hh"
#include "sim/cur_tick.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

namespace gem5
{

struct BaseCPUParams;

namespace o3
{

CPU::CPU(const BaseO3CPUParams &params)
    : BaseCPU(params),
      mmu(params.mmu),
      tickEvent([this]{ tick(); }, "O3CPU tick",
                false, Event::CPU_Tick_Pri),
      threadExitEvent([this]{ exitThreads(); }, "O3CPU exit threads",
                false, Event::CPU_Exit_Pri),
#ifndef NDEBUG
      instcount(0),
#endif
      removeInstsThisCycle(false),
      fetch(this, params),
      decode(this, params),
      rename(this, params),
      iew(this, params),
      commit(this, params),

      regFile(params.numPhysIntRegs,
              params.numPhysFloatRegs,
              params.numPhysVecRegs,
              params.numPhysVecPredRegs,
              params.numPhysMatRegs,
              params.numPhysCCRegs,
              params.isa[0]->regClasses()),

      freeList(name() + ".freelist", &regFile),

      rob(this, params),

      scoreboard(name() + ".scoreboard", regFile.totalNumPhysRegs()),

      isa(numThreads, NULL),

      timeBuffer(params.backComSize, params.forwardComSize),
      fetchQueue(params.backComSize, params.forwardComSize),
      decodeQueue(params.backComSize, params.forwardComSize),
      renameQueue(params.backComSize, params.forwardComSize),
      iewQueue(params.backComSize, params.forwardComSize),
      activityRec(name(), NumStages,
                  params.backComSize + params.forwardComSize,
                  params.activity),

      globalSeqNum(1),
      system(params.system),
      lastRunningCycle(curCycle()),
      cpuStats(this)
{
    fatal_if(FullSystem && params.numThreads > 1,
            "SMT is not supported in O3 in full system mode currently.");

    fatal_if(!FullSystem && params.numThreads < params.workload.size(),
            "More workload items (%d) than threads (%d) on CPU %s.",
            params.workload.size(), params.numThreads, name());

    if (!params.switched_out) {
        _status = Running;
    } else {
        _status = SwitchedOut;
    }

    if (params.checker) {
        BaseCPU *temp_checker = params.checker;
        checker = dynamic_cast<Checker<DynInstPtr> *>(temp_checker);
        checker->setIcachePort(&fetch.getInstPort());
        checker->setSystem(params.system);
    } else {
        checker = NULL;
    }

    if (!FullSystem) {
        thread.resize(numThreads);
        tids.resize(numThreads);
    }

    // The stages also need their CPU pointer setup.  However this
    // must be done at the upper level CPU because they have pointers
    // to the upper level CPU, and not this CPU.

    // Set up Pointers to the activeThreads list for each stage
    fetch.setActiveThreads(&activeThreads);
    decode.setActiveThreads(&activeThreads);
    rename.setActiveThreads(&activeThreads);
    iew.setActiveThreads(&activeThreads);
    commit.setActiveThreads(&activeThreads);

    // Give each of the stages the time buffer they will use.
    fetch.setTimeBuffer(&timeBuffer);
    decode.setTimeBuffer(&timeBuffer);
    rename.setTimeBuffer(&timeBuffer);
    iew.setTimeBuffer(&timeBuffer);
    commit.setTimeBuffer(&timeBuffer);

    // Also setup each of the stages' queues.
    fetch.setFetchQueue(&fetchQueue);
    decode.setFetchQueue(&fetchQueue);
    commit.setFetchQueue(&fetchQueue);
    decode.setDecodeQueue(&decodeQueue);
    rename.setDecodeQueue(&decodeQueue);
    rename.setRenameQueue(&renameQueue);
    iew.setRenameQueue(&renameQueue);
    iew.setIEWQueue(&iewQueue);
    commit.setIEWQueue(&iewQueue);
    commit.setRenameQueue(&renameQueue);

    commit.setIEWStage(&iew);
    rename.setIEWStage(&iew);
    rename.setCommitStage(&commit);

    ThreadID active_threads;
    if (FullSystem) {
        active_threads = 1;
    } else {
        active_threads = params.workload.size();

        if (active_threads > MaxThreads) {
            panic("Workload Size too large. Increase the 'MaxThreads' "
                  "constant in cpu/o3/limits.hh or edit your workload size.");
        }
    }

    // Make Sure That this a Valid Architeture
    assert(numThreads);
    const auto &regClasses = params.isa[0]->regClasses();

    assert(params.numPhysIntRegs >=
            numThreads * regClasses.at(IntRegClass)->numRegs());
    assert(params.numPhysFloatRegs >=
            numThreads * regClasses.at(FloatRegClass)->numRegs());
    assert(params.numPhysVecRegs >=
            numThreads * regClasses.at(VecRegClass)->numRegs());
    assert(params.numPhysVecPredRegs >=
            numThreads * regClasses.at(VecPredRegClass)->numRegs());
    assert(params.numPhysMatRegs >=
            numThreads * regClasses.at(MatRegClass)->numRegs());
    assert(params.numPhysCCRegs >=
            numThreads * regClasses.at(CCRegClass)->numRegs());

    // Just make this a warning and go ahead anyway, to keep from having to
    // add checks everywhere.
    warn_if(regClasses.at(CCRegClass)->numRegs() == 0 &&
            params.numPhysCCRegs != 0,
            "Non-zero number of physical CC regs specified, even though\n"
            "    ISA does not use them.");

    rename.setScoreboard(&scoreboard);
    iew.setScoreboard(&scoreboard);

    // Setup the rename map for whichever stages need it.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        isa[tid] = params.isa[tid];
        commitRenameMap[tid].init(regClasses, &regFile, &freeList);
        renameMap[tid].init(regClasses, &regFile, &freeList);
    }

    // Initialize rename map to assign physical registers to the
    // architectural registers for active threads only.
    for (ThreadID tid = 0; tid < active_threads; tid++) {
        for (auto type = (RegClassType)0; type <= CCRegClass;
                type = (RegClassType)(type + 1)) {
            for (auto &id: *regClasses.at(type)) {
                // Note that we can't use the rename() method because we don't
                // want special treatment for the zero register at this point
                PhysRegIdPtr phys_reg = freeList.getReg(type);
                renameMap[tid].setEntry(id, phys_reg);
                commitRenameMap[tid].setEntry(id, phys_reg);
            }
        }
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);
    rename.setFreeList(&freeList);

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);

    lastActivatedCycle = 0;

    DPRINTF(O3CPU, "Creating O3CPU object.\n");

    // Setup any thread state.
    thread.resize(numThreads);

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        if (FullSystem) {
            // SMT is not supported in FS mode yet.
            assert(numThreads == 1);
            thread[tid] = new ThreadState(this, 0, NULL);
        } else {
            if (tid < params.workload.size()) {
                DPRINTF(O3CPU, "Workload[%i] process is %#x", tid,
                        thread[tid]);
                thread[tid] = new ThreadState(this, tid, params.workload[tid]);
            } else {
                //Allocate Empty thread so M5 can use later
                //when scheduling threads to CPU
                Process* dummy_proc = NULL;

                thread[tid] = new ThreadState(this, tid, dummy_proc);
            }
        }

        gem5::ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        auto *o3_tc = new ThreadContext;

        tc = o3_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
        if (params.checker) {
            tc = new CheckerThreadContext<ThreadContext>(o3_tc, checker);
        }

        o3_tc->cpu = this;
        o3_tc->thread = thread[tid];

        // Give the thread the TC.
        thread[tid]->tc = tc;

        // Add the TC to the CPU's list of TC's.
        threadContexts.push_back(tc);
    }

    // O3CPU always requires an interrupt controller.
    if (!params.switched_out && interrupts.empty()) {
        fatal("O3CPU %s has no interrupt controller.\n"
              "Ensure createInterruptController() is called.\n", name());
    }
}

void
CPU::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppInstAccessComplete = new ProbePointArg<PacketPtr>(
            getProbeManager(), "InstAccessComplete");
    ppDataAccessComplete = new ProbePointArg<
        std::pair<DynInstPtr, PacketPtr>>(
                getProbeManager(), "DataAccessComplete");

    fetch.regProbePoints();
    rename.regProbePoints();
    iew.regProbePoints();
    commit.regProbePoints();
}

CPU::CPUStats::CPUStats(CPU *cpu)
    : statistics::Group(cpu),
      ADD_STAT(timesIdled, statistics::units::Count::get(),
               "Number of times that the entire CPU went into an idle state "
               "and unscheduled itself"),
      ADD_STAT(idleCycles, statistics::units::Cycle::get(),
               "Total number of cycles that the CPU has spent unscheduled due "
               "to idling"),
      ADD_STAT(quiesceCycles, statistics::units::Cycle::get(),
               "Total number of cycles that CPU has spent quiesced or waiting "
               "for an interrupt")
{
    // Register any of the O3CPU's stats here.
    timesIdled
        .prereq(timesIdled);

    idleCycles
        .prereq(idleCycles);

    quiesceCycles
        .prereq(quiesceCycles);
}

void
CPU::tick()
{
    DPRINTF(O3CPU, "\n\nO3CPU: Ticking main, O3CPU.\n");
    assert(!switchedOut());
    assert(drainState() != DrainState::Drained);

    ++baseStats.numCycles;
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

//    activity = false;

    //Tick each of the stages
    fetch.tick();

    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();

    // Now advance the time buffers
    timeBuffer.advance();

    fetchQueue.advance();
    decodeQueue.advance();
    renameQueue.advance();
    iewQueue.advance();

    activityRec.advance();

    if (removeInstsThisCycle) {
        cleanUpRemovedInsts();
    }

    if (!tickEvent.scheduled()) {
        if (_status == SwitchedOut) {
            DPRINTF(O3CPU, "Switched out!\n");
            // increment stat
            lastRunningCycle = curCycle();
        } else if (!activityRec.active() || _status == Idle) {
            DPRINTF(O3CPU, "Idle!\n");
            lastRunningCycle = curCycle();
            cpuStats.timesIdled++;
        } else {
            schedule(tickEvent, clockEdge(Cycles(1)));
            DPRINTF(O3CPU, "Scheduling next tick!\n");
        }
    }

    if (!FullSystem)
        updateThreadPriority();

    tryDrain();
}

void
CPU::init()
{
    BaseCPU::init();

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        // Set noSquashFromTC so that the CPU doesn't squash when initially
        // setting up registers.
        thread[tid]->noSquashFromTC = true;
    }

    // Clear noSquashFromTC.
    for (int tid = 0; tid < numThreads; ++tid)
        thread[tid]->noSquashFromTC = false;

    commit.setThreads(thread);
}

void
CPU::startup()
{
    BaseCPU::startup();

    fetch.startupStage();
    decode.startupStage();
    iew.startupStage();
    rename.startupStage();
    commit.startupStage();
}

void
CPU::activateThread(ThreadID tid)
{
    std::list<ThreadID>::iterator isActive =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i] Calling activate thread.\n", tid);
    assert(!switchedOut());

    if (isActive == activeThreads.end()) {
        DPRINTF(O3CPU, "[tid:%i] Adding to active threads list\n", tid);

        activeThreads.push_back(tid);
    }
}

void
CPU::deactivateThread(ThreadID tid)
{
    // hardware transactional memory
    // shouldn't deactivate thread in the middle of a transaction
    assert(!commit.executingHtmTransaction(tid));

    //Remove From Active List, if Active
    std::list<ThreadID>::iterator thread_it =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i] Calling deactivate thread.\n", tid);
    assert(!switchedOut());

    if (thread_it != activeThreads.end()) {
        DPRINTF(O3CPU,"[tid:%i] Removing from active threads list\n",
                tid);
        activeThreads.erase(thread_it);
    }

    fetch.deactivateThread(tid);
    commit.deactivateThread(tid);
}

Counter
CPU::totalInsts() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numInst;

    return total;
}

Counter
CPU::totalOps() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numOp;

    return total;
}

void
CPU::activateContext(ThreadID tid)
{
    assert(!switchedOut());

    // Needs to set each stage to running as well.
    activateThread(tid);

    // We don't want to wake the CPU if it is drained. In that case,
    // we just want to flag the thread as active and schedule the tick
    // event from drainResume() instead.
    if (drainState() == DrainState::Drained)
        return;

    // If we are time 0 or if the last activation time is in the past,
    // schedule the next tick and wake up the fetch unit
    if (lastActivatedCycle == 0 || lastActivatedCycle < curTick()) {
        scheduleTickEvent(Cycles(0));

        // Be sure to signal that there's some activity so the CPU doesn't
        // deschedule itself.
        activityRec.activity();
        fetch.wakeFromQuiesce();

        Cycles cycles(curCycle() - lastRunningCycle);
        // @todo: This is an oddity that is only here to match the stats
        if (cycles != 0)
            --cycles;
        cpuStats.quiesceCycles += cycles;

        lastActivatedCycle = curTick();

        _status = Running;

        BaseCPU::activateContext(tid);
    }
}

void
CPU::suspendContext(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Suspending Thread Context.\n", tid);
    assert(!switchedOut());

    deactivateThread(tid);

    // If this was the last thread then unschedule the tick event.
    if (activeThreads.size() == 0) {
        unscheduleTickEvent();
        lastRunningCycle = curCycle();
        _status = Idle;
    }

    DPRINTF(Quiesce, "Suspending Context\n");

    BaseCPU::suspendContext(tid);
}

void
CPU::haltContext(ThreadID tid)
{
    //For now, this is the same as deallocate
    DPRINTF(O3CPU,"[tid:%i] Halt Context called. Deallocating\n", tid);
    assert(!switchedOut());

    deactivateThread(tid);
    removeThread(tid);

    // If this was the last thread then unschedule the tick event.
    if (activeThreads.size() == 0) {
        if (tickEvent.scheduled())
        {
            unscheduleTickEvent();
        }
        lastRunningCycle = curCycle();
        _status = Idle;
    }
    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}

void
CPU::insertThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Initializing thread into CPU");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the ThreadContext.
    gem5::ThreadContext *src_tc;
    if (FullSystem)
        src_tc = system->threads[tid];
    else
        src_tc = tcBase(tid);

    //Bind Int Regs to Rename Map
    const auto &regClasses = isa[tid]->regClasses();

    for (auto type = (RegClassType)0; type <= CCRegClass;
            type = (RegClassType)(type + 1)) {
        for (auto &id: *regClasses.at(type)) {
            PhysRegIdPtr phys_reg = freeList.getReg(type);
            renameMap[tid].setEntry(id, phys_reg);
            scoreboard.setReg(phys_reg);
        }
    }

    //Copy Thread Data Into RegFile
    //copyFromTC(tid);

    //Set PC/NPC/NNPC
    pcState(src_tc->pcState(), tid);

    src_tc->setStatus(gem5::ThreadContext::Active);

    activateContext(tid);

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
}

void
CPU::removeThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Removing thread context from CPU.\n", tid);

    // Copy Thread Data From RegFile
    // If thread is suspended, it might be re-allocated
    // copyToTC(tid);


    // @todo: 2-27-2008: Fix how we free up rename mappings
    // here to alleviate the case for double-freeing registers
    // in SMT workloads.

    // clear all thread-specific states in each stage of the pipeline
    // since this thread is going to be completely removed from the CPU
    commit.clearStates(tid);
    fetch.clearStates(tid);
    decode.clearStates(tid);
    rename.clearStates(tid);
    iew.clearStates(tid);

    // Flush out any old data from the time buffers.
    for (int i = 0; i < timeBuffer.getSize(); ++i) {
        timeBuffer.advance();
        fetchQueue.advance();
        decodeQueue.advance();
        renameQueue.advance();
        iewQueue.advance();
    }

    // at this step, all instructions in the pipeline should be already
    // either committed successfully or squashed. All thread-specific
    // queues in the pipeline must be empty.
    assert(iew.instQueue.getCount(tid) == 0);
    assert(iew.ldstQueue.getCount(tid) == 0);
    assert(commit.rob->isEmpty(tid));

    // Reset ROB/IQ/LSQ Entries

    // Commented out for now.  This should be possible to do by
    // telling all the pipeline stages to drain first, and then
    // checking until the drain completes.  Once the pipeline is
    // drained, call resetEntries(). - 10-09-06 ktlim
/*
    if (activeThreads.size() >= 1) {
        commit.rob->resetEntries();
        iew.resetEntries();
    }
*/
}

Fault
CPU::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return interrupts[0]->getInterrupt();
}

void
CPU::processInterrupts(const Fault &interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
    interrupts[0]->updateIntrInfo();

    DPRINTF(O3CPU, "Interrupt %s being handled\n", interrupt->name());
    trap(interrupt, 0, nullptr);
}

void
CPU::trap(const Fault &fault, ThreadID tid, const StaticInstPtr &inst)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(threadContexts[tid], inst);
}

void
CPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    thread[tid]->serialize(cp);
}

void
CPU::unserializeThread(CheckpointIn &cp, ThreadID tid)
{
    thread[tid]->unserialize(cp);
}

DrainState
CPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    // If the CPU isn't doing anything, then return immediately.
    if (switchedOut())
        return DrainState::Drained;

    DPRINTF(Drain, "Draining...\n");

    // We only need to signal a drain to the commit stage as this
    // initiates squashing controls the draining. Once the commit
    // stage commits an instruction where it is safe to stop, it'll
    // squash the rest of the instructions in the pipeline and force
    // the fetch stage to stall. The pipeline will be drained once all
    // in-flight instructions have retired.
    commit.drain();

    // Wake the CPU and record activity so everything can drain out if
    // the CPU was not able to immediately drain.
    if (!isCpuDrained())  {
        // If a thread is suspended, wake it up so it can be drained
        for (auto t : threadContexts) {
            if (t->status() == gem5::ThreadContext::Suspended){
                DPRINTF(Drain, "Currently suspended so activate %i \n",
                        t->threadId());
                t->activate();
                // As the thread is now active, change the power state as well
                activateContext(t->threadId());
            }
        }

        wakeCPU();
        activityRec.activity();

        DPRINTF(Drain, "CPU not drained\n");

        return DrainState::Draining;
    } else {
        DPRINTF(Drain, "CPU is already drained\n");
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        // Flush out any old data from the time buffers.  In
        // particular, there might be some data in flight from the
        // fetch stage that isn't visible in any of the CPU buffers we
        // test in isCpuDrained().
        for (int i = 0; i < timeBuffer.getSize(); ++i) {
            timeBuffer.advance();
            fetchQueue.advance();
            decodeQueue.advance();
            renameQueue.advance();
            iewQueue.advance();
        }

        drainSanityCheck();
        return DrainState::Drained;
    }
}

bool
CPU::tryDrain()
{
    if (drainState() != DrainState::Draining || !isCpuDrained())
        return false;

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();

    return true;
}

void
CPU::drainSanityCheck() const
{
    assert(isCpuDrained());
    fetch.drainSanityCheck();
    decode.drainSanityCheck();
    rename.drainSanityCheck();
    iew.drainSanityCheck();
    commit.drainSanityCheck();
}

bool
CPU::isCpuDrained() const
{
    bool drained(true);

    if (!instList.empty() || !removeList.empty()) {
        DPRINTF(Drain, "Main CPU structures not drained.\n");
        drained = false;
    }

    if (!fetch.isDrained()) {
        DPRINTF(Drain, "Fetch not drained.\n");
        drained = false;
    }

    if (!decode.isDrained()) {
        DPRINTF(Drain, "Decode not drained.\n");
        drained = false;
    }

    if (!rename.isDrained()) {
        DPRINTF(Drain, "Rename not drained.\n");
        drained = false;
    }

    if (!iew.isDrained()) {
        DPRINTF(Drain, "IEW not drained.\n");
        drained = false;
    }

    if (!commit.isDrained()) {
        DPRINTF(Drain, "Commit not drained.\n");
        drained = false;
    }

    return drained;
}

void CPU::commitDrained(ThreadID tid) { fetch.drainStall(tid); }

void
CPU::drainResume()
{
    if (switchedOut())
        return;

    DPRINTF(Drain, "Resuming...\n");
    verifyMemoryMode();

    fetch.drainResume();
    commit.drainResume();

    _status = Idle;
    for (ThreadID i = 0; i < thread.size(); i++) {
        if (thread[i]->status() == gem5::ThreadContext::Active) {
            DPRINTF(Drain, "Activating thread: %i\n", i);
            activateThread(i);
            _status = Running;
        }
    }

    assert(!tickEvent.scheduled());
    if (_status == Running)
        schedule(tickEvent, nextCycle());

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

void
CPU::switchOut()
{
    DPRINTF(O3CPU, "Switching out\n");
    BaseCPU::switchOut();

    activityRec.reset();

    _status = SwitchedOut;

    if (checker)
        checker->switchOut();
}

void
CPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    fetch.takeOverFrom();
    decode.takeOverFrom();
    rename.takeOverFrom();
    iew.takeOverFrom();
    commit.takeOverFrom();

    assert(!tickEvent.scheduled());

    auto *oldO3CPU = dynamic_cast<CPU *>(oldCPU);
    if (oldO3CPU)
        globalSeqNum = oldO3CPU->globalSeqNum;

    lastRunningCycle = curCycle();
    _status = Idle;
}

void
CPU::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The O3 CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

RegVal
CPU::readMiscRegNoEffect(int misc_reg, ThreadID tid) const
{
    return isa[tid]->readMiscRegNoEffect(misc_reg);
}

RegVal
CPU::readMiscReg(int misc_reg, ThreadID tid)
{
    executeStats[tid]->numMiscRegReads++;
    return isa[tid]->readMiscReg(misc_reg);
}

void
CPU::setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid)
{
    isa[tid]->setMiscRegNoEffect(misc_reg, val);
}

void
CPU::setMiscReg(int misc_reg, RegVal val, ThreadID tid)
{
    executeStats[tid]->numMiscRegWrites++;
    isa[tid]->setMiscReg(misc_reg, val);
}

RegVal
CPU::getReg(PhysRegIdPtr phys_reg, ThreadID tid)
{
    switch (phys_reg->classValue()) {
      case IntRegClass:
        executeStats[tid]->numIntRegReads++;
        break;
      case FloatRegClass:
        executeStats[tid]->numFpRegReads++;
        break;
      case CCRegClass:
        executeStats[tid]->numCCRegReads++;
        break;
      case VecRegClass:
      case VecElemClass:
        executeStats[tid]->numVecRegReads++;
        break;
      case VecPredRegClass:
        executeStats[tid]->numVecPredRegReads++;
        break;
      default:
        break;
    }
    return regFile.getReg(phys_reg);
}

void
CPU::getReg(PhysRegIdPtr phys_reg, void *val, ThreadID tid)
{
    switch (phys_reg->classValue()) {
      case IntRegClass:
        executeStats[tid]->numIntRegReads++;
        break;
      case FloatRegClass:
        executeStats[tid]->numFpRegReads++;
        break;
      case CCRegClass:
        executeStats[tid]->numCCRegReads++;
        break;
      case VecRegClass:
      case VecElemClass:
        executeStats[tid]->numVecRegReads++;
        break;
      case VecPredRegClass:
        executeStats[tid]->numVecPredRegReads++;
        break;
      default:
        break;
    }
    regFile.getReg(phys_reg, val);
}

void *
CPU::getWritableReg(PhysRegIdPtr phys_reg, ThreadID tid)
{
    switch (phys_reg->classValue()) {
      case VecRegClass:
        executeStats[tid]->numVecRegReads++;
        break;
      case VecPredRegClass:
        executeStats[tid]->numVecPredRegReads++;
        break;
      default:
        break;
    }
    return regFile.getWritableReg(phys_reg);
}

void
CPU::setReg(PhysRegIdPtr phys_reg, RegVal val, ThreadID tid)
{
    switch (phys_reg->classValue()) {
      case IntRegClass:
        executeStats[tid]->numIntRegWrites++;
        break;
      case FloatRegClass:
        executeStats[tid]->numFpRegWrites++;
        break;
      case CCRegClass:
        executeStats[tid]->numCCRegWrites++;
        break;
      case VecRegClass:
      case VecElemClass:
        executeStats[tid]->numVecRegWrites++;
        break;
      case VecPredRegClass:
        executeStats[tid]->numVecPredRegWrites++;
        break;
      default:
        break;
    }
    regFile.setReg(phys_reg, val);
}

void
CPU::setReg(PhysRegIdPtr phys_reg, const void *val, ThreadID tid)
{
    switch (phys_reg->classValue()) {
      case IntRegClass:
        executeStats[tid]->numIntRegWrites++;
        break;
      case FloatRegClass:
        executeStats[tid]->numFpRegWrites++;
        break;
      case CCRegClass:
        executeStats[tid]->numCCRegWrites++;
        break;
      case VecRegClass:
      case VecElemClass:
        executeStats[tid]->numVecRegWrites++;
        break;
      case VecPredRegClass:
        executeStats[tid]->numVecPredRegWrites++;
        break;
      default:
        break;
    }
    regFile.setReg(phys_reg, val);
}

RegVal
CPU::getArchReg(const RegId &reg, ThreadID tid)
{
    const RegId flat = reg.flatten(*isa[tid]);
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(flat);
    return regFile.getReg(phys_reg);
}

void
CPU::getArchReg(const RegId &reg, void *val, ThreadID tid)
{
    const RegId flat = reg.flatten(*isa[tid]);
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(flat);
    regFile.getReg(phys_reg, val);
}

void *
CPU::getWritableArchReg(const RegId &reg, ThreadID tid)
{
    const RegId flat = reg.flatten(*isa[tid]);
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(flat);
    return regFile.getWritableReg(phys_reg);
}

void
CPU::setArchReg(const RegId &reg, RegVal val, ThreadID tid)
{
    const RegId flat = reg.flatten(*isa[tid]);
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(flat);
    regFile.setReg(phys_reg, val);
}

void
CPU::setArchReg(const RegId &reg, const void *val, ThreadID tid)
{
    const RegId flat = reg.flatten(*isa[tid]);
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(flat);
    regFile.setReg(phys_reg, val);
}

const PCStateBase &
CPU::pcState(ThreadID tid)
{
    return commit.pcState(tid);
}

void
CPU::pcState(const PCStateBase &val, ThreadID tid)
{
    commit.pcState(val, tid);
}

void
CPU::squashFromTC(ThreadID tid)
{
    thread[tid]->noSquashFromTC = true;
    commit.generateTCEvent(tid);
}

CPU::ListIt
CPU::addInst(const DynInstPtr &inst)
{
    instList.push_back(inst);

    return --(instList.end());
}

void
CPU::instDone(ThreadID tid, const DynInstPtr &inst)
{
    // Keep an instruction count.
    if (!inst->isMicroop() || inst->isLastMicroop()) {
        thread[tid]->numInst++;
        thread[tid]->threadStats.numInsts++;
        commitStats[tid]->numInstsNotNOP++;

        // Check for instruction-count-based events.
        thread[tid]->comInstEventQueue.serviceEvents(thread[tid]->numInst);
    }
    thread[tid]->numOp++;
    thread[tid]->threadStats.numOps++;
    commitStats[tid]->numOpsNotNOP++;

    probeInstCommit(inst->staticInst, inst->pcState().instAddr());
}

void
CPU::removeFrontInst(const DynInstPtr &inst)
{
    DPRINTF(O3CPU, "Removing committed instruction [tid:%i] PC %s "
            "[sn:%lli]\n",
            inst->threadNumber, inst->pcState(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}

void
CPU::removeInstsNotInROB(ThreadID tid)
{
    DPRINTF(O3CPU, "Thread %i: Deleting instructions from instruction"
            " list.\n", tid);

    ListIt end_it;

    bool rob_empty = false;

    if (instList.empty()) {
        return;
    } else if (rob.isEmpty(tid)) {
        DPRINTF(O3CPU, "ROB is empty, squashing all insts.\n");
        end_it = instList.begin();
        rob_empty = true;
    } else {
        end_it = (rob.readTailInst(tid))->getInstListIt();
        DPRINTF(O3CPU, "ROB is not empty, squashing insts not in ROB.\n");
    }

    removeInstsThisCycle = true;

    ListIt inst_it = instList.end();

    inst_it--;

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction iterator, end_it.
    while (inst_it != end_it) {
        assert(!instList.empty());

        squashInstIt(inst_it, tid);

        inst_it--;
    }

    // If the ROB was empty, then we actually need to remove the first
    // instruction as well.
    if (rob_empty) {
        squashInstIt(inst_it, tid);
    }
}

void
CPU::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
{
    assert(!instList.empty());

    removeInstsThisCycle = true;

    ListIt inst_iter = instList.end();

    inst_iter--;

    DPRINTF(O3CPU, "Deleting instructions from instruction "
            "list that are from [tid:%i] and above [sn:%lli] (end=%lli).\n",
            tid, seq_num, (*inst_iter)->seqNum);

    while ((*inst_iter)->seqNum > seq_num) {

        bool break_loop = (inst_iter == instList.begin());

        squashInstIt(inst_iter, tid);

        inst_iter--;

        if (break_loop)
            break;
    }
}

void
CPU::squashInstIt(const ListIt &instIt, ThreadID tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(O3CPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->pcState());

        // Mark it as squashed.
        (*instIt)->setSquashed();

        // @todo: Formulate a consistent method for deleting
        // instructions from the instruction list
        // Remove the instruction from the list.
        removeList.push(instIt);
    }
}

void
CPU::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(O3CPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->pcState());

        instList.erase(removeList.front());

        removeList.pop();
    }

    removeInstsThisCycle = false;
}
/*
void
CPU::removeAllInsts()
{
    instList.clear();
}
*/
void
CPU::dumpInsts()
{
    int num = 0;

    ListIt inst_list_it = instList.begin();

    cprintf("Dumping Instruction List\n");

    while (inst_list_it != instList.end()) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->pcState().instAddr(),
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->seqNum, (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}
/*
void
CPU::wakeDependents(const DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}
*/
void
CPU::wakeCPU()
{
    if (activityRec.active() || tickEvent.scheduled()) {
        DPRINTF(Activity, "CPU already running.\n");
        return;
    }

    DPRINTF(Activity, "Waking up CPU\n");

    Cycles cycles(curCycle() - lastRunningCycle);
    // @todo: This is an oddity that is only here to match the stats
    if (cycles > 1) {
        --cycles;
        cpuStats.idleCycles += cycles;
        baseStats.numCycles += cycles;
    }

    schedule(tickEvent, clockEdge());
}

void
CPU::wakeup(ThreadID tid)
{
    if (thread[tid]->status() != gem5::ThreadContext::Suspended)
        return;

    wakeCPU();

    DPRINTF(Quiesce, "Suspended Processor woken\n");
    threadContexts[tid]->activate();
}

ThreadID
CPU::getFreeTid()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!tids[tid]) {
            tids[tid] = true;
            return tid;
        }
    }

    return InvalidThreadID;
}

void
CPU::updateThreadPriority()
{
    if (activeThreads.size() > 1) {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        std::list<ThreadID>::iterator list_begin = activeThreads.begin();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}

void
CPU::addThreadToExitingList(ThreadID tid)
{
    DPRINTF(O3CPU, "Thread %d is inserted to exitingThreads list\n", tid);

    // the thread trying to exit can't be already halted
    assert(tcBase(tid)->status() != gem5::ThreadContext::Halted);

    // make sure the thread has not been added to the list yet
    assert(exitingThreads.count(tid) == 0);

    // add the thread to exitingThreads list to mark that this thread is
    // trying to exit. The boolean value in the pair denotes if a thread is
    // ready to exit. The thread is not ready to exit until the corresponding
    // exit trap event is processed in the future. Until then, it'll be still
    // an active thread that is trying to exit.
    exitingThreads.emplace(std::make_pair(tid, false));
}

bool
CPU::isThreadExiting(ThreadID tid) const
{
    return exitingThreads.count(tid) == 1;
}

void
CPU::scheduleThreadExitEvent(ThreadID tid)
{
    assert(exitingThreads.count(tid) == 1);

    // exit trap event has been processed. Now, the thread is ready to exit
    // and be removed from the CPU.
    exitingThreads[tid] = true;

    // we schedule a threadExitEvent in the next cycle to properly clean
    // up the thread's states in the pipeline. threadExitEvent has lower
    // priority than tickEvent, so the cleanup will happen at the very end
    // of the next cycle after all pipeline stages complete their operations.
    // We want all stages to complete squashing instructions before doing
    // the cleanup.
    if (!threadExitEvent.scheduled()) {
        schedule(threadExitEvent, nextCycle());
    }
}

void
CPU::exitThreads()
{
    // there must be at least one thread trying to exit
    assert(exitingThreads.size() > 0);

    // terminate all threads that are ready to exit
    auto it = exitingThreads.begin();
    while (it != exitingThreads.end()) {
        ThreadID thread_id = it->first;
        bool readyToExit = it->second;

        if (readyToExit) {
            DPRINTF(O3CPU, "Exiting thread %d\n", thread_id);
            haltContext(thread_id);
            tcBase(thread_id)->setStatus(gem5::ThreadContext::Halted);
            it = exitingThreads.erase(it);
        } else {
            it++;
        }
    }
}

void
CPU::htmSendAbortSignal(ThreadID tid, uint64_t htm_uid,
        HtmFailureFaultCause cause)
{
    const Addr addr = 0x0ul;
    const int size = 8;
    const Request::Flags flags =
      Request::PHYSICAL|Request::STRICT_ORDER|Request::HTM_ABORT;

    // O3-specific actions
    iew.ldstQueue.resetHtmStartsStops(tid);
    commit.resetHtmStartsStops(tid);

    // notify l1 d-cache (ruby) that core has aborted transaction
    RequestPtr req =
        std::make_shared<Request>(addr, size, flags, _dataRequestorId);

    req->taskId(taskId());
    req->setContext(thread[tid]->contextId());
    req->setHtmAbortCause(cause);

    assert(req->isHTMAbort());

    PacketPtr abort_pkt = Packet::createRead(req);
    uint8_t *memData = new uint8_t[8];
    assert(memData);
    abort_pkt->dataStatic(memData);
    abort_pkt->setHtmTransactional(htm_uid);

    // TODO include correct error handling here
    if (!iew.ldstQueue.getDataPort().sendTimingReq(abort_pkt)) {
        panic("HTM abort signal was not sent to the memory subsystem.");
    }
}



// ==============================================================
//
//                        JV PRECOMPUTED BTB
//
// ==============================================================
//
/**
 * JV ADDITION: TODO find somewhere more appropriate to stick this
 * Keeps track of branch sources, targets, conditions
 * Set by bmovs, bmovt, bmovc ops
 *
 * FOR NOW: set synchronously at execute time (commit time?), no handling for
 * speculation/squashing
 */

const char* PrecomputedBTB::BranchTypeCodes[] = {"N/A", "TKN", "LOP",
                                                 "BIT", "B_C"};
const char* PrecomputedBTB::BranchTypeStrs[] = { "NoBranch", "Taken", "Loop",
                                                 "ShiftBit", "ShiftBit_Clear"};

void PrecomputedBTB::debugDump(int regstart, int regstop) {
    //TODO: make these DPRINTF? (need to define my own debug flag somewhere)
    //printf("PBTB (%d CPs, fst=%d, lst=%d): {\n",
    //        numActiveCheckpoints(), first_checkp, last_checkp);

    regstart = std::max(0, regstart);
    regstop = std::min(NUM_REGS, regstop);
    int numtoprint = regstop - regstart;

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstart > 0) { printf("  ...\n"); }

    struct pbtb_map *mapsToPrint[] = {&map_fetch, &map_final};
    struct pbtb_map *curr_map = mapsToPrint[0];
    for (int reg = regstart; reg < regstop; reg++) {
        auto branchCode = BranchTypeCodes[curr_map->cond_type[reg]];

        printf("  %2i: SRC=%8lx TGT=%8lx | %s: %ld %ld\n", reg,
                (uint64_t)curr_map->source[reg],
                (uint64_t)curr_map->target[reg],
                branchCode,
                curr_map->cond_val[reg],
                curr_map->cond_aux_val[reg]);
    }

    // If skipping some regs at the start, show that
    if (numtoprint > 0 && regstop < NUM_REGS) { printf("  ...\n"); }

    // I guess also handle the edge case where we omit them all
    if (numtoprint == 0) { printf("  ...\n"); }

    printf("}\n");
}

void PrecomputedBTB::debugDump() { debugDump(0, NUM_REGS); }


// =============== Per-Map functions (PRIVATE)

void PrecomputedBTB::m_setSource(struct pbtb_map *pmap,
                                 int breg, Addr source_addr) {
    assert(breg > 0 && breg < NUM_REGS); //breg should be 0-31
    pmap->source[breg] = source_addr;
    pmap->version[breg]++; //TODO: set this to seqNum instead

    // setSource also resets it to an invalid branch
    pmap->cond_type[breg]    = NoBranch;
    pmap->cond_val[breg]     = 0;
    pmap->cond_aux_val[breg] = 0;
}
void PrecomputedBTB::m_setTarget(struct pbtb_map *pmap,
                                 int breg, Addr target_addr) {
    assert(breg > 0 && breg < NUM_REGS); //breg should be 0-31
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

    assert(breg > 0 && breg < NUM_REGS); //breg should be 0-31
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
            DPRINTF(Exec, "PBTB (X): bmovc_bit b%d: data{%s}(%d)\n",
                    breg, debugPrintBottomBits(bits,numbits), numbits);

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
        *p_breg_out = -1;
        return PR_NoMatch;
    }

    // ==== Check based on branch type whether or not it's taken
    // NOTE: breg might be -1, only guaranteed for breg to be valid
    //       if type != NoBranch
    if (brType == Taken) {
        resType = PR_Taken;

        DPRINTF(Decode, "PBTB (%s): HIT b%d: TKN 0x%x -> 0x%x\n",
                debugWhichMap, breg, pcAddr, pmap->target[breg]);

    } else if (brType == LoopN) {
        DPRINTF(Decode, "PBTB (%s): HIT b%d: LOOP{%d} (%s) 0x%x -?> 0x%x\n",
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
            DPRINTF(Decode, "PBTB (F): Loop at -1: Exhausted\n");
            //printf("PBTB: Loop at -1: ERROR\n");
        }

    } else if (brType == ShiftBit) {
        uint64_t bits = pmap->cond_val[breg];
        uint64_t numbits = pmap->cond_aux_val[breg];
        DPRINTF(Decode, "PBTB (%s): HIT b%d: BIT{%s} (%s) 0x%x -?> 0x%x\n",
                debugWhichMap, breg, debugPrintBottomBits(bits,numbits),
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
            DPRINTF(Decode, "PBTB (%s): ShiftBit out of bits: Exhausted\n",
                            debugWhichMap);
        }
    } else {
        assert(false); // Unrecognized pbtb entry type, crash out
        return PR_NoMatch;
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
