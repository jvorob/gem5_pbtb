/*
 * Copyright (c) 2011-2013, 2016-2020 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_CPU_HH__
#define __CPU_O3_CPU_HH__

#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/statistics.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/commit.hh"
#include "cpu/o3/decode.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/free_list.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/limits.hh"
#include "cpu/o3/rename.hh"
#include "cpu/o3/rob.hh"
#include "cpu/o3/scoreboard.hh"
#include "cpu/o3/thread_state.hh"
#include "cpu/activity.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/timebuf.hh"
#include "debug/Decode.hh" //JV TODO DEBUG TEMP
#include "debug/Exec.hh" //JV TODO DEBUG TEMP
#include "debug/Fetch.hh" //JV TODO DEBUG TEMP
#include "params/BaseO3CPU.hh"
#include "sim/process.hh"

namespace gem5
{

template <class>
class Checker;
class ThreadContext;

class Checkpoint;
class Process;

namespace o3
{

/**
 * JV ADDITION: TODO find somewhere more appropriate to stick this
 * Keeps track of branch sources, targets, conditions
 * Set by bmovs, bmovt, bmovc ops
 *
 * FOR NOW: set synchronously at execute time (commit time?), no
 * handling for speculation/squashing
 */
class PrecomputedBTB
{
  public:
    enum BranchType
    {
        NoBranch = 0, //Init value? Mostly for debugging
        Taken,
        LoopN, //Sets a counter, loops N times, then stops
        ShiftBit, //Shifts a bit into the fifo, branching
                  //consumes 1 bit each time

        // PSEUDO-BRANCH-TYPE: can be passed in to specify different
        // behavior but shouldn't be put into reg_cond_type
        ShiftBit_Clear, // ShiftBit, but also resets the fifo??

        //TODO: IF ADDING NEW BRANCH TYPE: UPDATE
        //      BRANCHTYPECODES AND BRANCHTYPETOSTR
    };

    enum PBTBResultType
    {
        PR_NoMatch = 0,
        PR_NotTaken,
        PR_Taken,
        PR_Exhaust,
    };

    //3-character codes for each branch type
    const static char* BranchTypeCodes[];
    const static char* BranchTypeStrs[];

    const static int NUM_REGS = 32;
    //const static int MAX_CHECKPOINTS = 32;

    //Each checkpoint stores all this data
    struct pbtb_map
    {
        //InstSeqNum seqNum; //The branch inst that started this checkpoint
        int64_t    version[NUM_REGS]; // ++ each time this breg is modified.
                                      // (except for bmovc_bit, if appending)
        Addr       source[NUM_REGS]; // All addresses are absolute
        Addr       target[NUM_REGS];
        BranchType cond_type[NUM_REGS];  //defaults to NoBranch
        int64_t   cond_val[NUM_REGS];
        int64_t   cond_aux_val[NUM_REGS]; //for shiftreg, counts number
                                                    //of bits held
    };

    struct pbtb_map map_fetch = {};
    struct pbtb_map map_final = {}; // init to all 0s


    //struct pbtp_map map;


    //TODO remove this? I'm using shortcodes??
    const static char* BranchTypeToStr(BranchType b, bool verbose){
      switch(b) {
        case NoBranch:        return "NoBranch";
        case Taken:           return "Taken";
        case LoopN:           return "Loop";
        case ShiftBit:        return "ShiftBit";
        case ShiftBit_Clear : return "ShiftBit_Clear";
        default:              return "ERROR_UNKNOWN";
      }
    }

  private:
    ////Constructor: initializes all to 0
    //Addr       reg_source[NUM_REGS] = {}; // All addresses are absolute
    //Addr       reg_target[NUM_REGS] = {};
    //BranchType reg_cond_type[NUM_REGS] = {};  //defaults to NoBranch
    //uint64_t   reg_cond_val[NUM_REGS] = {};
    //uint64_t   reg_cond_aux_val[NUM_REGS] = {}; //for shiftreg, counts number
                                                //of bits held


    static inline char debug_bit_buff[65];
    const static char* debugPrintBottomBits(uint64_t bits, int n) {
        debug_bit_buff[64] = '\0'; //null terminate just to be safe
        n = std::min(n, 64);

        int i;
        for (i = 0; i < n; i++) {
            int bit_offset = n - i - 1; //1st digit is at (bits >> (n-1))
            int bit = (bits >> bit_offset) & 1;
            debug_bit_buff[i] = bit ? '1':'0';
        }
        debug_bit_buff[i] = '\0';
        return debug_bit_buff;
    }


  private:
    // ==================== PER-MAP Functions:
    // If the system wasn't pipelined, these would be sufficient
    // Each of these acts on a single pbtb_map in the straightforward way


    // Note: map_fetch and map_finalize should ONLY EVER DIFFER in number
    // of loop iterations / shifted bits. All other modifications should apply
    // simultaneously to both. map_commit (once it's in) might differ though

    void m_setSource(struct pbtb_map *pmap, int breg, Addr source_addr);
    void m_setTarget(struct pbtb_map *pmap, int breg, Addr target_addr);

    // Sets the condition for a given breg
    // val treated differently for different types?
    // - Taken or NotTaken ignore it
    // - LoopTaken will be taken (val) times, then NT, than stall
    // - ShiftBit takes a bitstring in val, bottom n bits shifted in
    void m_setCondition(struct pbtb_map *pmap, int breg,
            BranchType conditionType, uint64_t val, int64_t n);

    // Queries a pmap (consuming bits in the process)
    // returns a resultType describing whether a match was found and what kind
    // If a match is found, sets p_breg_out, p_version_out
    // If the match is a taken branch, sets p_targetAddr_out
    PBTBResultType m_queryPC(struct pbtb_map *pmap, Addr pcAddr,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out);

  public:
    // ================== External-interface functions
    // These should modify both map_fetch and map_final
    // TODO: eventually these might need to modify commit?
    // NOTE: we SHOULDNT need a seq num for bmovc anymore, but we might need
    // it if we do checkpointing again and I'm too lazy to take it out rn
    void setSource(int breg, Addr source_addr);
    void setTarget(int breg, Addr target_addr);
    void setCondition(int breg,
                      BranchType conditionType, uint64_t val);
    void setCondition(int breg,
                      BranchType conditionType, uint64_t val, int64_t n);


    // Overwrite map_fetch with map_finalize
    void squashFinalizeToFetch();


    // handles the PCstatebase nonsense, otherwise passthru to m_query_PC
    //Note: if not taken, will advance pc
    PBTBResultType queryFromFetch(
            const StaticInstPtr inst, PCStateBase &pc,
            int *p_breg_out, uint64_t *p_version_out);

    //Note: if not taken, will instead
    //advance pcAddr and return in targetAddr_out
    PBTBResultType queryFromDecode(
            const StaticInstPtr inst, Addr pcAddr,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out);

    // === Squash behavior:
    // resetToFinalizedMap() // for when we squash from decode
    // resetToCommittedMap? // for when we are able to handle exceptions
    //print out
    void debugDump();
    // Limits which regs to print (to limit output). Inclusive/exclusive
    void debugDump(int regstart, int regstop);



}; // class PrecomputedBTB

class ThreadContext;

/**
 * O3CPU class, has each of the stages (fetch through commit)
 * within it, as well as all of the time buffers between stages.  The
 * tick() function for the CPU is defined here.
 */
class CPU : public BaseCPU
{
  public:
    typedef std::list<DynInstPtr>::iterator ListIt;

    friend class ThreadContext;

  public:
    enum Status
    {
        Running,
        Idle,
        Halted,
        Blocked,
        SwitchedOut
    };

    BaseMMU *mmu;
    using LSQRequest = LSQ::LSQRequest;

    /** Overall CPU status. */
    Status _status;


  private:

    /** The tick event used for scheduling CPU ticks. */
    EventFunctionWrapper tickEvent;

    /** The exit event used for terminating all ready-to-exit threads */
    EventFunctionWrapper threadExitEvent;

    /** Schedule tick event, regardless of its current state. */
    void
    scheduleTickEvent(Cycles delay)
    {
        if (tickEvent.squashed())
            reschedule(tickEvent, clockEdge(delay));
        else if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(delay));
    }

    /** Unschedule tick event, regardless of its current state. */
    void
    unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

    /**
     * Check if the pipeline has drained and signal drain done.
     *
     * This method checks if a drain has been requested and if the CPU
     * has drained successfully (i.e., there are no instructions in
     * the pipeline). If the CPU has drained, it deschedules the tick
     * event and signals the drain manager.
     *
     * @return False if a drain hasn't been requested or the CPU
     * hasn't drained, true otherwise.
     */
    bool tryDrain();

    /**
     * Perform sanity checks after a drain.
     *
     * This method is called from drain() when it has determined that
     * the CPU is fully drained when gem5 is compiled with the NDEBUG
     * macro undefined. The intention of this method is to do more
     * extensive tests than the isDrained() method to weed out any
     * draining bugs.
     */
    void drainSanityCheck() const;

    /** Check if a system is in a drained state. */
    bool isCpuDrained() const;

  public:
    /** Constructs a CPU with the given parameters. */
    CPU(const BaseO3CPUParams &params);

    ProbePointArg<PacketPtr> *ppInstAccessComplete;
    ProbePointArg<std::pair<DynInstPtr, PacketPtr> > *ppDataAccessComplete;

    /** Register probe points. */
    void regProbePoints() override;

    void
    demapPage(Addr vaddr, uint64_t asn)
    {
        mmu->demapPage(vaddr, asn);
    }

    /** Ticks CPU, calling tick() on each stage, and checking the overall
     *  activity to see if the CPU should deschedule itself.
     */
    void tick();

    /** Initialize the CPU */
    void init() override;

    void startup() override;

    /** Returns the Number of Active Threads in the CPU */
    int
    numActiveThreads()
    {
        return activeThreads.size();
    }

    /** Add Thread to Active Threads List */
    void activateThread(ThreadID tid);

    /** Remove Thread from Active Threads List */
    void deactivateThread(ThreadID tid);

    /** Setup CPU to insert a thread's context */
    void insertThread(ThreadID tid);

    /** Remove all of a thread's context from CPU */
    void removeThread(ThreadID tid);

    /** Count the Total Instructions Committed in the CPU. */
    Counter totalInsts() const override;

    /** Count the Total Ops (including micro ops) committed in the CPU. */
    Counter totalOps() const override;

    /** Add Thread to Active Threads List. */
    void activateContext(ThreadID tid) override;

    /** Remove Thread from Active Threads List */
    void suspendContext(ThreadID tid) override;

    /** Remove Thread from Active Threads List &&
     *  Remove Thread Context from CPU.
     */
    void haltContext(ThreadID tid) override;

    /** Update The Order In Which We Process Threads. */
    void updateThreadPriority();

    /** Is the CPU draining? */
    bool isDraining() const { return drainState() == DrainState::Draining; }

    void serializeThread(CheckpointOut &cp, ThreadID tid) const override;
    void unserializeThread(CheckpointIn &cp, ThreadID tid) override;

    /** Insert tid to the list of threads trying to exit */
    void addThreadToExitingList(ThreadID tid);

    /** Is the thread trying to exit? */
    bool isThreadExiting(ThreadID tid) const;

    /**
     *  If a thread is trying to exit and its corresponding trap event
     *  has been completed, schedule an event to terminate the thread.
     */
    void scheduleThreadExitEvent(ThreadID tid);

    /** Terminate all threads that are ready to exit */
    void exitThreads();

  public:
    /** Starts draining the CPU's pipeline of all instructions in
     * order to stop all memory accesses. */
    DrainState drain() override;

    /** Resumes execution after a drain. */
    void drainResume() override;

    /**
     * Commit has reached a safe point to drain a thread.
     *
     * Commit calls this method to inform the pipeline that it has
     * reached a point where it is not executed microcode and is about
     * to squash uncommitted instructions to fully drain the pipeline.
     */
    void commitDrained(ThreadID tid);

    /** Switches out this CPU. */
    void switchOut() override;

    /** Takes over from another CPU. */
    void takeOverFrom(BaseCPU *oldCPU) override;

    void verifyMemoryMode() const override;

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq() { return globalSeqNum++; }

    /** Traps to handle given fault. */
    void trap(const Fault &fault, ThreadID tid, const StaticInstPtr &inst);

    /** Returns the Fault for any valid interrupt. */
    Fault getInterrupts();

    /** Processes any an interrupt fault. */
    void processInterrupts(const Fault &interrupt);

    /** Halts the CPU. */
    void halt() { panic("Halt not implemented!\n"); }

    /** Register accessors.  Index refers to the physical register index. */

    /** Reads a miscellaneous register. */
    RegVal readMiscRegNoEffect(int misc_reg, ThreadID tid) const;

    /** Reads a misc. register, including any side effects the read
     * might have as defined by the architecture.
     */
    RegVal readMiscReg(int misc_reg, ThreadID tid);

    /** Sets a miscellaneous register. */
    void setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid);

    /** Sets a misc. register, including any side effects the write
     * might have as defined by the architecture.
     */
    void setMiscReg(int misc_reg, RegVal val, ThreadID tid);

    RegVal getReg(PhysRegIdPtr phys_reg, ThreadID tid);
    void getReg(PhysRegIdPtr phys_reg, void *val, ThreadID tid);
    void *getWritableReg(PhysRegIdPtr phys_reg, ThreadID tid);

    void setReg(PhysRegIdPtr phys_reg, RegVal val, ThreadID tid);
    void setReg(PhysRegIdPtr phys_reg, const void *val, ThreadID tid);

    /** Architectural register accessors.  Looks up in the commit
     * rename table to obtain the true physical index of the
     * architected register first, then accesses that physical
     * register.
     */

    RegVal getArchReg(const RegId &reg, ThreadID tid);
    void getArchReg(const RegId &reg, void *val, ThreadID tid);
    void *getWritableArchReg(const RegId &reg, ThreadID tid);

    void setArchReg(const RegId &reg, RegVal val, ThreadID tid);
    void setArchReg(const RegId &reg, const void *val, ThreadID tid);

    /** Sets the commit PC state of a specific thread. */
    void pcState(const PCStateBase &new_pc_state, ThreadID tid);

    /** Reads the commit PC state of a specific thread. */
    const PCStateBase &pcState(ThreadID tid);

    /** Initiates a squash of all in-flight instructions for a given
     * thread.  The source of the squash is an external update of
     * state through the TC.
     */
    void squashFromTC(ThreadID tid);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    ListIt addInst(const DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone(ThreadID tid, const DynInstPtr &inst);

    /** Remove an instruction from the front end of the list.  There's
     *  no restriction on location of the instruction.
     */
    void removeFrontInst(const DynInstPtr &inst);

    /** Remove all instructions that are not currently in the ROB.
     *  There's also an option to not squash delay slot instructions.*/
    void removeInstsNotInROB(ThreadID tid);

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid);

    /** Removes the instruction pointed to by the iterator. */
    void squashInstIt(const ListIt &instIt, ThreadID tid);

    /** Cleans up all instructions on the remove list. */
    void cleanUpRemovedInsts();

    /** Debug function to print all instructions on the list. */
    void dumpInsts();

  public:
#ifndef NDEBUG
    /** Count of total number of dynamic instructions in flight. */
    int instcount;
#endif

    /** List of all the instructions in flight. */
    std::list<DynInstPtr> instList;

    /** List of all the instructions that will be removed at the end of this
     *  cycle.
     */
    std::queue<ListIt> removeList;

#ifdef GEM5_DEBUG
    /** Debug structure to keep track of the sequence numbers still in
     * flight.
     */
    std::set<InstSeqNum> snList;
#endif

    /** Records if instructions need to be removed this cycle due to
     *  being retired or squashed.
     */
    bool removeInstsThisCycle;

  protected:
    /** The fetch stage. */
    Fetch fetch;

    /** The decode stage. */
    Decode decode;

    /** The dispatch stage. */
    Rename rename;

    /** The issue/execute/writeback stages. */
    IEW iew;

    /** The commit stage. */
    Commit commit;

    /** The register file. */
    PhysRegFile regFile;

    /** The free list. */
    UnifiedFreeList freeList;

    /** The rename map. */
    UnifiedRenameMap renameMap[MaxThreads];

    /** The commit rename map. */
    UnifiedRenameMap commitRenameMap[MaxThreads];

    /** The re-order buffer. */
    ROB rob;

    /** Active Threads List */
    std::list<ThreadID> activeThreads;

    /**
     *  This is a list of threads that are trying to exit. Each thread id
     *  is mapped to a boolean value denoting whether the thread is ready
     *  to exit.
     */
    std::unordered_map<ThreadID, bool> exitingThreads;

    /** Integer Register Scoreboard */
    Scoreboard scoreboard;

    std::vector<BaseISA *> isa;

  public:
    /** Enum to give each stage a specific index, so when calling
     *  activateStage() or deactivateStage(), they can specify which stage
     *  is being activated/deactivated.
     */
    enum StageIdx
    {
        FetchIdx,
        DecodeIdx,
        RenameIdx,
        IEWIdx,
        CommitIdx,
        NumStages
    };

    /** The main time buffer to do backwards communication. */
    TimeBuffer<TimeStruct> timeBuffer;

    /** The fetch stage's instruction queue. */
    TimeBuffer<FetchStruct> fetchQueue;

    /** The decode stage's instruction queue. */
    TimeBuffer<DecodeStruct> decodeQueue;

    /** The rename stage's instruction queue. */
    TimeBuffer<RenameStruct> renameQueue;

    /** The IEW stage's instruction queue. */
    TimeBuffer<IEWStruct> iewQueue;

  private:
    /** The activity recorder; used to tell if the CPU has any
     * activity remaining or if it can go to idle and deschedule
     * itself.
     */
    ActivityRecorder activityRec;

  public:
    /** Records that there was time buffer activity this cycle. */
    void activityThisCycle() { activityRec.activity(); }

    /** Changes a stage's status to active within the activity recorder. */
    void
    activateStage(const StageIdx idx)
    {
        activityRec.activateStage(idx);
    }

    /** Changes a stage's status to inactive within the activity recorder. */
    void
    deactivateStage(const StageIdx idx)
    {
        activityRec.deactivateStage(idx);
    }

    /** Wakes the CPU, rescheduling the CPU if it's not already active. */
    void wakeCPU();

    virtual void wakeup(ThreadID tid) override;

    /** Gets a free thread id. Use if thread ids change across system. */
    ThreadID getFreeTid();

  public:
    /** Returns a pointer to a thread context. */
    gem5::ThreadContext *
    tcBase(ThreadID tid)
    {
        return thread[tid]->getTC();
    }

    /** The global sequence number counter. */
    InstSeqNum globalSeqNum;//[MaxThreads];

    /** Pointer to the checker, which can dynamically verify
     * instruction results at run time.  This can be set to NULL if it
     * is not being used.
     */
    gem5::Checker<DynInstPtr> *checker;

    /** Pointer to the system. */
    System *system;

    /** Pointers to all of the threads in the CPU. */
    std::vector<ThreadState *> thread;

    /** Threads Scheduled to Enter CPU */
    std::list<int> cpuWaitList;

    /** The cycle that the CPU was last running, used for statistics. */
    Cycles lastRunningCycle;

    /** The cycle that the CPU was last activated by a new thread*/
    Tick lastActivatedCycle;

    /** Mapping for system thread id to cpu id */
    std::map<ThreadID, unsigned> threadMap;

    /** Available thread ids in the cpu*/
    std::vector<ThreadID> tids;

    /** CPU pushRequest function, forwards request to LSQ. */
    Fault
    pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
                unsigned int size, Addr addr, Request::Flags flags,
                uint64_t *res, AtomicOpFunctorPtr amo_op = nullptr,
                const std::vector<bool>& byte_enable=std::vector<bool>())

    {
        return iew.ldstQueue.pushRequest(inst, isLoad, data, size, addr,
                flags, res, std::move(amo_op), byte_enable);
    }

    /** Used by the fetch unit to get a hold of the instruction port. */
    Port &
    getInstPort() override
    {
        return fetch.getInstPort();
    }

    /** Get the dcache port (used to find block size for translations). */
    Port &
    getDataPort() override
    {
        return iew.ldstQueue.getDataPort();
    }

    struct CPUStats : public statistics::Group
    {
        CPUStats(CPU *cpu);

        /** Stat for total number of times the CPU is descheduled. */
        statistics::Scalar timesIdled;
        /** Stat for total number of cycles the CPU spends descheduled. */
        statistics::Scalar idleCycles;
        /** Stat for total number of cycles the CPU spends descheduled due to a
         * quiesce operation or waiting for an interrupt. */
        statistics::Scalar quiesceCycles;
    } cpuStats;

  public:
    // hardware transactional memory
    void htmSendAbortSignal(ThreadID tid, uint64_t htm_uid,
                            HtmFailureFaultCause cause) override;

  public:
    PrecomputedBTB PBTB;
};



} // namespace o3
} // namespace gem5

#endif // __CPU_O3_CPU_HH__
