
/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 * CITE IT IF YOU COPY IT.
 */

#ifndef __CPU_O3_PBTB_HH__
#define __CPU_O3_PBTB_HH__

#include <cassert>

#include "arch/generic/pcstate.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/static_inst.hh"

const int NUM_PBTB_REGS=32;

namespace gem5
{

namespace o3
{

  class CPU; // forward declare? Need for getting ptr to cpu


// ==============================================================
//
//                    JV Bmov tracker
//
// ==============================================================

/**
 * One component of the PBTB? This keeps track of inflight bmovs, bmov
 *  completions, squashes etc.
 *
 * This primarily needs to interact with decode?
 * as that's when pb stalls happen?
 */
class BmovTracker
{
  private:
    InstSeqNum lastCommittedInst = 0;

    // per-breg
    InstSeqNum lastExecBmov      [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecBmov       [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecNonbitBmov [NUM_PBTB_REGS] = {};

    // misc?
    InstSeqNum lastDecAny = 0;
    InstSeqNum lastDecPb = 0;
    InstSeqNum lastDecMutPb = 0;


  public:
    void reset(); //clear out all state


    // ============ tracking functions ( should be set in appropriate places
    void recordDecodeInst       (ThreadID tid, DynInstConstPtr inst);
    void recordExecBmovFromIew  (ThreadID tid, InstSeqNum bmovSeq, int breg);
    void recordCommit           (ThreadID tid, InstSeqNum commitSeq);

    // Note: this doesn't count squashes the Decode itself generates,
    // only squashed from ahead of it, e.g. from IEW or commit
    void recordSquashFromAhead (ThreadID tid, InstSeqNum squashSeq);

    // ============ Query Functions
    // if is pb or predicted-as-pb, need to stall for finalize
    bool instNeedsToStall(ThreadID tid, DynInstConstPtr inst) const;
};



// ==============================================================
//
//                        JV PRECOMPUTED BTB
//
// ==============================================================

/**
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

    const static int NUM_REGS = NUM_PBTB_REGS;
    //const static int MAX_CHECKPOINTS = 32;


  public:
    PrecomputedBTB(CPU *_cpu);

    /** Returns the name of PBTB (for DPRINTF?) */
    std::string name() const;

    // Maybe not the best place for it, but it's nice to have

  private:
    /** CPU Interface */
    CPU *cpu;

  public:
    BmovTracker tracker;
  private:
    // ================================================================
    //
    //                         PBTB MAPS
    //
    // ================================================================
    // Each of these is a single pbtb_map
    // TODO: maybe put these in their own class?

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

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_PBTB_HH__
