
/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 *
 * CITE IT IF YOU COPY IT.
 */

#ifndef __CPU_O3_PBTB_HH__
#define __CPU_O3_PBTB_HH__

#include <cassert>

#include "arch/generic/pcstate.hh"
#include "base/cprintf.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/bitvec64.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/pbtb_map.hh"
#include "cpu/static_inst.hh"

// We should be importing NUM_PBTB_REGS from pbtb_map.hh

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
class PrecomputedBTB;

/**
 * One component of the PBTB? This keeps track of inflight bmovs, bmov
 *  completions, squashes etc.
 *
 * This primarily needs to interact with decode?
 * as that's when pb stalls happen?
 */
class BmovTracker
{
  public:
    BmovTracker(const PrecomputedBTB *pbtb): pbtb(pbtb) {}

  private:
    const PrecomputedBTB *pbtb;

    InstSeqNum lastCommittedInst = 0;

    // per-breg
    InstSeqNum lastExecBmov      [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecBmov       [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecNonBitBmov [NUM_PBTB_REGS] = {};

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
    // TODO: for now I'll just define them all in PBTBMap and import to here,
    // but really both classes are using these types, so maybe they should
    //      just be loose in a PBTB namespace?
    using BranchType = PBTBMap::BranchType;
    using PBTBResultType = PBTBMap::PBTBResultType;

    using undo_action = PBTBMap::undo_action;
    using utype =       PBTBMap::utype;

  public:
    PrecomputedBTB(CPU *_cpu): cpu(_cpu), tracker(this) {}


    /** Returns the name of PBTB (for DPRINTF?) */
    std::string name() const;

    // Maybe not the best place for it, but it's nice to have
    const static int NUM_REGS = NUM_PBTB_REGS;

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
    // (i.e. 1 for decode, 1 for finalize, etc)
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


    // ================================================================
    //
    //                         UNDO STATES
    //
    // ================================================================
    // We need to be able to squash instructions that modify the map_final pmap
    // Most bmovs can be undone by simply reverting to the previous pbtb state
    // (for bmovs that can't be interleaved, e.g. that update the version)
    //   However we must also be able to undo bit-pushes and bit-consumes
    // individually, as they can be interleaved (bit-type bmovs or pbs).
    //   To support this, an undo_entry for a breg holds an undo_state, which
    // can be either a full breg's data (for overwriting), or a partial
    // undo-state (specifying which bits were consumed, or how many bits were
    // pushed)

    // Stores the state of a single breg: not used in pbtb_map, but used
    // for stashing/passing around full value of a single breg in undo history
/*
pmap {
  PmapAction // action applied to a single struct pbtb_map
  PBTBAction // action applied to the pbtb (fetch vs decode, )

  UndoToken applyAction(PBTBAction) {
  }
}
*/

  public:

  private:
    // ================ UNDO ACTIONS

    struct undo_entry
    {
        InstSeqNum seqnum;
        undo_action action;
    };

    static std::string undoEntryToString(const struct undo_entry &ent) {
        std::string prev_str; // descriptor of what will be undone
        switch(ent.action.type) {
          case utype::U_FULL:
              prev_str = csprintf("prev state: %s",
                  PBTBMap::bdataToString(ent.action.as_overwrite));
              break;
          case utype::U_UNPOP_BITS:
              prev_str = csprintf("pb had consumed: fst<%s>lst",
                  ent.action.as_consumed_bits.toString());
              break;
          case utype::U_UNPUSH_BITS:
              prev_str = csprintf("bmov had pushed %d bits",
                  ent.action.as_pushed_bits);
              break;
          case utype::U_NONE:
              prev_str = csprintf("noop");
              break;
        }

        std::string v_str; // describes version (either v->v, or just v+);
        v_str = csprintf("v%d->v%d",
            ent.action.undone_ver, ent.action.done_ver);

        return csprintf("UNDO entry [sn:%d]: b%d %s (%s)",
            ent.seqnum, ent.action.breg, v_str, prev_str);
    };

    // ==================== PBTB Maps:
    // If the system wasn't pipelined, one of these would be sufficient

    PBTBMap map_fetch{"F"};
    PBTBMap map_final{"D"}; // init to all 0s

    // Note: map_fetch and map_finalize should ONLY EVER DIFFER in number
    // of loop iterations / shifted bits. All other modifications should apply
    // simultaneously to both. map_commit (once it's in) might differ though

    std::vector<struct undo_entry> undo_stack;

    // ========= UNDO STUFF
    // Any methods that modify a pmap should return an undo_action, which
    // can be applied to a pmap to revert it to its exact state before
    // the modification (assuming no intervening edits)
    //
    // If multiple m_ modifications return non-empty undo actions,
    // those undo actions can be applied in reverse order to yield
    // the initial state.
    //
    // Undo actions can also be arbitratily reordered if they are for
    // different bregs (as long as order within a breg is maintained)
    //
    // Also: push_bits and consume_bits can always be reordered so that
    // they are undone in reverse sequence-number order
    // (can't reorder arbitrarily, might overflow the bitvec)

    // Given an undo action and the seqnum, records it onto the history
    // TODO: accept 0 or more undo_actions?
    void savePrevState(int breg, InstSeqNum seqnum, undo_action undo);
    // undoes back to and including squashingSeqNum
    void unwindSquash(InstSeqNum squashingSeqNum);

  public:
    // ================== External-interface functions
    // These should modify both map_fetch and map_final
    // NOTE: ALL OF THESE SHOULD SAVE STATE IF THEY MODIFY MAP_FINAL
    // TODO: eventually these might need to modify commit?
    void setSource(int breg, InstSeqNum seqnum, Addr source_addr);
    void setTarget(int breg, InstSeqNum seqnum, Addr target_addr);
    void setCondition(int breg, InstSeqNum seqnum,
                      BranchType conditionType, uint64_t val);
    void setCondition(int breg, InstSeqNum seqnum,
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
            const StaticInstPtr inst, Addr pcAddr, InstSeqNum seqnum,
            int *p_breg_out, uint64_t *p_version_out, Addr *p_targetAddr_out);


    // Checks if breg is ready to finalize a bit-type branch
    bool isBregBitTypeAndReady(int breg) const;


    // === Squash behavior:
    // resetToFinalizedMap() // for when we squash from decode
    // resetToCommittedMap? // for when we are able to handle exceptions

    //=========== Pretty printing
    void debugDump();
    // Limits which regs to print (to limit output). Inclusive/exclusive
    void debugDump(int regstart, int regstop);


}; // class PrecomputedBTB

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_PBTB_HH__
