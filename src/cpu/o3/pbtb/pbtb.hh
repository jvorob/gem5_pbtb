
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
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/pbtb/bitvec64.hh"
#include "cpu/o3/pbtb/pbtb_map.hh"
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
class PBTB;

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
    BmovTracker(const PBTB *pbtb): pbtb(pbtb) {}

  private:
    const PBTB *pbtb;

    InstSeqNum lastCommittedInst = 0;

    // per-breg
    InstSeqNum lastExecBmov      [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecBmov       [NUM_PBTB_REGS] = {};
    InstSeqNum lastDecNonBitBmov [NUM_PBTB_REGS] = {};

    // misc?
    InstSeqNum lastDecAny = 0;
    InstSeqNum lastDecPb = 0;
    InstSeqNum lastDecMutPb = 0;


    // === New tracker: shadows the ROB: keeps track of all decoded insts:
    struct tracker_entry
    {
      // not sure if I can keep ptrs to the actual insts, but
      // I'll keep copies of the info
      InstSeqNum seqNum;
      bool isBmov;
      bool isBitBmov;
      bool isPb; // TODO: unneeded?
      bool isExecuted;
      bool isSquashed;
      int breg;
    };

    // numbering will be in the same direction as seqnum order,
    // so newly decoded insts will be inserted at end, and committed
    // insts will be dropped from start
    std::vector<struct tracker_entry> allFlyingBmovs;


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

    // Prints out all in-flight bmovs
    void debugDump();
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

enum class PBTB_pred_conf_t
{
  PBTB_Pred_None = 0,
  PBTB_Pred_2bit,
  PBTB_Pred_vanilla
};

const PBTB_pred_conf_t PBTB_PREDICTOR_CONF =
                          PBTB_pred_conf_t::PBTB_Pred_vanilla;
                          //PBTB_pred_conf_t::PBTB_Pred_2bit;
                          //PBTB_pred_conf_t::PBTB_Pred_None;
class PBTB
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
    PBTB(CPU *_cpu): cpu(_cpu), tracker(this) {
      clear_predictor();
    }


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
    // ================ UNDO ACTIONS

    struct undo_entry
    {
        InstSeqNum seqnum;
        undo_action action;
    };

    static std::string undoEntryToString(const struct undo_entry &ent) {
        return csprintf("UNDO entry [sn:%d]: %s",
            ent.seqnum, PBTBMap::undoActionToString(ent.action));
    };

    // Store undo history for each breg separately
    // Within a single breg's undo stack, undo_entries should be stored in
    // seqnum order (oldest to newest).
     std::vector<struct undo_entry> undo_stacks[NUM_REGS];

    // ==================== PBTB Maps:
    // If the system wasn't pipelined, one of these would be sufficient

    PBTBMap map_fetch{"F"};
    PBTBMap map_final{"D"}; // init to all 0s

    // Note: map_fetch and map_finalize should ONLY EVER DIFFER in number
    // of loop iterations / shifted bits. All other modifications should apply
    // simultaneously to both. map_commit (once it's in) might differ though

    // ======= PREDICTOR
    // this is kindof a tacked-on hack, but it doesn't really matter I think?

    // a set of saturating counters, one per breg
    // from 0 (strong not taken) to 3 (strong taken)
    int predictor_ctrs[NUM_REGS];
    void clear_predictor();
    void write_predictor(int breg, bool taken);
    bool query_predictor(int breg);

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

  public:
    // undoes back to (but not including) squashingSeqNum
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
    // Note: if not taken, will advance pc
    PBTBResultType queryFromFetch(
            const StaticInstPtr inst, PCStateBase &pc_inout,
            bool vanilla_pred_taken,
            int *p_breg_out, uint64_t *p_version_out, bool *p_exhaust_out);

    //Note: if not taken, will instead
    //advance pcAddr and return in targetAddr_out
    PBTBResultType queryFromDecode(
            const StaticInstPtr inst, PCStateBase &pc_inout, InstSeqNum seqnum,
            int *p_breg_out, uint64_t *p_version_out);


    // Checks if breg is ready to finalize a bit-type branch
    bool isBregBitTypeAndReady(int breg) const;


    // === Squash behavior:
    // resetToFinalizedMap() // for when we squash from decode
    // resetToCommittedMap? // for when we are able to handle exceptions

    //=========== Pretty printing
    void debugDump();
    // Limits which regs to print (to limit output). Inclusive/exclusive
    void debugDump(int regstart, int regstop);

    void debugDumpUndo(int breg); //prints out undo actions for given breg
    void debugDumpAllUndo(); // Does all of above


}; // class PBTB

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_PBTB_HH__
