
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
#include "cpu/static_inst.hh"

const int NUM_PBTB_REGS=32;

namespace gem5
{

namespace o3
{

class CPU; // forward declare? Need for getting ptr to cpu

// ==============================================================
//
//                     JV Bitvec
//
// ==============================================================

class BitVec64
{
    /* Minimal abstraction over a queue of bits (uin64_t + a count)
     * To be used as a value type
     * Can push/pop at front/back
     * (front is LSB-side, back is MSB-side)
     */

    private:
        int num_valid; // how many of the bits, from LSB are valid
        uint64_t data; // the underlying 64 bits, treat LSB as [0], MSB as [63]
                       // [0] is LSB, push/pop at LSB, push_back/popback at MSB
        const static int capacity = 64; // max # of bits `data` can hold

    public:
        BitVec64(int n, uint64_t bits): num_valid(n), data(bits)
          { assert(n >= 0 && n <= capacity); };
        BitVec64(): BitVec64(0, 0) {};

        // default constructors for move, move=, copy, copy= (memberwise)
        BitVec64(std::initializer_list<bool> lst): BitVec64(0,0) {
            assert(lst.size() <= 64);
            for (auto b : lst) {
                push_back(b);
            }
        }
    public:

        int size() { return num_valid; }
        // true means this can hold at least n more bits
        bool has_capacity(int n) { return (n + num_valid) <= capacity; }

        // i must be already within the size of the vector
        bool at(int i);
        void write_bit(int i, bool is_set );

        // Panic if full / empty
        void push_back(bool bit);
        void push_front(bool bit);
        bool pop_back();
        bool pop_front();

        // modifies self, other gets placed at back of self
        void append(BitVec64 other);

        // print full debug info
        std::string toDebugString(bool verbose=false) const;

        // Formats as just "110011" or "", LSB-first
        std::string toString() const;

        // To avoid putting this in gem5 main, just call this somewhere?
        static void test_bitvecs();
};

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
    PrecomputedBTB(CPU *_cpu): cpu(_cpu), tracker(this) {}


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
    struct breg_data
    {
        int64_t   version;
        Addr      source;
        Addr      target;
        BranchType cond_type;
        int64_t   cond_val;
        int64_t   cond_aux_val;
    };

  private:
    // ================ UNDO ACTIONS

    enum class utype
    {
      U_NONE,        // Nothing to undo, can discard

      // U_SET_PART, // arg( which field? (src/tgt/etc)), needed to implement
      //                fwd actions properly, but not neede for undo
      U_FULL,        // Used to undo a version?

      //U_PUSH_BITS, // arg(bitvec) used by bit-bmovs (pushes at back of queue)
      U_UNPUSH_BITS, // arg(n) used to undo bit-bmov (removes 1 from BACK)

      //U_POP_BITS,   // arg(n), used by pb, pops from front of queue
      U_UNPOP_BITS, // arg(bit), used to undo pb
    };

  //class UndoAction {
  //  public:
  //    // version: before/after the action
  //    // for ops that don't modify version, these will be equal
  //    int64_t undone_ver;
  //    int64_t done_ver;
  //    int8_t breg;

  //    // type-specific stuff
  //    utype type;
  //    union {
  //      struct breg_data as_overwrite;
  //      BitVec64 as_consumed_bits; // undo by returning the bit to hd
  //      int as_pushed_bits; // undo by un-pushing those bits from tail
  //    };

  //    static UndoAction MkFull(int8_t breg,
  //                             int64_t new_version, breg_data old_data) {
  //      auto u = UndoAction(breg, old_data.version,
  //    }

  //  private:
  //};


    struct undo_action
    {
        // version: before/after the action
        // for ops that don't modify version, these will be equal
        int breg;
        int64_t undone_ver;
        int64_t done_ver;

        // type-specific stuff
        utype type;
        union
        {
          struct breg_data as_overwrite;
          BitVec64 as_consumed_bits; // undo by returning the bits to head
          int as_pushed_bits; // undo by un-pushing those bits from tail
        };
    };

    struct undo_entry
    {
        InstSeqNum seqnum;
        struct undo_action action;
    };

    // Gets/sets a single bregs value?
    static struct breg_data breg_get(const struct pbtb_map *pmap, int breg);
    static void breg_set(struct pbtb_map *pmap, int breg,
                  const struct breg_data *data);

    static std::string bdataToString(const struct breg_data &bdata);
    static std::string undoEntryToString(const struct undo_entry &ent) {
        std::string prev_str; // descriptor of what will be undone
        switch(ent.action.type) {
          case utype::U_FULL:
              prev_str = csprintf("prev state: %s",
                  bdataToString(ent.action.as_overwrite));
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



    struct pbtb_map map_fetch = {};
    struct pbtb_map map_final = {}; // init to all 0s

    // ==================== PER-MAP Functions:
    // If the system wasn't pipelined, these would be sufficient
    // Each of these acts on a single pbtb_map in the straightforward way


    // Note: map_fetch and map_finalize should ONLY EVER DIFFER in number
    // of loop iterations / shifted bits. All other modifications should apply
    // simultaneously to both. map_commit (once it's in) might differ though

    undo_action m_setSource(struct pbtb_map *pmap, int breg, Addr source_addr);
    undo_action m_setTarget(struct pbtb_map *pmap, int breg, Addr target_addr);

    // Sets the condition for a given breg
    // val treated differently for different types?
    // - Taken or NotTaken ignore it
    // - LoopTaken will be taken (val) times, then NT, than stall
    // - ShiftBit takes a bitstring in val, bottom n bits shifted in
    undo_action m_setCondition(struct pbtb_map *pmap, int breg,
            BranchType conditionType, uint64_t val, int64_t n);


    // Returns the index of the first nonempty breg matching PC,
    // or -1 if none found
    int m_findMatchingBreg(const struct pbtb_map *pmap, Addr pcAddr) const;


    /**
     * Returns the outcome of a pb op if it queried the given pmap.
     * NOTE: This does not actually modify the pbtb, MUST ALSO CALL
     *       m_consumeIter(breg) to actually consume that iteration.
     * version / targetAddr_out will be written only if match found
     * @param pcAddr The pc of branches to look for (source PC)
     * @param p_breg_out If a breg match is found, then the breg, else -1
     * @param p_version_out If a breg match is found, version will be here
     * @param p_targetAddr_out If match is a tkn branch, tgt addr will be here
     * @return Returns PBTBResultType depending on the kind of match
     */
    PBTBResultType m_queryPC(const struct pbtb_map *pmap, Addr pcAddr,
            int *p_breg_out,
            uint64_t *p_version_out,
            Addr *p_targetAddr_out) const;

    //  A pb queries the PBTB, but also modifies it (conditonal or
    //  loop branches will have different behavior each time a pb goes through)
    //  This function does the modification, consuming one bit from a specified
    //  breg. If breg is empty/invalid(-1)/exhausted, does nothing
    //  TODO: make this return a result? how many iters consumed?
    //  TODO: make this a more general clear method?
    undo_action m_consumeIter(struct pbtb_map *pmap, int breg);

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
    void m_apply_undo(struct pbtb_map *pmap, undo_action undo);


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

    // if a Pb hits in this breg, is there any chance that it will
    // modify the map's state in a way that might need undoing
    // (note: false is always accurate, but true might be conservative)
    bool pbCouldModifyState(const struct pbtb_map *pmap, int breg) const;

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
