#ifndef __CPU_O3_PBTB_MAP_HH__
#define __CPU_O3_PBTB_MAP_HH__

#include <cstdint>
#include <string>

#include "base/cprintf.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/pbtb/bitvec64.hh"

namespace gem5
{

namespace o3
{


constexpr int NUM_PBTB_REGS=32;

// =========== Loose functions: (should go in pbtb_common.hh?)
const std::string debugPrintBottomBits(
    uint64_t bits, int n, bool lsb_first=false);



class PBTBMap
{
  public:

  // Friends:
  friend class PBTB;

// ==============================================================
//
//                     COMPONENT TYPES
//
// ==============================================================

    enum class BranchType
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

    const static char* BranchTypeToShortCode(BranchType type) {
        return BranchTypeCodes[static_cast<int>(type)];
    }

    const static int NUM_REGS = NUM_PBTB_REGS;
    //const static int MAX_CHECKPOINTS = 32;



    struct breg_data
    {
        int64_t version;
        Addr source;
        Addr target;
        BranchType cond_type;
        int64_t cond_val;
        int64_t cond_aux_val;
    };

    // Gets/sets a single bregs value?
    struct breg_data breg_get(int breg);
    void breg_set(int breg, const struct breg_data *data);

    static std::string bdataToString(const struct breg_data &bdata);

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

    // ================================================================
    //
    //                         UNDO STATES
    //
    // ================================================================
    // We need to be able to squash instructions that modify the map_final pmap
    // Most bmovs can be undone by simply reverting to the previous pbtb state
    // (for bmovs that can't be interleaved, i.e. ones that update the version)
    //   However we must also be able to undo bit-pushes and bit-consumes
    // individually, as they can be interleaved (bit-type bmovs or pbs).
    //   To support this, an undo_entry for a breg holds an undo_state, which
    // can be either a full breg's data (for overwriting), or a partial
    // undo-state (specifying which bits were consumed, or how many bits were
    // pushed)

    // Stores the state of a single breg: not used in pbtb_map, but used
    // for stashing/passing around full value of a single breg in undo history

    struct undo_action
    {
        int breg;
        int64_t undone_ver;
        int64_t done_ver;
        utype type;
        union
        {
            struct breg_data as_overwrite;
            BitVec64 as_consumed_bits;
            int as_pushed_bits;
        };
    };

    // ===========  Undo constructors:  ===================

    // Overwrite: assumes version++, reverts to current state
    // (should be called before modifying state so it can save it)
    undo_action makeUndoFull(int breg) {
        struct undo_action ret_undo =
        {
            .breg = breg,
            .undone_ver = this->version[breg],
            .done_ver = this->version[breg] + 1,
            .type = utype::U_FULL,
            .as_overwrite = this->breg_get(breg),
        };
        return ret_undo;
    }

    // Push_bits: assumes no version change, reverts by discarding n bits
    undo_action makeUndoPushN(int breg, int n) {
        struct undo_action ret_undo =
        {
            .breg = breg,
            .undone_ver = this->version[breg],
            .done_ver   = this->version[breg], // ver doesn't change
            .type = utype::U_UNPUSH_BITS,
            .as_pushed_bits = n,
        };
        return ret_undo;
    }

    // Consume_bits: assumes no version change
    // Reverts by pushing the n bits we had consumed (at front)
    undo_action makeUndoConsumeBits(int breg, BitVec64 consumed_bv) {
        struct undo_action ret_undo =
        {
            .breg = breg,
            .undone_ver = this->version[breg],
            .done_ver   = this->version[breg], // ver doesn't change
            .type = utype::U_UNPOP_BITS,
            .as_consumed_bits = consumed_bv,
        };
        return ret_undo;
    }

    // No-op (let's set version correctly so it doesn't crash
    //         even if we don't handle it)
    undo_action makeUndoBlank(int breg) {
        struct undo_action ret_undo =
        {
            .breg = breg,
            .undone_ver = this->version[breg],
            .done_ver   = this->version[breg], // ver doesn't change
            .type = utype::U_NONE,
            .as_pushed_bits = 0, // shut up the type checker
        };
        return ret_undo;
    }

    static std::string undoActionToString(const struct undo_action &und) {
        std::string prev_str; // descriptor of what will be undone
        switch(und.type) {
          case utype::U_FULL:
              prev_str = csprintf("prev state: %s",
                  bdataToString(und.as_overwrite));
              break;
          case utype::U_UNPOP_BITS:
              prev_str = csprintf("pb had consumed: fst<%s>lst",
                  und.as_consumed_bits.toString());
              break;
          case utype::U_UNPUSH_BITS:
              prev_str = csprintf("bmov had pushed %d bits",
                  und.as_pushed_bits);
              break;
          case utype::U_NONE:
              prev_str = csprintf("noop");
              break;
        }

        std::string v_str; // describes version (either v->v, or just v+);
        v_str = csprintf("v%d->v%d",
            und.undone_ver, und.done_ver);

        return csprintf("(UND: b%d %s (%s))", und.breg, v_str, prev_str);
    };


// ==============================================================
//
//                     ACTUAL START OF PBTB_MAP
//
// ==============================================================
  public:
    PBTBMap(const std::string dbgId) : dbgId(dbgId) {};

    const std::string dbgId;
  private:

    // Member data fields
    int64_t    version      [NUM_REGS] = {};
    Addr       source       [NUM_REGS] = {};
    Addr       target       [NUM_REGS] = {};
    BranchType cond_type    [NUM_REGS] = {};
    int64_t    cond_val     [NUM_REGS] = {};
    int64_t    cond_aux_val [NUM_REGS] = {};
    struct breg_data bregs[NUM_REGS];


  private:
    // Add methods that were previously in pbtb_map
    breg_data breg_get(int breg) const;
    void      breg_set(int breg, const breg_data &data);
    //UndoToken applyAction(PBTBAction action); w

  public:
    PBTBMap() {
        for (int i = 0; i <NUM_REGS; i++) {
            cond_type[i] = BranchType::NoBranch;
            version[i] = -1;
        }
    }

    void setFrom(const PBTBMap &other) {
        for (int i = 0; i < NUM_REGS; ++i) {
            version[i]      = other.version[i];
            source[i]       = other.source[i];
            target[i]       = other.target[i];
            cond_type[i]    = other.cond_type[i];
            cond_val[i]     = other.cond_val[i];
            cond_aux_val[i] = other.cond_aux_val[i];
        }
        // Note: dbgId is not copied
    };

    undo_action setSource(int breg, Addr source_addr);
    undo_action setTarget(int breg, Addr target_addr);

    /**
     * Sets the condition for a given breg
     * val treated differently for different types?
     * - Taken or NotTaken ignore it
     * - LoopTaken will be taken (val) times, then NT, than stall
     * - ShiftBit takes a bitstring in val, bottom n bits shifted in
     */
    undo_action setCondition(int breg,
        BranchType conditionType, uint64_t val, int64_t n);

    /**
     * Returns the index of the first nonempty breg matching PC,
     * or -1 if none found
     */
    int findMatchingBreg(Addr pcAddr) const;

    /**
     * Returns the outcome of a pb op if it queried the given pmap.
     * NOTE: This does not actually modify the pbtb, MUST ALSO CALL
     *       consumeIter(breg) to actually consume that iteration.
     * version / targetAddr_out will be written only if match found
     * @param pcAddr The pc of branches to look for (source PC)
     * @param p_breg_out If a breg match is found, then the breg, else -1
     * @param p_version_out If a breg match is found, version will be here
     * @param p_targetAddr_out If match is a tkn branch, tgt addr will be here
     * @return Returns PBTBResultType depending on the kind of match
     */
    PBTBResultType queryPC(Addr pcAddr,
            int *p_breg_out,
            uint64_t *p_version_out,
            Addr *p_targetAddr_out) const;

    /**
     * A pb queries the PBTB, but also modifies it (conditional or
     * loop branches will have different behavior each time a pb goes through)
     * This function does the modification, consuming one bit from a specified
     * breg. If breg is empty/invalid(-1)/exhausted, does nothing
     * TODO: make this return a result? how many iters consumed?
     * TODO: make this a more general clear method?
     */
    undo_action consumeIter(int breg);

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
    void apply_undo(undo_action undo);

    //=========== Pretty printing
    void debugDump();
    // Limits which regs to print (to limit output). Inclusive/exclusive
    void debugDump(int regstart, int regstop);

    // =============== MISC

    // if a Pb hits in this breg, is there any chance that it will
    // modify the map's state in a way that might need undoing
    // (note: false is always accurate, but true might be conservative)
    //bool pbCouldModifyState(int breg) const;
};

}; // namespace o3
}; // namespace gem5

#endif //__CPU_O3_PBTB_MAP_HH__
