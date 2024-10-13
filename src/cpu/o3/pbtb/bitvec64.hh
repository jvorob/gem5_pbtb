#ifndef __CPU_O3_BITVEC64_HH__
#define __CPU_O3_BITVEC64_HH__

#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <string>

namespace gem5
{

namespace o3
{

class BitVec64
{
    /* Minimal abstraction over a queue of bits (uin64_t + a count)
     * To be used as a value type
     * Can push/pop at front/back
     * (front is LSB-side, back is MS wB-side)
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
}; // namespace o3
}; // namespace gem5

#endif // __CPU_O3_BITVEC64_HH__
