/*
 * 2024 Janet Vorobyeva
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" BY ITS LONE GRAD-STUDENT AUTHOR,
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO
 * LOREM IPSUM DOLOR SIT AMET GLORIAM BLAH BLAH ETC ARE DISCLAIMED.
 * CITE IT IF YOU COPY IT.
 */

#include "cpu/o3/pbtb/bitvec64.hh"

#include "base/cprintf.hh"
#include "cpu/o3/pbtb/pbtb_map.hh" // just for debugPrintBottomBits?

namespace gem5
{

namespace o3
{

// ==============================================================
//
//                        JV Bitvecs
//
// ==============================================================



bool BitVec64::at(int i) {
    assert(i >= 0 && i < size());
    return (data >> i) & 1;
}

// i must be already within the size of the vector
void BitVec64::write_bit(int i, bool is_set ) {
    assert(i >= 0 && i < size());
    data &= ~( ((uint64_t)1) << i ); // clear i-th bit
    data |= ( ((uint64_t)is_set) << i ); // write i-th bit
}

// append at MSB-side, (panics if full)
void BitVec64::push_back(bool bit) {
    assert(has_capacity(1));
    num_valid++; // need to increment num_valid first (write_bit checks it)
    write_bit(num_valid-1, bit);
}

// push at LSB-side (panic if full);
void BitVec64::push_front(bool bit) {
    assert(has_capacity(1));
    data <<= 1;
    data |= bit;
    num_valid++;
}

bool BitVec64::pop_back() {
    assert(size() >= 1);
    bool bit = at(num_valid-1);
    write_bit(num_valid-1, 0); // unnecessary? (but prevents garbage)
    num_valid--;
    return bit;
}

bool BitVec64::pop_front() {
    assert(size() >= 1);
    bool bit = at(0);
    data >>=1;
    num_valid--;
    return bit;
}

// modifies self, other gets placed at back of self
void BitVec64::append(BitVec64 other) {
    assert(has_capacity(other.size()));
    data |= other.data<<size();
    num_valid += other.size();
}

// print full debug info
std::string BitVec64::toDebugString(bool verbose) const {
    std::string bitstring = debugPrintBottomBits(data, num_valid, true);

    std::string verbose_info = (!verbose) ? std::string("") :
        csprintf(", data: %s",
            debugPrintBottomBits(data, capacity).c_str());
    return csprintf("<BitVec64 LSB:%s:MSB, size:%d%s> ",
            bitstring.c_str(), num_valid, verbose_info.c_str());
}

// Formats as just "110011" or "", LSB-first
std::string BitVec64::toString() const {
    std::string bitstring = debugPrintBottomBits(data, num_valid, true);
    //return csprintf("<bv '%s' n=%d> ", bitstring.c_str(), num_valid);
    return csprintf("%s", bitstring.c_str());
}

// aaa I don't want to figure out how to replace gem5's main,
// we can just call this from somewhere
static void test_bitvecs() {
    BitVec64 empty = BitVec64();
    BitVec64 t = BitVec64(1, 1);
    //BitVec64 f = BitVec64(1, 0);
    BitVec64 vec_7 = BitVec64(3, 0x7);
    //BitVec64 vec_07 = BitVec64(4, 0x7);

    assert(empty.size() == 0);
    assert(t.size() == 1);
    assert(vec_7.size() == 3);

    BitVec64 vinit = BitVec64 {1,1,0,0};
    assert(vinit.pop_front() == 1);
    assert(vinit.pop_front() == 1);
    assert(vinit.pop_front() == 0);
    assert(vinit.pop_front() == 0);
    assert(vinit.size() == 0);

    // should be able to hold 64
    BitVec64 v64 = BitVec64(); // clear v64
    for (int i = 0; i < 32; i++) {
        v64.push_back(1);
        v64.push_back(0);
    }
    assert(v64.size() == 64);
    assert(!v64.has_capacity(1));

    // append 0101 to vec7, should be 1110101
    //printf("=== testing append\n");
    vec_7.append(BitVec64{0,1,0,1});
    //show(vec_7);

    assert(vec_7.size() == 7);
    for (auto i : { 1, 1, 1, 0, 1, 0, 1 }) {
        assert(vec_7.pop_front() == i);
    }
}

} // namespace o3
} // namespace gem5
