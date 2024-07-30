/* === Janet Vorobyeva 2024
 * Adding branch registers to riscv isa
 */


#ifndef __ARCH_RISCV_REGS_BRANCH_HH__
#define __ARCH_RISCV_REGS_BRANCH_HH__

#include <string>
#include <vector>

#include "cpu/reg_class.hh"
#include "debug/BranchRegs.hh"

namespace gem5
{

namespace RiscvISA
{

namespace branch_reg
{

enum : RegIndex
{
    _B0Idx,  _B1Idx,  _B2Idx,  _B3Idx,  _B4Idx,
    _B5Idx,  _B6Idx,  _B7Idx,  _B8Idx,  _B9Idx,
    _B10Idx, _B11Idx, _B12Idx, _B13Idx, _B14Idx,
    _B15Idx, _B16Idx, _B17Idx, _B18Idx, _B19Idx,
    _B20Idx, _B21Idx, _B22Idx, _B23Idx, _B24Idx,
    _B25Idx, _B26Idx, _B27Idx, _B28Idx, _B29Idx,
    _B30Idx, _B31Idx,

    NumRegs
};

} // namespace branch_reg


inline constexpr RegClass branchRegClass(BranchRegClass, BranchRegClassName,
        branch_reg::NumRegs, debug::BranchRegs);

namespace branch_reg
{

inline constexpr RegId
    B0  = branchRegClass[_B0Idx],
    B1  = branchRegClass[_B1Idx],
    B2  = branchRegClass[_B2Idx],
    B3  = branchRegClass[_B3Idx],
    B4  = branchRegClass[_B4Idx],
    B5  = branchRegClass[_B5Idx],
    B6  = branchRegClass[_B6Idx],
    B7  = branchRegClass[_B7Idx],
    B8  = branchRegClass[_B8Idx],
    B9  = branchRegClass[_B9Idx],

    B10  = branchRegClass[_B10Idx],
    B11  = branchRegClass[_B11Idx],
    B12  = branchRegClass[_B12Idx],
    B13  = branchRegClass[_B13Idx],
    B14  = branchRegClass[_B14Idx],
    B15  = branchRegClass[_B15Idx],
    B16  = branchRegClass[_B16Idx],
    B17  = branchRegClass[_B17Idx],
    B18  = branchRegClass[_B18Idx],
    B19  = branchRegClass[_B19Idx],

    B20  = branchRegClass[_B20Idx],
    B21  = branchRegClass[_B21Idx],
    B22  = branchRegClass[_B22Idx],
    B23  = branchRegClass[_B23Idx],
    B24  = branchRegClass[_B24Idx],
    B25  = branchRegClass[_B25Idx],
    B26  = branchRegClass[_B26Idx],
    B27  = branchRegClass[_B27Idx],
    B28  = branchRegClass[_B28Idx],
    B29  = branchRegClass[_B29Idx],

    B30  = branchRegClass[_B30Idx],
    B31  = branchRegClass[_B31Idx];

const std::vector<std::string> RegNames = {
    "b0", "b1", "b2", "b3", "b4",
    "b5", "b6", "b7", "b8", "b9",

    "b10", "b11", "b12", "b13", "b14",
    "b15", "b16", "b17", "b18", "b19",

    "b20", "b21", "b22", "b23", "b24",
    "b25", "b26", "b27", "b28", "b29",

    "b30", "b31"
};




} // namespace branch_reg

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_BRANCH_HH__
