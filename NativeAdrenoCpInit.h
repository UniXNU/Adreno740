/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_CPINIT_H
#define NATIVE_ADRENO_CPINIT_H
#include "NativeAdrenoRing.h"
struct NativeAdrenoCpInitConfig {
    uint64_t sqe_iova;             /* CP_SQE_INSTR_BASE, 4 KiB aligned, nonzero */
    uint64_t pwrup_reglist_iova;   /* Power-up register list, 4-byte aligned */
    /* Nonzero when the caller already wrote CP_SQE_INSTR_BASE before CP_SQE_CNTL, as a6xx hw_init does: the base is
     * then read back and compared instead of rewritten after the SQE started. */
    uint32_t sqe_base_programmed;
    /* 0 or 7: the A7XX sequence below. 6: a6xx_cp_init for an A6XX without APRIV or a second CP thread (Adreno 610):
     * CP_ME_INIT alone with its eight words 0x2f, 3, 0x20000000 and zeros; no power-up list (pwrup_reglist_iova may be 0). */
    uint32_t generation=0;
};
/* One-shot A7XX command-processor start sequence, not a GPU power or reset
 * driver. The caller must have completed GMU boot, powered GX/core clocks,
 * released the CP reset, loaded and hash-verified the SQE microcode into the
 * real IOVA given here, and hold exclusive MMIO. This programs
 * CP_SQE_INSTR_BASE, then submits the pinned a7xx_cp_init sequence directly
 * through the ring: CP_THREAD_CONTROL with concurrent binning disabled and
 * CP_ME_INIT with the Linux mask, hardware-context count, operation mode and
 * power-up register list. Completion is the ring timestamp fence, so a hung
 * CP is reported as a timeout instead of a fabricated success. A second call
 * cannot re-initialize the same object. */
class NativeAdrenoCpInit final {
    bool done_=false,poisoned_=false;
    IOReturn poison();
public:
    NativeAdrenoCpInit()=default;
    ~NativeAdrenoCpInit()=default;
    NativeAdrenoCpInit(const NativeAdrenoCpInit &)=delete;
    NativeAdrenoCpInit &operator=(const NativeAdrenoCpInit &)=delete;
    IOReturn run(const NativeAdrenoRingIo *,NativeAdrenoRing *,
        const NativeAdrenoCpInitConfig *,uint32_t *fence);
    bool completed() const {return done_;}
    bool poisoned() const {return poisoned_;}
    static uint32_t meInitMask();
};
#endif
