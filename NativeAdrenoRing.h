/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_RING_H
#define NATIVE_ADRENO_RING_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>

/* A7XX command processor ring and timestamp fence, not a scheduler.
 * Register indices are dword offsets; the platform adds the MMIO base and
 * multiplies by four. Caller must already own an exclusive, powered GPU with
 * SQE microcode loaded (CP_SQE_INSTR_BASE), global reset released and all
 * masters quiesced while attach programs the ring. Ring and fence CPU
 * addresses must be actual writable mappings of the same IOVAs that were
 * mapped into the active native-4K GPU context. No MMIO base, IRQ, power,
 * GMU, context switching or per-client validation is implemented here.
 * Every submit publishes ring memory before advancing WPTR and completes
 * only when the CP timestamp value is observed at the fence address.
 * A failed WPTR write or cache publication is uncertain: the ring is poisoned
 * and every resource must be retained until an independently verified reset. */
struct NativeAdrenoRingIo {
    void *cookie;
    /* Return zero only on a completed access. */
    int (*write32)(void *,uint32_t reg,uint32_t value);
    int (*read32)(void *,uint32_t reg,uint32_t *value);
    int (*barrier)(void *);
};
struct NativeAdrenoRingCache {
    void *cookie;
    /* Clean owned ring bytes to PoC before the device reads them. */
    int (*publish)(void *,const volatile uint32_t *words,size_t bytes);
    /* Invalidate the fence page before the CPU reads CP-written values. */
    int (*acquire)(void *,volatile uint32_t *words,size_t bytes);
};
struct NativeAdrenoRingClock {
    void *cookie;
    uint64_t (*now_us)(void *);
    /* Optional; may sleep. Never used before a successful submit. */
    int (*idle)(void *);
};
enum NativeAdrenoRingFence {
    NativeAdrenoRingFenceComplete = 0,
    NativeAdrenoRingFenceTimeout,
    NativeAdrenoRingFencePoisoned,
    NativeAdrenoRingFenceArgument
};
class NativeAdrenoRing final {
    NativeAdrenoRingIo io_{};
    NativeAdrenoRingCache cache_{};
    volatile uint32_t *ring_=nullptr,*fence_=nullptr;
    size_t ring_bytes_=0,fence_bytes_=0;
    uint64_t ring_iova_=0,fence_iova_=0;
    uint32_t next_=0,rptr_shadow_=0,seqno_=0,completed_=0,submissions_=0;
    unsigned generation_=7;
    bool attached_=false,poisoned_=false,reset_pending_=false;
    uint32_t prologueWords(bool reset) const;
    IOReturn poison();
    IOReturn readRptr(uint32_t *);
    IOReturn emitPrologue(uint32_t at,bool reset,uint32_t *end);
    IOReturn emitFence(uint32_t at,uint32_t seqno,uint32_t *end);
    IOReturn emitFramedFence(uint32_t at,uint32_t seqno,uint32_t *end);
    IOReturn submitWords(const uint32_t *words,size_t word_count,uint32_t *fence,bool framed);
public:
    NativeAdrenoRing()=default;
    ~NativeAdrenoRing()=default;
    NativeAdrenoRing(const NativeAdrenoRing &)=delete;
    NativeAdrenoRing &operator=(const NativeAdrenoRing &)=delete;
    /* 7 (the default) emits the A7XX forms. 6 emits the A6XX ones of the pinned a6xx_gpu.c for a CP without APRIV or a
     * second thread (Adreno 610): CP_RB_CNTL with NO_UPDATE, the fence as CP_EVENT_WRITE CACHE_FLUSH_TS with IRQ, and
     * user submissions preceded by CACHE_INVALIDATE and the CCU depth/colour invalidates (a6xx_submit,
     * a6xx_set_pagetable) instead of the BR/BV framing. Only before attach. */
    IOReturn setGeneration(unsigned generation);
    /* ring_bytes is a power of two in [8KiB,1MiB]; ring_iova must be aligned
     * to ring_bytes. fence_bytes >=64 and fence_iova 8-aligned. The fence page
     * layout is: +0 CP read-pointer shadow, +4 completed timestamp, +8 spare.
     * All outputs are prevalidated before any register write. */
    IOReturn attach(const NativeAdrenoRingIo *,const NativeAdrenoRingCache *,
        void *ring,size_t ring_bytes,uint64_t ring_iova,
        void *fence,size_t fence_bytes,uint64_t fence_iova);
    /* Programs nothing. Refuses while submissions are unobserved or poisoned,
     * because only the caller can establish real GPU quiescence and unmap. */
    IOReturn detach();
    /* Emit an entire command stream followed by the timestamp fence and commit
     * it with one WPTR update. Returns a nonzero fence value on success. The
     * words are copied; the caller's buffer is never retained. */
    IOReturn submit(const uint32_t *words,size_t word_count,uint32_t *fence);
    /* User command buffers: the A7XX BR/BV thread framing of a7xx_submit() around the words (CP init and the kernel's own
     * bring-up packets keep the plain submit() above, as Linux sends them outside a7xx_submit()). */
    IOReturn submitFramed(const uint32_t *words,size_t word_count,uint32_t *fence);
    /* The next framed submission starts with the A7XX context entry of a6xx_set_pagetable() (BR/BV counters, on-chip
     * timestamps and resource table cleared between thread syncs); a new user client requests it, as Linux does per context. */
    void requestContextReset() {reset_pending_=true;}
    /* Polls the CP-written timestamp after invalidating the fence page. Never
     * writes registers and never advances the ring. Timeout is not poison. */
    NativeAdrenoRingFence wait(uint32_t fence,const NativeAdrenoRingClock *,
        uint32_t timeout_us,uint32_t *observed);
    /* Observe a previously issued fence without retiring or accounting for it.
     * Used before a new submission waits on an earlier batch; only fence values
     * this ring issued are accepted. Never writes a register. */
    NativeAdrenoRingFence observeFence(uint32_t fence,const NativeAdrenoRingClock *,
        uint32_t timeout_us,uint32_t *observed);
    bool poisoned() const {return poisoned_;}
    uint32_t outstanding() const {return seqno_-completed_;}
    uint32_t lastSubmitted() const {return seqno_;}      /* the newest fence number handed out */
    uint32_t completedFence() const {return completed_;}
    uint32_t submissions() const {return submissions_;}
    uint32_t readPointer() const {return rptr_shadow_;}
};
#endif
