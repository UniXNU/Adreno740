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
    /* Optional: the GMU's AHB fence status (bit 0: a write to a fenced GPU register was dropped while the GPU power domain was not
     * ready); with clear set, also clears it. Without it, WPTR writes are not checked. */
    void *fence_cookie;
    int (*fence_status)(void *,int clear,uint32_t *status);
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
enum : uint32_t {NativeAdrenoRingStallWords = 22};   /* NativeAdrenoRing::stallState */
class NativeAdrenoRing final {
    NativeAdrenoRingIo io_{};
    NativeAdrenoRingCache cache_{};
    volatile uint32_t *ring_=nullptr,*fence_=nullptr;
    size_t ring_bytes_=0,fence_bytes_=0;
    uint64_t ring_iova_=0,fence_iova_=0;
    uint32_t next_=0,rptr_shadow_=0,seqno_=0,completed_=0,submissions_=0;
    bool attached_=false,poisoned_=false,reset_pending_=false;
    uint32_t context_=0;
    /* The GPU page table the CP runs (TTBR0 with its ASID in bits 63:48) and whether the next framed prologue must switch to it. */
    uint64_t table_=0;
    bool table_pending_=false;
    IOReturn poison();
    IOReturn writeWptr(uint32_t value);
    IOReturn readRptr(uint32_t *);
    IOReturn emitPrologue(uint32_t at,bool reset,bool table,uint32_t *end);
    IOReturn emitFence(uint32_t at,uint32_t seqno,uint32_t *end);
    IOReturn emitFramedFence(uint32_t at,uint32_t seqno,uint32_t *end);
    IOReturn submitWords(const uint32_t *words,size_t word_count,uint32_t *fence,bool framed);
public:
    NativeAdrenoRing()=default;
    ~NativeAdrenoRing()=default;
    NativeAdrenoRing(const NativeAdrenoRing &)=delete;
    NativeAdrenoRing &operator=(const NativeAdrenoRing &)=delete;
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
    /* The client whose submission comes next. A change of client is a context switch: as a6xx_set_pagetable() does whenever the
     * submitting context differs from the ring's last one, the next framed prologue carries the context entry. Turnip's BR/BV barriers
     * keep on-chip timestamps and counters that only that entry resets, so clients sharing the ring without it left one client's
     * counters to the next, and the threads waited on each other for good (int-249..254: BR in a later IB than BV, no fault). */
    void useContext(uint32_t context) {if(context!=context_) {context_=context;reset_pending_=true;}}
    /* The GPU page table of the submission that comes next (a process's own table: TTBR0 with its ASID in bits 63:48). A change is
     * a context switch with a table switch: the next framed prologue carries a6xx_set_pagetable()'s CP_SMMU_TABLE_UPDATE, which the CP
     * executes after every earlier submission has finished (its CP_WAIT_TIMESTAMP), so each submission runs on its own table. */
    void useTable(uint64_t ttbr0) {if(ttbr0 && ttbr0!=table_) {table_=ttbr0;table_pending_=true;reset_pending_=true;}}
    /* What the bank holds now, set by the CPU (bring-up, or a table written while the GPU was idle): no packet is due for it. */
    void tableInstalled(uint64_t ttbr0) {table_=ttbr0;table_pending_=false;}
    uint64_t table() const {return table_;}
    bool tablePending() const {return table_pending_;}
    /* Polls the CP-written timestamp after invalidating the fence page. Never
     * writes registers and never advances the ring. Timeout is not poison. */
    NativeAdrenoRingFence wait(uint32_t fence,const NativeAdrenoRingClock *,
        uint32_t timeout_us,uint32_t *observed);
    /* Observe a previously issued fence without retiring or accounting for it.
     * Used before a new submission waits on an earlier batch; only fence values
     * this ring issued are accepted. Never writes a register. */
    NativeAdrenoRingFence observeFence(uint32_t fence,const NativeAdrenoRingClock *,
        uint32_t timeout_us,uint32_t *observed);
    /* Reads the CP's state for a fence that stopped advancing into out[NativeAdrenoRingStallWords]: RBBM_STATUS..STATUS3,
     * RBBM_INT_0_STATUS, CP_HW_FAULT, CP_STATUS_1, CP_PROTECT_STATUS, CP_RB_RPTR, CP_RB_WPTR, CP_IB1_BASE (lo, hi),
     * CP_IB1_REM_SIZE, CP_IB2_BASE (lo, hi), CP_IB2_REM_SIZE, CP_BV_RB_RPTR, CP_BV_IB1_BASE lo, CP_BV_IB1_REM_SIZE,
     * CP_BV_IB2_REM_SIZE. Never writes a register; a failed read is poison. */
    IOReturn stallState(uint32_t *out);
    /* Writes CP_RB_WPTR again with the value already published: for a CP that went idle with work in the ring (a lost write pointer
     * notification); harmless when the CP already saw it. */
    IOReturn kick();
    /* a6xx_bus_clear_pending_transactions without the GX side: GBIF client requests halted, then all AXI requests, each until its
     * acknowledge or a bounded spin, then the halt cleared. ack[0] and ack[1] are the acknowledge words (bit 0 client, bit 1 arbiter). */
    IOReturn drainBus(uint32_t *ack);
    /* The CP_INDIRECT_BUFFER packet that ends just before ring index rptr (what a thread whose read pointer is rptr executes): its GPU
     * address and size in dwords. kIOReturnUnsupported when the words there are not that packet. */
    IOReturn ibBefore(uint32_t rptr,uint64_t *iova,uint32_t *dwords) const;
    bool poisoned() const {return poisoned_;}
    uint32_t outstanding() const {return seqno_-completed_;}
    uint32_t lastSubmitted() const {return seqno_;}      /* the newest fence number handed out */
    uint32_t completedFence() const {return completed_;}
    uint32_t submissions() const {return submissions_;}
    uint32_t readPointer() const {return rptr_shadow_;}
};
#endif
