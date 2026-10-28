/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_SUBMIT_H
#define NATIVE_ADRENO_SUBMIT_H
#include "NativeAdrenoRing.h"
#include "NativeAdrenoVm.h"

/* IBs of one submission: a whole frame, so the ring prologue (cache flush, idle wait, thread switch) runs once per frame. */
enum : size_t {NativeAdrenoSubmitMaxBuffers=128};
struct NativeAdrenoSubmitBuffer {
    uint64_t handle;
    uint64_t iova;
    size_t bytes;      /* Complete indirect-buffer range in bytes. */
};
/* Coherence samples (int-070): [0] submissions sampled; [1..3] the last one's first IB word through the kernel's cacheable alias,
 * through DRAM before and through DRAM after the CPU cache maintenance of every mapped BO (~0 when unreadable); [4] samples whose
 * three values differ and [5..7] the first such sample. */
extern "C" volatile uint32_t native_adreno_coherence[8];
/* The write-combine audit (int-100), see NativeAdrenoSubmit.cpp, and the submission (1-based) that ended the trust; 0 = never. */
extern "C" volatile uint32_t native_adreno_wc_audit[6];
extern "C" volatile uint32_t native_adreno_wc_distrusted;
struct NativeAdrenoSubmitStats {
    uint32_t submissions,completed,timeouts,poisoned_jobs;
    size_t references_held,references_retired,peak_jobs;
};
/* Kernel-side execbuf: turns validated indirect buffers into one ring
 * submission and owns a device reference per distinct buffer until the
 * timestamp fence is actually observed. This is not a userspace ABI and not a
 * scheduler: at most eight jobs are outstanding, callers serialize every
 * method, and the ring/VM must already be attached to the same live context.
 * Buffers may be sub-ranges of a live VM mapping but never a different IOVA,
 * unmapped handle, table pool or a range that includes unreadable bytes.
 * A failed ring commit that poisons the ring keeps every reference and the
 * submit object unusable; only an independent reset may reclaim them. */
class NativeAdrenoSubmit final {
    struct Job {
        uint32_t fence=0;
        size_t count=0;
        uint64_t handles[NativeAdrenoSubmitMaxBuffers]{};
        uint32_t queue=0;
        bool active=false;
        uint64_t vm_next=0;   /* NativeAdrenoVm::nextHandle() at submission: a mapping with a handle from it on came later */
        NativeAdrenoVm *vm=nullptr;   /* the GPU address space (process page table) the job's buffers belong to */
    } jobs_[8];
    NativeAdrenoVm *vm_=nullptr;   /* the default VM: the kernel's table, installed at bring-up */
    /* Page table switching between processes: mode 0 none (one table for everything), 1 the CPU writes TTBR0 while the GPU is idle,
     * 2 the CP switches in the ring (CP_SMMU_TABLE_UPDATE). table_vm_ is the VM whose table the GPU runs after the queued work. */
    unsigned table_mode_=0;
    NativeAdrenoVm *table_vm_=nullptr;
    void *table_cookie_=nullptr;
    IOReturn (*write_table_)(void *,uint64_t ttbr0)=nullptr;
    IOReturn useVm(NativeAdrenoVm *,const NativeAdrenoRingClock *);
    NativeAdrenoRing *ring_=nullptr;
    NativeAdrenoSubmitStats stats_{};
    void noteStall(uint32_t observed,const NativeAdrenoRingClock *clock);   /* the CP's state for a fence that stopped advancing (native_adreno_stall) */
    bool attached_=false,poisoned_=false;
    uint32_t audit_tick_=0;   /* submissions since attach; every eighth one audits its IB lines */
    /* submit()'s distinct buffers and ring words: too large for the kernel stack at 128 IBs (callers serialize every method) */
    uint64_t scratch_handles_[NativeAdrenoSubmitMaxBuffers]{};
    uint32_t scratch_words_[NativeAdrenoSubmitMaxBuffers*4]{};
    Job *find(uint32_t fence);
    Job *free();
    IOReturn release(Job &);
    IOReturn poison();
public:
    NativeAdrenoSubmit()=default;
    ~NativeAdrenoSubmit()=default;
    NativeAdrenoSubmit(const NativeAdrenoSubmit &)=delete;
    NativeAdrenoSubmit &operator=(const NativeAdrenoSubmit &)=delete;
    IOReturn attach(NativeAdrenoVm *,NativeAdrenoRing *);
    /* Refuses while jobs are outstanding or poisoned; the caller must have
     * stopped the GPU and observed every fence before calling this. */
    IOReturn detach();
    /* queue is bookkeeping for one shared ring; it never reorders or isolates
     * submissions and is validated by the caller's open queue set. */
    IOReturn submit(const NativeAdrenoSubmitBuffer *,size_t,uint32_t queue,
        uint32_t *fence);
    /* The same for the buffers of one process's VM (null: the default VM); with table switching on, the GPU runs the job on that
     * VM's table. The clock is for the CPU switch's wait for the GPU to go idle. */
    IOReturn submit(NativeAdrenoVm *,const NativeAdrenoSubmitBuffer *,size_t,uint32_t queue,uint32_t *fence,
        const NativeAdrenoRingClock *);
    /* Turns on table switching (mode 1 or 2, see table_mode_); write installs a TTBR0 from the CPU (used by mode 1 and to leave a
     * table that is about to be freed). Only with no job outstanding. */
    IOReturn setTableSwitching(unsigned mode,IOReturn (*write)(void *,uint64_t),void *cookie);
    unsigned tableMode() const {return table_mode_;}
    /* Before a VM is freed: waits for every job, and when the GPU's table is that VM's, installs the default table from the CPU. */
    IOReturn forgetVm(NativeAdrenoVm *,const NativeAdrenoRingClock *,uint32_t timeout_us);
    NativeAdrenoVm *defaultVm() const {return vm_;}
    bool queueOutstanding(uint32_t queue) const;
    /* Queues [first, first+count): whether any of their jobs is outstanding, and a wait for every one of them (oldest first) that
     * leaves other queues' jobs alone. Each client of the shared ring owns one such range. */
    bool rangeOutstanding(uint32_t first,uint32_t count) const;
    IOReturn waitRange(uint32_t first,uint32_t count,const NativeAdrenoRingClock *,uint32_t timeout_us);
    /* Retires (without waiting) every job whose fence the CP has already passed, so completed work never pins buffers or
     * job slots just because nobody called wait(). */
    IOReturn retireCompleted();
    /* Waits for every outstanding job in submission order and retires its references. */
    IOReturn waitAll(const NativeAdrenoRingClock *,uint32_t timeout_us);
    /* Polls the CP timestamp; on completion retires every buffer reference. A fence whose job was already retired (by
     * wait or retireCompleted) is checked against the ring and succeeds once the CP has passed it. */
    IOReturn wait(uint32_t fence,const NativeAdrenoRingClock *,uint32_t timeout_us);
    /* Wait for an earlier fence without touching job references; a timeout
     * leaves every job and reference exactly as they were. */
    IOReturn observeFence(uint32_t fence,const NativeAdrenoRingClock *,uint32_t timeout_us);
    bool poisoned() const {return poisoned_;}
    size_t outstanding() const;
    NativeAdrenoSubmitStats stats() const {return stats_;}
};
#endif
