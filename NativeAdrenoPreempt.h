/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_PREEMPT_H
#define NATIVE_ADRENO_PREEMPT_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>

/* A6XX..A7XX CP context switch between two rings, as msm's a6xx_preempt.c drives it: the CP keeps one ring loaded and saves
 * and restores the other's state through a preemption record the CPU fills in. Every constant below is from the pinned
 * registers/adreno/a6xx.xml and a6xx_gpu.c/a6xx_preempt.c, never guessed:
 *   CP_CONTEXT_SWITCH_CNTL 0x08a0 (STOP bit 0, LEVEL bits 6..7, USES_GMEM bit 8, SKIP_SAVE_RESTORE bit 9)
 *   CP_CONTEXT_SWITCH_SMMU_INFO 0x08a1 (64 bit): the a7xx_cp_smmu_info the CP reads on the way in
 *   CP_CONTEXT_SWITCH_PRIV_NON_SECURE_RESTORE_ADDR 0x08a3 (64 bit): the incoming ring's preemption record
 *   RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE 0x8e50
 *   GMU_PWR_COL_PREEMPT_KEEPALIVE: a6xx_gmu.xml dword offset 0x1f8c4 in the GMU's own register window
 *   RBBM_INT_0_MASK bit 8 CP_SW: the switch finished
 * A record is 4192 KiB (the CP stores its state there) and the SMMU info page 4 KiB; a6xx_preempt.c allocates exactly these
 * per ring. A record's CPU-aware fields are the first 56 bytes (a6xx_preempt_record in a6xx_gpu.h).
 *
 * This object writes the trigger registers and judges the completion; the caller owns the memory (records, SMMU info pages,
 * ring and fence pages), fills a record through writeRecord() and an SMMU info page through writeSmmuInfo(), and gives the
 * write pointer again after a switch, because only it knows which ring the CP now runs. */
enum : uint32_t {NativeAdrenoPreemptRingCount = 2};

/* The GPU addresses of the per-ring state. */
struct NativeAdrenoPreemptRings {
    uint64_t record[NativeAdrenoPreemptRingCount];      /* the preemption record of each ring */
    uint64_t smmu_info[NativeAdrenoPreemptRingCount];   /* the a7xx_cp_smmu_info page of each ring */
    /* The CPU's own mappings of both, which the caller fills before each switch. */
    uint8_t *record_cpu[NativeAdrenoPreemptRingCount];
    uint8_t *smmu_cpu[NativeAdrenoPreemptRingCount];
};

struct NativeAdrenoPreemptIo {
    void *cookie;
    int (*write32)(void *, uint32_t reg, uint32_t value);
    int (*read32)(void *, uint32_t reg, uint32_t *value);
    int (*barrier)(void *);
    /* The GMU's window: the preemption keepalive vote lives there, not in the GPU's register window. Its register is a dword index,
     * as a6xx_gmu.xml writes it (0x1f8c4). */
    void *gmu_cookie;
    int (*gmu_write32)(void *, uint32_t dword_offset, uint32_t value);
};

struct NativeAdrenoPreemptStats {
    uint32_t triggers;        /* switches the CPU asked for */
    uint32_t completed;       /* switches whose STOP bit had cleared when the interrupt came */
    uint32_t skipped;         /* a higher-priority ring was waiting and no switch was possible */
    uint32_t faulted;         /* a switch that did not finish */
    uint32_t disjoint_skips;  /* switches not taken because the rings' buffers overlap */
    uint32_t restored_wptr;   /* write pointers given again after a switch */
    uint32_t last_us;         /* how long the last completed switch took */
};

class NativeAdrenoPreempt final {
    NativeAdrenoPreemptIo io_{};
    NativeAdrenoPreemptRings rings_{};
    uint32_t loaded_=0;   /* the ring the CP runs */
    uint32_t next_=0;     /* the ring a switch in flight goes to */
    bool armed_=false;    /* a switch was triggered and its STOP bit not yet seen cleared */
    bool attached_=false;
    uint64_t (*now_us_)(void *)=nullptr;
    void *clock_cookie_=nullptr;
    uint64_t trigger_us_=0;
    NativeAdrenoPreemptStats stats_{};
public:
    NativeAdrenoPreempt()=default;
    ~NativeAdrenoPreempt()=default;
    NativeAdrenoPreempt(const NativeAdrenoPreempt &)=delete;
    NativeAdrenoPreempt &operator=(const NativeAdrenoPreempt &)=delete;
    /* Programmes RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE and clears CP_CONTEXT_SWITCH_SMMU_INFO, as a6xx_preempt_hw_init
     * does, with `loaded` the ring the CP runs. The caller must already own a powered GPU with that ring up. `now` times the
     * switches (optional). */
    IOReturn attach(const NativeAdrenoPreemptIo *,const NativeAdrenoPreemptRings &,uint32_t loaded,uint64_t (*now)(void *),void *clock);
    IOReturn detach();
    bool attached() const {return attached_;}
    /* The CPU-aware fields of a record (a6xx_preempt_record, first 56 bytes): the magic the CP checks, the ring's address, its
     * read-pointer shadow and BV read-pointer shadow, its CP_RB_CNTL value and the write pointer the CP resumes at. */
    static bool writeRecord(uint8_t *record,uint64_t rbase,uint64_t rptr_addr,uint64_t bv_rptr_addr,uint32_t cntl,uint32_t wptr);
    /* An a7xx_cp_smmu_info page: its magic, the page table the ring's CP runs, the address-space id and context idr. */
    static bool writeSmmuInfo(uint8_t *page,uint64_t ttbr0,uint32_t asid,uint32_t context_idr);
    /* The write pointer a record resumes at (the CP writes it back after saving), for the caller to give again. */
    static uint32_t recordWptr(const uint8_t *record);
    /* Switch the CP to `ring`, which must not be the loaded one: its SMMU info and record go out first, then the control word
     * with STOP. `uses_gmem` asks for the GMEM save and restore. False where a switch is still in flight or a register refused. */
    bool trigger(uint32_t ring,bool uses_gmem);
    /* The interrupt's side: read CP_CONTEXT_SWITCH_CNTL; a cleared STOP bit completes the switch and makes its ring the loaded
     * one, a set one records a fault. False where nothing was in flight. */
    bool complete();
    bool inFlight() const {return armed_;}
    uint32_t loaded() const {return loaded_;}
    uint32_t target() const {return next_;}
    /* After a GPU reset: `ring` is the one the CP runs and nothing is in flight. */
    void restart(uint32_t ring) {loaded_=ring;armed_=false;}
    NativeAdrenoPreemptStats stats() const {return stats_;}
    void noteDisjointSkip() {stats_.disjoint_skips=stats_.disjoint_skips+1;}
    void noteSkipped() {stats_.skipped=stats_.skipped+1;}
    void noteRestoredWptr() {stats_.restored_wptr=stats_.restored_wptr+1;}
};
#endif
