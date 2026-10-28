/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_VM_H
#define NATIVE_ADRENO_VM_H
#include "NativeAdrenoSmmuMemory.h"
#include "ArmSmmu4KStreams.h"
#include "NativeAdrenoHandleIndex.h"
struct NativeAdrenoVmRegion {
    NativeAdrenoTablePages *tables;
    ArmSmmu4KLiveArena *arena;
    ArmSmmu4KLiveState *state;
};
struct NativeAdrenoVmResult {
    ArmSmmu4KStatus operation,quarantine;
    size_t completed_spans;
    bool complete;
};
/* Kernel-owned BO mapping transactions, not a userspace ABI or GPU scheduler.
 * Attach only under the actual bank/ASID/stream and powered-MMIO lease, with
 * fault IRQ handling installed and empty native4K arenas already active.
 * The real table allocations, context/streams and BO objects remain retained
 * by the owning driver. No implicit destructor cleanup; detach must succeed.
 * Serialize all methods, table updates and submission under the DRIVER gate.
 * ALL masters must stay genuinely quiesced through each map/unmap transaction.
 * Job counts are lifetime accounting; neither they nor these APIs are fences.
 * Each active region is audited, including the unmodified peer. Partial failure
 * keeps every old/new BO reference, poisons both arenas and attempts to FAULT
 * only the owned streams. Uncertain resources require independent real reset.
 * No VM may be advertised to apps until per-client isolation, submission
 * validation/protection, genuine IRQ fences and reset are implemented. */
/* Board hook for sampleWord(): reads one aligned word of physical memory through a fresh uncached mapping; 0 on success. */
extern "C" int (*native_adreno_dram_read32)(uint64_t physical,uint32_t *value);
/* The board's reader through the kernel's cacheable physical aperture (null on the host: the BO's own mapping is read). */
extern "C" int (*native_adreno_cached_read32)(uint64_t physical,uint32_t *value);
/* Board hook for auditRange(): [physical, physical+bytes), whole 64-byte lines inside one page, as the cacheable physical aperture and
 * DRAM (a fresh uncached mapping) read it. A line that differs is cleaned and invalidated and DRAM read again: when DRAM then holds the
 * cached copy the line was dirty (*dirty), when DRAM is unchanged it was a stale clean copy (*stale). 0 on success. */
extern "C" int (*native_adreno_line_audit)(uint64_t physical,size_t bytes,uint32_t *dirty,uint32_t *stale);
class NativeAdrenoVm final {
    struct Mapping {
        NativeAdrenoMemory *memory=nullptr;
        uint64_t handle=0,iova=0;
        size_t spans=0,mapped_spans=0;
        unsigned permissions=0,writeback=0,shareability=0,jobs=0;
    };
    Mapping *mappings_=nullptr;
    NativeAdrenoHandleIndex index_;   /* live handle -> its mappings_ slot */
    size_t free_hint_=0;   /* no free mappings_ slot below it */
    size_t high_=0;   /* no live mappings_ slot at or above it: the walks over every mapping stop there */
    size_t class_spans_[16]{};   /* ranges_ per alias class, so a map counts the other classes without a walk */
    size_t import_spans_[16]{};   /* the imported mappings' ranges per alias class */
    NativeAdrenoVmRegion regions_[2]{};
    ArmSmmu4KLiveVisibility visibility_[2]{};
    ArmSmmu4KIo io_{};
    ArmSmmu4KContextConfig *context_=nullptr;
    ArmSmmu4KStreamState *streams_=nullptr;
    ArmSmmu4KRange *ranges_=nullptr;
    /* The kit's physical-alias sort scratch (ArmSmmu4KLiveArena::alias_scratch) for this VM's regions: one entry per range, grown with
     * ranges_, as the smaller compared side never holds more ranges than the table. */
    uint32_t *alias_scratch_=nullptr;
    size_t capacity_=0,lower_count_=0,upper_count_=0,objects_=0;
    /* Both tables grow on demand with no limit but memory (a VM per GPU process: most use a few hundred objects; a fixed table filled up
     * at 1024, then 4096 mappings: int-237, i567). */
    size_t mapping_capacity_=0;
    bool growRanges(size_t needed);
    void freeTables();
    bool growMappings();
    unsigned budget_=0;
    uint64_t next_handle_=1;
    bool attached_=false,poisoned_=false,distrust_write_combined_=false,unusable_=false;
    uint64_t faults_=0,hangs_=0;
    bool fault_streams_=true;   /* a failure faults the bank's streams; off for a process table, whose streams are the kernel table's too */
    NativeAdrenoVmResult result_{};
    ArmSmmu4KLiveRegion region(unsigned) const;
    ArmSmmu4KStatus observe() const;
    /* The route check alone: the table walk is the update's own (it verifies every region before the first store and after the last), so
     * map/unmap pay a full extra audit only every 64th operation (int-139: 4 full walks of the 2 GiB aperture made every map 70 ms). */
    ArmSmmu4KStatus observeStreamsOnly() const;
    ArmSmmu4KStatus observeCheap();
    unsigned since_audit_=0;
    IOReturn fail(ArmSmmu4KStatus);
    Mapping *find(uint64_t);
    const Mapping *find(uint64_t) const;
    size_t rangeAt(uint64_t iova) const;
    void insert(size_t,unsigned);
    void erase(size_t,size_t,unsigned);
    IOReturn preflight(const NativeAdrenoMemory *,unsigned,uint64_t,unsigned,unsigned,unsigned,bool exclusive) const;
    bool exclusivePages(const NativeAdrenoMemory *,unsigned cls) const;
public:
    NativeAdrenoVm()=default;
    ~NativeAdrenoVm()=default;
    NativeAdrenoVm(const NativeAdrenoVm &)=delete;
    NativeAdrenoVm &operator=(const NativeAdrenoVm &)=delete;
    IOReturn attach(const ArmSmmu4KIo *,ArmSmmu4KContextConfig *,ArmSmmu4KStreamState *,
        const NativeAdrenoVmRegion *lower,const NativeAdrenoVmRegion *upper,
        size_t first_spans,unsigned poll_budget);
    /* Frees CPU metadata only, with no BOs remaining. Never releases roots,
     * disables a bank or returns the platform lease; board lifecycle owns it. */
    IOReturn detach();
    IOReturn findAddress(unsigned upper,size_t bytes,uint64_t alignment,uint64_t *iova) const;
    IOReturn map(NativeAdrenoMemory *,unsigned upper,uint64_t iova,
        unsigned permissions,unsigned writeback,unsigned shareability,uint64_t *handle);
    IOReturn unmap(uint64_t handle);
    IOReturn holdJob(uint64_t handle);
    /* TLBIALL for the context bank plus TLBSYNC: drops every cached translation of the bank whatever its ASID. Unmap already
     * synchronizes by ASID; this is the stronger form the submit path uses before each submission (int-070: on sheng the CP read
     * zeros from an IB whose table entry and DRAM were both correct, after the address had been mapped to other pages before). */
    IOReturn invalidateTlb();
    /* Cleans and invalidates the CPU caches over every mapped BO (whole objects; imported ones through their physical pages), so
     * no CPU cache line can hold data the GPU should see or hide what it wrote. Lines are physically tagged, so this also reaches
     * lines a user mapping of the same pages left behind. int-070 runs it before every submission. */
    IOReturn publishMapped();
    /* The submit path saw a CPU-cached word that DRAM did not hold yet in a client's BO: from now on write-combined BOs are maintained
     * like any other (sticky; the safe behaviour of int-070..097). */
    void distrustWriteCombined() {distrust_write_combined_=true;}
    bool writeCombinedTrusted() const {return !distrust_write_combined_;}
    /* After a submission completed: cleans and invalidates the CPU caches over every GPU-writable imported BO, so the owner's mapping
     * (an IOSurface) reads what the GPU wrote, not a line prefetched while the GPU ran. Jobs name only their IBs, so every such BO
     * is covered. int-071: an imported render target read back 672 of 4096 texels stale. */
    IOReturn acquireImported();
    /* Diagnostic, never a proof of coherence: the 32-bit word at a mapped GPU address as the kernel's cacheable alias reads it
     * and, when the board installed native_adreno_dram_read32, as DRAM reads it through an uncached mapping (else ~0). */
    IOReturn sampleWord(uint64_t iova,uint32_t *cached,uint32_t *dram) const;
    /* Every line of [iova, iova+bytes) that lies in a write-combined BO, through native_adreno_line_audit: *dirty counts lines a CPU
     * cache held newer than DRAM (a client view that caches after all; each such line is written back by the audit), *stale clean
     * copies, *lines the lines compared. kIOReturnUnsupported without a board hook. int-099 sampled one word and wrote its line back
     * before reading DRAM, so it could never see a dirty line. */
    /* Before a submission while write-combined BOs are trusted: every page of every mapped write-combined BO that XNU set back to the
     * default attribute is write-combined again (and its lines written back); *reset counts the pages. */
    IOReturn reassertWriteCombined(uint32_t *reset) const;
    IOReturn auditRange(uint64_t iova,size_t bytes,uint32_t *lines,uint32_t *dirty,uint32_t *stale) const;
    /* Read-only description of a live mapping; never pins or extends it. */
    IOReturn describe(uint64_t handle,uint64_t *iova,size_t *bytes,unsigned *permissions) const;
    /* Up to `count` dwords of a mapped kernel-owned BO at a GPU address (through the kernel's own mapping), for the board log. */
    IOReturn peekWords(uint64_t iova,uint32_t *out,size_t count) const;
    /* The same after cleaning and invalidating the CPU cache lines over the words: a BO zeroed through the kernel's cacheable mapping keeps
     * clean lines of those zeros there, which a plain read returns instead of what the client wrote through its own mapping since. */
    IOReturn peekWordsUncached(uint64_t iova,uint32_t *out,size_t count) const;
    /* The physical address of the leaf descriptor that maps the page at target (the table walk coherence proof); false when none does. */
    bool leafPhysical(uint64_t target,uint64_t *physical) const;
    /* The context bank's fault record for the board log: FSR, FAR (lo, hi) and FSYNR0. Reads only; never clears a fault. */
    IOReturn faultState(uint32_t out[4]) const;
    /* Map handles count up and are never reused, so they order mappings in time: the next one, and the one over a GPU address (0: none). */
    uint64_t nextHandle() const {return next_handle_;}
    uint64_t handleAt(uint64_t iova) const;
    /* Only a scheduler's actual successful GPU completion may call this.
     * A timeout, interrupt bit alone or CPU-written timestamp is insufficient. */
    IOReturn retireJobAfterCompletion(uint64_t handle);
    bool poisoned() const {return poisoned_;}
    /* A GPU hang this VM's submission caused. As Metal does after a GPU restart, only the hung command buffer is lost: the process keeps
     * its device, its queued submissions run again and new ones are taken (msm instead makes the VM unusable, and the client then
     * reports its device lost; when that client was the render server, backboardd ended and SpringBoard's restart stalled, untagged-1f4ba0a24).
     * hangs() counts them; faults(), which a client turns into a lost device, stays for a VM whose table failed. */
    void noteHang(bool) {++hangs_;}
    uint64_t hangs() const {return hangs_;}
    uint64_t faults() const {return faults_;}
    bool unusable() const {return unusable_;}
    /* A process address space: its failure poisons only itself (its tables and BOs), never the streams the kernel table shares. */
    void keepStreamsOnFailure() {fault_streams_=false;}
    /* The lower region's table root with this VM's ASID in bits 63:48: what the bank's TTBR0 holds while the GPU runs this VM. */
    uint64_t ttbr0() const {return regions_[0].state && context_?regions_[0].state->tables.root_physical|(uint64_t)context_->asid<<48:0;}
    unsigned asid() const {return context_?context_->asid:0;}
    /* TLBIASID of this VM's ASID (its translations only), for an ASID about to be given to another table. */
    IOReturn invalidateAsid();
    size_t objectCount() const {return objects_;}
    size_t spanCount() const {return lower_count_+upper_count_;}
    NativeAdrenoVmResult lastResult() const {return result_;}
};
#endif
