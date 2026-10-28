/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_MEMORY_H
#define NATIVE_ADRENO_MEMORY_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>
class IOBufferMemoryDescriptor;
class IOMemoryDescriptor;
struct NativeAdrenoPhysicalSpan {uint64_t offset,physical,bytes;};
struct NativeAdrenoCpuState {
    uint64_t tcr,ttbr0,ttbr1,sctlr;
    uint64_t kernel_page_bytes,kernel_map_page_bytes;
};
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *);
enum NativeAdrenoCacheOperation {NativeAdrenoClean,NativeAdrenoCleanInvalidate,NativeAdrenoInvalidate};
IOReturn NativeAdrenoMaintainOwnedPages(void *,size_t,NativeAdrenoCacheOperation);
/* The same over whole physical pages of DRAM, through the kernel's physical aperture: data cache lines are physically tagged, so this
 * reaches lines another task's mapping left (an imported IOSurface). Apple's sys_dcache_flush is only a DSB, so user space cannot. */
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t physical,size_t bytes,NativeAdrenoCacheOperation);
/* Every CPU mapping of the spans' pages becomes write-combined (true) or takes the default cacheable attribute again (false). */
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *,size_t count,bool write_combined);
/* int-100 found a write-combined BO's page back at the default (cacheable) attribute with dirty IB lines: XNU's view machinery sets a page's
 * attribute when it populates or tears down a device-pager view. Every page of the spans no longer write-combined is made so again (the pmap
 * then rewrites every mapping it tracks and writes the page's cache lines back); *reset counts them. */
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *,size_t count,uint32_t *reset);
/* Whether every page of the spans has a non-cacheable attribute in the pmap (write-combined, device, real-time): no CPU mapping of them can
 * hold a line, so the GPU needs no CPU cache maintenance around them. */
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t count);
/* CPU cache maintenance by the phase that asked for it, for the board log: [phase][0] passes, [phase][1] KiB. */
enum NativeAdrenoMaintenancePhase {NativeAdrenoPhasePool,NativeAdrenoPhaseAllocate,NativeAdrenoPhaseMap,NativeAdrenoPhaseSubmit,
    NativeAdrenoPhaseUnmap,NativeAdrenoPhaseComplete,NativeAdrenoPhaseTable,NativeAdrenoPhaseDiagnostic};
void NativeAdrenoNotePhase(unsigned phase,size_t bytes);

/* Native XNU backing store, not a Metal device or user ABI. No IOMMU mapper,
 * Linux fd/ioctl, hardware address, guessed PA or implicit destructor cleanup.
 * Driver owns this object until release() succeeds; do not destroy/copy it
 * with outstanding memory. Serialize every call under the queue/resource gate.
 * Every published mapping/job must hold a device reference BEFORE publication,
 * and retire it only after actual DMA quiescence and completed native TLB unmap.
 * References are lifetime accounting, never fences or proof of quiescence. */
class NativeAdrenoMemory final {
    IOBufferMemoryDescriptor *descriptor_=nullptr;
    IOMemoryDescriptor *imported_=nullptr;
    bool read_only_=false;
    /* An import whose every page the pmap holds non-cacheable (iPadOS 27's display surfaces are write-combined): nothing to maintain. */
    bool owner_uncached_=false;
    void *mapping_=nullptr;
    NativeAdrenoPhysicalSpan *spans_=nullptr;
    size_t bytes_=0,span_count_=0,span_capacity_=0;
    uint32_t device_references_=0;
    bool prepared_=false,quarantined_=false,write_combined_=false,pooled_=false,borrowed_=false;
    unsigned pool_chunk_=0;
    size_t pool_offset_=0;
    IOReturn maintainImported(size_t offset,size_t bytes,NativeAdrenoCacheOperation);
public:
    NativeAdrenoMemory()=default;
    ~NativeAdrenoMemory()=default;
    NativeAdrenoMemory(const NativeAdrenoMemory &)=delete;
    NativeAdrenoMemory &operator=(const NativeAdrenoMemory &)=delete;
    /* Explicit driver budget (the caller's limit: a client's GPU address window).
     * Error may retain quarantined allocation state if complete() fails.
     * Caller always keeps this object and checks quarantined(), even on error.
     * CPU native4K state and kernel page ABI are checked before allocation.
     * prepare(), raw unmapped physical segments, span/PA bounds and optional
     * physical contiguity are verified before zeroing/PoC publication. */
    /* `pooled`: take the bytes from the BO pool (a chunk with room, or a new chunk). Only user BOs do: the pool is write-combined for
     * the CPU (writeCombinePool), and the kernel's own objects (SMMU tables, ring, fence and firmware pages) are walked or read by the
     * CPU through their cacheable mapping (int-153: page tables from the write-combined pool made each map's table update 2.6x slower). */
    IOReturn allocate(size_t bytes,unsigned output_bits,bool contiguous,size_t byte_budget,bool pooled=false);
    /* The BO pool: physically contiguous chunks cut into BOs, so a BO is one span of the GPU's table instead of one per scattered page
     * (int-137: 59474 spans for 332 MiB, the span capacity ran out, BoMap failed and the compositor fell back to the CPU). Enables the
     * pool for pages below 2^output_bits and takes `bytes` (0: none) as a first chunk while memory is not yet fragmented; that chunk stays.
     * Every other chunk is taken from the system when no chunk has room (2 MiB, halved down to 64 KiB, or a large BO's own size) and given
     * back when it empties. allocate(..., pooled=true) falls back to scattered pages only when the system has no such chunk. Board only. */
    static IOReturn reservePool(unsigned output_bits,size_t bytes);
    static size_t poolBytes();
    /* Memory pressure: the system could not give the pool a chunk (the last BO fell back to scattered pages); cleared once the pool serves
     * an allocation again. Clients read it in Query and give their cached BOs back. */
    static bool poolPressure();
    /* Whether [physical, physical+bytes) touches a pool chunk: its pages belong to pooled BOs alone. */
    static bool poolContains(uint64_t physical,size_t bytes);
    /* Every chunk write-combined once, for good, and every later chunk when it is made: a pooled BO then takes no page-attribute change
     * on allocation or release. Board only, after reservePool, when user BOs are write-combined. */
    static IOReturn writeCombinePool();
    /* Pins bytes at address in task (a user task's own pages) instead of allocating; prepare() wires them until release().
     * There is no kernel mapping: the owner keeps its CPU mapping and does its own cache maintenance around GPU use, so
     * publishForDevice/acquireDeviceWrites have nothing to do for this object. Spans and PA bounds are checked as for allocate. */
    IOReturn importTask(void *task,uint64_t address,size_t bytes,unsigned output_bits,size_t byte_budget);
    /* Takes over a contiguous block another object owns (its kernel mapping, physical address and size), zeroed here; release() only
     * forgets it. The process address spaces' page tables come from such blocks, reserved cacheable at bring-up. */
    IOReturn borrow(void *mapping,uint64_t physical,size_t bytes);
    bool imported() const {return imported_!=nullptr;}
    bool pooled() const {return pooled_;}
    /* An import of memory the owner maps read-only (the kernel could pin it for reading only): the GPU may read it, never write it. */
    bool readOnly() const {return read_only_;}
    bool ownerUncached() const {return owner_uncached_;}
    /* After allocate(), before any device or client mapping: cleans and invalidates the pages and makes every CPU mapping of them
     * write-combined, so publishForDevice/acquireDeviceWrites have nothing left to do. release() restores the default attribute. */
    IOReturn makeWriteCombined();
    bool writeCombined() const {return write_combined_;}
    /* NativeAdrenoReassertWriteCombined over this object's pages; nothing for an object that is not write-combined, nor for one from the
     * BO pool: XNU set scattered BOs' pages back to the default attribute (int-160: 123847 pages), never a pool page (int-161: none in
     * 1280 submissions, while walking the pool's pages took 0.65 ms of each). The submissions' line audit still covers every object. */
    IOReturn reassertWriteCombined(uint32_t *reset) const
    {return write_combined_ && span_count_ && !pooled_?NativeAdrenoReassertWriteCombined(spans_,span_count_,reset):kIOReturnSuccess;}
    bool backed() const {return descriptor_ || imported_ || pooled_ || borrowed_;}
    /* Refuses device references/quarantine; only clears ownership after real
     * descriptor complete() succeeds. No automatic reset or resource recycling. */
    IOReturn release();
    IOReturn holdForDevice();
    IOReturn retireAfterQuiescence();
    void quarantine() {quarantined_=true;}
    /* Requires real CPU/device exclusion and retained ownership. Before GPU
     * writes, clean+invalidate; after genuine completion, invalidate only.
     * CPU must not dirty these pages while the device owns access. */
    /* A write-combined object is skipped unless include_write_combined: then its pages are maintained through the physical aperture,
     * for a client whose view turned out to be cacheable after all (NativeAdrenoVm::distrustWriteCombined). */
    IOReturn publishForDevice(size_t offset,size_t bytes,bool device_writes,bool include_write_combined=false,unsigned phase=NativeAdrenoPhaseMap);
    IOReturn acquireDeviceWrites(size_t offset,size_t bytes,unsigned phase=NativeAdrenoPhaseUnmap);
    bool quarantined() const {return quarantined_;}
    uint32_t deviceReferences() const {return device_references_;}
    size_t bytes() const {return bytes_;}
    size_t spanCount() const {return span_count_;}
    const NativeAdrenoPhysicalSpan *spans() const {return spans_;}
    void *mapping() const {return mapping_;}
    /* Borrowed descriptor, never release independently or expose table pools
     * through clientMemoryForType. Sharing requires a separate owner/task ABI. */
    IOBufferMemoryDescriptor *descriptor() const {return descriptor_;}
};
#endif
