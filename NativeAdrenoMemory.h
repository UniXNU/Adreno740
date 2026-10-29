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
enum NativeAdrenoMaintenancePhase {NativeAdrenoPhaseUnused,NativeAdrenoPhaseAllocate,NativeAdrenoPhaseMap,NativeAdrenoPhaseSubmit,
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
    bool prepared_=false,quarantined_=false,write_combined_=false,system_=false,borrowed_=false,owned_=false;
    IOReturn takeSegments(IOBufferMemoryDescriptor *,size_t bytes,uint64_t limit,bool contiguous);
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
    IOReturn allocate(size_t bytes,unsigned output_bits,bool contiguous,size_t byte_budget);
    /* A user BO's memory from the system, as Metal's resources on iPadOS have it (pageable VM memory the GPU driver wires: a resource can be
     * made purgeable, which only VM objects can be): an IOBufferMemoryDescriptor of pageable memory with no kernel mapping, wired by
     * prepare(). The VM gives zero-filled pages and, for write-combined memory, sets the pages' attribute while it wires them, in one batch
     * (vm_object_iopl_request, pmap_batch_set_cache_attributes, which also cleans the pages' lines), and gives the pages back with the
     * default attribute (vm_object_reap). No kernel address range, no physical contiguity search, no fixed pool: the BO pool of 1536 MiB
     * (untagged-ed1611b1) filled with the runtime's 4-64 MiB blocks, 404 BOs fell back to scattered kernel memory, 96 allocations of
     * 96 MiB failed and the pressure it reported stayed on, so the clients' BO caches flushed and allocated again (5.6 GB of maintenance).
     * Client views map this descriptor itself (the VM object and its attribute), not its physical pages. */
    IOReturn allocateSystem(size_t bytes,unsigned output_bits,size_t byte_budget,bool write_combined);
    /* Memory pressure: the system could not give a user BO its memory (allocateSystem failed); cleared by the next one that succeeds.
     * Clients read it in Query and give their cached BOs back. */
    static bool memoryPressure();
    /* Whether any page of the spans belongs to a BO this driver allocated (allocate, allocateSystem) and has not released. */
    static bool ownsAnyPage(const NativeAdrenoPhysicalSpan *,size_t count);
    /* Pins bytes at address in task (a user task's own pages) instead of allocating; prepare() wires them until release().
     * There is no kernel mapping: the owner keeps its CPU mapping and does its own cache maintenance around GPU use, so
     * publishForDevice/acquireDeviceWrites have nothing to do for this object. Spans and PA bounds are checked as for allocate. */
    IOReturn importTask(void *task,uint64_t address,size_t bytes,unsigned output_bits,size_t byte_budget);
    /* Takes over a contiguous block another object owns (its kernel mapping, physical address and size), zeroed here; release() only
     * forgets it. The process address spaces' page tables come from such blocks, reserved cacheable at bring-up. */
    IOReturn borrow(void *mapping,uint64_t physical,size_t bytes);
    bool imported() const {return imported_!=nullptr;}
    /* Memory from allocateSystem: the VM owns its pages' attribute, and its client views map the descriptor. */
    bool system() const {return system_;}
    /* An import of memory the owner maps read-only (the kernel could pin it for reading only): the GPU may read it, never write it. */
    bool readOnly() const {return read_only_;}
    bool ownerUncached() const {return owner_uncached_;}
    /* After allocate(), before any device or client mapping: cleans and invalidates the pages and makes every CPU mapping of them
     * write-combined, so publishForDevice/acquireDeviceWrites have nothing left to do. release() restores the default attribute. */
    IOReturn makeWriteCombined();
    bool writeCombined() const {return write_combined_;}
    /* NativeAdrenoReassertWriteCombined over this object's pages; nothing for an object that is not write-combined, nor for system memory:
     * XNU set the pages of BOs whose views mapped their physical pages back to the default attribute as it tore a device-pager view down
     * (int-160: 123847 pages; untagged-2319cf3e5: 685208 pages, 3.8 s of submissions), while a view of a VM object maps it with the
     * object's own attribute. The submissions' line audit still covers every object. */
    IOReturn reassertWriteCombined(uint32_t *reset) const
    {return write_combined_ && span_count_ && !system_?NativeAdrenoReassertWriteCombined(spans_,span_count_,reset):kIOReturnSuccess;}
    bool backed() const {return descriptor_ || imported_ || borrowed_;}
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
