/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_MEMORY_H
#define NATIVE_ADRENO_MEMORY_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>
class IOBufferMemoryDescriptor;
struct NativeAdrenoPoolGrant;
/* System memory prepared outside the driver's lock (NativeAdrenoMemory::prepareSystem): a wired descriptor of the VM's pages, or a
 * grant of zero-filled pool pages (NativeAdrenoPool.h), never both. */
/* made, wired: when the descriptor was made and when its pages were wired (mach units), for the allocation trace. */
struct NativeAdrenoSystemPages {IOBufferMemoryDescriptor *descriptor=nullptr;size_t bytes=0;bool write_combined=false,cleaned=false;
    uint64_t made=0,wired=0;NativeAdrenoPoolGrant *grant=nullptr;
    bool held() const {return descriptor || grant;}};
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
/* NativeAdrenoPhaseWrite: a writer of the driver's own memory cleaning what it stored through a cacheable mapping. */
enum NativeAdrenoMaintenancePhase {NativeAdrenoPhaseWrite,NativeAdrenoPhaseAllocate,NativeAdrenoPhaseMap,NativeAdrenoPhaseSubmit,
    NativeAdrenoPhaseUnmap,NativeAdrenoPhaseComplete,NativeAdrenoPhaseTable,NativeAdrenoPhaseDiagnostic};
void NativeAdrenoNotePhase(unsigned phase,size_t bytes);
/* One holder's change in a table of 24 rows {pid, live KiB, live objects, peak KiB} (native_adreno_bo_owners and the tables shaped like
 * it): bytes > 0 adds an object of that size to pid's row, bytes < 0 takes one away. The 24 largest holders are kept; a process that
 * holds nothing gives its row up. The caller holds the driver's lock. */
void NativeAdrenoNoteHolder(volatile uint32_t (*table)[4],int32_t pid,int64_t bytes);

/* Native XNU backing store, not a Metal device or user ABI. No IOMMU mapper,
 * Linux fd/ioctl, hardware address, guessed PA or implicit destructor cleanup.
 * Driver owns this object until release() succeeds; do not destroy/copy it
 * with outstanding memory. Serialize every call under the queue/resource gate.
 * Every published mapping/job must hold a device reference BEFORE publication,
 * and retire it only after actual DMA quiescence and completed native TLB unmap.
 * References are lifetime accounting, never fences or proof of quiescence. */
class NativeAdrenoMemory final {
    IOBufferMemoryDescriptor *descriptor_=nullptr;
    /* A system BO of pool pages (NativeAdrenoPool.h) has this grant and no descriptor: pages outside the kernel's DRAM, which only
     * the SMMU tables and non-cacheable CPU mappings name. No CPU cache holds a line of them, so nothing is ever maintained. */
    NativeAdrenoPoolGrant *grant_=nullptr;
    IOMemoryDescriptor *imported_=nullptr;
    bool read_only_=false;
    /* An import whose every page the pmap holds non-cacheable (iPadOS 27's display surfaces are write-combined): nothing to maintain. */
    bool owner_uncached_=false;
    void *mapping_=nullptr;
    NativeAdrenoPhysicalSpan *spans_=nullptr;
    size_t bytes_=0,span_count_=0,span_capacity_=0;
    uint32_t device_references_=0;
    int32_t owner_pid_=0;   /* the process that allocated a system BO or made an import, for the holders' tables */
    bool prepared_=false,quarantined_=false,write_combined_=false,system_=false,borrowed_=false,owned_=false;
    bool import_noted_=false;   /* an import counted in native_adreno_import_memory until release() */
    bool synchronized_=false;   /* declareSynchronized(): its writers keep the device's view current themselves */
    IOReturn takeSegments(IOBufferMemoryDescriptor *,size_t bytes,uint64_t limit,bool contiguous);
    IOReturn takeRuns(const NativeAdrenoPoolGrant *,size_t bytes,uint64_t limit);
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
     * Client views map this descriptor itself (the VM object and its attribute), not its physical pages.
     * A write-combined BO is first asked of the pool of DRAM outside the kernel's (NativeAdrenoPool.h), when the board registered
     * one: such a BO has a grant instead of a descriptor, costs the VM no page, and its client views map its physical runs. Whatever
     * the pool does not serve takes the descriptor path above. */
    IOReturn allocateSystem(size_t bytes,unsigned output_bits,size_t byte_budget,bool write_combined,
        struct NativeAdrenoSystemPages *prepared=nullptr);
    /* allocateSystem's slow part, which needs no driver state: the descriptor made and wired (the VM's zero fill and attribute) and, for
     * default-attribute pages, the zero fill written back. The caller does it before taking the driver's lock (i14aef633e: 5454 BoAllocs
     * held it 4.0 s, every other process's submit waiting); allocateSystem adopts it, and discardSystem gives back one it did not take. */
    /* output_bits: the client's physical address width, for pool pages (0: the descriptor path only). */
    static IOReturn prepareSystem(size_t bytes,bool write_combined,struct NativeAdrenoSystemPages *out,unsigned output_bits=0);
    static void discardSystem(struct NativeAdrenoSystemPages *);
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
    /* The grant of a system BO made of pool pages; null for any other memory. A client view of it describes the grant's runs and
     * holds a reference of its own (the pages have no VM object that could keep them for the view). */
    NativeAdrenoPoolGrant *poolGrant() const {return grant_;}
    /* An import of memory the owner maps read-only (the kernel could pin it for reading only): the GPU may read it, never write it. */
    bool readOnly() const {return read_only_;}
    bool ownerUncached() const {return owner_uncached_;}
    /* After allocate(), before any device or client mapping: cleans and invalidates the pages and makes every CPU mapping of them
     * write-combined, so publishForDevice/acquireDeviceWrites have nothing left to do. release() restores the default attribute. */
    IOReturn makeWriteCombined();
    bool writeCombined() const {return write_combined_;}
    /* The driver's own kernel-mapped buffer whose users keep the device's view current themselves, as a driver of a non-coherent device
     * does with dma_sync_single_for_device and dma_sync_single_for_cpu: whoever stores into it through a cacheable mapping cleans what
     * it stored before the device can read it (publishForDevice), whoever loads what the device wrote invalidates first
     * (acquireDeviceWrites), and a user with a non-cacheable view needs neither. A dirty line can then exist only between a store and
     * its writer's own clean, so the passes over every mapping (at a map, an unmap, a submission) have nothing to do for it and leave it
     * out. Called after allocate() and before any mapping: the zero fill is cleaned and dropped here, so no line of the buffer is in a
     * CPU cache when its users take it over. */
    IOReturn declareSynchronized();
    bool synchronized() const {return synchronized_;}
    /* Whether a CPU cache can hold a line of this memory that the device has not seen: not for an import whose owner maps it
     * non-cacheable, and not for write-combined memory unless a view of it was found caching (include_write_combined). The passes that
     * decide without calling publishForDevice use the same answer. Never for pool pages: the kernel has no cacheable mapping of them
     * to maintain them through, and the pool drops whatever a client's own view cached before the pages change hands. */
    bool holdsCpuLines(bool include_write_combined) const
    {return !grant_ && !(imported_ && owner_uncached_) && !(write_combined_ && !include_write_combined);}
    /* Whether reassertWriteCombined has pages to look at. */
    bool reassertable() const {return write_combined_ && span_count_ && !system_;}
    /* NativeAdrenoReassertWriteCombined over this object's pages; nothing for an object that is not write-combined, nor for system memory:
     * XNU set the pages of BOs whose views mapped their physical pages back to the default attribute as it tore a device-pager view down
     * (int-160: 123847 pages; untagged-2319cf3e5: 685208 pages, 3.8 s of submissions), while a view of a VM object maps it with the
     * object's own attribute. The submissions' line audit still covers every object. */
    IOReturn reassertWriteCombined(uint32_t *reset) const
    {return reassertable()?NativeAdrenoReassertWriteCombined(spans_,span_count_,reset):kIOReturnSuccess;}
    bool backed() const {return descriptor_ || grant_ || imported_ || borrowed_;}
    /* Refuses device references/quarantine; only clears ownership after real
     * descriptor complete() succeeds. No automatic reset or resource recycling. */
    IOReturn release();
    /* release() for a BO of system memory whose pages may go after the driver's lock: its descriptor (still wired) or its pool grant
     * is handed to out, for discardSystem to unwire and free, or to give back to the pool, once the lock is let go. Any other BO, or
     * an out that holds pages, is released here. A quarantined BO is refused before either: its pool pages stay out of the pool. */
    IOReturn releaseSystemInto(struct NativeAdrenoSystemPages *out);
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
