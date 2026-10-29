/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoMemory.h"
/* Where a call that never returns is stuck: the free/unmap/release path stores a stage number here, the board page shows it. */
extern "C" {volatile uint32_t native_adreno_trace_stage;}
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <kern/task.h>

static bool outputWidth(unsigned bits)
{return bits==32 || bits==36 || bits==40 || bits==42 || bits==44 || bits==48;}
/* Where the last import stopped (10..19, 19 = done), and for a refused page the physical address and offset: the board log shows them. */
enum : IOReturn {kNativeAdrenoProtectionFailure=2};   /* KERN_PROTECTION_FAILURE, as prepare() reports a range that cannot be wired for writing */
extern "C" {volatile uint32_t native_adreno_import_stage;volatile uint64_t native_adreno_import_fail_pa,native_adreno_import_fail_offset;}
/* BO memory for the board log: [0] user BOs from the system (allocateSystem), [1] their spans, [2] refused, [3] the last refusal's status,
 * [4] MiB of them live, [5] the peak, [6] kernel-mapped BOs (allocate), [7] import spans. */
extern "C" {volatile uint32_t native_adreno_bo_memory[8];}
namespace {
uint64_t systemBytes,systemPeak;
bool pressure;
void noteSystem(int64_t bytes)
{
    systemBytes+=bytes;if(systemBytes>systemPeak)systemPeak=systemBytes;
    native_adreno_bo_memory[4]=(uint32_t)(systemBytes>>20);native_adreno_bo_memory[5]=(uint32_t)(systemPeak>>20);
}
}
bool NativeAdrenoMemory::memoryPressure() {return pressure;}
/* Which DRAM pages this driver's own BOs hold, a bit a page over the kernel's physical range: an import (another task's pages) aliases
 * one of them only where a bit is set, so proving an import exclusive costs its own pages, not the ranges mapped so far (every BO is
 * scattered pages: a pairwise check over them ran out of budget on each import). Made on first use; without it every page counts as
 * owned (the full alias check). */
extern "C" uint64_t gPhysBase,gPhysSize;
namespace {
uint64_t *ownedBits;
uint64_t *ownedMap()
{
    static bool tried;
    if(!ownedBits && !tried) {tried=true;ownedBits=static_cast<uint64_t *>(IOMallocZero((size_t)((gPhysSize/4096+63)/64*8)));}
    return ownedBits;
}
void noteOwned(const NativeAdrenoPhysicalSpan *spans,size_t count,bool owned)
{
    uint64_t *bits=ownedMap();
    if(!bits)return;
    for(size_t i=0;i<count;++i)for(uint64_t page=spans[i].physical;page<spans[i].physical+spans[i].bytes;page+=4096) {
        if(page<gPhysBase || page-gPhysBase>=gPhysSize)continue;
        const uint64_t at=(page-gPhysBase)/4096,bit=UINT64_C(1)<<(at%64);
        if(owned)__atomic_fetch_or(&bits[at/64],bit,__ATOMIC_RELAXED);else __atomic_fetch_and(&bits[at/64],~bit,__ATOMIC_RELAXED);
    }
}
}
bool NativeAdrenoMemory::ownsAnyPage(const NativeAdrenoPhysicalSpan *spans,size_t count)
{
    const uint64_t *bits=ownedMap();
    if(!bits)return true;
    for(size_t i=0;i<count;++i)for(uint64_t page=spans[i].physical;page<spans[i].physical+spans[i].bytes;page+=4096) {
        if(page<gPhysBase || page-gPhysBase>=gPhysSize)continue;
        const uint64_t at=(page-gPhysBase)/4096;
        if(__atomic_load_n(&bits[at/64],__ATOMIC_RELAXED)>>(at%64)&1)return true;
    }
    return false;
}
/* The descriptor's pages as spans (neighbours merged), each below `limit` and, for a contiguous allocation, following the first. */
IOReturn NativeAdrenoMemory::takeSegments(IOBufferMemoryDescriptor *descriptor,size_t bytes,uint64_t limit,bool contiguous)
{
    span_capacity_=bytes/4096;
    spans_=IONewData(NativeAdrenoPhysicalSpan,span_capacity_);
    if(!spans_)return kIOReturnNoMemory;
    for(size_t offset=0;offset<bytes;offset+=4096) {
        IOByteCount length=0;
        addr64_t physical=descriptor->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
        if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical)return kIOReturnUnsupported;
        if(contiguous && offset && physical!=spans_[0].physical+offset)return kIOReturnUnsupported;
        if(span_count_ && physical==spans_[span_count_-1].physical+spans_[span_count_-1].bytes)
            spans_[span_count_-1].bytes+=4096;
        else spans_[span_count_++]={offset,physical,4096};
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::allocate(size_t bytes,unsigned bits,bool contiguous,size_t budget)
{
    if(descriptor_ || spans_ || quarantined_)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits) || !budget)
        return kIOReturnBadArgument;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    uint64_t limit=UINT64_C(1)<<bits;
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone);
    if(contiguous)options|=kIOMemoryPhysicallyContiguous;
    descriptor_=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,bytes,(limit-1)&~UINT64_C(4095));
    if(!descriptor_)return kIOReturnNoMemory;
    status=descriptor_->prepare();
    if(status!=kIOReturnSuccess) {descriptor_->release();descriptor_=nullptr;return status;}
    prepared_=true;bytes_=bytes;mapping_=descriptor_->getBytesNoCopy();
    status=kIOReturnUnsupported;
    if(!mapping_ || ((uintptr_t)mapping_&4095) || bytes>UINTPTR_MAX-(uintptr_t)mapping_ ||
        descriptor_->getLength()!=bytes)goto failed;
    status=takeSegments(descriptor_,bytes,limit,contiguous);
    if(status!=kIOReturnSuccess)goto failed;
    bzero(mapping_,bytes);
    status=NativeAdrenoMaintainOwnedPages(mapping_,bytes,NativeAdrenoClean);
    if(status!=kIOReturnSuccess)goto failed;
    NativeAdrenoNotePhase(NativeAdrenoPhaseAllocate,bytes);
    noteOwned(spans_,span_count_,true);owned_=true;
    native_adreno_bo_memory[6]=native_adreno_bo_memory[6]+1;
    return kIOReturnSuccess;
failed:
    /* release() deliberately retains descriptor/metadata if complete fails.
     * Preserve the allocation's original failure instead of reporting cleanup
     * as success. No object has been exposed to a GPU or userspace yet. */
    (void)release();return status;
}
IOReturn NativeAdrenoMemory::allocateSystem(size_t bytes,unsigned bits,size_t budget,bool write_combined)
{
    if(descriptor_ || spans_ || quarantined_)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits) || !budget)return kIOReturnBadArgument;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    /* No task: the memory is mapped nowhere until a client view maps it. The cache mode is the memory entry's, so every mapping of it
     * (the client views) and the pages themselves take it. */
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|
        static_cast<IOOptionBits>(kIOMemoryPageable);
    if(write_combined)options|=kIOMapWriteCombineCache;
    descriptor_=IOBufferMemoryDescriptor::inTaskWithOptions(nullptr,options,bytes,4096);
    status=kIOReturnNoMemory;
    if(descriptor_) {
        system_=true;
        status=descriptor_->prepare();
        if(status!=kIOReturnSuccess) {descriptor_->release();descriptor_=nullptr;system_=false;}
    }
    if(status==kIOReturnSuccess) {
        prepared_=true;bytes_=bytes;write_combined_=write_combined;noteSystem((int64_t)bytes);
        status=descriptor_->getLength()==bytes?takeSegments(descriptor_,bytes,UINT64_C(1)<<bits,false):kIOReturnUnsupported;
        /* Default-attribute pages may still hold the zero fill in this CPU's cache: written back before the GPU reads them. */
        if(status==kIOReturnSuccess && !write_combined) {
            status=maintainImported(0,bytes,NativeAdrenoClean);
            NativeAdrenoNotePhase(NativeAdrenoPhaseAllocate,bytes);
        }
        if(status!=kIOReturnSuccess) {write_combined_=false;(void)release();}
    }
    if(status!=kIOReturnSuccess) {
        pressure=true;
        native_adreno_bo_memory[2]=native_adreno_bo_memory[2]+1;native_adreno_bo_memory[3]=(uint32_t)status;
        return status;
    }
    pressure=false;
    noteOwned(spans_,span_count_,true);owned_=true;
    native_adreno_bo_memory[0]=native_adreno_bo_memory[0]+1;native_adreno_bo_memory[1]=native_adreno_bo_memory[1]+(uint32_t)span_count_;
    return kIOReturnSuccess;
}
/* An import holds the pages through a memory entry of its own (kIOMemoryPersistent), which references the VM object: the owner may unmap
 * its range while the GPU still uses it (Core Animation frees an image's Shmem once its no-copy buffer is released, before the runtime frees
 * the import), and the object then lives on until the import completes instead of being torn down under a wired UPL (int-233..235: the render
 * thread stuck in vm_deallocate). A range no memory entry can be made of is imported as before; [0] counts the persistent imports, [1] those. */
extern "C" {volatile uint32_t native_adreno_import_persistent[2];}
/* Imports whose pages were all non-cacheable ([1]: no CPU maintenance for them) or not ([0]). */
extern "C" {volatile uint32_t native_adreno_pool_stats_uncached[2];}
static IOMemoryDescriptor *importRange(uint64_t address,size_t bytes,IODirection direction,void *task)
{
    const IOOptionBits options=static_cast<IOOptionBits>(direction)|static_cast<IOOptionBits>(kIOMemoryMapperNone);
    IOMemoryDescriptor *range=IOMemoryDescriptor::withAddressRange(address,bytes,options|kIOMemoryPersistent,static_cast<task_t>(task));
    if(range) {native_adreno_import_persistent[0]=native_adreno_import_persistent[0]+1;return range;}
    range=IOMemoryDescriptor::withAddressRange(address,bytes,options,static_cast<task_t>(task));
    if(range)native_adreno_import_persistent[1]=native_adreno_import_persistent[1]+1;
    return range;
}
IOReturn NativeAdrenoMemory::importTask(void *task,uint64_t address,size_t bytes,unsigned bits,size_t budget)
{
    native_adreno_import_stage=10;
    if(descriptor_ || imported_ || spans_ || quarantined_)return kIOReturnBusy;
    native_adreno_import_stage=11;
    if(!task || !address || (address&4095) || !bytes || (bytes&4095) || address>UINT64_MAX-bytes || !outputWidth(bits) ||
        !budget)return kIOReturnBadArgument;
    native_adreno_import_stage=12;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    native_adreno_import_stage=13;
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    const uint64_t limit=UINT64_C(1)<<bits;
    imported_=importRange(address,bytes,kIODirectionInOut,task);
    native_adreno_import_stage=14;
    if(!imported_)return kIOReturnNoMemory;
    native_adreno_import_stage=15;
    status=imported_->prepare();
    if(status==kNativeAdrenoProtectionFailure) {
        /* Read-only memory of the owner (a mapped file, constant data): cannot be pinned for writing. Pin it for reading only; the GPU then maps it
         * read-only. Metal's no-copy buffers over such memory are only ever read by shaders. */
        imported_->release();imported_=nullptr;
        imported_=importRange(address,bytes,kIODirectionOut,task);
        if(!imported_)return kIOReturnNoMemory;
        status=imported_->prepare();
        if(status==kIOReturnSuccess)read_only_=true;
    }
    if(status!=kIOReturnSuccess) {imported_->release();imported_=nullptr;return status;}
    prepared_=true;bytes_=bytes;
    status=kIOReturnUnsupported;
    native_adreno_import_stage=16;
    if(imported_->getLength()!=bytes)goto failed;
    native_adreno_import_stage=17;
    span_capacity_=bytes/4096;
    spans_=IONewData(NativeAdrenoPhysicalSpan,span_capacity_);
    if(!spans_) {status=kIOReturnNoMemory;goto failed;}
    for(size_t offset=0;offset<bytes;offset+=4096) {
        IOByteCount length=0;
        addr64_t physical=imported_->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
        if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical)
            {native_adreno_import_stage=18;native_adreno_import_fail_pa=physical;native_adreno_import_fail_offset=offset;
             status=kIOReturnUnsupported;goto failed;}
        if(span_count_ && physical==spans_[span_count_-1].physical+spans_[span_count_-1].bytes)
            spans_[span_count_-1].bytes+=4096;
        else spans_[span_count_++]={offset,physical,4096};
    }
    native_adreno_import_stage=19;
    owner_uncached_=NativeAdrenoPagesUncached(spans_,span_count_);
    native_adreno_pool_stats_uncached[owner_uncached_]=native_adreno_pool_stats_uncached[owner_uncached_]+1;
    native_adreno_bo_memory[7]=native_adreno_bo_memory[7]+(uint32_t)span_count_;
    return kIOReturnSuccess;
failed:
    (void)release();return status;
}
IOReturn NativeAdrenoMemory::borrow(void *mapping,uint64_t physical,size_t bytes)
{
    if(descriptor_ || imported_ || spans_ || quarantined_ || borrowed_)return kIOReturnBusy;
    if(!mapping || ((uintptr_t)mapping&4095) || !physical || (physical&4095) || !bytes || (bytes&4095))return kIOReturnBadArgument;
    spans_=IONewData(NativeAdrenoPhysicalSpan,1);
    if(!spans_)return kIOReturnNoMemory;
    span_capacity_=1;span_count_=1;spans_[0]={0,physical,bytes};
    mapping_=mapping;bytes_=bytes;prepared_=true;borrowed_=true;
    bzero(mapping_,bytes);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::makeWriteCombined()
{
    if(!descriptor_ || system_ || imported_ || !prepared_ || quarantined_ || device_references_ || write_combined_ || !mapping_ || !span_count_)
        return kIOReturnNotReady;
    IOReturn status=NativeAdrenoMaintainOwnedPages(mapping_,bytes_,NativeAdrenoCleanInvalidate);
    if(status)return status;
    NativeAdrenoNotePhase(NativeAdrenoPhaseAllocate,bytes_);
    status=NativeAdrenoSetPagesCacheMode(spans_,span_count_,true);
    if(status)return status;
    write_combined_=true;return kIOReturnSuccess;
}
/* Frees of an owned BO whose first pages a user task still maps: [0] count, [1] the last mapping's page number. */
extern "C" {volatile uint32_t native_adreno_free_user_mapped[2];}
extern "C" unsigned int native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) __attribute__((weak_import));
IOReturn NativeAdrenoMemory::release()
{
    native_adreno_trace_stage=30;
    if(device_references_ || quarantined_)return kIOReturnBusy;
    /* System memory's pages go back to the VM with the default attribute (vm_object_reap), as they came. */
    if(system_)write_combined_=false;
    if(write_combined_) {
        /* The pages go back to the VM cacheable, as they came. */
        const IOReturn status=NativeAdrenoSetPagesCacheMode(spans_,span_count_,false);
        if(status) {quarantined_=true;return status;}
        write_combined_=false;
    }
    if(imported_) {
        if(prepared_) {
            IOReturn status=imported_->complete();
            if(status!=kIOReturnSuccess) {quarantined_=true;return status;}
            prepared_=false;
        }
        imported_->release();imported_=nullptr;
    }
    if(descriptor_ && !system_ && native_pmap_user_mappings && spans_) {
        /* A page a user task still maps (a BO view over physical pages holds no reference to them) is counted before the pages are freed:
         * the first page of up to 64 spans, then which process maps it. XNU frees such a page under the mapping (pmap_assert_free). A view
         * of system memory maps its VM object, which keeps the pages until the view goes. */
        for(size_t i=0;i<span_count_ && i<64;++i) {
            char name[17]={};void *pmap=nullptr;uint64_t va=0;
            if(native_pmap_user_mappings((ppnum_t)(spans_[i].physical>>12),&pmap,&va,name)) {
                native_adreno_free_user_mapped[0]=native_adreno_free_user_mapped[0]+1;
                native_adreno_free_user_mapped[1]=(uint32_t)(va>>12);
                break;
            }
        }
    }
    if(descriptor_) {
        if(prepared_) {
            native_adreno_trace_stage=31;
            IOReturn status=descriptor_->complete();
            native_adreno_trace_stage=32;
            if(status!=kIOReturnSuccess) {quarantined_=true;return status;}
            prepared_=false;
        }
        native_adreno_trace_stage=33;
        descriptor_->release();descriptor_=nullptr;
        native_adreno_trace_stage=34;
    }
    if(owned_) {noteOwned(spans_,span_count_,false);owned_=false;}
    if(system_) {noteSystem(-(int64_t)bytes_);system_=false;}
    if(borrowed_) {borrowed_=false;prepared_=false;}
    if(spans_)IODeleteData(spans_,NativeAdrenoPhysicalSpan,span_capacity_);
    mapping_=nullptr;spans_=nullptr;bytes_=span_count_=span_capacity_=0;read_only_=false;owner_uncached_=false;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::holdForDevice()
{
    if(!backed() || !prepared_ || !span_count_ || quarantined_)return kIOReturnNotReady;
    if(device_references_==UINT32_MAX)return kIOReturnNoResources;
    ++device_references_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::retireAfterQuiescence()
{
    if(!device_references_)return kIOReturnBadArgument;
    if(quarantined_)return kIOReturnNotReady;
    --device_references_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::publishForDevice(size_t offset,size_t bytes,bool writes,bool include_write_combined,unsigned phase)
{
    if(!backed() || !prepared_ || quarantined_)return kIOReturnNotReady;
    if(!bytes || ((offset|bytes)&4095) || offset>bytes_ || bytes>bytes_-offset)return kIOReturnBadArgument;
    /* Pages every CPU mapping holds non-cacheable carry no line to clean. */
    if(imported_ && owner_uncached_)return kIOReturnSuccess;
    if(imported_) {NativeAdrenoNotePhase(phase,bytes);return maintainImported(offset,bytes,writes?NativeAdrenoCleanInvalidate:NativeAdrenoClean);}
    if(write_combined_) {  /* no CPU mapping should cache these pages; if one does, reach its lines through the physical pages */
        if(!include_write_combined)return kIOReturnSuccess;
        NativeAdrenoNotePhase(phase,bytes);
        return maintainImported(offset,bytes,writes?NativeAdrenoCleanInvalidate:NativeAdrenoClean);
    }
    NativeAdrenoNotePhase(phase,bytes);
    if(!mapping_)return maintainImported(offset,bytes,writes?NativeAdrenoCleanInvalidate:NativeAdrenoClean);   /* system memory */
    return NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(mapping_)+offset,bytes,
        writes?NativeAdrenoCleanInvalidate:NativeAdrenoClean);
}
IOReturn NativeAdrenoMemory::acquireDeviceWrites(size_t offset,size_t bytes,unsigned phase)
{
    if(!backed() || !prepared_ || quarantined_)return kIOReturnNotReady;
    if(!bytes || ((offset|bytes)&4095) || offset>bytes_ || bytes>bytes_-offset)return kIOReturnBadArgument;
    if(imported_ && owner_uncached_)return kIOReturnSuccess;
    /* Imported pages may hold the owner's unrelated dirty lines next to the GPU's writes: clean as well as invalidate. */
    if(imported_) {NativeAdrenoNotePhase(phase,bytes);return maintainImported(offset,bytes,NativeAdrenoCleanInvalidate);}
    if(write_combined_)return kIOReturnSuccess;
    NativeAdrenoNotePhase(phase,bytes);
    if(!mapping_)return maintainImported(offset,bytes,NativeAdrenoInvalidate);   /* system memory */
    return NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(mapping_)+offset,bytes,NativeAdrenoInvalidate);
}
/* Imported pages have no kernel mapping here (and write-combined ones only a non-cacheable one); maintain each span's part of
 * [offset, offset+bytes) by physical address. */
IOReturn NativeAdrenoMemory::maintainImported(size_t offset,size_t bytes,NativeAdrenoCacheOperation op)
{
    for(size_t i=0;i<span_count_;++i) {
        const NativeAdrenoPhysicalSpan &span=spans_[i];
        const size_t begin=offset>span.offset?offset:span.offset;
        const size_t end=offset+bytes<span.offset+span.bytes?offset+bytes:span.offset+span.bytes;
        if(begin>=end)continue;
        const IOReturn status=NativeAdrenoMaintainPhysicalPages(span.physical+(begin-span.offset),end-begin,op);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
