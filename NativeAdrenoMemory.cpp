/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoMemory.h"
/* Where a call that never returns is stuck: the free/unmap/release path stores a stage number here, the board page shows it. */
extern "C" {volatile uint32_t native_adreno_trace_stage;}
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <kern/task.h>
#include <kern/clock.h>

static bool outputWidth(unsigned bits)
{return bits==32 || bits==36 || bits==40 || bits==42 || bits==44 || bits==48;}
/* Where the last import stopped (10..19, 19 = done), and for a refused page the physical address and offset: the board log shows them. */
enum : IOReturn {kNativeAdrenoProtectionFailure=2};   /* KERN_PROTECTION_FAILURE, as prepare() reports a range that cannot be wired for writing */
extern "C" {volatile uint32_t native_adreno_pool_mib=0,native_adreno_pool_status=0;volatile uint32_t native_adreno_import_stage;volatile uint64_t native_adreno_import_fail_pa,native_adreno_import_fail_offset;}
/* The pool (see reservePool): a free list of page extents over one contiguous block; best fit, neighbours merged on release. */
extern "C" {volatile uint32_t native_adreno_pool_stats[8];}
/* The pool's write-combine (writeCombinePool): bit 31 once tried, the low bits its status. */
extern "C" {volatile uint32_t native_adreno_pool_wc;}   /* [0] pool MiB, [1] BOs from the pool, [2] fell back, [3] MiB in use, [4] peak MiB, [5] scattered BOs, [6] their spans, [7] import spans */
namespace {
struct PoolExtent {size_t offset,bytes;};
struct Pool {
    IOBufferMemoryDescriptor *descriptor=nullptr;
    unsigned char *virt=nullptr;uint64_t physical=0;size_t bytes=0,used=0,peak=0;
    PoolExtent free[512];unsigned count=0;
    bool write_combined=false;
} pool;
bool poolTake(size_t bytes,size_t *offset) {
    int best=-1;
    for(unsigned i=0;i<pool.count;++i)if(pool.free[i].bytes>=bytes && (best<0 || pool.free[i].bytes<pool.free[best].bytes))best=(int)i;
    if(best<0)return false;
    *offset=pool.free[best].offset;
    pool.free[best].offset+=bytes;pool.free[best].bytes-=bytes;
    if(!pool.free[best].bytes) {for(unsigned i=(unsigned)best;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];--pool.count;}
    pool.used+=bytes;if(pool.used>pool.peak)pool.peak=pool.used;
    native_adreno_pool_stats[3]=(uint32_t)(pool.used>>20);native_adreno_pool_stats[4]=(uint32_t)(pool.peak>>20);
    return true;
}
void poolGive(size_t offset,size_t bytes) {
    unsigned at=0;while(at<pool.count && pool.free[at].offset<offset)++at;
    if(pool.count>=512) {pool.used-=bytes;return;}   /* cannot happen with bounded fragmentation; the extent is lost, not reused */
    for(unsigned i=pool.count;i>at;--i)pool.free[i]=pool.free[i-1];
    pool.free[at]={offset,bytes};++pool.count;
    if(at+1<pool.count && pool.free[at].offset+pool.free[at].bytes==pool.free[at+1].offset) {
        pool.free[at].bytes+=pool.free[at+1].bytes;
        for(unsigned i=at+1;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];
        --pool.count;
    }
    if(at && pool.free[at-1].offset+pool.free[at-1].bytes==pool.free[at].offset) {
        pool.free[at-1].bytes+=pool.free[at].bytes;
        for(unsigned i=at;i+1<pool.count;++i)pool.free[i]=pool.free[i+1];
        --pool.count;
    }
    pool.used-=bytes;native_adreno_pool_stats[3]=(uint32_t)(pool.used>>20);
}
}
size_t NativeAdrenoMemory::poolBytes() {return pool.bytes;}
static uint64_t gPoolMissAt;   /* mach_absolute_time of the last allocation the pool could not serve, 0 none */
bool NativeAdrenoMemory::poolPressure()
{
    if(!pool.descriptor || !pool.bytes)return false;
    if(pool.used>pool.bytes-pool.bytes/8)return true;
    const uint64_t miss=gPoolMissAt;
    if(!miss)return false;
    uint64_t window=0;nanoseconds_to_absolutetime(2000000000ull,&window);
    return mach_absolute_time()-miss<window;
}
uint64_t NativeAdrenoMemory::poolPhysical() {return pool.descriptor?pool.physical:0;}
/* int-152: a user BO's pages changed attribute one pmap_set_cache_attributes call at a time (each walks the page's mappings, flushes the
 * TLB and cleans the page), on allocation and again on release, under the connection lock: 3 ms a BO on average and 90-130 ms for a
 * surface-sized one, while every other GPU call of every client waited. The pool is the BOs' own block and nothing else maps it, so
 * its pages take the attribute once here and keep it while the pool lives. */
IOReturn NativeAdrenoMemory::writeCombinePool()
{
    if(!pool.descriptor || !pool.virt || !pool.bytes)return kIOReturnNotReady;
    if(pool.write_combined)return kIOReturnSuccess;
    IOReturn status=NativeAdrenoMaintainOwnedPages(pool.virt,pool.bytes,NativeAdrenoCleanInvalidate);
    if(status)return status;
    NativeAdrenoNotePhase(NativeAdrenoPhasePool,pool.bytes);
    const NativeAdrenoPhysicalSpan span={0,pool.physical,pool.bytes};
    status=NativeAdrenoSetPagesCacheMode(&span,1,true);
    if(status)return status;
    pool.write_combined=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::reservePool(unsigned bits,size_t bytes)
{
    if(pool.descriptor)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits))return kIOReturnBadArgument;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    const uint64_t limit=UINT64_C(1)<<bits;
    IOBufferMemoryDescriptor *descriptor=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|kIOMemoryPhysicallyContiguous,
        bytes,(limit-1)&~UINT64_C(4095));
    if(!descriptor)return kIOReturnNoMemory;
    status=descriptor->prepare();
    if(status!=kIOReturnSuccess) {descriptor->release();return status;}
    IOByteCount length=0,tail=0;
    const addr64_t first=descriptor->getPhysicalSegment(0,&length,kIOMemoryMapperNone);
    const addr64_t last=descriptor->getPhysicalSegment(bytes-4096,&tail,kIOMemoryMapperNone);
    unsigned char *virt=static_cast<unsigned char *>(descriptor->getBytesNoCopy());
    if(!first || (first&4095) || !virt || ((uintptr_t)virt&4095) || last!=first+bytes-4096 || first>=limit || bytes>limit-first ||
        descriptor->getLength()!=bytes) {
        (void)descriptor->complete();descriptor->release();return kIOReturnUnsupported;
    }
    pool.descriptor=descriptor;pool.virt=virt;pool.physical=first;pool.bytes=bytes;pool.used=pool.peak=0;
    pool.free[0]={0,bytes};pool.count=1;
    native_adreno_pool_stats[0]=(uint32_t)(bytes>>20);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoMemory::allocate(size_t bytes,unsigned bits,bool contiguous,size_t budget,bool pooled)
{
    if(descriptor_ || spans_ || quarantined_)return kIOReturnBusy;
    if(!bytes || (bytes&4095) || !outputWidth(bits) || !budget || budget>256u*1024u*1024u)
        return kIOReturnBadArgument;
    if(bytes>budget)return kIOReturnNoResources;
    NativeAdrenoCpuState cpu{};
    IOReturn status=NativeAdrenoQueryCpu4K(&cpu);if(status!=kIOReturnSuccess)return status;
    uint64_t limit=UINT64_C(1)<<bits;
    if(pooled && pool.descriptor && bytes<=pool.bytes) {
        size_t offset=0;
        if(poolTake(bytes,&offset)) {
            const uint64_t physical=pool.physical+offset;
            if(physical>=limit || bytes>limit-physical) {poolGive(offset,bytes);}
            else {
                spans_=IONewData(NativeAdrenoPhysicalSpan,1);
                if(!spans_) {poolGive(offset,bytes);return kIOReturnNoMemory;}
                span_capacity_=1;span_count_=1;spans_[0]={0,physical,bytes};
                mapping_=pool.virt+offset;bytes_=bytes;pooled_=true;pool_offset_=offset;prepared_=true;
                bzero(mapping_,bytes);
                /* Through a write-combined pool's own mapping the zeros go to DRAM; the barrier completes them. */
                if(pool.write_combined)__asm__ volatile("dsb sy":::"memory");
                else {status=NativeAdrenoMaintainOwnedPages(mapping_,bytes,NativeAdrenoClean);NativeAdrenoNotePhase(NativeAdrenoPhaseAllocate,bytes);}
                if(status!=kIOReturnSuccess) {(void)release();return status;}
                native_adreno_pool_stats[1]=native_adreno_pool_stats[1]+1;
                return kIOReturnSuccess;
            }
        }
        native_adreno_pool_stats[2]=native_adreno_pool_stats[2]+1;
        gPoolMissAt=mach_absolute_time();
    }
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
    span_capacity_=bytes/4096;
    spans_=IONewData(NativeAdrenoPhysicalSpan,span_capacity_);
    if(!spans_) {status=kIOReturnNoMemory;goto failed;}
    for(size_t offset=0;offset<bytes;offset+=4096) {
        IOByteCount length=0;
        addr64_t physical=descriptor_->getPhysicalSegment(offset,&length,kIOMemoryMapperNone);
        if(!physical || (physical&4095) || length<4096 || physical>=limit || 4096>limit-physical)
            {status=kIOReturnUnsupported;goto failed;}
        if(contiguous && offset && physical!=spans_[0].physical+offset)
            {status=kIOReturnUnsupported;goto failed;}
        if(span_count_ && physical==spans_[span_count_-1].physical+spans_[span_count_-1].bytes)
            spans_[span_count_-1].bytes+=4096;
        else spans_[span_count_++]={offset,physical,4096};
    }
    bzero(mapping_,bytes);
    status=NativeAdrenoMaintainOwnedPages(mapping_,bytes,NativeAdrenoClean);
    if(status!=kIOReturnSuccess)goto failed;
    NativeAdrenoNotePhase(NativeAdrenoPhaseAllocate,bytes);
    native_adreno_pool_stats[5]=native_adreno_pool_stats[5]+1;native_adreno_pool_stats[6]=native_adreno_pool_stats[6]+(uint32_t)span_count_;
    return kIOReturnSuccess;
failed:
    /* release() deliberately retains descriptor/metadata if complete fails.
     * Preserve the allocation's original failure instead of reporting cleanup
     * as success. No object has been exposed to a GPU or userspace yet. */
    (void)release();return status;
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
        !budget || budget>256u*1024u*1024u)return kIOReturnBadArgument;
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
    native_adreno_pool_stats[7]=native_adreno_pool_stats[7]+(uint32_t)span_count_;
    return kIOReturnSuccess;
failed:
    (void)release();return status;
}
IOReturn NativeAdrenoMemory::borrow(void *mapping,uint64_t physical,size_t bytes)
{
    if(descriptor_ || imported_ || spans_ || quarantined_ || pooled_ || borrowed_)return kIOReturnBusy;
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
    if(!(descriptor_ || pooled_) || imported_ || !prepared_ || quarantined_ || device_references_ || write_combined_ || !mapping_ || !span_count_)
        return kIOReturnNotReady;
    if(pooled_ && pool.write_combined) {write_combined_=true;return kIOReturnSuccess;}
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
    if(write_combined_ && pooled_ && pool.write_combined)write_combined_=false;   /* the pool's pages keep the attribute */
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
    if(descriptor_ && native_pmap_user_mappings && spans_) {
        /* A page a user task still maps (a BO view over physical pages holds no reference to them) is counted before the pages are freed:
         * the first page of up to 64 spans, then which process maps it. XNU frees such a page under the mapping (pmap_assert_free). */
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
    if(pooled_) {poolGive(pool_offset_,bytes_);pooled_=false;prepared_=false;pool_offset_=0;}
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
