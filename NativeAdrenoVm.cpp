/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoVm.h"
#include "NativeAdrenoVmAlias.h"
extern "C" volatile uint32_t native_adreno_trace_stage;
extern "C" {int (*native_adreno_dram_read32)(uint64_t,uint32_t *);int (*native_adreno_cached_read32)(uint64_t,uint32_t *);
    int (*native_adreno_line_audit)(uint64_t,size_t,uint32_t *,uint32_t *);}
#include <string.h>
#include <IOKit/IOLib.h>
/* 1 once the board proved the GPU's cacheable inner-shareable accesses coherent with the CPU's caches in both directions (the A740's
 * ADRENO_QUIRK_HAS_CACHED_COHERENT behind the dma-coherent GPU SMMU, which Linux maps with IOMMU_CACHE): a mapping with write-back
 * attributes then needs no CPU cache maintenance. 0 keeps every mapping maintained as before. Counters: maintenance passes skipped for
 * coherent mappings at map, unmap, submission and completion. */
extern "C" {volatile uint32_t native_adreno_io_coherent;volatile uint32_t native_adreno_coherent_skips[4];}
static bool coherentMapping(unsigned writeback) {return writeback && native_adreno_io_coherent;}
/* GPU-written imports (render targets) are mapped non-coherent (the GPU stopped inside its end-of-pass clean writing them through
 * write-back mappings), and the CPU never touches their pixels while the GPU owns them: they were cleaned when mapped and need no
 * maintenance per submission, which cost about 60 ms per frame. */
static bool importedUntouched(const NativeAdrenoMemory *memory) {return memory->imported() && native_adreno_io_coherent;}
/* Mappings of one cache/shareability/privilege class may share pages; only pages shared across classes refuse. */
static unsigned aliasClass(unsigned permissions,unsigned writeback,unsigned shareability)
{return writeback|(shareability<<1)|((permissions&ArmSmmu4KPrivileged)?8u:0u);}
static bool validBinding(const NativeAdrenoVmRegion &r,unsigned upper)
{
    return r.tables && r.arena && r.state && !r.tables->quarantined() && r.tables->provider() &&
        r.arena->pages==r.tables->provider() && r.arena->upper_region==upper &&
        r.tables->provider()->find(r.tables->provider()->cookie,r.arena->config.table_physical) &&
        r.state->prepared && !r.state->poisoned && !r.state->tables.leaf_pages;
}
ArmSmmu4KLiveRegion NativeAdrenoVm::region(unsigned upper,const ArmSmmu4KRange *ranges,size_t count) const
{
    return {regions_[upper].arena,regions_[upper].state,ranges,count};
}
/* Board hook and per-phase time of the two table transactions, for the log: [0] map, [1] unmap; each row is count, the phases in order, the slowest call
 * and the bytes mapped (int-134: BoMap averaged 135 ms and the desktop crawled). */
extern "C" {uint64_t (*native_adreno_vm_now_us)(void)=nullptr;volatile uint64_t native_adreno_vm_phase_us[2][8];}
/* {count, us, spans} per kind: 0 map of driver memory, 1 import map without the alias pass, 2 import map with it, 3 unmap dropping its
 * pages from the TLB one by one, 4 unmap invalidating the ASID. */
extern "C" {volatile uint64_t native_adreno_vm_kind_us[5][3];}
namespace {
struct PhaseClock {
    uint64_t last=native_adreno_vm_now_us?native_adreno_vm_now_us():0,began=last;
    void lap(unsigned row,unsigned slot) {
        if(!native_adreno_vm_now_us)return;
        const uint64_t now=native_adreno_vm_now_us();native_adreno_vm_phase_us[row][slot]+=now-last;last=now;
    }
    uint64_t elapsed() const {return native_adreno_vm_now_us?native_adreno_vm_now_us()-began:0;}
    void done(unsigned row,uint64_t bytes) {
        if(!native_adreno_vm_now_us)return;
        const uint64_t total=native_adreno_vm_now_us()-began;
        native_adreno_vm_phase_us[row][0]+=1;native_adreno_vm_phase_us[row][7]+=bytes;
        if(total>native_adreno_vm_phase_us[row][6])native_adreno_vm_phase_us[row][6]=total;
    }
};
}
ArmSmmu4KStatus NativeAdrenoVm::observe() const
{
    ArmSmmu4KStreamResult routes;
    ArmSmmu4KStatus status=ArmSmmu4KObserveStreams(&io_,streams_,context_,&routes);if(status)return status;
    ArmSmmu4KLiveRegion lower=region(0),upper=region(1);ArmSmmu4KLiveResult audit;
    status=ArmSmmu4KLiveAuditContext(context_,lower.arena?&lower:nullptr,upper.arena?&upper:nullptr,&audit);
    if(status)return status;
    return ArmSmmu4KObserveStreams(&io_,streams_,context_,&routes);
}
/* Global policy, the context image and the owned routes only. The scan of every other slot for a conflicting route read each
 * SMR/S2CR of the SMMU twice per map and unmap (int-152: 0.75 ms a pass, 1.3 s of the 3.2 s of 807 maps); observe(), which
 * observeCheap runs every 64 operations, keeps that scan. */
ArmSmmu4KStatus NativeAdrenoVm::observeStreamsOnly() const
{
    ArmSmmu4KStreamResult routes;
    return ArmSmmu4KObserveOwnedStreams(&io_,streams_,context_,&routes);
}
/* Every 64th table operation scans the SMMU's routes for a conflict with the owned ones; the ones between read no SMMU register
 * before the table transaction (int-159: a map read the owned routes before and after its transaction, 0.4 ms of its 1.7 ms). The
 * tables are not walked here: attach proved them whole and each transaction proves what it changes (i557u: a whole walk of both
 * regions every 64th operation stalled a map 37 ms). */
ArmSmmu4KStatus NativeAdrenoVm::observeCheap()
{
    if(++since_audit_>=64) {
        since_audit_=0;
        ArmSmmu4KStreamResult routes;
        return ArmSmmu4KObserveStreams(&io_,streams_,context_,&routes);
    }
    return ArmSmmu4KSuccess;
}
IOReturn NativeAdrenoVm::fail(ArmSmmu4KStatus status)
{
    poisoned_=true;result_.operation=status;result_.complete=false;
    for(size_t i=0;mappings_ && i<high_;++i)if(mappings_[i].memory)mappings_[i].memory->quarantine();
    for(auto &r:regions_)if(r.arena) {r.tables->quarantine();r.state->poisoned=1;}
    if(fault_streams_) {
        ArmSmmu4KStreamResult routes;
        result_.quarantine=ArmSmmu4KFaultStreams(&io_,streams_,&routes);
    }
    return kIOReturnIOError;
}
IOReturn NativeAdrenoVm::attach(const ArmSmmu4KIo *io,ArmSmmu4KContextConfig *context,
    ArmSmmu4KStreamState *streams,const NativeAdrenoVmRegion *lower,const NativeAdrenoVmRegion *upper,
    size_t capacity,unsigned budget)
{
    if(attached_ || changes_ || poisoned_)return kIOReturnBusy;
    if(!io || !context || !streams || (!lower && !upper) || !capacity ||
        !budget || budget>1048576 || (lower && !validBinding(*lower,0)) || (upper && !validBinding(*upper,1)))
        return kIOReturnBadArgument;
    const size_t first=capacity<1024?capacity:1024;
    changes_=IONewData(ArmSmmu4KRange,first);alias_scratch_=IONewData(uint32_t,first);mappings_=IONewData(Mapping,64);
    order_=IONewData(uint32_t,64);
    capacity_=first;mapping_capacity_=64;
    if(!changes_ || !alias_scratch_ || !mappings_ || !order_ || !index_.allocate()) {freeTables();return kIOReturnNoMemory;}
    memset(mappings_,0,sizeof(Mapping)*64);
    free_hint_=0;high_=0;memset(class_spans_,0,sizeof(class_spans_));order_count_[0]=order_count_[1]=0;
    io_=*io;context_=context;streams_=streams;budget_=budget;
    if(lower)regions_[0]=*lower;if(upper)regions_[1]=*upper;
    for(unsigned i=0;i<2;++i)if(regions_[i].arena) {
        if(NativeAdrenoBindSmmuVisibility(regions_[i].tables,&visibility_[i])!=kIOReturnSuccess) {
            freeTables();regions_[0]={};regions_[1]={};return kIOReturnBadArgument;
        }
        regions_[i].arena->alias_scratch=alias_scratch_;regions_[i].arena->alias_scratch_entries=capacity_;
    }
    attached_=true;
    ArmSmmu4KStatus status=observe();if(status)return fail(status);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::detach()
{
    if(poisoned_ || objects_ || spanCount())return kIOReturnBusy;
    if(!attached_)return kIOReturnNotReady;
    for(auto &r:regions_)if(r.arena) {r.arena->alias_scratch=nullptr;r.arena->alias_scratch_entries=0;}
    freeTables();
    attached_=false;regions_[0]={};regions_[1]={};context_=nullptr;streams_=nullptr;capacity_=mapping_capacity_=0;
    return kIOReturnSuccess;
}
void NativeAdrenoVm::freeTables()
{
    if(changes_)IODeleteData(changes_,ArmSmmu4KRange,capacity_);
    if(alias_scratch_)IODeleteData(alias_scratch_,uint32_t,capacity_);
    if(mappings_)IODeleteData(mappings_,Mapping,mapping_capacity_);
    if(order_)IODeleteData(order_,uint32_t,mapping_capacity_);
    index_.release();
    changes_=nullptr;alias_scratch_=nullptr;mappings_=nullptr;order_=nullptr;capacity_=mapping_capacity_=0;
}
bool NativeAdrenoVm::growScratch(size_t needed)
{
    if(needed<=capacity_)return true;
    if(needed>UINT32_MAX || needed>SIZE_MAX/2/sizeof(ArmSmmu4KRange))return false;
    size_t size=capacity_*2;if(size<needed)size=needed;
    ArmSmmu4KRange *made=IONewData(ArmSmmu4KRange,size);
    if(!made)return false;
    uint32_t *scratch=IONewData(uint32_t,size);
    if(!scratch) {IODeleteData(made,ArmSmmu4KRange,size);return false;}
    IODeleteData(changes_,ArmSmmu4KRange,capacity_);IODeleteData(alias_scratch_,uint32_t,capacity_);
    changes_=made;alias_scratch_=scratch;capacity_=size;
    for(auto &r:regions_)if(r.arena) {r.arena->alias_scratch=alias_scratch_;r.arena->alias_scratch_entries=capacity_;}
    return true;
}
bool NativeAdrenoVm::growMappings()
{
    if(mapping_capacity_>SIZE_MAX/2/sizeof(Mapping))return false;
    const size_t size=mapping_capacity_*2;
    Mapping *made=IONewData(Mapping,size);
    if(!made)return false;
    uint32_t *order=IONewData(uint32_t,size);
    if(!order) {IODeleteData(made,Mapping,size);return false;}
    memset(made,0,sizeof(Mapping)*size);
    memcpy(made,mappings_,sizeof(Mapping)*mapping_capacity_);
    memcpy(order,order_,sizeof(uint32_t)*(order_count_[0]+order_count_[1]));
    IODeleteData(mappings_,Mapping,mapping_capacity_);IODeleteData(order_,uint32_t,mapping_capacity_);
    mappings_=made;order_=order;mapping_capacity_=size;
    return true;
}
IOReturn NativeAdrenoVm::invalidateAsid()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    uint64_t offset=0;
    for(const auto &region:regions_)if(region.state && region.state->context_offset)offset=region.state->context_offset;
    if(!offset)return kIOReturnNotReady;
    ArmSmmu4KProgramResult r;
    const ArmSmmu4KStatus status=ArmSmmu4KSyncContextTlb(&io_,offset,context_->asid,0,budget_,&r);
    return status?fail(status):kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::findAddress(unsigned upper,size_t bytes,uint64_t alignment,uint64_t *out) const
{
    if(!out)return kIOReturnBadArgument;*out=0;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(upper>1 || !regions_[upper].arena || !bytes || (bytes&4095) || alignment<4096 ||
        (alignment&(alignment-1)))return kIOReturnBadArgument;
    const ArmSmmu4KLiveArena *a=regions_[upper].arena;
    uint64_t offset=0;
    for(size_t i=orderBegin(upper);;++i) {
        uint64_t rem=(a->iova+offset)&(alignment-1),padding=rem?alignment-rem:0;
        if(padding>a->bytes-offset)return kIOReturnNoResources;offset+=padding;
        if(bytes>a->bytes-offset)return kIOReturnNoResources;
        const Mapping *m=i<orderEnd(upper)?&mappings_[order_[i]]:nullptr;
        if(!m || (m->iova-a->iova>=offset && bytes<=m->iova-a->iova-offset))
            {*out=a->iova+offset;return kIOReturnSuccess;}
        uint64_t end=m->iova-a->iova+m->memory->bytes();
        if(end>offset)offset=end;
    }
}
IOReturn NativeAdrenoVm::preflight(const NativeAdrenoMemory *memory,unsigned upper,uint64_t iova,
    unsigned permissions,unsigned writeback,unsigned shareability,bool exclusive) const
{
    if(!memory || upper>1 || !regions_[upper].arena || !memory->backed() || memory->quarantined() ||
        !memory->bytes() || !memory->spanCount() || !(permissions&ArmSmmu4KRead) || (permissions&~15u) ||
        writeback>1 || (shareability!=2 && shareability!=3))return kIOReturnBadArgument;
    const ArmSmmu4KLiveArena *a=regions_[upper].arena;
    if((iova&4095) || iova<a->iova || iova-a->iova>=a->bytes || memory->bytes()>a->bytes-(iova-a->iova))
        return kIOReturnBadArgument;
    {
        /* Mappings are sorted by address and disjoint: only the first one not wholly below iova can overlap (native addresses may end at
         * 2^64, so ends are compared as last bytes). */
        const size_t i=orderAt(upper,iova);
        if(i<orderEnd(upper) && mappings_[order_[i]].iova<=iova+(memory->bytes()-1))return kIOReturnBusy;
    }
    if(a->config.output_bits<32 || a->config.output_bits>48)return kIOReturnBadArgument;
    uint64_t offset=0,limit=UINT64_C(1)<<a->config.output_bits;
    for(size_t j=0;j<memory->spanCount();++j) {
        const auto &span=memory->spans()[j];
        if(span.offset!=offset || !span.physical || !span.bytes || ((span.physical|span.bytes)&4095) ||
            span.bytes>memory->bytes()-offset || span.physical>=limit || span.bytes>limit-span.physical)
            return kIOReturnBadArgument;
        for(const auto &r:regions_)if(r.arena && r.tables->overlaps(span.physical,span.bytes))return kIOReturnBadArgument;
        offset+=span.bytes;
    }
    if(offset!=memory->bytes())return kIOReturnBadArgument;
    /* exclusive (exclusivePages): no mapped range can alias these pages in conflict, so the pass over every other class's ranges is
     * not needed. i568: this pass and the table transaction's walked every one of about 50000 ranges on each map, 0.6 ms of its 0.66. */
    if(exclusive)return kIOReturnSuccess;
    /* Only another cache/shareability/privilege class can conflict. The new spans are indexed by physical address in the scratch (the
     * caller grew it to them) and every mapped span of another class is looked up in O(log n). Pages of this driver's own buffers are
     * all in its ownership map: an import none of whose pages is there can only alias another import. */
    const unsigned mine=aliasClass(permissions,writeback,shareability);
    if(spanCount()==class_spans_[mine&15])return kIOReturnSuccess;
    if(!alias_scratch_ || memory->spanCount()>capacity_)return kIOReturnNoResources;
    const bool importsOnly=memory->imported() && !NativeAdrenoMemory::ownsAnyPage(memory->spans(),memory->spanCount());
    for(size_t i=0;i<memory->spanCount();++i)alias_scratch_[i]=(uint32_t)i;
    NativeAdrenoPhysicalIndex<NativeAdrenoPhysicalSpan> index(memory->spans(),alias_scratch_,memory->spanCount());
    for(size_t k=0;k<high_;++k) {
        const Mapping &m=mappings_[k];
        if(!m.memory || !m.mapped_spans || aliasClass(m.permissions,m.writeback,m.shareability)==mine)continue;
        if(importsOnly && !m.memory->imported())continue;
        for(size_t j=0;j<m.memory->spanCount();++j)
            if(index.overlaps(m.memory->spans()[j].physical,m.memory->spans()[j].bytes))return kIOReturnBadArgument;
    }
    return kIOReturnSuccess;
}
/* Whether no mapped range can alias memory's pages with another cache/shareability/privilege class. A buffer this driver allocated
 * owns its pages until its unmap has completed (the free waits for it). An import (a user range, usually an IOSurface, which any
 * process may import again) can alias only another import or a range of driver memory it covers: it is exclusive when every mapped
 * import has its class and none of its pages lies in a buffer of this driver (NativeAdrenoMemory::ownsAnyPage). i568: about 100
 * surface imports of 5000-6000 spans each, every one checked against every other range. */
bool NativeAdrenoVm::exclusivePages(const NativeAdrenoMemory *memory,unsigned cls) const
{
    if(!memory || !memory->imported())return true;   /* preflight refuses a null one */
    size_t imports=0;for(size_t k=0;k<16;++k)imports+=import_spans_[k];
    if(imports!=import_spans_[cls&15])return false;
    return !NativeAdrenoMemory::ownsAnyPage(memory->spans(),memory->spanCount());
}
NativeAdrenoVm::Mapping *NativeAdrenoVm::find(uint64_t handle)
{return const_cast<Mapping *>(static_cast<const NativeAdrenoVm *>(this)->find(handle));}
const NativeAdrenoVm::Mapping *NativeAdrenoVm::find(uint64_t handle) const
{
    const uint32_t slot=mappings_?index_.find(handle):UINT32_MAX;
    if(slot>=mapping_capacity_)return nullptr;
    const Mapping *m=&mappings_[slot];
    return m->handle==handle && m->memory?m:nullptr;
}
size_t NativeAdrenoVm::orderAt(unsigned upper,uint64_t iova) const
{
    size_t low=orderBegin(upper),high=orderEnd(upper);
    while(low<high) {
        const size_t mid=low+(high-low)/2;
        const Mapping &m=mappings_[order_[mid]];
        if(m.iova+(m.memory->bytes()-1)<iova)low=mid+1;else high=mid;
    }
    return low;
}
/* A mapping whose spans the table now holds takes its place in its region's order; its spans count for their region and class. */
void NativeAdrenoVm::insert(Mapping *record)
{
    const size_t at=orderAt(record->upper,record->iova),total=order_count_[0]+order_count_[1];
    memmove(order_+at+1,order_+at,(total-at)*sizeof(*order_));
    order_[at]=(uint32_t)(record-mappings_);++order_count_[record->upper];
    class_spans_[aliasClass(record->permissions,record->writeback,record->shareability)&15]+=record->spans;
    if(record->upper)upper_count_+=record->spans;else lower_count_+=record->spans;
}
bool NativeAdrenoVm::erase(Mapping *record)
{
    const size_t at=orderAt(record->upper,record->iova),total=order_count_[0]+order_count_[1];
    if(at>=orderEnd(record->upper) || order_[at]!=(uint32_t)(record-mappings_))return false;
    memmove(order_+at,order_+at+1,(total-at-1)*sizeof(*order_));
    --order_count_[record->upper];
    class_spans_[aliasClass(record->permissions,record->writeback,record->shareability)&15]-=record->spans;
    if(record->upper)upper_count_-=record->spans;else lower_count_-=record->spans;
    return true;
}
IOReturn NativeAdrenoVm::map(NativeAdrenoMemory *memory,unsigned upper,uint64_t iova,
    unsigned permissions,unsigned writeback,unsigned shareability,uint64_t *handle)
{
    if(!handle)return kIOReturnBadArgument;*handle=0;result_={};
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    PhaseClock clock;
    const unsigned cls=aliasClass(permissions,writeback,shareability)&15;
    /* Room for the new spans' table transaction and their alias index. */
    if(memory && memory->spanCount() && !growScratch(memory->spanCount()))return kIOReturnNoResources;
    const bool exclusive=exclusivePages(memory,cls);
    IOReturn checked=preflight(memory,upper,iova,permissions,writeback,shareability,exclusive);if(checked)return checked;
    if(next_handle_==UINT64_MAX)return kIOReturnNoResources;
    clock.lap(0,5); /* Previously omitted preflight; map slot 5 was unused. */
    ArmSmmu4KStatus status=observeCheap();if(status)return fail(status);
    clock.lap(0,1);
    if(!index_.reserve(objects_+1))return kIOReturnNoMemory;   /* the handle's index cell is had before anything changes */
    checked=memory->holdForDevice();if(checked)return checked;
    Mapping *record=nullptr;
    for(;;) {
        for(size_t i=free_hint_;i<mapping_capacity_;++i)if(!mappings_[i].memory) {
            record=&mappings_[i];free_hint_=i+1;if(high_<i+1)high_=i+1;break;
        }
        if(record || !growMappings())break;
    }
    if(!record) {memory->quarantine();return fail(ArmSmmu4KStateMismatch);}
    *record={memory,0,iova,memory->spanCount(),0,permissions,writeback,shareability,0,upper};++objects_;
    if(coherentMapping(writeback))native_adreno_coherent_skips[0]=native_adreno_coherent_skips[0]+1;
    else {
        checked=memory->publishForDevice(0,memory->bytes(),permissions&ArmSmmu4KWrite,distrust_write_combined_);
        if(checked)return fail(ArmSmmu4KIoError);
    }
    clock.lap(0,2);
    /* Every span in one table transaction (the scratch was grown to them). */
    const size_t spans=record->spans;
    ArmSmmu4KRange *changes=changes_;
    for(size_t i=0;i<spans;++i) {
        const auto &span=memory->spans()[i];
        changes[i]={iova+span.offset,span.physical,span.bytes,permissions,writeback,shareability};
    }
    /* preflight cleared the pages of conflicting aliases with every range of both regions (exclusive, or checked against the other
     * classes' ranges there), so the table transaction does not check them against the existing ranges a second time: that pass
     * sorted every other-class range on each import map (i35abaf83: imports not exclusive 1679 maps, 3.56 ms each, 79% of all map
     * time). */
    ArmSmmu4KLiveRegion lower=region(0),high=region(1);ArmSmmu4KLiveResult update;
    status=ArmSmmu4KLiveMapContextBatchExclusive(&io_,streams_->bank,context_,lower.arena?&lower:nullptr,
        high.arena?&high:nullptr,upper,&visibility_[upper],changes,spans,budget_,&update);
    if(status==ArmSmmu4KCapacity) {
        /* No table page could be had: the kit linked only empty tables and wrote no leaf, so the map is undone and the VM stays usable,
         * as a failed io-pgtable map returns -ENOMEM. */
        const size_t slot=size_t(record-mappings_);*record={};--objects_;if(slot<free_hint_)free_hint_=slot;
        if(memory->retireAfterQuiescence()!=kIOReturnSuccess)return fail(ArmSmmu4KStateMismatch);
        result_.operation=status;
        return kIOReturnNoMemory;
    }
    if(status)return fail(status);
    clock.lap(0,3);
    insert(record);record->mapped_spans=spans;result_.completed_spans=spans;
    if(memory->imported())import_spans_[cls]+=spans;
    clock.lap(0,4);clock.done(0,memory->bytes());
    {
        const unsigned kind=!memory->imported()?0:exclusive?1:2;
        native_adreno_vm_kind_us[kind][0]=native_adreno_vm_kind_us[kind][0]+1;
        native_adreno_vm_kind_us[kind][1]=native_adreno_vm_kind_us[kind][1]+clock.elapsed();
        native_adreno_vm_kind_us[kind][2]=native_adreno_vm_kind_us[kind][2]+spans;
    }
    record->handle=next_handle_++;*handle=record->handle;index_.insert(record->handle,(uint32_t)(record-mappings_));
    result_.complete=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::unmap(uint64_t handle)
{
    native_adreno_trace_stage=10;
    result_={};if(!attached_ || poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record)return kIOReturnBadArgument;
    if(record->jobs)return kIOReturnBusy;
    PhaseClock clock;
    ArmSmmu4KStatus status=observeCheap();
    native_adreno_trace_stage=11;
    if(status)return fail(status);
    clock.lap(1,1);
    const unsigned upper=record->upper;
    if(coherentMapping(record->writeback))native_adreno_coherent_skips[1]=native_adreno_coherent_skips[1]+1;
    else if(record->permissions&ArmSmmu4KWrite) {
        if(record->memory->acquireDeviceWrites(0,record->memory->bytes())!=kIOReturnSuccess)
            return fail(ArmSmmu4KIoError);
    }
    clock.lap(1,2);
    /* The mapping's spans, as it mapped them, in one transaction; they are also the existing ranges the kit checks each leaf against
     * (the descriptor it must hold before it is cleared, which the kit then reads from the table). */
    const size_t spans=record->spans;
    if(spans!=record->mapped_spans || spans>capacity_ || record->memory->spanCount()!=spans)return fail(ArmSmmu4KStateMismatch);
    for(size_t i=0;i<spans;++i) {
        const auto &span=record->memory->spans()[i];
        changes_[i]={record->iova+span.offset,span.physical,span.bytes,record->permissions,record->writeback,record->shareability};
    }
    ArmSmmu4KLiveRegion lower=upper?region(0):region(0,changes_,spans),high=upper?region(1,changes_,spans):region(1);
    ArmSmmu4KLiveResult update;
    native_adreno_trace_stage=12;
    status=ArmSmmu4KLiveUpdateContextBatch(&io_,streams_->bank,context_,lower.arena?&lower:nullptr,
        high.arena?&high:nullptr,upper,&visibility_[upper],changes_,spans,0,budget_,&update);
    native_adreno_trace_stage=13;
    if(status)return fail(status);
    clock.lap(1,3);
    if(!erase(record))return fail(ArmSmmu4KStateMismatch);
    record->mapped_spans=0;result_.completed_spans=spans;
    native_adreno_trace_stage=14;
    clock.lap(1,4);
    if(record->memory->retireAfterQuiescence()!=kIOReturnSuccess)return fail(ArmSmmu4KStateMismatch);
    clock.lap(1,5);clock.done(1,record->memory->bytes());
    if(record->memory->imported())import_spans_[aliasClass(record->permissions,record->writeback,record->shareability)&15]-=spans;
    {
        const unsigned kind=record->memory->bytes()<=(size_t)ArmSmmu4KTlbPageLimit*4096?3:4;   /* page TLBIs, else the ASID */
        native_adreno_vm_kind_us[kind][0]=native_adreno_vm_kind_us[kind][0]+1;
        native_adreno_vm_kind_us[kind][1]=native_adreno_vm_kind_us[kind][1]+clock.elapsed();
        native_adreno_vm_kind_us[kind][2]=native_adreno_vm_kind_us[kind][2]+spans;
    }
    index_.erase(record->handle);
    const size_t slot=(size_t)(record-mappings_);if(slot<free_hint_)free_hint_=slot;
    *record={};--objects_;
    while(high_ && !mappings_[high_-1].memory)--high_;result_.complete=true;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::describe(uint64_t handle,uint64_t *iova,size_t *bytes,
    unsigned *permissions) const
{
    if(!iova || !bytes || !permissions)return kIOReturnBadArgument;
    *iova=0;*bytes=0;*permissions=0;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    const Mapping *mapping=find(handle);if(!mapping)return kIOReturnBadArgument;
    *iova=mapping->iova;*bytes=mapping->memory->bytes();*permissions=mapping->permissions;
    return mapping->mapped_spans==mapping->spans?kIOReturnSuccess:kIOReturnNotReady;
}
/* TLBIALL of the bank before a submission. Where a table update has observed the bank's image (each region's whole-verification
 * update remembers its register offset), the invalidation alone: the image and faults were observed there and every 64th table
 * operation since (int-159: a submission spent 1.3 ms in the kernel, most of it reading SMMU registers). */
IOReturn NativeAdrenoVm::invalidateTlb()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    ArmSmmu4KProgramResult r;
    uint64_t offset=0;
    for(const auto &region:regions_)if(region.state && region.state->context_offset)offset=region.state->context_offset;
    const ArmSmmu4KStatus status=offset?ArmSmmu4KSyncContextTlb(&io_,offset,context_->asid,1,budget_,&r):
        ArmSmmu4KInvalidateAllOwnedContext(&io_,streams_->bank,context_,budget_,&r);
    return status?fail(status):kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::publishMapped()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory)continue;
        if(coherentMapping(m.writeback) || importedUntouched(m.memory)) {native_adreno_coherent_skips[2]=native_adreno_coherent_skips[2]+1;continue;}
        if(m.memory->publishForDevice(0,m.memory->bytes(),true,distrust_write_combined_,NativeAdrenoPhaseSubmit))return fail(ArmSmmu4KIoError);
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::acquireImported()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || !m.memory->imported() || !(m.permissions&ArmSmmu4KWrite))continue;
        if(coherentMapping(m.writeback) || importedUntouched(m.memory)) {native_adreno_coherent_skips[3]=native_adreno_coherent_skips[3]+1;continue;}
        if(m.memory->acquireDeviceWrites(0,m.memory->bytes(),NativeAdrenoPhaseComplete))return fail(ArmSmmu4KIoError);
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::sampleWord(uint64_t iova,uint32_t *cached,uint32_t *dram) const
{
    if(!cached || !dram)return kIOReturnBadArgument;
    *cached=*dram=~0u;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(iova&3)return kIOReturnBadArgument;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        const size_t offset=(size_t)(iova-m.iova);
        if(m.memory->mapping())*cached=static_cast<const volatile uint32_t *>(m.memory->mapping())[offset/4];
        for(size_t j=0;j<m.memory->spanCount();++j) {
            const auto &span=m.memory->spans()[j];
            if(offset<span.offset || offset-span.offset>=span.bytes)continue;
            uint32_t value=0;
            /* DRAM first: the cached reader writes its line back, so read after it DRAM would always agree (int-099). */
            if(native_adreno_dram_read32 && !native_adreno_dram_read32(span.physical+(offset-span.offset),&value))*dram=value;
            /* A write-combined BO's own mapping is non-cacheable: read the cacheable alias instead, so a dirty line is still seen. */
            if(native_adreno_cached_read32 && !native_adreno_cached_read32(span.physical+(offset-span.offset),&value))*cached=value;
            break;
        }
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
IOReturn NativeAdrenoVm::reassertWriteCombined(uint32_t *reset) const
{
    if(!reset)return kIOReturnBadArgument;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory)continue;
        const IOReturn status=m.memory->reassertWriteCombined(reset);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::auditRange(uint64_t iova,size_t bytes,uint32_t *lines,uint32_t *dirty,uint32_t *stale) const
{
    if(!lines || !dirty || !stale)return kIOReturnBadArgument;
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!native_adreno_line_audit)return kIOReturnUnsupported;
    if(!bytes || iova>UINT64_MAX-bytes)return kIOReturnBadArgument;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        if(!m.memory->writeCombined())return kIOReturnSuccess;   /* maintained before every submission anyway */
        /* Whole lines; the range is clipped to its BO. */
        size_t at=(size_t)(iova-m.iova)&~(size_t)63;
        const size_t end=(size_t)(iova-m.iova)+bytes>m.memory->bytes()?m.memory->bytes():(size_t)(iova-m.iova)+bytes;
        while(at<end) {
            const NativeAdrenoPhysicalSpan *span=nullptr;
            for(size_t j=0;j<m.memory->spanCount();++j) {
                const auto &s=m.memory->spans()[j];
                if(at>=s.offset && at-s.offset<s.bytes) {span=&s;break;}
            }
            if(!span)return kIOReturnBadArgument;
            const uint64_t physical=span->physical+(at-span->offset);
            size_t chunk=4096-(size_t)(physical&4095);
            if(chunk>span->offset+span->bytes-at)chunk=span->offset+span->bytes-at;
            if(chunk>end-at)chunk=(end-at+63)&~(size_t)63;
            if(native_adreno_line_audit(physical,chunk,dirty,stale))return kIOReturnIOError;
            *lines+=(uint32_t)(chunk/64);at+=chunk;
        }
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
/* The DRAM word at offset of memory, through the board's uncached reader: system memory has no kernel mapping. */
static bool dramWord(const NativeAdrenoMemory &memory,size_t offset,uint32_t *value)
{
    if(!native_adreno_dram_read32)return false;
    for(size_t j=0;j<memory.spanCount();++j) {
        const auto &span=memory.spans()[j];
        if(offset>=span.offset && offset-span.offset<span.bytes)return !native_adreno_dram_read32(span.physical+(offset-span.offset),value);
    }
    return false;
}
IOReturn NativeAdrenoVm::peekWords(uint64_t iova,uint32_t *out,size_t count) const
{
    if(!out || !attached_ || poisoned_)return kIOReturnBadArgument;
    for(size_t k=0;k<count;++k)out[k]=0;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        const size_t offset=(size_t)(iova-m.iova);
        if(!m.memory->mapping()) {
            for(size_t k=0;k<count && offset+4*k+4<=m.memory->bytes();++k)
                if(!dramWord(*m.memory,offset+4*k,&out[k]))return kIOReturnUnsupported;
            return kIOReturnSuccess;
        }
        for(size_t k=0;k<count && offset+4*k+4<=m.memory->bytes();++k)out[k]=static_cast<const volatile uint32_t *>(m.memory->mapping())[offset/4+k];
        return kIOReturnSuccess;
    }
    return kIOReturnBadArgument;
}
IOReturn NativeAdrenoVm::peekWordsUncached(uint64_t iova,uint32_t *out,size_t count) const
{
    if(!out || !attached_ || poisoned_ || !count)return kIOReturnBadArgument;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(!m.memory || iova<m.iova || iova-m.iova>=m.memory->bytes())continue;
        if(!m.memory->mapping())return peekWords(iova,out,count);   /* read from DRAM already */
        const size_t offset=(size_t)(iova-m.iova),first=offset&~(size_t)4095;
        const size_t end=offset+4*count<m.memory->bytes()?offset+4*count:m.memory->bytes();
        const size_t bytes=((end+4095)&~(size_t)4095)-first;
        IOReturn status=NativeAdrenoMaintainOwnedPages(static_cast<unsigned char *>(m.memory->mapping())+first,bytes,NativeAdrenoCleanInvalidate);
        NativeAdrenoNotePhase(NativeAdrenoPhaseDiagnostic,bytes);
        if(status)return status;
        return peekWords(iova,out,count);
    }
    return kIOReturnBadArgument;
}
bool NativeAdrenoVm::leafPhysical(uint64_t target,uint64_t *physical) const
{
    if(!physical || !attached_)return false;
    for(const auto &region:regions_) {
        const NativeAdrenoTablePages *tables=region.tables;
        if(!tables)continue;
        for(size_t p=0;p<tables->pageCount();++p) {
            const volatile uint64_t *words=tables->pageMapping(p);
            for(size_t i=0;i<512;++i) {
                const uint64_t d=words[i];
                if((d&3)==3 && (d&UINT64_C(0x0000fffffffff000))==target) {*physical=tables->pagePhysical(p)+8*i;return true;}
            }
        }
    }
    return false;
}
uint64_t NativeAdrenoVm::handleAt(uint64_t iova) const
{
    if(!attached_ || !mappings_)return 0;
    for(size_t i=0;i<high_;++i) {
        const Mapping &m=mappings_[i];
        if(m.memory && iova>=m.iova && iova-m.iova<m.memory->bytes())return m.handle;
    }
    return 0;
}
IOReturn NativeAdrenoVm::faultState(uint32_t out[4]) const
{
    if(!out || !attached_ || poisoned_)return kIOReturnBadArgument;
    uint64_t offset=0;
    for(const auto &region:regions_)if(region.state && region.state->context_offset)offset=region.state->context_offset;
    if(!offset)return kIOReturnNotReady;
    static const uint32_t regs[4]={0x58,0x60,0x64,0x68};
    for(unsigned i=0;i<4;++i)if(io_.read32(io_.cookie,(size_t)offset+regs[i],&out[i]))return kIOReturnIOError;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::holdJob(uint64_t handle)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record)return kIOReturnBadArgument;
    if(record->jobs==UINT32_MAX)return kIOReturnNoResources;
    IOReturn status=record->memory->holdForDevice();if(status)return status;
    ++record->jobs;return kIOReturnSuccess;
}
IOReturn NativeAdrenoVm::retireJobAfterCompletion(uint64_t handle)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    Mapping *record=find(handle);if(!record || !record->jobs)return kIOReturnBadArgument;
    IOReturn status=record->memory->retireAfterQuiescence();if(status)return status;
    --record->jobs;return kIOReturnSuccess;
}
