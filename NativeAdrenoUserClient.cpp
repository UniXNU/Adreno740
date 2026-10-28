/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoUserClient.h"
extern "C" volatile uint32_t native_adreno_trace_stage;
/* 0 keeps user BOs cacheable and maintained before every submission (int-070..097); the board sets it from the sheng-gpu-wc boot argument. */
/* sheng-gpu-owned-coherent=1: the GPU's own BOs use the IO-coherent path (CPU write-back view, GPU write-back inner-shareable mapping) instead of
 * write-combine: no write-combined audit and no cache maintenance per submission. Effective only once the board proved IO coherence. */
extern "C" {volatile uint32_t native_adreno_wc_enabled=1,native_adreno_owned_coherent=0;extern volatile uint32_t native_adreno_io_coherent;}
static bool ownedCoherent() {return native_adreno_owned_coherent && native_adreno_io_coherent;}
/* BoImport record: [0] calls, [1] refused, [2..5] last refusal (status, stage, bytes, address), [6..9] first refusal. Stages 1..3 are this file's
 * checks, 10..19 NativeAdrenoMemory::importTask's. */
extern "C" {volatile uint64_t native_adreno_import_diag[10];}
extern "C" volatile uint32_t native_adreno_import_stage;
/* Imports and their release (int-233/234: Core Animation's render thread waited in vm_deallocate for pages a kernel wiring held): [0] imports
 * alive, [1] frees of an import refused, [2] last refusal status; then the 8 newest imports as {address, pid<<40|bytes, handle, state}, state
 * 1 alive, 2 released, 3 free refused. */
extern "C" {volatile uint64_t native_adreno_import_live[3+8*4];}
extern "C" int proc_selfpid(void);
static unsigned gImportNext;
static void noteImportState(uint64_t handle,uint64_t state)
{
    for(unsigned i=0;i<8;++i)if(native_adreno_import_live[3+i*4+2]==handle) {native_adreno_import_live[3+i*4+3]=state;return;}
}
extern "C" volatile uint32_t native_adreno_io_coherent;
/* Per kernel method: [method][0] calls, [1] calls that returned non-zero, [2] last status, [3] total time (us, 32-bit). Method numbers are the ABI's (1..16). */
extern "C" {extern uint64_t (*native_adreno_vm_now_us)(void);volatile uint32_t native_adreno_method_stats[20][4];volatile uint32_t native_adreno_map_fail[7][2];}
static void noteImport(IOReturn status,uint32_t stage,uint64_t bytes,uint64_t address)
{
    if(!status)return;
    native_adreno_import_diag[1]=native_adreno_import_diag[1]+1;
    native_adreno_import_diag[2]=(uint32_t)status;native_adreno_import_diag[3]=stage;native_adreno_import_diag[4]=bytes;native_adreno_import_diag[5]=address;
    if(native_adreno_import_diag[1]==1) {
        native_adreno_import_diag[6]=(uint32_t)status;native_adreno_import_diag[7]=stage;native_adreno_import_diag[8]=bytes;native_adreno_import_diag[9]=address;
    }
}
#include <IOKit/IOLib.h>
#include <string.h>
extern "C" {volatile uint32_t native_adreno_window_overflow[4][3];}   /* {count, last size KiB, last end MiB} per client window */
NativeAdrenoUserClient::Entry *NativeAdrenoUserClient::find(uint64_t handle)
{
    const uint32_t index=entry_count_?index_.find(handle):UINT32_MAX;
    if(index>=entry_count_)return nullptr;
    Entry &entry=at(index);
    return entry.used && entry.handle==handle?&entry:nullptr;
}
/* A new live entry with a handle: counted and findable. */
void NativeAdrenoUserClient::publish(Entry &entry)
{
    entry.used=true;entry.handle=next_handle_++;entry.token=entry.index+1;++live_;
    index_.insert(entry.handle,entry.index);
}
void NativeAdrenoUserClient::clear(Entry &entry)
{
    if(entry.used && live_)--live_;
    index_.erase(entry.handle);
    const size_t index=entry.index;if(index<free_hint_)free_hint_=index;
    entry.handle=0;entry.mapped=0;entry.iova=0;entry.used=false;
    entry.mapped_state=false;entry.unreleasable=false;entry.imported=false;entry.metadata_bytes=0;
    entry.token=0;entry.mapping_leases=0;
}
/* A free entry, with room in the handle index for it to be published; null without memory. */
NativeAdrenoUserClient::Entry *NativeAdrenoUserClient::free()
{
    if(!index_.reserve(live_+1))return nullptr;
    for(;;) {
        for(size_t i=free_hint_;i<entry_count_;++i)if(!at(i).used) {free_hint_=i;return &at(i);}
        free_hint_=entry_count_;
        if(!grow())return nullptr;
    }
}
bool NativeAdrenoUserClient::grow()
{
    if(entry_count_>=UINT32_MAX-EntryChunk)return false;   /* tokens are entry indices plus one */
    const size_t chunk_index=entry_count_/EntryChunk;
    if(chunk_index==chunk_slots_) {
        const size_t slots=chunk_slots_?chunk_slots_*2:16;
        Entry **made=IONewData(Entry *,slots);
        if(!made)return false;
        for(size_t i=0;i<slots;++i)made[i]=i<chunk_slots_?chunks_[i]:nullptr;
        if(chunks_)IODeleteData(chunks_,Entry *,chunk_slots_);
        chunks_=made;chunk_slots_=slots;
    }
    Entry *chunk=IONewData(Entry,EntryChunk);
    if(!chunk)return false;
    memset(chunk,0,sizeof(Entry)*EntryChunk);
    for(size_t i=0;i<EntryChunk;++i)chunk[i].index=(uint32_t)(entry_count_+i);
    chunks_[chunk_index]=chunk;entry_count_+=EntryChunk;
    return true;
}
void NativeAdrenoUserClient::releaseEntries()
{
    for(size_t i=0;i<chunk_slots_;++i)if(chunks_[i])IODeleteData(chunks_[i],Entry,EntryChunk);
    if(chunks_)IODeleteData(chunks_,Entry *,chunk_slots_);
    chunks_=nullptr;chunk_slots_=0;entry_count_=0;
}
bool NativeAdrenoUserClient::growQueues()
{
    if(queue_capacity_>=UINT32_MAX/2)return false;
    const uint32_t size=queue_capacity_?queue_capacity_*2:8;
    Queue *made=IONewData(Queue,size);
    if(!made)return false;
    for(uint32_t i=0;i<size;++i)made[i]=i<queue_capacity_?queues_[i]:Queue{};
    if(queues_)IODeleteData(queues_,Queue,queue_capacity_);
    queues_=made;queue_capacity_=size;
    return true;
}
size_t NativeAdrenoUserClient::liveObjects() const
{
    return live_;
}
bool NativeAdrenoUserClient::mapped(uint64_t handle)
{
    Entry *entry=find(handle);return entry && entry->mapped_state;
}
NativeAdrenoMemory *NativeAdrenoUserClient::memoryOf(uint64_t handle)
{
    Entry *entry=find(handle);return entry && !entry->unreleasable?&entry->memory:nullptr;
}
IOReturn NativeAdrenoUserClient::open(uint32_t abi_version,NativeAdrenoVm *vm,
    NativeAdrenoSubmit *submit,NativeAdrenoRing *ring,uint32_t input_bits,uint32_t output_bits)
{
    if(open_ || vm_ || submit_)return kIOReturnBusy;
    if(abi_version!=NativeAdrenoAbiVersion)return kIOReturnUnsupported;
    if(!vm || !submit || !ring || input_bits<31 || input_bits>48 ||
        output_bits<32 || output_bits>48 || vm->poisoned() || submit->poisoned() ||
        ring->poisoned())return kIOReturnBadArgument;
    if(!grow())return kIOReturnNoMemory;
    if(!index_.allocate()) {releaseEntries();return kIOReturnNoMemory;}
    if(!growQueues()) {releaseEntries();index_.release();return kIOReturnNoMemory;}
    free_hint_=0;live_=0;
    vm_=vm;submit_=submit;ring_=ring;input_bits_=input_bits;output_bits_=output_bits;
    next_handle_=1;open_=true;
    queues_[0]={0,true};
    /* Each client is a new GPU context for the CP (useContext before each submission). */
    static uint32_t contexts;
    do context_=__atomic_add_fetch(&contexts,1,__ATOMIC_RELAXED); while(!context_);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::setScope(uint64_t queue_base,uint64_t window_start,uint64_t window_bytes)
{
    if(!open_)return kIOReturnNotReady;
    if(queue_base%NativeAdrenoQueueRange || (window_start&4095) || (window_bytes&4095) ||
        (window_bytes && (window_start+window_bytes<window_start || window_start+window_bytes>(UINT64_C(1)<<input_bits_))))
        return kIOReturnBadArgument;
    /* A client that owns no range yet has none to drain: its queue_base_ is the default 0, the render server's range, and the render server
     * nearly always has a job in flight, so every other process was refused here with kIOReturnBusy (int-200: WidgetRenderer_Default). */
    if(liveObjects() || ownRangeOutstanding() || submit_->rangeOutstanding(queue_base,NativeAdrenoQueueRange))return kIOReturnBusy;
    queue_base_=queue_base;scoped_=true;window_start_=window_start;window_bytes_=window_bytes;window_peak_=0;
    return kIOReturnSuccess;
}
bool NativeAdrenoUserClient::ownRangeOutstanding() const
{
    return (scoped_ || used_) && submit_->rangeOutstanding(queue_base_,NativeAdrenoQueueRange);
}
IOReturn NativeAdrenoUserClient::close()
{
    if(!open_)return kIOReturnNotReady;
    if(liveObjects() || ownRangeOutstanding())return kIOReturnBusy;
    for(uint32_t i=1;i<queue_capacity_;++i)if(queues_[i].active)return kIOReturnBusy;
    releaseEntries();index_.release();
    if(queues_)IODeleteData(queues_,Queue,queue_capacity_);
    queues_=nullptr;queue_capacity_=0;
    queue_base_=0;scoped_=used_=false;window_start_=window_bytes_=window_peak_=0;
    open_=false;vm_=nullptr;submit_=nullptr;ring_=nullptr;timer_=nullptr;status_=nullptr;info_=nullptr;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::releaseAll(const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!open_)return kIOReturnNotReady;
    if(!clock)return kIOReturnBadArgument;
    IOReturn status=(scoped_ || used_)?submit_->waitRange(queue_base_,NativeAdrenoQueueRange,clock,timeout_us):kIOReturnSuccess;
    if(status)return status;
    for(size_t index=0;index<entry_count_;++index) {
        Entry &entry=at(index);
        if(!entry.used)continue;
        entry.mapping_leases=0;
        NativeAdrenoAbiBoFree request{entry.handle};
        status=freeBo(&request);
        if(status)return status;
    }
    for(uint32_t i=1;i<queue_capacity_;++i)queues_[i]=Queue{};
    return close();
}
NativeAdrenoMemory *NativeAdrenoUserClient::memoryForToken(uint32_t token)
{
    if(!open_ || !token || token>entry_count_)return nullptr;
    Entry &entry=at(token-1);
    return entry.used && !entry.unreleasable && entry.token==token && entry.mapping_leases?&entry.memory:nullptr;
}
uint32_t NativeAdrenoUserClient::mappingLeases(uint32_t token) const
{
    if(!open_ || !token || token>entry_count_)return 0;
    const Entry &entry=at(token-1);
    return entry.used && entry.token==token?entry.mapping_leases:0;
}
IOReturn NativeAdrenoUserClient::query(NativeAdrenoAbiParams *params)
{
    if(!params)return kIOReturnBadArgument;
    *params={};
    if(!open_)return kIOReturnNotReady;
    params->abi_version=NativeAdrenoAbiVersion;
    params->input_bits=input_bits_;params->output_bits=output_bits_;
    params->flags=NativeAdrenoParamsHostImport|(ownedCoherent()?NativeAdrenoParamsOwnedCoherent:0u)|
        (NativeAdrenoMemory::poolPressure()?NativeAdrenoParamsPoolPressure:0u);params->max_bo_bytes=maxBoBytes();
    return kIOReturnSuccess;
}
/* Per client slot (4, the kernel's own client last): [0] live objects now, [1] their peak, [2] refused BoAllocs, [3] last refusal status,
 * [4] bytes of the last refused request, [5] the window's highest mapped end in MiB above its start. Turnip turns any refusal into
 * VK_ERROR_OUT_OF_DEVICE_MEMORY, which its command streams do not check (int-237: backboardd wrote through a NULL stream pointer). */
extern "C" {volatile uint32_t native_adreno_client_stats[5][6];}
static volatile uint32_t *clientStats(uint32_t slot) {
    return native_adreno_client_stats[slot<4?slot:4];
}
static void noteLive(uint32_t slot,size_t live) {
    volatile uint32_t *stats=clientStats(slot);
    stats[0]=(uint32_t)live;if(live>stats[1])stats[1]=(uint32_t)live;
}
IOReturn NativeAdrenoUserClient::allocBo(const NativeAdrenoAbiBoAlloc *request,
    NativeAdrenoAbiBoAllocReply *reply)
{
    const IOReturn status=allocBoInner(request,reply);
    if(status && request) {
        volatile uint32_t *stats=clientStats(stats_slot_);
        stats[2]=stats[2]+1;stats[3]=(uint32_t)status;stats[4]=(uint32_t)request->bytes;
    }
    if(entry_count_)noteLive(stats_slot_,liveObjects());
    return status;
}
IOReturn NativeAdrenoUserClient::allocBoInner(const NativeAdrenoAbiBoAlloc *request,
    NativeAdrenoAbiBoAllocReply *reply)
{
    if(!request || !reply)return kIOReturnBadArgument;reply->handle=0;
    if(!open_)return kIOReturnNotReady;
    const uint64_t limit=maxBoBytes();
    if(!request->bytes || (request->bytes&4095) || request->bytes>limit ||
        request->align_log2<12 || request->align_log2>24 || request->contiguous>1)
        return kIOReturnBadArgument;
    Entry *entry=free();
    if(!entry)return kIOReturnNoResources;
    /* A scattered allocation costs one VM span per physical page (int-030: 16 MiB came back as 4096 single-page spans, the
     * whole span table), so larger objects try one contiguous run first and fall back to scattered pages unless the
     * caller demanded contiguity. */
    IOReturn status=kIOReturnUnsupported;
    const bool prefer_contiguous=request->contiguous==0 && request->bytes>=NativeAdrenoClientContiguousFirstBytes;
    if(prefer_contiguous) {
        status=entry->memory.allocate((size_t)request->bytes,output_bits_,true,(size_t)request->bytes,true);
        if(status && !entry->memory.quarantined() && entry->memory.release()==kIOReturnSuccess)status=kIOReturnUnsupported;
    }
    if(status)
        status=entry->memory.allocate((size_t)request->bytes,output_bits_,request->contiguous!=0,(size_t)request->bytes,true);
    /* User BOs are write-combined for the CPU, as the GPU maps them non-cacheable: no per-submission cache maintenance.
     * int-147 kept them cacheable under IO coherence and mapped the GPU side write-back: the first Turnip IB executed zeros and the GPU hung
     * (the kernel's cacheable alias read zeros past the first line too). Turnip maps every BO asking kIOMapWriteCombineCache, the user's
     * cache bits replace the kernel's (IOUserClient::mapClientMemory64), and XNU's view machinery changes the page attribute to match, so
     * the aliases disagreed. A coherent own BO needs a client that asks for a write-back view. */
    if(!status && native_adreno_wc_enabled && !ownedCoherent())status=entry->memory.makeWriteCombined();
    if(status) {
        /* A failed allocation can leave quarantine behind; never reuse it. */
        clear(*entry);
        if(entry->memory.release()!=kIOReturnSuccess) {
            entry->used=true;entry->unreleasable=true;++live_;
        }
        return status==kIOReturnUnsupported?kIOReturnUnsupported:status;
    }
    publish(*entry);
    entry->mapped=0;entry->iova=0;entry->mapped_state=false;
    reply->handle=entry->handle;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::importBo(const NativeAdrenoAbiBoImport *request,void *task,
    NativeAdrenoAbiBoAllocReply *reply)
{
    if(!request || !reply)return kIOReturnBadArgument;reply->handle=0;
    if(!open_)return kIOReturnNotReady;
    native_adreno_import_diag[0]=native_adreno_import_diag[0]+1;
    const uint64_t limit=maxBoBytes();
    if(!task || !request->address || ((request->address|request->bytes)&4095) || !request->bytes || request->bytes>limit) {
        noteImport(kIOReturnBadArgument,1,request->bytes,request->address);
        return kIOReturnBadArgument;
    }
    Entry *entry=free();
    if(!entry) {noteImport(kIOReturnNoResources,2,request->bytes,request->address);return kIOReturnNoResources;}
    const IOReturn status=entry->memory.importTask(task,request->address,(size_t)request->bytes,output_bits_,(size_t)request->bytes);
    if(status) {
        noteImport(status,native_adreno_import_stage,request->bytes,request->address);
        clear(*entry);
        if(entry->memory.release()!=kIOReturnSuccess) {entry->used=true;entry->unreleasable=true;++live_;}
        return status;
    }
    publish(*entry);
    entry->mapped=0;entry->iova=0;entry->mapped_state=false;entry->imported=true;
    reply->handle=entry->handle;
    native_adreno_import_live[0]=native_adreno_import_live[0]+1;
    {
        const unsigned slot=3+(gImportNext++%8)*4;
        native_adreno_import_live[slot]=request->address;native_adreno_import_live[slot+1]=(uint64_t)proc_selfpid()<<40|request->bytes;
        native_adreno_import_live[slot+2]=entry->handle;native_adreno_import_live[slot+3]=1;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::mapBo(const NativeAdrenoAbiBoMap *request,
    NativeAdrenoAbiBoMapReply *reply)
{
    if(!request || !reply)return kIOReturnBadArgument;reply->iova=0;
    if(!open_)return kIOReturnNotReady;
    if(!(request->permissions&1) || (request->permissions&~3u) || request->writeback>1 ||
        (request->shareability!=2 && request->shareability!=3) || request->reserved ||
        (request->iova&4095))return kIOReturnBadArgument;
    Entry *entry=find(request->handle);
    if(!entry || entry->unreleasable)return kIOReturnBadArgument;
    if(entry->mapped_state)return kIOReturnBusy;
    const uint64_t end=request->iova+entry->memory.bytes();
    if(window_bytes_ && (request->iova<window_start_ || end<request->iova || end>window_start_+window_bytes_)) {
        /* Per window: how often a mapping fell outside it, and the last one's size and end (MiB), for the boot log's [gpu windows] row. */
        const uint32_t slot=stats_slot_;
        if(slot<4) {
            native_adreno_window_overflow[slot][0]=native_adreno_window_overflow[slot][0]+1;
            native_adreno_window_overflow[slot][1]=uint32_t(entry->memory.bytes()>>10);
            native_adreno_window_overflow[slot][2]=uint32_t(end>>20);
        }
        return kIOReturnBadArgument;
    }
    uint64_t handle=0;
    /* Memory the owner maps read-only could only be pinned for reading: the GPU gets the read half of what was asked. */
    const unsigned permissions=entry->memory.readOnly()?(request->permissions&~2u):request->permissions;
    /* Imported memory is the owner's cacheable memory (IOSurfaces, no-copy buffers): once the board proved IO coherence, the GPU maps it
     * write-back inner-shareable (Linux's IOMMU_CACHE PTE) and the CPU caches over it need no maintenance. The GPU's own BOs stay
     * non-cacheable: their CPU view is write-combined, and a snoop could find a stale line of the kernel's cacheable alias. */
    /* Only memory the GPU never writes is mapped coherent: sysmem passes written through write-back inner-shareable mappings stopped
     * the GPU inside its end-of-pass cache clean (BR and BV idle at the clean events), while non-coherent mappings never did. */
    const bool coherent=(entry->memory.imported() && native_adreno_io_coherent && !(permissions&2u)) || (!entry->memory.imported() && ownedCoherent());
    const IOReturn status=vm_->map(&entry->memory,0,request->iova,permissions,
        coherent?1u:request->writeback,coherent?3u:request->shareability,&handle);
    if(status)return status;
    entry->mapped=handle;entry->iova=request->iova;entry->mapped_state=true;
    if(end>window_peak_)window_peak_=end;
    if(window_bytes_ && window_peak_>window_start_)clientStats(stats_slot_)[5]=(uint32_t)((window_peak_-window_start_)>>20);
    reply->iova=request->iova;
    return kIOReturnSuccess;
}
static void noteFreeRefused(bool imported,uint64_t handle,IOReturn status)
{
    if(!imported)return;
    native_adreno_import_live[1]=native_adreno_import_live[1]+1;native_adreno_import_live[2]=(uint32_t)status;noteImportState(handle,3);
}
IOReturn NativeAdrenoUserClient::freeBo(const NativeAdrenoAbiBoFree *request)
{
    native_adreno_trace_stage=1;
    if(!request)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    Entry *entry=find(request->handle);
    if(!entry)return kIOReturnBadArgument;
    if(entry->unreleasable) {noteFreeRefused(entry->imported,entry->handle,kIOReturnBusy);return kIOReturnBusy;}
    if(entry->mapping_leases) {noteFreeRefused(entry->imported,entry->handle,kIOReturnBusy);return kIOReturnBusy;}
    native_adreno_trace_stage=2;
    if(entry->mapped_state) {
        /* Completed jobs must not pin the buffer: retire what the CP has passed before the unmap decides. */
        submit_->retireCompleted(); /* a poisoned execbuf refuses and the unmap below stays refused too */
        native_adreno_trace_stage=3;
        const IOReturn unmap=vm_->unmap(entry->mapped);
        native_adreno_trace_stage=4;
        if(unmap) {noteFreeRefused(entry->imported,entry->handle,unmap);return unmap;}
        entry->mapped_state=false;entry->mapped=0;entry->iova=0;
    }
    const IOReturn release=entry->memory.release();
    if(release) {noteFreeRefused(entry->imported,entry->handle,release);return release;}
    if(entry->imported) {native_adreno_import_live[0]=native_adreno_import_live[0]-1;noteImportState(entry->handle,2);}
    clear(*entry);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::submit(const NativeAdrenoAbiSubmit *request,
    NativeAdrenoAbiSubmitReply *reply,const NativeAdrenoRingClock *clock)
{
    if(!request || !reply)return kIOReturnBadArgument;reply->fence=0;
    if(!open_)return kIOReturnNotReady;
    if(!request->buffer_count || request->buffer_count>NativeAdrenoAbiMaxBuffers ||
        request->reserved)return kIOReturnBadArgument;
    if(request->queue>=queue_capacity_ || !queues_[request->queue].active)
        return kIOReturnBadArgument;
    if(request->wait_fence) {
        if(!clock)return kIOReturnBadArgument;
        const uint32_t wait_us=request->wait_timeout_ns==0?0:
            (request->wait_timeout_ns>UINT64_C(0xffffffff)*1000u?0xffffffffu:
            (uint32_t)(request->wait_timeout_ns/1000));
        const IOReturn waited=submit_->observeFence((uint32_t)request->wait_fence,clock,wait_us);
        if(waited)return waited;
    }
    NativeAdrenoSubmitBuffer *buffers=submit_buffers_;
    for(uint32_t i=0;i<request->buffer_count;++i) {
        const NativeAdrenoAbiSubmitBuffer &wire=request->buffers[i];
        if(!wire.bytes || (wire.bytes&3))return kIOReturnBadArgument;
        Entry *entry=find(wire.handle);
        if(!entry || !entry->mapped_state)return kIOReturnBadArgument;
        buffers[i]={entry->mapped,wire.iova,(size_t)wire.bytes};
    }
    uint32_t fence=0;
    used_=true;
    ring_->useContext(context_);
    const IOReturn status=submit_->submit(vm_,buffers,request->buffer_count,
        queue_base_+request->queue,&fence,clock);
    if(status)return status;
    reply->fence=fence;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::attachTimer(const NativeAdrenoTimer *timer)
{
    if(!open_)return kIOReturnNotReady;
    if(!timer || timer_ )return kIOReturnBadArgument;
    timer_=timer;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::timestamp(NativeAdrenoAbiTimestamp *reply)
{
    if(!reply)return kIOReturnBadArgument;*reply={};
    if(!open_)return kIOReturnNotReady;
    if(!timer_)return kIOReturnUnsupported;
    return timer_->read(&reply->ticks);
}
IOReturn NativeAdrenoUserClient::attachDeviceInfo(const NativeAdrenoAbiDeviceInfo *info)
{
    if(!open_)return kIOReturnNotReady;
    if(!info || info_ || info->reserved || !info->chip_id || !info->va_size)return kIOReturnBadArgument;
    info_=info;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::deviceInfo(NativeAdrenoAbiDeviceInfo *reply)
{
    if(!reply)return kIOReturnBadArgument;*reply={};
    if(!open_)return kIOReturnNotReady;
    if(!info_)return kIOReturnUnsupported;
    *reply=*info_;
    if(window_bytes_){reply->va_start=window_start_;reply->va_size=window_bytes_;}
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::attachStatus(const NativeAdrenoStatus *status)
{
    if(!open_)return kIOReturnNotReady;
    if(!status || status_)return kIOReturnBadArgument;
    status_=status;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::status(NativeAdrenoAbiStatus *reply)
{
    if(!reply)return kIOReturnBadArgument;*reply={};
    if(!open_)return kIOReturnNotReady;
    if(!status_)return kIOReturnUnsupported;
    /* msm MSM_PARAM_FAULTS: the global count plus this client's own VM's (a hang its submission caused). */
    const NativeAdrenoStatusCounts counts=status_->counts();
    reply->faults=counts.faults+(vm_?vm_->faults():0);reply->suspends=counts.suspends;
    return kIOReturnSuccess;
}
size_t NativeAdrenoUserClient::inputSize(uint32_t method)
{
    switch(method) {
    case NativeAdrenoMethodQuery:
    case NativeAdrenoMethodGetTimestamp:
    case NativeAdrenoMethodGetStatus:
    case NativeAdrenoMethodGetDeviceInfo:return 0;
    case NativeAdrenoMethodBoAlloc:return sizeof(NativeAdrenoAbiBoAlloc);
    case NativeAdrenoMethodBoMap:return sizeof(NativeAdrenoAbiBoMap);
    case NativeAdrenoMethodBoImport:return sizeof(NativeAdrenoAbiBoImport);
    case NativeAdrenoMethodBoFree:return sizeof(NativeAdrenoAbiBoFree);
    case NativeAdrenoMethodSubmit:return sizeof(NativeAdrenoAbiSubmit);
    case NativeAdrenoMethodSetMetadata:
    case NativeAdrenoMethodGetMetadata:return sizeof(NativeAdrenoAbiMetadata);
    case NativeAdrenoMethodCreateQueue:
    case NativeAdrenoMethodCloseQueue:return sizeof(NativeAdrenoAbiQueue);
    case NativeAdrenoMethodBoCpuMap:return sizeof(NativeAdrenoAbiBoCpuMap);
    case NativeAdrenoMethodBoCpuUnmap:return sizeof(NativeAdrenoAbiBoCpuUnmap);
    case NativeAdrenoMethodWaitFence:return sizeof(NativeAdrenoAbiWaitFence);
    default:return SIZE_MAX;
    }
}
size_t NativeAdrenoUserClient::outputSize(uint32_t method)
{
    switch(method) {
    case NativeAdrenoMethodQuery:return sizeof(NativeAdrenoAbiParams);
    case NativeAdrenoMethodBoAlloc:return sizeof(NativeAdrenoAbiBoAllocReply);
    case NativeAdrenoMethodBoMap:return sizeof(NativeAdrenoAbiBoMapReply);
    case NativeAdrenoMethodBoFree:return 0;
    case NativeAdrenoMethodSubmit:return sizeof(NativeAdrenoAbiSubmitReply);
    case NativeAdrenoMethodWaitFence:return 0;
    case NativeAdrenoMethodGetTimestamp:return sizeof(NativeAdrenoAbiTimestamp);
    case NativeAdrenoMethodGetStatus:return sizeof(NativeAdrenoAbiStatus);
    case NativeAdrenoMethodGetDeviceInfo:return sizeof(NativeAdrenoAbiDeviceInfo);
    case NativeAdrenoMethodGetMetadata:return sizeof(NativeAdrenoAbiMetadata);
    case NativeAdrenoMethodSetMetadata:return 0;
    case NativeAdrenoMethodCreateQueue:return sizeof(NativeAdrenoAbiQueueReply);
    case NativeAdrenoMethodCloseQueue:return 0;
    case NativeAdrenoMethodBoCpuMap:return sizeof(NativeAdrenoAbiBoCpuMapReply);
    case NativeAdrenoMethodBoCpuUnmap:return 0;
    case NativeAdrenoMethodBoImport:return sizeof(NativeAdrenoAbiBoAllocReply);
    default:return SIZE_MAX;
    }
}
IOReturn NativeAdrenoUserClient::dispatch(uint32_t method,const void *input,
    size_t input_size,void *output,size_t output_size,const NativeAdrenoRingClock *clock)
{
    struct Tally {
        uint32_t method;uint64_t began;IOReturn *result;const NativeAdrenoRingClock *clock;
        Tally(uint32_t m,IOReturn *r,const NativeAdrenoRingClock *c):method(m),began(c && c->now_us?c->now_us(c->cookie):0),result(r),clock(c) {}
        ~Tally() {
            if(method>=20)return;
            const uint64_t now=clock && clock->now_us?clock->now_us(clock->cookie):0;
            native_adreno_method_stats[method][0]=native_adreno_method_stats[method][0]+1;
            if(*result)native_adreno_method_stats[method][1]=native_adreno_method_stats[method][1]+1;
            native_adreno_method_stats[method][2]=(uint32_t)*result;
            native_adreno_method_stats[method][3]=native_adreno_method_stats[method][3]+(uint32_t)(now-began);
        }
    };
    /* The same clock times the VM's table transactions (the board service's probe never ran on this path, so nothing else installed it). */
    static const NativeAdrenoRingClock *vm_clock=nullptr;
    vm_clock=clock;
    native_adreno_vm_now_us=[]()->uint64_t{return vm_clock && vm_clock->now_us?vm_clock->now_us(vm_clock->cookie):0;};
    IOReturn tally_result=kIOReturnSuccess;
    Tally tally(method,&tally_result,clock);
    const size_t expected_in=inputSize(method),expected_out=outputSize(method);
    if(expected_in==SIZE_MAX || expected_out==SIZE_MAX)return kIOReturnUnsupported;
    if(input_size!=expected_in || output_size!=expected_out)return kIOReturnBadArgument;
    if((expected_in && !input) || (expected_out && !output))return kIOReturnBadArgument;
    IOReturn status=kIOReturnSuccess;
    switch(method) {
    case NativeAdrenoMethodQuery: {
        NativeAdrenoAbiParams reply{};
        status=query(&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodBoAlloc: {
        NativeAdrenoAbiBoAlloc request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiBoAllocReply reply{};
        status=allocBo(&request,&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodBoImport:
        status=kIOReturnUnsupported;
        break;
    case NativeAdrenoMethodBoMap: {
        NativeAdrenoAbiBoMap request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiBoMapReply reply{};
        status=mapBo(&request,&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        else {
            /* Why maps fail (int-135: 55 of 191, and the translator answers each as a device out-of-memory): status, count, and what the table
             * held and the failed request asked for the last time. */
            unsigned at=0;while(at<5 && native_adreno_map_fail[at][0] && native_adreno_map_fail[at][0]!=(uint32_t)status)++at;
            native_adreno_map_fail[at][0]=(uint32_t)status;native_adreno_map_fail[at][1]=native_adreno_map_fail[at][1]+1;
            native_adreno_map_fail[5][0]=(uint32_t)vm_->spanCount();native_adreno_map_fail[5][1]=(uint32_t)vm_->objectCount();
            native_adreno_map_fail[6][0]=(uint32_t)(request.iova>>12);native_adreno_map_fail[6][1]=(uint32_t)(request.handle);
        }
        break;
    }
    case NativeAdrenoMethodBoFree: {
        NativeAdrenoAbiBoFree request{};
        memcpy(&request,input,sizeof(request));
        status=freeBo(&request);
        break;
    }
    case NativeAdrenoMethodSubmit: {
        NativeAdrenoAbiSubmit &request=submit_request_;
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiSubmitReply reply{};
        status=submit(&request,&reply,clock);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodWaitFence: {
        NativeAdrenoAbiWaitFence request{};
        memcpy(&request,input,sizeof(request));
        status=waitFence(&request,clock);
        break;
    }
    case NativeAdrenoMethodGetDeviceInfo: {
        NativeAdrenoAbiDeviceInfo reply{};
        status=deviceInfo(&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodGetTimestamp: {
        NativeAdrenoAbiTimestamp reply{};
        status=timestamp(&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodBoCpuMap: {
        NativeAdrenoAbiBoCpuMap request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiBoCpuMapReply reply{};
        status=cpuMap(&request,&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodBoCpuUnmap: {
        NativeAdrenoAbiBoCpuUnmap request{};
        memcpy(&request,input,sizeof(request));
        status=cpuUnmap(&request);
        break;
    }
    case NativeAdrenoMethodCreateQueue: {
        NativeAdrenoAbiQueueReply reply{};
        status=createQueue(&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodCloseQueue: {
        NativeAdrenoAbiQueue request{};
        memcpy(&request,input,sizeof(request));
        status=closeQueue(&request);
        break;
    }
    case NativeAdrenoMethodSetMetadata: {
        NativeAdrenoAbiMetadata request{};
        memcpy(&request,input,sizeof(request));
        status=setMetadata(&request);
        break;
    }
    case NativeAdrenoMethodGetMetadata: {
        NativeAdrenoAbiMetadata request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiMetadata reply{};
        status=getMetadata(&request,&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    case NativeAdrenoMethodGetStatus: {
        NativeAdrenoAbiStatus reply{};
        status=this->status(&reply);
        if(!status)memcpy(output,&reply,sizeof(reply));
        break;
    }
    default:return kIOReturnUnsupported;
    }
    tally_result=status;
    return status;
}
IOReturn NativeAdrenoUserClient::cpuMap(const NativeAdrenoAbiBoCpuMap *request,
    NativeAdrenoAbiBoCpuMapReply *reply)
{
    if(!request || !reply)return kIOReturnBadArgument;*reply={};
    if(!open_)return kIOReturnNotReady;
    Entry *entry=find(request->handle);
    if(!entry || entry->unreleasable)return kIOReturnBadArgument;
    /* An imported BO is the caller's own memory, already mapped where it came from. */
    if(entry->memory.imported())return kIOReturnUnsupported;
    if(entry->mapping_leases==UINT32_MAX)return kIOReturnNoResources;
    ++entry->mapping_leases;
    reply->token=entry->token;reply->bytes=entry->memory.bytes();
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::cpuUnmap(const NativeAdrenoAbiBoCpuUnmap *request)
{
    if(!request)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    if(request->reserved)return kIOReturnBadArgument;
    Entry *entry=find(request->handle);
    if(!entry || entry->unreleasable || !entry->token)return kIOReturnBadArgument;
    if(request->token!=entry->token)return kIOReturnBadArgument;
    if(!entry->mapping_leases)return kIOReturnBusy;
    --entry->mapping_leases;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::createQueue(NativeAdrenoAbiQueueReply *reply)
{
    if(!reply)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    for(;;) {
        for(uint32_t i=1;i<queue_capacity_;++i)if(!queues_[i].active) {
            queues_[i]={i,true};
            reply->queue=i;
            return kIOReturnSuccess;
        }
        if(!growQueues())return kIOReturnNoMemory;
    }
}
IOReturn NativeAdrenoUserClient::closeQueue(const NativeAdrenoAbiQueue *request)
{
    if(!request)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    if(!request->queue || request->queue>=queue_capacity_)return kIOReturnBadArgument;
    Queue &queue=queues_[request->queue];
    if(!queue.active)return kIOReturnBadArgument;
    /* A queue must be drained by the caller before it disappears. */
    if(submit_->queueOutstanding(queue_base_+request->queue))
        return kIOReturnBusy;
    queue={};
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::setMetadata(const NativeAdrenoAbiMetadata *request)
{
    if(!request)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    if(request->reserved || request->bytes>NativeAdrenoAbiMetadataBytes)return kIOReturnBadArgument;
    Entry *entry=find(request->handle);
    if(!entry || entry->unreleasable)return kIOReturnBadArgument;
    if(request->bytes)memcpy(entry->metadata,request->text,request->bytes);
    entry->metadata_bytes=request->bytes;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::getMetadata(const NativeAdrenoAbiMetadata *request,
    NativeAdrenoAbiMetadata *reply)
{
    if(!request || !reply)return kIOReturnBadArgument;
    /* Snapshot the request so an in/out alias is still well defined. */
    const uint64_t handle=request->handle;
    const uint32_t asked=request->bytes;
    const uint32_t reserved=request->reserved;
    *reply={};
    if(!open_)return kIOReturnNotReady;
    if(reserved || asked>NativeAdrenoAbiMetadataBytes)return kIOReturnBadArgument;
    Entry *entry=find(handle);
    if(!entry || entry->unreleasable)return kIOReturnBadArgument;
    if(asked<entry->metadata_bytes)return kIOReturnBadArgument;
    reply->handle=entry->handle;reply->bytes=entry->metadata_bytes;
    if(entry->metadata_bytes)memcpy(reply->text,entry->metadata,entry->metadata_bytes);
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoUserClient::waitFence(const NativeAdrenoAbiWaitFence *request,
    const NativeAdrenoRingClock *clock)
{
    if(!request || !clock)return kIOReturnBadArgument;
    if(!open_)return kIOReturnNotReady;
    if(!request->fence)return kIOReturnBadArgument;
    const uint32_t timeout_us=request->timeout_ns==0?0:
        (request->timeout_ns>UINT64_C(0xffffffff)*1000u?0xffffffffu:
        (uint32_t)(request->timeout_ns/1000));
    return submit_->wait((uint32_t)request->fence,clock,timeout_us);
}
