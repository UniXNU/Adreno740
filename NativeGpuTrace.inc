#include "NativeGpuThreadTargets.h"
#include "NativeGpuLatency.h"
static NativeGpuLatencyWaits gpuLatencyWaits;
static NativeGpuThreadTargets gpuThreadTargets;
extern "C" {void (*native_gpu_dispatch_hook)(uint32_t,const uint64_t *,uint32_t);}
extern "C" uint64_t native_gpu_render_thread(uint32_t pid) {return gpuThreadTargets.find(pid);}
/* SPDX-License-Identifier: MIT */
/* Included by the IOKit publisher; no allocation or GPU lock in a trace write. */
#include "NativeGpuTraceStore.h"
static NativeGpuTraceStore gpuTrace;
static uint32_t gpuTraceOn,gpuPixelsOn;
static NativeGpuTraceRecord *gpuMetadata;
static uint32_t gpuMetadataBusy;
struct GpuPixelSlot { uint32_t busy,used,received,complete; NativeGpuPixelHeader header; uint8_t data[NativeGpuPixelBytes]; uint32_t trace_count; NativeGpuTraceRecord trace[NativeGpuCaptureRecords]; };
static GpuPixelSlot *gpuPixels;
static uint32_t gpuPixelWriter;
struct GpuPixelWriteGuard { ~GpuPixelWriteGuard(){__atomic_store_n(&gpuPixelWriter,0,__ATOMIC_RELEASE);} };
static void gpuTraceInit()
{
    unsigned enabled=0,pixels=0;
    PE_parse_boot_argn("sheng-gpu-trace",&enabled,sizeof(enabled));
    /* Pixel capture is outside latency diagnosis. */
    if(enabled!=1)return;
    gpuTrace.slots=static_cast<NativeGpuTraceSlot *>(IOMallocZero(sizeof(NativeGpuTraceSlot)*NativeGpuTraceCapacity));
    if(!gpuTrace.slots)return;
    gpuTrace.capacity=NativeGpuTraceCapacity;
    gpuMetadata=static_cast<NativeGpuTraceRecord *>(IOMallocZero(sizeof(NativeGpuTraceRecord)*NativeGpuTraceMetadata));
    if(pixels==1)gpuPixels=static_cast<GpuPixelSlot *>(IOMallocZero(sizeof(GpuPixelSlot)*NativeGpuPixelSlots));
    __atomic_store_n(&gpuPixelsOn,gpuPixels?1:0,__ATOMIC_RELEASE);
    __atomic_store_n(&gpuTraceOn,1,__ATOMIC_RELEASE);
}
extern "C" int native_gpu_trace_enabled(void) {return __atomic_load_n(&gpuTraceOn,__ATOMIC_ACQUIRE);}
extern "C" void native_gpu_trace_record(uint32_t event,uint32_t pid,const uint64_t *values,uint32_t count)
{
    if(!native_gpu_trace_enabled() || count>12 || (count && !values) || !native_gpu_latency_event(event,values,count))return;
    NativeGpuTraceRecord record{};uint64_t ns=0;
    absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
    record.us=ns/1000;record.event=event;record.pid=pid;
    for(uint32_t i=0;i<count;++i)record.value[i]=values[i];
    if(event==22 && count>=9 && !gpuLatencyWaits.keep(record.us,pid,record.value[0],(uint32_t)record.value[1],record.value[2],record.value[9]))return;
    record.sequence=gpuTrace.push(record);
    if(event==68 && count>=6 && native_gpu_dispatch_hook)native_gpu_dispatch_hook(pid,values,count);
    if(event==63 && count>=2 && record.value[1]==1)gpuThreadTargets.record(pid,record.value[0]);
    const bool meta=event<=3 || event==GpuTraceImport || event==46 || event==47 || event==48 || event==56 || event==57 || event==58 || event==59 || event==63 || event==67 || (event==68 && ((record.value[1]&255)==0 || (record.value[1]&255)==5));
    uint32_t free=0;
    if(meta && gpuMetadata && __atomic_compare_exchange_n(&gpuMetadataBusy,&free,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED)) {
        unsigned pick=NativeGpuTraceMetadata;
        for(unsigned i=0;i<NativeGpuTraceMetadata;++i) {
            auto &m=gpuMetadata[i];
            if(!m.event && pick==NativeGpuTraceMetadata)pick=i;
            if(m.event==event && m.pid==pid && (event!=63 || m.value[0]==record.value[0]) && (event!=67 || (m.value[0]==record.value[0] && m.value[1]==record.value[1])) && (event!=68 || (m.value[0]==record.value[0] && m.value[1]==record.value[1] && m.value[2]==record.value[2])) && (event!=GpuTraceImport || m.value[3]==record.value[3]) &&
               ((event!=56 && event!=57) || (m.value[0]==record.value[0] && m.value[1]==record.value[1] && m.value[2]==record.value[2]))){pick=i;break;}
        }
        if(pick<NativeGpuTraceMetadata) {
            if(event==GpuTraceImport && record.value[0]==0)gpuMetadata[pick]={};
            else gpuMetadata[pick]=record;
        } else __atomic_add_fetch(&gpuTrace.dropped,1,__ATOMIC_RELAXED);
        __atomic_store_n(&gpuMetadataBusy,0,__ATOMIC_RELEASE);
    }

}
extern "C" uint32_t native_gpu_trace_snapshot(void *out,uint32_t bytes)
{
    if(!native_gpu_trace_enabled() || !out || bytes<sizeof(NativeGpuTraceHeader))return 0;
    auto *header=static_cast<NativeGpuTraceHeader *>(out);
    *header={0x31545047,NativeGpuTraceVersion,sizeof(NativeGpuTraceRecord),0,
             __atomic_load_n(&gpuTrace.issued,__ATOMIC_ACQUIRE),__atomic_load_n(&gpuTrace.dropped,__ATOMIC_RELAXED)};
    auto *records=reinterpret_cast<NativeGpuTraceRecord *>(header+1);
    const uint32_t room=(bytes-sizeof(*header))/sizeof(*records);
    uint32_t free=0;
    if(gpuMetadata && __atomic_compare_exchange_n(&gpuMetadataBusy,&free,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED)) {
        for(unsigned i=0;i<NativeGpuTraceMetadata && header->count<room;++i)
            if(gpuMetadata[i].event)records[header->count++]=gpuMetadata[i];
        __atomic_store_n(&gpuMetadataBusy,0,__ATOMIC_RELEASE);
    }
    const uint64_t available=room-header->count;
    const uint64_t keep=available<gpuTrace.capacity?available:gpuTrace.capacity;
    const uint64_t first=header->issued>keep?header->issued-keep+1:1;
    for(uint64_t n=first;n<=header->issued;++n)
        if(gpuTrace.read(n,records+header->count))++header->count;
    // Each captured image retains the preceding trace window after the live ring wraps.
    if(gpuPixels)for(unsigned i=0;i<NativeGpuPixelSlots && header->count<room;++i) {
        auto &slot=gpuPixels[i];uint32_t unlocked=0;
        if(!__atomic_compare_exchange_n(&slot.busy,&unlocked,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))continue;
        if(slot.complete)for(unsigned n=0;n<slot.trace_count && header->count<room;++n)records[header->count++]=slot.trace[n];
        __atomic_store_n(&slot.busy,0,__ATOMIC_RELEASE);
    }
    return sizeof(*header)+header->count*sizeof(*records);
}
extern "C" int native_gpu_pixel_put(const NativeGpuPixelPiece *piece,uint32_t pid)
{
    if(!__atomic_load_n(&gpuPixelsOn,__ATOMIC_ACQUIRE))return -1;
    if(!piece || !piece->bytes || piece->bytes>NativeGpuPixelChunk)return -2;
    NativeGpuPixelHeader h=piece->header;h.pid=pid;
    if(h.magic!=0x31585047 || h.version!=1 || !(h.flags&1) || !h.capture || !h.bytes ||
       h.bytes>NativeGpuPixelBytes || h.stage<1 || h.stage>3 || (pid==0 ? h.stage!=3 : h.stage==3) || !h.grid_width || !h.grid_height ||
       h.grid_width>h.width || h.grid_height>h.height ||
       (h.bytes_per_pixel!=4 && h.bytes_per_pixel!=8) ||
       uint64_t(h.row_bytes)!=uint64_t(h.grid_width)*h.bytes_per_pixel ||
       uint64_t(h.bytes)!=uint64_t(h.row_bytes)*h.grid_height ||
       piece->offset>h.bytes || piece->bytes>h.bytes-piece->offset)return -2;
    uint32_t unlocked=0;
    if(!__atomic_compare_exchange_n(&gpuPixelWriter,&unlocked,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))return -4;
    GpuPixelWriteGuard guard;
    // Each pid owns at most three captures per stage. An untrusted client cannot
    // allocate or overwrite another client's retained samples.
    unsigned owned=0;
    for(unsigned i=0;i<NativeGpuPixelSlots;++i) {
        auto &s=gpuPixels[i];uint32_t empty=0;
        if(!__atomic_compare_exchange_n(&s.busy,&empty,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))return -4;
        if(s.used && s.header.pid==pid && s.header.stage==h.stage)++owned;
        if(s.used && s.header.pid==pid && s.header.capture==h.capture && s.header.stage==h.stage) {
            const bool valid=!s.complete && s.received==piece->offset && !memcmp(&s.header,&h,sizeof(h));
            if(valid){memcpy(s.data+s.received,piece->data,piece->bytes);s.received+=piece->bytes;s.complete=s.received==h.bytes;}
            __atomic_store_n(&s.busy,0,__ATOMIC_RELEASE);return valid?0:-2;
        }
        __atomic_store_n(&s.busy,0,__ATOMIC_RELEASE);
    }
    if(piece->offset || owned>=3)return -3;
    for(unsigned i=0;i<NativeGpuPixelSlots;++i) {
        auto &s=gpuPixels[i];uint32_t empty=0;
        if(!__atomic_compare_exchange_n(&s.busy,&empty,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))return -4;
        if(!s.used) {
            const uint64_t issued=__atomic_load_n(&gpuTrace.issued,__ATOMIC_ACQUIRE);
            const uint64_t first=issued>NativeGpuCaptureRecords?issued-NativeGpuCaptureRecords+1:1;
            for(uint64_t n=first;n<=issued;++n)if(gpuTrace.read(n,s.trace+s.trace_count))++s.trace_count;
            s.used=1;s.header=h;memcpy(s.data,piece->data,piece->bytes);s.received=piece->bytes;s.complete=s.received==h.bytes;
            __atomic_store_n(&s.busy,0,__ATOMIC_RELEASE);return 0;
        }
        __atomic_store_n(&s.busy,0,__ATOMIC_RELEASE);
    }
    return -3;
}
extern "C" uint32_t native_gpu_pixel_snapshot(void *out,uint32_t bytes)
{
    if(!gpuPixels || !out || bytes<16)return 0;
    uint32_t *head=static_cast<uint32_t *>(out);head[0]=0x31535047;head[1]=1;head[2]=0;head[3]=0;
    uint32_t used=16;
    for(unsigned i=0;i<NativeGpuPixelSlots;++i) {
        auto &s=gpuPixels[i];uint32_t empty=0;
        if(!__atomic_compare_exchange_n(&s.busy,&empty,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))continue;
        const uint32_t size=sizeof(s.header)+s.header.bytes;
        if(s.complete && size<=bytes-used) {
            memcpy(static_cast<uint8_t *>(out)+used,&s.header,sizeof(s.header));
            memcpy(static_cast<uint8_t *>(out)+used+sizeof(s.header),s.data,s.header.bytes);
            used+=size;++head[2];
        } else if(s.used)++head[3];
        __atomic_store_n(&s.busy,0,__ATOMIC_RELEASE);
    }
    return used;
}
