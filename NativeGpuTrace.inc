#include "NativeGpuThreadTargets.h"
#include "NativeGpuLatency.h"
static NativeGpuLatencyWaits gpuLatencyWaits;
static NativeGpuThreadTargets gpuThreadTargets;
extern "C" uint64_t native_gpu_role_thread(uint32_t pid,uint32_t role) {return gpuThreadTargets.find(pid,role);}
/* SPDX-License-Identifier: MIT */
/* Included by the IOKit publisher; no allocation or GPU lock in a trace write. */
#include "NativeGpuTraceStore.h"
static NativeGpuTraceStore gpuTrace;
static uint32_t gpuTraceOn;
static NativeGpuTraceRecord *gpuMetadata;
static uint32_t gpuMetadataBusy;
static void gpuTraceInit()
{
    unsigned enabled=0;
    PE_parse_boot_argn("sheng-gpu-trace",&enabled,sizeof(enabled));
    if(enabled!=1)return;
    gpuTrace.slots=static_cast<NativeGpuTraceSlot *>(IOMallocZero(sizeof(NativeGpuTraceSlot)*NativeGpuTraceCapacity));
    if(!gpuTrace.slots)return;
    gpuTrace.capacity=NativeGpuTraceCapacity;
    gpuMetadata=static_cast<NativeGpuTraceRecord *>(IOMallocZero(sizeof(NativeGpuTraceRecord)*NativeGpuTraceMetadata));
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
    if(event==GpuTraceThread && count>=2)gpuThreadTargets.record(pid,(uint32_t)record.value[1],record.value[0]);
    const bool meta=event<=3 || event==GpuTraceThread || event==GpuTraceProblem;
    uint32_t free=0;
    if(meta && gpuMetadata && __atomic_compare_exchange_n(&gpuMetadataBusy,&free,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED)) {
        unsigned pick=NativeGpuTraceMetadata;
        for(unsigned i=0;i<NativeGpuTraceMetadata;++i) {
            auto &m=gpuMetadata[i];
            if(!m.event && pick==NativeGpuTraceMetadata)pick=i;
            if(m.event==event && m.pid==pid && (event!=GpuTraceThread || m.value[0]==record.value[0]) &&
               (event!=GpuTraceProblem || !memcmp(m.value,record.value,sizeof m.value))){pick=i;break;}
        }
        if(pick<NativeGpuTraceMetadata)gpuMetadata[pick]=record;
        else __atomic_add_fetch(&gpuTrace.dropped,1,__ATOMIC_RELAXED);
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
    return sizeof(*header)+header->count*sizeof(*records);
}
