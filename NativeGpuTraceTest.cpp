// SPDX-License-Identifier: MIT
// Host checks for the diagnostic store, with the allocation/clock boundary replaced.
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <vector>
#include "NativeGpuTrace.h"
static void PE_parse_boot_argn(const char *,unsigned *out,size_t){*out=1;}
static void *IOMallocZero(size_t n){return calloc(1,n);}
static uint64_t mach_absolute_time(){return std::chrono::steady_clock::now().time_since_epoch().count();}
static void absolutetime_to_nanoseconds(uint64_t t,uint64_t *out){*out=t;}
#include "NativeGpuTrace.inc"
int main() {
    NativeGpuThreadTargets targets;assert(targets.find(61)==0);targets.record(61,100);
    targets.record(61,200);targets.record(0,300);assert(targets.find(61)==200);assert(targets.find(0)==0);
    gpuTraceInit();
    static unsigned dispatch_seen=0;
    native_gpu_dispatch_hook=[](uint32_t pid,const uint64_t *v,uint32_t count){assert(pid==61 && count==12 && v[0]==123);++dispatch_seen;};
    uint64_t dispatch[12]={123,1};native_gpu_trace_record(68,61,dispatch,12);assert(dispatch_seen==1);native_gpu_dispatch_hook=nullptr;
    uint64_t thread[6]={3456,1,0,0,0,0};native_gpu_trace_record(63,61,thread,6);
    assert(native_gpu_render_thread(61)==3456);

    NativeGpuTraceSlot slots[8]{};NativeGpuTraceStore store;store.slots=slots;store.capacity=8;
    for(unsigned i=0;i<32;++i){NativeGpuTraceRecord r{};r.value[0]=i;assert(store.push(r));}
    NativeGpuTraceRecord row{};assert(!store.read(24,&row));assert(store.read(25,&row) && row.value[0]==24);
    std::vector<std::thread> writers;
    for(unsigned t=0;t<4;++t)writers.emplace_back([&,t]{for(unsigned i=0;i<10000;++i){NativeGpuTraceRecord r{};r.pid=t;r.value[0]=i;r.value[1]=~uint64_t(i);store.push(r);}});
    for(auto &t:writers)t.join();
    for(uint64_t i=store.issued-7;i<=store.issued;++i)if(store.read(i,&row))assert(row.value[1]==~row.value[0]);
    gpuTraceInit();gpuTraceEvent(GpuTraceBuild,0,1234);
    for(unsigned i=0;i<NativeGpuTraceCapacity+10;++i)gpuTraceEvent(GpuTraceSubmit,7,i);
    std::vector<uint8_t> output(32+(NativeGpuTraceCapacity+NativeGpuTraceMetadata)*120);
    unsigned bytes=native_gpu_trace_snapshot(output.data(),output.size());assert(bytes>32);
    auto *head=(NativeGpuTraceHeader *)output.data();auto *rows=(NativeGpuTraceRecord *)(head+1);
    bool build=false;for(unsigned i=0;i<head->count;++i)build|=rows[i].event==GpuTraceBuild && rows[i].value[0]==1234;
    assert(build);
    auto issued=gpuTrace.issued;
    uint64_t noise[3]={0,20,0};native_gpu_trace_record(39,61,noise,3);
    native_gpu_trace_record(GpuTraceFenceQuery,0,noise,3);
    native_gpu_trace_record(GpuTraceGmem,61,noise,3);
    assert(gpuTrace.issued==issued);
    noise[0]=123;noise[1]=53;native_gpu_trace_record(39,61,noise,3);assert(gpuTrace.issued==issued+1);
    NativeGpuLatencyWaits waits;uint64_t omitted=0;
    assert(waits.keep(1,0,55,3,99,omitted));
    for(unsigned i=2;i<100001;++i)assert(!waits.keep(i,0,55,3,99,omitted));
    assert(waits.keep(100002,0,55,3,99,omitted) && omitted==99999);
    assert(waits.keep(100003,0,55,4,99,omitted));
    assert(waits.keep(100004,0,55,1,99,omitted));
    assert(waits.keep(100005,0,55,3,100,omitted));
    NativeGpuPixelPiece p{};assert(native_gpu_pixel_put(&p,42)!=0);
    uint8_t pixels[16];assert(native_gpu_pixel_snapshot(pixels,sizeof pixels)==0);
    return 0;
}
