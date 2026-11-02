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
    NativeGpuPixelPiece p{};auto &h=p.header;h.magic=0x31585047;h.version=1;h.stage=1;h.capture=1;
    h.width=16;h.height=32;h.grid_width=16;h.grid_height=32;h.row_bytes=64;h.bytes_per_pixel=4;h.bytes=2048;h.flags=1;
    p.bytes=1024;memset(p.data,0x12,sizeof(p.data));
    assert(native_gpu_pixel_put(&p,42)==0);
    assert(native_gpu_pixel_put(&p,42)!=0); // duplicate fragment
    p.offset=1024;assert(native_gpu_pixel_put(&p,43)!=0); // different process cannot append
    assert(native_gpu_pixel_put(&p,42)==0);
    assert(native_gpu_pixel_put(&p,42)!=0); // completed slot is immutable
    std::vector<uint8_t> pixels(16+24*(112+65536));bytes=native_gpu_pixel_snapshot(pixels.data(),pixels.size());
    assert(bytes==16+112+2048);auto *ph=(NativeGpuPixelHeader *)(pixels.data()+16);assert(ph->pid==42 && ph->capture==1);
    p.offset=0;h.flags=0;assert(native_gpu_pixel_put(&p,42)!=0);h.flags=1;
    for(unsigned n=2;n<=3;++n){h.capture=n;assert(native_gpu_pixel_put(&p,42)==0);p.offset=1024;assert(native_gpu_pixel_put(&p,42)==0);p.offset=0;}
    h.capture=4;assert(native_gpu_pixel_put(&p,42)!=0); // per-stage quota
    h.capture=9;h.row_bytes=1;assert(native_gpu_pixel_put(&p,77)!=0);
    return 0;
}
