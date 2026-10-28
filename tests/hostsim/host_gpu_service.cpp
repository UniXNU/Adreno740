/* SPDX-License-Identifier: MIT */
/* The kernel half of the NativeAdreno stack on the host: the real GPU context bring-up (software SMMU register peer), VM, ring,
 * submit object and user-client core over the authored memory doubles, with a software "CP" that retires every submission at
 * once. It runs no shader and touches no hardware; it exists so the Turnip darwin backend can be driven through a real
 * Vulkan instance before the userspace side ever runs on the tablet. Narrow C interface for the IOKit shim translation unit. */
#include <unistd.h>
#include "XnuMemoryFixture.h"
#include "NativeAdrenoGpuContext.h"
#include "NativeAdrenoUserClient.h"
#include "NativeAdrenoTimer.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <mutex>

IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *state) {*state={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *,size_t,NativeAdrenoCacheOperation) {return kIOReturnSuccess;}

namespace {
constexpr size_t kSmmuBytes=0x40000;
alignas(8) uint32_t mmio[kSmmuBytes/4];
int sread32(void *,size_t at,uint32_t *v) {*v=mmio[at/4];return 0;}
int sread64(void *,size_t at,uint64_t *v) {std::memcpy(v,mmio+at/4,8);return 0;}
int swrite32(void *,size_t at,uint32_t v) {
    if((at&0xfff)==0x58 && at>=0x20000) {mmio[at/4]&=~v;return 0;}   /* context FSR is write-one-to-clear */
    mmio[at/4]=v;return 0;
}
int swrite64(void *,size_t at,uint64_t v) {std::memcpy(mmio+at/4,&v,8);return 0;}
int sbarrier(void *) {return 0;}

/* Software CP: a write of the ring write pointer retires everything up to it, and the fence the ring reads is the number of
 * submissions so far (one write pointer update per submission). */
std::vector<uint32_t> regs(0x10000);
volatile uint32_t *memptrs=nullptr;
unsigned submissions=0;
std::atomic<bool> holdCompletion{false};
std::recursive_mutex serviceLock;
unsigned calls[32]={},successes[32]={};
uint32_t faultRemaining[32]={};
IOReturn faultStatus[32]={};
uint64_t lastMapIova=0;
void completeIfReleased() {
    if(!holdCompletion.load(std::memory_order_acquire)) {
        regs[0x0806]=regs[0x0807];
        if(memptrs)memptrs[1]=submissions;
    }
}
int cwrite(void *,uint32_t reg,uint32_t v) {
    regs[reg]=v;
    if(reg==0x0807) {++submissions;completeIfReleased();}
    return 0;
}
int cread(void *,uint32_t reg,uint32_t *v) {completeIfReleased();*v=regs[reg];return 0;}
int cbarrier(void *) {return 0;}
int cpublish(void *,const volatile uint32_t *,size_t) {return 0;}
int cacquire(void *,volatile uint32_t *,size_t) {completeIfReleased();return 0;}
uint64_t cnow(void *) {
    return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
int cidle(void *) {std::this_thread::sleep_for(std::chrono::microseconds(50));return 0;}
int timeread(void *,uint32_t address,uint32_t *value) {
    static uint64_t ticks=1000;
    if(address==0x0980)ticks+=1000;
    *value=address==0x0980?uint32_t(ticks):(address==0x0981?uint32_t(ticks>>32):0);return 0;
}
const NativeAdrenoAbiDeviceInfo info={0x43050a01u,3u<<20,0x1000000ull,0x1fffffffff000ull,0x20000000ull,0x60000000ull,
    16,6,1,4,NativeAdrenoDeviceClientIova,0};
const NativeAdrenoRingIo ringio{nullptr,cwrite,cread,cbarrier};
const NativeAdrenoRingCache ringcache{nullptr,cpublish,cacquire};
const NativeAdrenoRingClock clock_{nullptr,cnow,cidle};
const NativeAdrenoTimeIo timeio{nullptr,timeread};

NativeAdrenoGpuContext *context;
NativeAdrenoRing ring;
NativeAdrenoSubmit execbuf;
NativeAdrenoUserClient client;
NativeAdrenoTimer timer;
bool built=false,opened=false;

bool build() {
    if(built)return true;
    /* HOSTSIM_SCATTER=1: non-contiguous BOs come back one physical page per span, as on the tablet (int-070's bridge ran out of
     * device memory there while the contiguous host model passed). */
    if(const char *scatter=std::getenv("HOSTSIM_SCATTER");scatter && *scatter=='1')fixture.mode=ScatterUnlessContiguous;
    /* HOSTSIM_CONTIGUOUS_MAX=<bytes>: larger physically contiguous allocations fail, so they fall back to scattered pages. */
    if(const char *limit=std::getenv("HOSTSIM_CONTIGUOUS_MAX"))fixture.contiguous_limit=std::strtoull(limit,nullptr,0);
    std::memset(mmio,0,sizeof(mmio));
    mmio[0]=0x002f0406;mmio[0x20/4]=0x4c017e18;mmio[0x24/4]=0x40000016;mmio[0x28/4]=0x1433;
    for(unsigned i=0;i<24;++i)mmio[(0xc00+4*i)/4]=0x20000;
    static const ArmSmmu4KIo io{nullptr,kSmmuBytes,sread32,sread64,swrite32,swrite64,sbarrier};
    context=new NativeAdrenoGpuContext;
    NativeAdrenoGmuContextReport report;
    const unsigned slots[2]={2,3};
    const uint16_t sids[2]={0,1};
    const IOReturn up=context->bringUp(&io,1,2,slots,sids,nullptr,&report);
    if(up!=kIOReturnSuccess) {std::fprintf(stderr,"host gpu: context bring-up failed %d at step %u\n",up,report.step);return false;}
    const NativeAdrenoMemory &ringmem=context->buffer(1),&memory=context->buffer(2);
    memptrs=static_cast<volatile uint32_t *>(memory.mapping());
    if(ring.attach(&ringio,&ringcache,ringmem.mapping(),ringmem.bytes(),GpuRingIova,memory.mapping(),memory.bytes(),
        GpuMemptrsIova)!=kIOReturnSuccess) {std::fprintf(stderr,"host gpu: ring attach failed\n");return false;}
    if(execbuf.attach(context->vm(),&ring)!=kIOReturnSuccess) {std::fprintf(stderr,"host gpu: submit attach failed\n");return false;}
    if(timer.attach(&timeio,NativeAdrenoTimeCp)!=kIOReturnSuccess)return false;
    /* Ring attach already wrote the write pointer once (the ring's own start-up packets); only submissions after this count as fences. */
    submissions=0;memptrs[1]=0;
    built=true;
    return true;
}
IOReturn translate(IOReturn status) {return status;}
}

extern "C" {
/* The kernel KPI the user client names an import's task by; the host stand-in is this process. */
int proc_selfpid(void) {return getpid();}
/* Return codes are the fixture's IOReturn values; the shim translates them. */
int host_gpu_open() {
    std::lock_guard<std::recursive_mutex> guard(serviceLock);
    if(!build() || opened)return kIOReturnBusy;
    const IOReturn st=client.open(NativeAdrenoAbiVersion,context->vm(),&execbuf,&ring,31,40);
    if(st) {std::fprintf(stderr,"host gpu: client open failed %d\n",st);return kIOReturnNotReady;}
    std::setvbuf(stderr,nullptr,_IONBF,0);
    client.attachTimer(&timer);
    client.attachDeviceInfo(&info);
    opened=true;
    return kIOReturnSuccess;
}
int host_gpu_close() {
    std::lock_guard<std::recursive_mutex> guard(serviceLock);
    if(!opened)return kIOReturnNotReady;
    const IOReturn status=client.releaseAll(&clock_,2000000);
    if(!status)opened=false;
    return translate(status);
}
int host_gpu_call(uint32_t method,const void *in,size_t in_size,void *out,size_t out_size) {
    std::lock_guard<std::recursive_mutex> guard(serviceLock);
    if(!opened)return kIOReturnNotReady;
    if(method<32)++calls[method];
    if(method<32 && faultRemaining[method]){--faultRemaining[method];return faultStatus[method];}
    /* As the IOKit wrapper does (NativeAdrenoPublish.cpp): an import is the calling task's pages, here a stand-in task. */
    if(method==NativeAdrenoMethodBoImport) {
        if(!in || in_size!=sizeof(NativeAdrenoAbiBoImport) || !out || out_size!=sizeof(NativeAdrenoAbiBoAllocReply))return kIOReturnBadArgument;
        NativeAdrenoAbiBoImport request{};std::memcpy(&request,in,sizeof(request));
        NativeAdrenoAbiBoAllocReply reply{};
        const IOReturn status=client.importBo(&request,reinterpret_cast<void *>(0x1000),&reply);
        if(!status)std::memcpy(out,&reply,sizeof(reply));
        return translate(status);
    }
    const IOReturn status=client.dispatch(method,in,in_size,out,out_size,&clock_);
    if(!status && method<32)++successes[method];
    if(!status && method==NativeAdrenoMethodBoMap && in)
        lastMapIova=static_cast<const NativeAdrenoAbiBoMap *>(in)->iova;
    /* HOSTSIM_TRACE=1: every failing call, and every allocation size, on stderr. */
    static const bool trace=std::getenv("HOSTSIM_TRACE") && *std::getenv("HOSTSIM_TRACE")=='1';
    if(trace && method==NativeAdrenoMethodBoAlloc && in)
        std::fprintf(stderr,"hostgpu alloc %llu -> %d spans-live %zu\n",(unsigned long long)static_cast<const NativeAdrenoAbiBoAlloc *>(in)->bytes,
            status,context->vm()->spanCount());
    else if(trace && status)std::fprintf(stderr,"hostgpu method %u -> %d\n",method,status);
    return translate(status);
}
int host_gpu_map(uint32_t token,void **address,uint64_t *bytes) {
    std::lock_guard<std::recursive_mutex> guard(serviceLock);
    NativeAdrenoMemory *memory=opened?client.memoryForToken(token):nullptr;
    if(!memory)return kIOReturnBadArgument;
    *address=memory->mapping();*bytes=memory->bytes();return kIOReturnSuccess;
}
unsigned host_gpu_submissions() {std::lock_guard<std::recursive_mutex> guard(serviceLock);return submissions;}
unsigned host_gpu_live_objects() {std::lock_guard<std::recursive_mutex> guard(serviceLock);return unsigned(client.liveObjects());}
unsigned host_gpu_calls(uint32_t method) {std::lock_guard<std::recursive_mutex> guard(serviceLock);return method<32?calls[method]:0;}
unsigned host_gpu_successes(uint32_t method) {std::lock_guard<std::recursive_mutex> guard(serviceLock);return method<32?successes[method]:0;}
uint64_t host_gpu_last_map_iova() {std::lock_guard<std::recursive_mutex> guard(serviceLock);return lastMapIova;}
// Only the atomic peer-release flag changes off-thread. The next actual ring read/acquire
// publishes the software peer's completion under the service lock, like the existing CP model.
void host_gpu_hold_completion(int hold) {holdCompletion.store(hold!=0,std::memory_order_release);}
void host_gpu_inject_failure(uint32_t method,int status,uint32_t count) {
    std::lock_guard<std::recursive_mutex> guard(serviceLock);
    if(method<32){faultStatus[method]=status;faultRemaining[method]=count;}
    else std::memset(faultRemaining,0,sizeof faultRemaining);
}
}
/* Physical-page maintenance (imported BOs): record it; no host page is touched. */
/* Host double: the page attribute change of user BOs has no host equivalent (no host page is touched). */
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *spans,size_t count,bool){return spans && count?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *spans,size_t count,uint32_t *reset)
{return spans && count && reset?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t physical,size_t bytes,NativeAdrenoCacheOperation op)
{
    if(!physical || !bytes || ((physical|bytes)&4095) || op>NativeAdrenoInvalidate)std::abort();
    ++fixture.physical_calls;fixture.physical_bytes+=bytes;return kIOReturnSuccess;
}
