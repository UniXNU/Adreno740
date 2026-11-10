// SPDX-License-Identifier: MIT
// Kernel services absent from the host model. Tests can replace these weak defaults.
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include "NativeAdrenoPool.h"
extern "C" __attribute__((weak)) boolean_t PE_parse_boot_argn(const char *,void *,int) {return 0;}
__attribute__((weak)) void NativeAdrenoNotePhase(unsigned,size_t) {}
__attribute__((weak)) bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
extern "C" __attribute__((weak)) unsigned native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
extern "C" {__attribute__((weak)) volatile uint32_t native_gpu_dcvs[15]{};}
extern "C" __attribute__((weak)) int proc_selfpid() {return 1;}
extern "C" __attribute__((weak)) uint64_t native_adreno_now_us() {return 0;}

extern "C" __attribute__((weak)) void native_gpu_trace_record(uint32_t,uint32_t,const uint64_t *,uint32_t) {}

// The pool's kernel services: a range's "kernel mapping" is host memory that stands for its pages, filled with a pattern no test
// writes, so a page handed out without its zero fill shows. The scrub is the kernel's: lines dropped (counted), then zeros.
extern "C" {__attribute__((weak)) unsigned native_adreno_host_pool_drops=0;}
__attribute__((weak)) void *NativeAdrenoPoolMapRange(uint64_t physical,size_t bytes,void **owner)
{
    if(!owner || !physical || !bytes)return nullptr;
    void *memory=nullptr;
    if(posix_memalign(&memory,4096,bytes))return nullptr;
    std::memset(memory,0xa5,bytes);
    *owner=memory;
    return memory;
}
__attribute__((weak)) IOReturn NativeAdrenoPoolScrub(void *mapping,size_t bytes,bool drop_lines)
{
    if(!mapping || !bytes || (((uintptr_t)mapping|bytes)&4095))return kIOReturnBadArgument;
    if(drop_lines)++native_adreno_host_pool_drops;
    std::memset(mapping,0,bytes);
    return kIOReturnSuccess;
}
// A host program built without NativeAdrenoPool.cpp has no pool: every system BO takes the descriptor path. NativeAdrenoPool.cpp's
// own definitions replace these wherever it is linked.
__attribute__((weak)) IOReturn NativeAdrenoPoolAllocate(size_t,uint64_t,bool,NativeAdrenoPoolGrant **out)
{
    if(out)*out=nullptr;
    return kIOReturnUnsupported;
}
__attribute__((weak)) void NativeAdrenoPoolRelease(NativeAdrenoPoolGrant *) {}
__attribute__((weak)) bool NativeAdrenoPoolContains(uint64_t,uint64_t) {return false;}
