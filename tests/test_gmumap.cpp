/* SPDX-License-Identifier: MIT */
/* Real lower-region mapping of the GMU payloads at their fixed a6xx addresses over a software SMMU peer. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoGmuMemory.h"
#include <cstdio>
#include <cstring>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
#define SMMU_BYTES 0x40000
#define CB 0x22000
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *state)
{CHECK(state);*state={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && op<=NativeAdrenoInvalidate);
    ++fixture.cache_calls;
    return fixture.cache_calls==fixture.cache_fail_call?kIOReturnIOError:kIOReturnSuccess;
}
static uint32_t mmio[SMMU_BYTES/4];
static struct {unsigned calls,fail_call,after;} mf;
static ArmSmmu4KIo io;
static ArmSmmu4KStreamState streams;
static ArmSmmu4KStream routes[1]={{2,0}}; /* slot 0 holds an unrelated route */
static ArmSmmu4KStreamResult route_result;
static ArmSmmu4KContextConfig context;
static ArmSmmu4KContextImage image;
static int event(void){return ++mf.calls==mf.fail_call;}
static int read32(void *c,size_t at,uint32_t *v)
{CHECK(c==&mf && at%4==0 && at<=SMMU_BYTES-4);if(event())return 1;*v=mmio[at/4];return 0;}
static int read64(void *c,size_t at,uint64_t *v)
{CHECK(c==&mf && at%8==0 && at<=SMMU_BYTES-8);if(event())return 1;std::memcpy(v,mmio+at/4,8);return 0;}
static int write32(void *c,size_t at,uint32_t v)
{CHECK(c==&mf && at%4==0 && at<=SMMU_BYTES-4);int f=event();if(f && !mf.after)return 1;mmio[at/4]=v;return f;}
static int write64(void *c,size_t at,uint64_t v)
{CHECK(c==&mf && at%8==0 && at<=SMMU_BYTES-8);int f=event();if(f && !mf.after)return 1;std::memcpy(mmio+at/4,&v,8);return f;}
static int barrier(void *c){CHECK(c==&mf);return event();}
int main()
{
    /* A GMU-only context: lower region covering the whole 0..2 GiB GMU space, no upper region. */
    NativeAdrenoTablePages tables;
    ArmSmmu4KLiveArena arena;
    ArmSmmu4KLiveState state{};
    ArmSmmu4KLiveVisibility visibility;
    NativeAdrenoVm vm;
    std::memset(mmio,0,sizeof(mmio));mf={};streams=(ArmSmmu4KStreamState){};
    io=(ArmSmmu4KIo){&mf,SMMU_BYTES,read32,read64,write32,write64,barrier};
    mmio[0x20/4]=(1u<<30)|(1u<<27)|(4u<<9)|8;
    mmio[0x24/4]=0x40000010;mmio[0x28/4]=0x1433;mmio[0]=0x436;
    for(unsigned i=0;i<8;++i)mmio[(0xc00+4*i)/4]=0x20000;
    mmio[0x800/4]=1u<<31|9;mmio[0xc00/4]=3;
    CHECK(NativeAdrenoPrepareSmmuMemory(&tables,32,40,0,GmuApertureBytes,0,GmuApertureBytes/4096,
        &arena,&state,&visibility)==kIOReturnSuccess);
    CHECK(state.tables.table_pages==1 && tables.pageCount()==1);
    context=(ArmSmmu4KContextConfig){.lower=&arena.config,.upper=nullptr,
        .lower_tables=&state.tables,.upper_tables=nullptr,.id0=mmio[0x20/4],
        .id2=mmio[0x28/4],.asid=1,.coherent_walk=1,.stall_on_fault=0,
        .continue_after_fault=1};
    ArmSmmu4KContextConfig actual=context;
    CHECK(ArmSmmu4KEncodeContext(&actual,&image)==ArmSmmu4KSuccess);
    mmio[0x1008/4]=image.cbar;mmio[0x1808/4]=image.cba2r;
    mmio[CB/4]=image.sctlr;mmio[(CB+0x10)/4]=image.tcr2;mmio[(CB+0x30)/4]=image.tcr;
    std::memcpy(mmio+(CB+0x20)/4,&image.ttbr0,8);
    std::memcpy(mmio+(CB+0x28)/4,&image.ttbr1,8);
    mmio[(CB+0x38)/4]=image.mair0;
    CHECK(ArmSmmu4KAttachStreams(&io,&streams,routes,1,2,&context,&route_result)==ArmSmmu4KSuccess);
    NativeAdrenoVmRegion lower={&tables,&arena,&state};
    CHECK(vm.attach(&io,&context,&streams,&lower,nullptr,64,2000)==kIOReturnSuccess);
    CHECK(vm.spanCount()==0 && vm.objectCount()==0);

    NativeAdrenoGmuSizes sizes{0x4000,0x1000,0x2000,0x1000,0x1000};
    NativeAdrenoGmuAllocator allocator;
    CHECK(allocator.mapInto(&vm)==kIOReturnBadArgument); /* not allocated yet */
    CHECK(allocator.allocate(sizes)==kIOReturnSuccess);
    CHECK(allocator.layout().icache_iova==0x4000 && allocator.layout().dummy_iova==0x60000000);
    CHECK(allocator.layout().log_iova==0x60001000 && allocator.layout().hfi_iova==0x60002000);
    CHECK(allocator.layout().debug_iova==0x60400000 && allocator.layout().end_iova==0x60402000);
    CHECK(allocator.mapInto(&vm)==kIOReturnSuccess);
    CHECK(vm.objectCount()==5 && vm.spanCount()==5);
    CHECK(allocator.mapInto(&vm)==kIOReturnBusy);
    uint64_t iova=0;
    CHECK(vm.findAddress(1,0x2000,0x2000,&iova)!=kIOReturnSuccess); /* no upper region */
    CHECK(allocator.unmapFrom(&vm)==kIOReturnSuccess);
    CHECK(vm.objectCount()==0 && vm.spanCount()==0);
    CHECK(allocator.unmapFrom(&vm)==kIOReturnSuccess);
    CHECK(allocator.release()==kIOReturnSuccess);
    CHECK(vm.detach()==kIOReturnSuccess);
    CHECK(tables.release()==kIOReturnSuccess);
    CHECK(fixture.live.empty() && fixture.metadata.empty());
    std::printf("PASS: 1 lower-region fixed-address GMU payload mapping and unmap, 1 pre-allocation refusal and 1 repeat-map refusal over a software SMMU peer\n");
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
