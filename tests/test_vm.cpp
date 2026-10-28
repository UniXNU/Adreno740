/* SPDX-License-Identifier: MIT */
/* Real VM/memory/SMMU code over a software SMMUv2 register peer.
 * Proves ownership/lifetime/refusal logic only; no hardware is touched. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoVm.h"
#include <chrono>
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
#define BYTES 0x40000
#define CB 0x22000
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));
    CHECK(op==NativeAdrenoClean || op==NativeAdrenoCleanInvalidate || op==NativeAdrenoInvalidate);
    ++fixture.cache_calls;
    return fixture.cache_calls==fixture.cache_fail_call?kIOReturnIOError:kIOReturnSuccess;
}
static uint32_t memory[BYTES/4];
static struct {unsigned calls,fail_call,after;} f;
static ArmSmmu4KIo io;
static ArmSmmu4KStreamState streams;
static ArmSmmu4KStream routes[2]={{2,0},{4,1}};
static ArmSmmu4KStreamResult route_result;
static ArmSmmu4KContextConfig context;
static ArmSmmu4KContextImage image;
extern "C" {extern uint64_t (*native_adreno_vm_now_us)(void);extern volatile uint64_t native_adreno_vm_phase_us[2][8];}
static uint64_t nowUs() {return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static int event(void){return ++f.calls==f.fail_call;}
static int read32(void *cookie,size_t at,uint32_t *v)
{CHECK(cookie==&f && at%4==0 && at<=BYTES-4);if(event())return 1;*v=memory[at/4];return 0;}
static int read64(void *cookie,size_t at,uint64_t *v)
{CHECK(cookie==&f && at%8==0 && at<=BYTES-8);if(event())return 1;std::memcpy(v,memory+at/4,8);return 0;}
static int write32(void *cookie,size_t at,uint32_t v)
{
    CHECK(cookie==&f && at%4==0 && at<=BYTES-4);
    int fail=event();if(fail && !f.after)return 1;
    memory[at/4]=v;return fail;
}
static int write64(void *cookie,size_t at,uint64_t v)
{
    CHECK(cookie==&f && at%8==0 && at<=BYTES-8);
    int fail=event();if(fail && !f.after)return 1;
    std::memcpy(memory+at/4,&v,8);return fail;
}
static int barrier(void *cookie){CHECK(cookie==&f);return event();}
struct Fixture {
    NativeAdrenoTablePages tables;
    NativeAdrenoMemory object;
    ArmSmmu4KLiveArena arena;
    ArmSmmu4KLiveState state{};
    ArmSmmu4KLiveVisibility visibility;
    NativeAdrenoVm vm;
};
static void mmioReset(void)
{
    std::memset(memory,0,sizeof(memory));f={};streams=(ArmSmmu4KStreamState){};
    io=(ArmSmmu4KIo){&f,BYTES,read32,read64,write32,write64,barrier};
    memory[0x20/4]=(1u<<30)|(1u<<27)|(4u<<9)|8;
    memory[0x24/4]=0x40000010;memory[0x28/4]=0x1433;memory[0]=0x436;
    for(unsigned i=0;i<8;++i)memory[(0xc00+4*i)/4]=0x20000;
    memory[0x800/4]=1u<<31|9;memory[0xc00/4]=3;
}
/* Real prepared arena, enabled actual context image and exact stream routes. */
static IOReturn exposed(Fixture &s,size_t capacity=32,size_t pages=16)
{
    mmioReset();
    IOReturn smmu=NativeAdrenoPrepareSmmuMemory(&s.tables,32,40,0x800000,pages*4096,0,pages,
        &s.arena,&s.state,&s.visibility);
    if(smmu)return smmu;
    context=(ArmSmmu4KContextConfig){.lower=&s.arena.config,.lower_tables=&s.state.tables,
        .id0=memory[0x20/4],.id2=memory[0x28/4],.asid=1,.coherent_walk=1,
        .stall_on_fault=0,.continue_after_fault=1};
    ArmSmmu4KContextConfig actual=context;
    CHECK(ArmSmmu4KEncodeContext(&actual,&image)==ArmSmmu4KSuccess);
    memory[0x1008/4]=image.cbar;memory[0x1808/4]=image.cba2r;
    memory[CB/4]=image.sctlr;memory[(CB+0x10)/4]=image.tcr2;memory[(CB+0x30)/4]=image.tcr;
    std::memcpy(memory+(CB+0x20)/4,&image.ttbr0,8);
    std::memcpy(memory+(CB+0x28)/4,&image.ttbr1,8);
    memory[(CB+0x38)/4]=image.mair0;
    if(ArmSmmu4KAttachStreams(&io,&streams,routes,2,2,&context,&route_result)!=ArmSmmu4KSuccess)
        return kIOReturnBadArgument;
    NativeAdrenoVmRegion lower={&s.tables,&s.arena,&s.state};
    return s.vm.attach(&io,&context,&streams,&lower,nullptr,capacity,1000);
}
static IOReturn object(Fixture &s,size_t pages)
{return s.object.allocate(pages*4096,40,false,pages*4096);}
int main()
{
    /* A real map/hold/unmap transaction across the actual code paths. */
    {
        Fixture s;
        CHECK(exposed(s)==kIOReturnSuccess);
        CHECK(object(s,2)==kIOReturnSuccess);
        uint64_t address=0,handle=0;
        CHECK(s.vm.findAddress(0,8192,8192,&address)==kIOReturnSuccess);
        CHECK(address==0x800000);
        CHECK(s.vm.objectCount()==0 && s.vm.spanCount()==0);
        CHECK(s.vm.map(&s.object,0,address,3,0,2,&handle)==kIOReturnSuccess);
        CHECK(handle && s.vm.objectCount()==1 && s.vm.spanCount()==s.object.spanCount());
        CHECK(s.object.deviceReferences()==1); /* one mapping reference */
        CHECK(s.vm.findAddress(0,8192,8192,&address)==kIOReturnSuccess && address==0x802000);
        CHECK(s.vm.holdJob(handle)==kIOReturnSuccess && s.object.deviceReferences()==2);
        CHECK(s.vm.unmap(handle)==kIOReturnBusy);
        CHECK(s.vm.retireJobAfterCompletion(handle)==kIOReturnSuccess);
        CHECK(s.vm.unmap(handle)==kIOReturnSuccess);
        CHECK(s.vm.objectCount()==0 && s.vm.spanCount()==0);
        CHECK(s.object.deviceReferences()==0 && s.object.release()==kIOReturnSuccess);
        CHECK(s.vm.detach()==kIOReturnSuccess);
        CHECK(s.tables.release()==kIOReturnSuccess);
        CHECK(fixture.live.empty() && fixture.metadata.empty());
    }
    /* Refusals precede any ownership or table mutation. */
    {
        Fixture s;
        CHECK(exposed(s)==kIOReturnSuccess);
        CHECK(object(s,2)==kIOReturnSuccess);
        unsigned refs=s.object.deviceReferences(),spans=s.vm.spanCount(),reads=f.calls;
        uint64_t handle=0;
        CHECK(s.vm.map(&s.object,0,0x800001,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(&s.object,1,0x800000,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(&s.object,0,0xec0000,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(&s.object,0,0x800000,2,0,2,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(&s.object,0,0x800000,3,2,2,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,1,&handle)==kIOReturnBadArgument);
        CHECK(s.vm.map(nullptr,0,0x800000,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(!handle && refs==s.object.deviceReferences() && spans==s.vm.spanCount() && reads==f.calls);
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,2,&handle)==kIOReturnSuccess);
        uint64_t mapped=handle;
        /* A refused map clears only its own out-parameter. */
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,2,&handle)==kIOReturnBusy && !handle);
        CHECK(s.vm.unmap(mapped+100)==kIOReturnBadArgument);
        CHECK(s.vm.holdJob(mapped+100)==kIOReturnBadArgument);
        CHECK(s.vm.retireJobAfterCompletion(mapped)==kIOReturnBadArgument);
        CHECK(s.vm.unmap(mapped)==kIOReturnSuccess);
        CHECK(s.object.release()==kIOReturnSuccess);
        CHECK(s.vm.detach()==kIOReturnSuccess);
        CHECK(s.tables.release()==kIOReturnSuccess);
        CHECK(fixture.live.empty() && fixture.metadata.empty());
    }
    /* Scattered BOs, one span per page, mapped in one transaction each into the middle, the end and the start of the table
     * (each map and unmap audits the whole table against the sorted spans); the range table grows past its first capacity. */
    {
        Fixture s;
        CHECK(exposed(s,8)==kIOReturnSuccess);   /* aperture 0x800000..0x810000, 8 spans */
        fixture.mode=Scattered;
        NativeAdrenoMemory first,middle,last,huge;
        CHECK(!first.allocate(4096,40,false,4096) && !middle.allocate(4*4096,40,false,4*4096) && !last.allocate(4096,40,false,4096));
        CHECK(middle.spanCount()==4);
        uint64_t a=0,b=0,c=0,iova=0;size_t bytes=0;unsigned permissions=0;
        CHECK(!s.vm.map(&first,0,0x800000,3,0,2,&a) && !s.vm.map(&last,0,0x808000,3,0,2,&c));
        CHECK(!s.vm.map(&middle,0,0x802000,3,0,2,&b) && s.vm.spanCount()==6 && s.vm.lastResult().completed_spans==4);
        CHECK(!s.vm.describe(b,&iova,&bytes,&permissions) && iova==0x802000 && bytes==4*4096);
        CHECK(!s.vm.unmap(b) && s.vm.spanCount()==2 && s.vm.lastResult().completed_spans==4);
        CHECK(!s.vm.map(&middle,0,0x80a000,3,0,2,&b) && s.vm.spanCount()==6);
        CHECK(!s.vm.unmap(a) && s.vm.spanCount()==5);
        CHECK(!s.vm.map(&first,0,0x800000,3,0,2,&a) && s.vm.spanCount()==6);
        CHECK(!huge.allocate(3*4096,40,false,3*4096) && huge.spanCount()==3);
        uint64_t grown=0;
        CHECK(!s.vm.map(&huge,0,0x802000,3,0,2,&grown) && grown && s.vm.spanCount()==9 && !s.vm.poisoned());
        CHECK(!s.vm.unmap(grown) && !s.vm.unmap(a) && !s.vm.unmap(b) && !s.vm.unmap(c) && s.vm.spanCount()==0);
        fixture.mode=Contiguous;
        CHECK(!first.release() && !middle.release() && !last.release() && !huge.release());
        CHECK(s.vm.detach()==kIOReturnSuccess);
        CHECK(s.tables.release()==kIOReturnSuccess);
        CHECK(fixture.live.empty() && fixture.metadata.empty());
    }
    /* Physical overlap remains allowed only within the same attribute class.
     * Check scratch and fallback, both smaller-side choices, and refusals before
     * MMIO/ownership changes. These are physical-page identities in the peer. */
    for(bool scratch:{false,true}) {
        Fixture s;CHECK(exposed(s)==kIOReturnSuccess);
        /* The VM owns its regions' alias scratch; without it (scratch false) the kit compares pairwise. */
        if(!scratch){s.arena.alias_scratch=nullptr;s.arena.alias_scratch_entries=0;}
        NativeAdrenoMemory first,second,many;
        CHECK(!first.allocate(4096,40,false,4096));
        fixture.next_physical=first.spans()[0].physical;
        CHECK(!second.allocate(4096,40,false,4096));
        fixture.mode=Scattered;fixture.next_physical=first.spans()[0].physical;
        CHECK(!many.allocate(8192,40,false,8192)&&many.spanCount()==2);fixture.mode=Contiguous;
        uint64_t a=0,b=0;CHECK(!s.vm.map(&first,0,0x800000,3,0,2,&a));
        CHECK(!s.vm.map(&second,0,0x802000,1,0,2,&b));CHECK(!s.vm.unmap(b));
        /* Driver-allocated pages are exclusive to their BO (exclusivePages): no cross-class alias pass runs for them, so a different
         * class over the same pages maps. Imports keep the pass (test_vm_alias covers the index). */
        CHECK(!s.vm.map(&second,0,0x802000,3,1,2,&b) && !s.vm.unmap(b));
        CHECK(!s.vm.unmap(a)&&!first.release()&&!second.release()&&!many.release());
        CHECK(!s.vm.detach()&&!s.tables.release());
        CHECK(fixture.live.empty()&&fixture.metadata.empty());
    }
    /* A large different-class scattered map: the old preflight did n*n physical
     * comparisons despite the table updater already having alias scratch. */
    {
        constexpr size_t n=8192;
        Fixture s;CHECK(exposed(s,2*n,2*n)==kIOReturnSuccess);
        fixture.mode=Scattered;NativeAdrenoMemory first,second;
        CHECK(!first.allocate(n*4096,40,false,n*4096)&&first.spanCount()==n);
        CHECK(!second.allocate(n*4096,40,false,n*4096)&&second.spanCount()==n);
        uint64_t a=0,b=0;CHECK(!s.vm.map(&first,0,0x800000,3,0,2,&a));
        native_adreno_vm_now_us=nowUs;uint64_t before=native_adreno_vm_phase_us[0][5];
        CHECK(!s.vm.map(&second,0,0x800000+n*4096,3,1,3,&b));
        std::printf("VM alias preflight diagnostic: %zu x %zu different-class spans, %llu host us (not hardware timing)\n",n,n,
            (unsigned long long)(native_adreno_vm_phase_us[0][5]-before));
        native_adreno_vm_now_us=nullptr;fixture.mode=Contiguous;
        CHECK(s.vm.spanCount()==2*n&&!s.vm.unmap(b)&&!s.vm.unmap(a)&&!first.release()&&!second.release());
        CHECK(!s.vm.detach()&&!s.tables.release());
        CHECK(fixture.live.empty()&&fixture.metadata.empty());
    }
    std::printf("PASS: VM alias index/fallback preserve attribute-class refusals before MMIO/ownership; 8192-span different-class map/unmap\n");
    /* A 48-bit aperture (msm's per-process window up to BIT(ias)): only the tables a mapping needs exist, maps far apart take their
     * own tables, an unmap keeps them (io-pgtable frees no table on a page unmap) and release() gives every page back. */
    {
        Fixture s;
        mmioReset();
        memory[0x28/4]=0x1533;   /* UBS 5 (48 bits), OAS 3, IAS 3, PTFS 4K */
        CHECK(NativeAdrenoSmmuInputBits(memory[0x28/4])==48 && NativeAdrenoSmmuInputBits(0x0233)==40 && NativeAdrenoSmmuInputBits(0xf33)==48);
        const uint64_t aperture=UINT64_C(1)<<48;
        CHECK(!NativeAdrenoPrepareSmmuMemory(&s.tables,48,40,0,aperture,0,size_t(aperture/4096),&s.arena,&s.state,&s.visibility));
        CHECK(s.tables.pageCount()==1 && s.state.tables.table_pages==1 && s.state.tables.start_level==0);
        context=(ArmSmmu4KContextConfig){.lower=&s.arena.config,.lower_tables=&s.state.tables,
            .id0=memory[0x20/4],.id2=memory[0x28/4],.asid=1,.coherent_walk=1,.stall_on_fault=0,.continue_after_fault=1};
        ArmSmmu4KContextConfig actual=context;
        CHECK(ArmSmmu4KEncodeContext(&actual,&image)==ArmSmmu4KSuccess && (image.tcr&63)==16);
        memory[0x1008/4]=image.cbar;memory[0x1808/4]=image.cba2r;
        memory[CB/4]=image.sctlr;memory[(CB+0x10)/4]=image.tcr2;memory[(CB+0x30)/4]=image.tcr;
        std::memcpy(memory+(CB+0x20)/4,&image.ttbr0,8);std::memcpy(memory+(CB+0x28)/4,&image.ttbr1,8);
        memory[(CB+0x38)/4]=image.mair0;
        CHECK(ArmSmmu4KAttachStreams(&io,&streams,routes,2,2,&context,&route_result)==ArmSmmu4KSuccess);
        NativeAdrenoVmRegion lower={&s.tables,&s.arena,&s.state};
        CHECK(!s.vm.attach(&io,&context,&streams,&lower,nullptr,64,1000));
        NativeAdrenoMemory low,high;
        CHECK(!low.allocate(8192,40,false,8192) && !high.allocate(8192,40,false,8192));
        uint64_t a=0,b=0,leaf=0;
        CHECK(!s.vm.map(&low,0,0x100000000ull,3,0,2,&a) && s.tables.pageCount()==4);
        CHECK(!s.vm.map(&high,0,aperture-8192,3,0,2,&b) && s.tables.pageCount()==7);
        CHECK(s.state.tables.table_pages==7 && s.state.tables.leaf_pages==4);
        CHECK(s.vm.leafPhysical(high.spans()[0].physical,&leaf) && leaf);
        CHECK(s.tables.overlaps(s.tables.pagePhysical(3),1) && !s.tables.overlaps(low.spans()[0].physical,8192));
        CHECK(!s.vm.unmap(a) && !s.vm.unmap(b) && s.tables.pageCount()==7 && !s.state.tables.leaf_pages);
        CHECK(!low.release() && !high.release() && !s.vm.detach() && !s.tables.release() && !s.tables.pageCount());
        CHECK(fixture.live.empty() && fixture.metadata.empty());
    }
    /* A table page the system cannot give refuses the map before any leaf is written; the VM stays usable. */
    {
        Fixture s;
        CHECK(exposed(s)==kIOReturnSuccess);
        CHECK(object(s,2)==kIOReturnSuccess);
        const size_t pages=s.tables.pageCount();
        fixture.allocation_fail=true;
        uint64_t handle=0;
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,2,&handle)!=kIOReturnSuccess && !handle);
        fixture.allocation_fail=false;
        CHECK(s.tables.pageCount()>=pages && !s.state.tables.leaf_pages && !s.object.deviceReferences());
        CHECK(!s.vm.poisoned() && !s.vm.map(&s.object,0,0x800000,3,0,2,&handle) && handle);
        CHECK(!s.vm.unmap(handle) && !s.object.release() && !s.vm.detach() && !s.tables.release());
        CHECK(fixture.live.empty() && fixture.metadata.empty());
    }
    std::printf("PASS: 48-bit sparse aperture with on-demand table pages, table-page refusal before any leaf\n");
    /* Post-hold CPU publication failure: poison, retain everything, refuse. */
    {
        Fixture s;
        CHECK(exposed(s)==kIOReturnSuccess);
        CHECK(object(s,2)==kIOReturnSuccess);
        fixture.cache_fail_call=fixture.cache_calls+1; /* next publication fails */
        uint64_t handle=0;
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,2,&handle)==kIOReturnIOError);
        CHECK(!handle && s.vm.poisoned() && s.vm.spanCount()==0); /* new BO reference is retained */
        CHECK(s.object.quarantined() && s.object.deviceReferences()>=1 && s.tables.quarantined());
        CHECK(s.state.prepared && s.state.poisoned);
        CHECK(s.vm.map(&s.object,0,0x800000,3,0,2,&handle)==kIOReturnNotReady);
        CHECK(s.vm.unmap(0)==kIOReturnNotReady && s.vm.detach()==kIOReturnBusy);
        CHECK(s.vm.attach(&io,&context,&streams,nullptr,nullptr,32,1000)==kIOReturnBusy);
        CHECK(s.object.release()==kIOReturnBusy && s.tables.release()==kIOReturnBusy);
        CHECK(!fixture.live.empty());
    }
    std::printf("PASS: 1 native map/hold/unmap transaction, 13 pre-ownership refusals, 1 scattered-span ordering and growth case and 1 post-hold poison/retention case over a software SMMU register peer\n");
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
void NativeAdrenoNotePhase(unsigned,size_t) {}
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
extern "C" unsigned int native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
