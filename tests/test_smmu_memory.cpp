/* SPDX-License-Identifier: MIT */
/* Real XNU memory adapter plus real table builder/auditor; IOKit peers only. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoSmmuMemory.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));
    CHECK(op==NativeAdrenoClean);++fixture.cache_calls;
    return fixture.cache_calls==fixture.cache_fail_call?kIOReturnIOError:kIOReturnSuccess;
}
void NativeAdrenoNotePhase(unsigned,size_t) {}
int main()
{
    unsigned profiles=0;
    for(unsigned bits=32;bits<=48;++bits)for(unsigned upper=0;upper<2;++upper) {
        NativeAdrenoTablePages pages;ArmSmmu4KLiveArena arena;ArmSmmu4KLiveState state{};
        ArmSmmu4KLiveVisibility visibility;
        uint64_t va=(upper?UINT64_MAX-(UINT64_C(1)<<bits)+1:0)+0x1ff000;
        CHECK(NativeAdrenoPrepareSmmuMemory(&pages,bits,40,va,8192,upper,16,&arena,&state,&visibility)==kIOReturnSuccess);
        CHECK(pages.pageCount()==1 && arena.config.table_physical==pages.pagePhysical(0) && arena.pages==pages.provider());
        CHECK(state.prepared && !state.poisoned && state.tables.table_pages==1);
        const volatile uint64_t *root=pages.pageMapping(0);
        for(unsigned i=0;i<512;++i)CHECK(!root[i]);
        ArmSmmu4KLiveResult audit;
        CHECK(ArmSmmu4KLiveAudit(&arena,&state,nullptr,0,&audit)==ArmSmmu4KSuccess);
        volatile uint64_t *writable=const_cast<volatile uint64_t *>(root);
        CHECK(!visibility.publish_table_page(visibility.cookie,writable,4096));
        CHECK(visibility.publish_table_page(visibility.cookie,writable,8192));
        CHECK(visibility.publish_table_page(visibility.cookie,writable+1,4096));
        uint64_t foreign[1024] __attribute__((aligned(4096)));
        CHECK(visibility.publish_table_page(visibility.cookie,reinterpret_cast<volatile uint64_t *>(foreign),4096));
        /* The provider's own lookups: a page it gave, nothing else. */
        CHECK(arena.pages->find(arena.pages->cookie,pages.pagePhysical(0))==root);
        CHECK(!arena.pages->find(arena.pages->cookie,pages.pagePhysical(0)+4096));
        CHECK(arena.pages->overlaps(arena.pages->cookie,pages.pagePhysical(0)-4096,4097));
        CHECK(!arena.pages->overlaps(arena.pages->cookie,pages.pagePhysical(0)-4096,4096));
        CHECK(pages.release()==kIOReturnSuccess && fixture.live.empty() && fixture.metadata.empty());++profiles;
    }
    unsigned refusals=0;
    for(unsigned failure=0;failure<4;++failure) {
        fixture=MemoryFixture{};
        NativeAdrenoTablePages pages;ArmSmmu4KLiveArena arena;ArmSmmu4KLiveState state{};
        ArmSmmu4KLiveVisibility visibility;
        if(failure==0)fixture.allocation_fail=true;
        if(failure==1)fixture.null_mapping=true;
        if(failure==2)fixture.cache_fail_call=1;
        if(failure==3)pages.quarantine();
        IOReturn expected=failure==0 || failure==1?kIOReturnNoMemory:failure==2?kIOReturnIOError:kIOReturnBusy;
        IOReturn got=NativeAdrenoPrepareSmmuMemory(&pages,48,40,0xffff0000001ff000,8192,1,16,&arena,&state,&visibility);
        CHECK(got==expected);
        CHECK(!visibility.publish_table_page && state.poisoned==(failure==2));
        CHECK(pages.release()==(failure==3?kIOReturnBusy:kIOReturnSuccess) && fixture.live.empty() && fixture.metadata.empty());++refusals;
    }
    std::printf("PASS: %u on-demand table-page bindings and %u allocation/range/publication/quarantine refusals; no hardware context enabled\n",profiles,refusals);
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
