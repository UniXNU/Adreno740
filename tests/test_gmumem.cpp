/* SPDX-License-Identifier: MIT */
/* Real allocator/layout over host IOKit doubles. No mapping or GMU access. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoGmuMemory.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *state)
{CHECK(state);*state={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && op<=NativeAdrenoInvalidate);
    return kIOReturnSuccess;
}
int main()
{
    {
        NativeAdrenoGmuAllocator allocator;
        CHECK(allocator.allocate(NativeAdrenoGmuDefaultSizes())==kIOReturnSuccess && allocator.ready());
        const NativeAdrenoGmuMemoryLayout &layout=allocator.layout();
        /* a6xx_gmu.c at the pinned tree: icache 0x4000, dummy 0x60000000, debug 0x60400000; log and hfi
         * take the lowest free pages after the 16 KiB dummy. */
        CHECK(layout.icache_iova==0x4000);
        CHECK(layout.dummy_iova==0x60000000);
        CHECK(layout.debug_iova==0x60400000);
        CHECK(layout.log_iova==0x60004000);
        CHECK(layout.hfi_iova==0x60008000);
        CHECK(layout.end_iova==0x60407000);
        CHECK(allocator.icache().bytes()==0xffc000 && allocator.icache().spanCount()==1);
        CHECK(allocator.hfi().bytes()==0x4000 && !allocator.tables().pageCount());
        CHECK(allocator.allocate(NativeAdrenoGmuDefaultSizes())==kIOReturnBusy);
        CHECK(allocator.release()==kIOReturnSuccess && !allocator.ready());
        CHECK(fixture.live.empty());
    }
    /* Bad sizes, payloads that leave their fixed range, and allocation failure leave nothing behind. */
    {
        NativeAdrenoGmuAllocator allocator;
        NativeAdrenoGmuSizes big=NativeAdrenoGmuDefaultSizes();
        big.icache=0x60000000;            /* would run into the uncached range */
        CHECK(allocator.allocate(big)==kIOReturnBadArgument);
        big=NativeAdrenoGmuDefaultSizes();
        big.hfi=0x400000;                 /* dummy+log+hfi past 0x60400000 */
        CHECK(allocator.allocate(big)==kIOReturnBadArgument);
        big=NativeAdrenoGmuDefaultSizes();
        big.debug=0x20000000;             /* past the 2 GiB GMU space */
        CHECK(allocator.allocate(big)==kIOReturnBadArgument);
        CHECK(!allocator.ready() && fixture.live.empty());
    }
    {
        fixture=MemoryFixture{};
        fixture.allocation_fail=true;
        NativeAdrenoGmuAllocator allocator;
        CHECK(allocator.allocate(NativeAdrenoGmuDefaultSizes())==kIOReturnNoMemory);
        CHECK(!allocator.ready() && fixture.live.empty());
        fixture.allocation_fail=false;
    }
    std::printf("PASS: 1 fixed a6xx GMU layout allocation and release, 3 size/range refusals, 1 allocation failure and 1 repeat-allocation refusal over host doubles\n");
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
