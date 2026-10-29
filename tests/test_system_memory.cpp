/* SPDX-License-Identifier: MIT */
/* User BOs from the system over host doubles: pageable memory with no task and no kernel mapping, the write-combined cache mode on the
 * memory entry, memory pressure while the system refuses one and the owned-page map that proves an import exclusive. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
extern "C" volatile uint32_t native_adreno_bo_memory[8];
static unsigned attribute_calls,physical_calls;
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out) {CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation)
{CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));++fixture.cache_calls;return kIOReturnSuccess;}
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *spans,size_t count,bool)
{++attribute_calls;return spans && count?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *,size_t,uint32_t *) {return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t,size_t,NativeAdrenoCacheOperation) {++physical_calls;return kIOReturnSuccess;}
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
void NativeAdrenoNotePhase(unsigned,size_t) {}
extern "C" unsigned int native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
int main()
{
    fixture.mode=ScatterUnlessContiguous;
    /* Write-combined: the entry carries the mode, the VM sets the pages' attribute; nothing here touches a page or its attribute. */
    NativeAdrenoMemory a,b,c,d;
    CHECK(!a.allocateSystem(96u<<20,40,96u<<20,true) && a.system() && a.writeCombined() && !a.mapping() && a.descriptor());
    CHECK(a.spanCount()==(96u<<20)/4096 && fixture.pageable==1 && fixture.write_combined_entries==1 && fixture.prepared==1);
    CHECK(attribute_calls==0 && fixture.cache_calls==0 && physical_calls==0 && !NativeAdrenoMemory::memoryPressure());
    CHECK(native_adreno_bo_memory[0]==1 && native_adreno_bo_memory[4]==96 && native_adreno_bo_memory[5]==96);
    CHECK(a.makeWriteCombined()==kIOReturnNotReady);   /* the entry carries the mode */
    uint32_t reset=0;CHECK(!a.reassertWriteCombined(&reset) && !reset);
    /* Default attribute: the zero fill is written back through the physical pages, no kernel mapping. */
    CHECK(!b.allocateSystem(64u<<10,40,64u<<10,false) && !b.writeCombined() && physical_calls==16 && fixture.write_combined_entries==1);
    CHECK(!b.publishForDevice(0,64u<<10,true) && physical_calls==32);
    CHECK(!b.acquireDeviceWrites(0,64u<<10) && physical_calls==48);
    /* The owned-page map: a's and b's pages, nothing else. */
    CHECK(NativeAdrenoMemory::ownsAnyPage(a.spans(),1) && NativeAdrenoMemory::ownsAnyPage(b.spans()+3,1));
    const NativeAdrenoPhysicalSpan foreign{0,b.spans()[0].physical+4096,4096};   /* the fixture scatters every other page */
    CHECK(!NativeAdrenoMemory::ownsAnyPage(&foreign,1));
    /* The system refuses: pressure until the next allocation succeeds. */
    fixture.allocation_fail_count=1;
    CHECK(c.allocateSystem(4096,40,4096,true)==kIOReturnNoMemory && NativeAdrenoMemory::memoryPressure());
    CHECK(native_adreno_bo_memory[2]==1 && native_adreno_bo_memory[3]==(uint32_t)kIOReturnNoMemory);
    CHECK(!c.release() && !d.allocateSystem(4096,40,4096,true) && !NativeAdrenoMemory::memoryPressure());
    /* Release: the VM restores the attribute, so no attribute call; the owned bits clear. */
    const NativeAdrenoPhysicalSpan first=a.spans()[0];
    CHECK(!a.release() && attribute_calls==0 && !NativeAdrenoMemory::ownsAnyPage(&first,1));
    CHECK(native_adreno_bo_memory[4]==0 && native_adreno_bo_memory[5]==96);
    CHECK(!b.release() && !d.release() && fixture.live.empty() && fixture.metadata.empty());
    /* A contiguous kernel BO still comes from allocate and changes its pages' attribute itself. */
    NativeAdrenoMemory e;
    CHECK(!e.allocate(64u<<10,40,true,64u<<10) && e.mapping() && !e.system() && e.spanCount()==1);
    CHECK(!e.makeWriteCombined() && attribute_calls==1 && NativeAdrenoMemory::ownsAnyPage(e.spans(),1));
    CHECK(!e.release() && attribute_calls==2 && fixture.live.empty());
    std::puts("PASS: system BOs: no task, no kernel mapping, write-combined entry, pressure, owned-page map, release");
    return 0;
}
