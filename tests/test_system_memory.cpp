/* SPDX-License-Identifier: MIT */
/* User BOs from the system over host doubles: pageable memory with no task and no kernel mapping, the write-combined cache mode on the
 * memory entry, memory pressure while the system refuses one and the owned-page map that proves an import exclusive. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
extern "C" volatile uint32_t native_adreno_bo_memory[8];
extern "C" volatile uint32_t native_adreno_bo_owners[24][4],native_adreno_import_memory[4],native_adreno_import_owners[24][4];
static int self_pid=7;
extern "C" int proc_selfpid() {return self_pid;}
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
    /* Pages prepared before the driver's lock: a default-attribute BO's zero fill is written back there, allocateSystem adopts the
     * descriptor without making or cleaning another, and pages it did not take go back whole. */
    NativeAdrenoSystemPages pages{};
    const unsigned prepared=fixture.prepared,cleans=physical_calls;
    CHECK(!NativeAdrenoMemory::prepareSystem(64u<<10,false,&pages) && pages.descriptor && pages.cleaned && fixture.prepared==prepared+1);
    CHECK(physical_calls>cleans);
    const unsigned afterPrepare=physical_calls;
    NativeAdrenoMemory f;
    CHECK(!f.allocateSystem(64u<<10,40,64u<<10,false,&pages) && !pages.descriptor && f.system() && f.spanCount()==16);
    CHECK(physical_calls==afterPrepare && fixture.prepared==prepared+1);
    CHECK(!NativeAdrenoMemory::prepareSystem(64u<<10,true,&pages) && pages.descriptor && !pages.cleaned);
    NativeAdrenoMemory g;   /* a request of another size makes its own */
    CHECK(!g.allocateSystem(128u<<10,40,128u<<10,true,&pages) && pages.descriptor && fixture.prepared==prepared+3);
    NativeAdrenoMemory::discardSystem(&pages);
    CHECK(!pages.descriptor && !f.release() && !g.release() && fixture.live.empty());
    /* Holders: every system BO above was pid 7's and is released, so its row holds nothing but the peak since it last held none
     * (f and g together). */
    CHECK(native_adreno_bo_owners[0][0]==7 && native_adreno_bo_owners[0][1]==0 && native_adreno_bo_owners[0][2]==0 &&
          native_adreno_bo_owners[0][3]==192);
    /* Imports: the bytes they keep wired, in all and by importing process, until each is released; a refused import counts nowhere. */
    NativeAdrenoMemory i1,i2,i3,i4;
    void *const task=reinterpret_cast<void *>(2);
    CHECK(!i1.importTask(task,0x10000000,1u<<20,40,1u<<30) && i1.imported());
    self_pid=9;
    CHECK(!i2.importTask(task,0x20000000,3u<<20,40,1u<<30));
    CHECK(native_adreno_import_memory[0]==2 && native_adreno_import_memory[1]==(4u<<10) && native_adreno_import_memory[2]==(4u<<10) &&
          native_adreno_import_memory[3]==2);
    CHECK(native_adreno_import_owners[0][0]==7 && native_adreno_import_owners[0][1]==(1u<<10) && native_adreno_import_owners[0][2]==1);
    CHECK(native_adreno_import_owners[1][0]==9 && native_adreno_import_owners[1][1]==(3u<<10) && native_adreno_import_owners[1][2]==1);
    fixture.prepare_fail=true;
    CHECK(i3.importTask(task,0x30000000,1u<<20,40,1u<<30)==kIOReturnIOError && native_adreno_import_memory[0]==2 &&
          native_adreno_import_memory[3]==2);
    fixture.prepare_fail=false;
    CHECK(i4.importTask(task,0x40000000,2u<<20,40,1u<<20)==kIOReturnNoResources && native_adreno_import_memory[1]==(4u<<10));
    self_pid=1;   /* the release may run in any process: the row is the importer's */
    CHECK(!i2.release() && native_adreno_import_memory[0]==1 && native_adreno_import_memory[1]==(1u<<10) &&
          native_adreno_import_memory[2]==(4u<<10));
    CHECK(native_adreno_import_owners[1][1]==0 && native_adreno_import_owners[1][2]==0 && native_adreno_import_owners[1][3]==(3u<<10));
    CHECK(!i1.release() && native_adreno_import_memory[0]==0 && native_adreno_import_memory[1]==0 && native_adreno_import_owners[0][2]==0);
    CHECK(fixture.imports_live==0);
    std::puts("PASS: system BOs: no task, no kernel mapping, write-combined entry, pressure, owned-page map, release, prepared pages, holders and import accounting");
    return 0;
}
