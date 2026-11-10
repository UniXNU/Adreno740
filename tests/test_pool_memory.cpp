/* SPDX-License-Identifier: MIT */
/* System BOs of pool pages over host doubles: a write-combined BO takes a grant and no descriptor, its spans are the grant's runs,
 * nothing of it is ever maintained, release and the release after the lock give the pages up, a quarantined BO keeps them, a viewed
 * grant is retired (gpu-pool=1), an import of pool pages is refused and whatever the pool cannot serve takes the descriptor path. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include "NativeAdrenoPool.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
extern "C" volatile uint32_t native_adreno_bo_memory[8];
extern "C" int proc_selfpid() {return 7;}
static unsigned physical_calls,owned_calls;
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out) {CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *,size_t,NativeAdrenoCacheOperation) {++owned_calls;return kIOReturnSuccess;}
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *,size_t,bool) {CHECK(false);return kIOReturnSuccess;}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *,size_t,uint32_t *) {CHECK(false);return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t physical,size_t,NativeAdrenoCacheOperation)
{CHECK(physical<UINT64_C(0xbab00000));++physical_calls;return kIOReturnSuccess;}   /* never a pool page */
int main()
{
    gPhysBase=0;gPhysSize=UINT64_C(0x80000000);   /* the fixture's descriptor pages are the kernel's DRAM, the pool lies above */
    const uint64_t P=UINT64_C(0xbab00000),PBytes=UINT64_C(8)<<20;
    fixture.mode=ScatterUnlessContiguous;
    CHECK(!native_adreno_pool_add(P,PBytes));
    /* A write-combined system BO: a grant, one span, no descriptor, no kernel mapping, nothing wired or cleaned. */
    NativeAdrenoMemory a;
    CHECK(!a.allocateSystem(2u<<20,40,2u<<20,true) && a.system() && a.writeCombined() && a.poolGrant() && !a.descriptor() && !a.mapping());
    CHECK(a.backed() && a.spanCount()==1 && a.spans()[0].physical==P && a.spans()[0].bytes==(2u<<20) && !fixture.created);
    CHECK(native_adreno_bo_memory[0]==1 && native_adreno_bo_memory[4]==2 && native_adreno_pool_state[4]==1 && native_adreno_pool_state[5]==2048);
    CHECK(!a.holdsCpuLines(false) && !a.holdsCpuLines(true) && !a.reassertable() && !NativeAdrenoMemory::ownsAnyPage(a.spans(),1));
    CHECK(!a.publishForDevice(0,2u<<20,true,true) && !a.acquireDeviceWrites(0,2u<<20) && !physical_calls && !owned_calls);
    CHECK(a.makeWriteCombined()==kIOReturnNotReady && a.declareSynchronized()==kIOReturnNotReady);
    CHECK(!a.holdForDevice() && a.release()==kIOReturnBusy && !a.retireAfterQuiescence());
    /* A default-attribute BO takes the descriptor path (it is cleaned through the kernel's aperture). */
    NativeAdrenoMemory c;
    CHECK(!c.allocateSystem(65536,40,65536,false) && !c.poolGrant() && c.descriptor() && fixture.created==1 && physical_calls==16);
    CHECK(native_adreno_pool_fallbacks[NativeAdrenoPoolKind]==1 && !c.release());
    /* The physical limit: a 32-bit client reaches the pool, a narrower table would not (the pool refuses, the VM serves). */
    NativeAdrenoMemory n;
    CHECK(!n.allocateSystem(65536,32,65536,true) && n.poolGrant() && !n.release());
    /* Prepared before the lock and adopted; pages not taken go back whole. */
    NativeAdrenoSystemPages pages{};
    CHECK(!NativeAdrenoMemory::prepareSystem(1u<<20,true,&pages,40) && pages.grant && !pages.descriptor && pages.cleaned && pages.held());
    NativeAdrenoMemory d;
    CHECK(!d.allocateSystem(1u<<20,40,1u<<20,true,&pages) && !pages.held() && d.poolGrant() && d.spans()[0].physical==P+(2u<<20));
    CHECK(!NativeAdrenoMemory::prepareSystem(1u<<20,true,&pages,40) && pages.grant && native_adreno_pool_state[4]==3);
    NativeAdrenoMemory::discardSystem(&pages);
    CHECK(!pages.held() && native_adreno_pool_state[4]==2);
    CHECK(!NativeAdrenoMemory::prepareSystem(1u<<20,true,&pages) && pages.descriptor && !pages.grant);   /* no width given: the VM's pages */
    NativeAdrenoMemory::discardSystem(&pages);
    /* The release after the lock hands the grant out; the pages return when it is discarded. */
    NativeAdrenoSystemPages freed{};
    const uint32_t freeBefore=native_adreno_pool_state[2];
    CHECK(!d.releaseSystemInto(&freed) && freed.grant && !freed.descriptor && !d.backed() && !d.poolGrant() && !d.spanCount());
    CHECK(native_adreno_pool_state[2]==freeBefore && native_adreno_bo_memory[4]==2);
    NativeAdrenoMemory::discardSystem(&freed);
    CHECK(native_adreno_pool_state[2]==freeBefore+1024 && native_adreno_pool_state[4]==1);
    /* More than the pool has free: the descriptor path, unchanged, and the pool is whole. */
    NativeAdrenoMemory big;
    CHECK(!big.allocateSystem(7u<<20,40,7u<<20,true) && !big.poolGrant() && big.descriptor() && big.spanCount()==(7u<<20)/4096);
    CHECK(native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted]==1 && !big.release());
    /* An import whose pages lie in the pool (a client's view of a pool BO) is refused; fixture imports begin at next_physical. */
    NativeAdrenoMemory imported;
    const uint64_t saved=fixture.next_physical;
    fixture.next_physical=P+(4u<<20);
    CHECK(imported.importTask(reinterpret_cast<void *>(2),0x10000,8192,40,8192)==kIOReturnNotPermitted && !imported.backed() && !fixture.imports_live);
    fixture.next_physical=saved;
    CHECK(!imported.importTask(reinterpret_cast<void *>(2),0x10000,8192,40,8192) && !imported.release());
    /* A quarantined BO keeps its pages: nothing returns to the pool. */
    NativeAdrenoMemory q;
    CHECK(!q.allocateSystem(65536,40,65536,true) && q.poolGrant());
    const uint32_t held=native_adreno_pool_state[2];
    q.quarantine();
    CHECK(q.release()==kIOReturnBusy && q.releaseSystemInto(&freed)==kIOReturnBusy && !freed.held() && q.poolGrant());
    CHECK(native_adreno_pool_state[2]==held && native_adreno_pool_state[4]==2);
    /* A client view holds the grant past the BO; with the default gpu-pool the viewed pages are then retired, not reused. */
    NativeAdrenoPoolGrant *const view=a.poolGrant();
    NativeAdrenoPoolRetain(view);NativeAdrenoPoolNoteView(view);
    CHECK(!a.release() && !a.backed() && native_adreno_pool_state[4]==2 && native_adreno_pool_state[2]==held);
    NativeAdrenoPoolRelease(view);
    CHECK(native_adreno_pool_state[4]==1 && native_adreno_pool_state[8]==1 && native_adreno_pool_state[9]==2048 && native_adreno_pool_state[2]==held);
    CHECK(fixture.live.empty() && !owned_calls);
    std::printf("pool memory: %u served, fallbacks kind %u exhausted %u\n",native_adreno_pool_state[6],
        native_adreno_pool_fallbacks[NativeAdrenoPoolKind],native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted]);
    return 0;
}
