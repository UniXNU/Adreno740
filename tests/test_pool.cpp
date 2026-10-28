/* SPDX-License-Identifier: MIT */
/* The BO pool's chunks over host doubles: a reserved first chunk, chunks taken on demand and given back when empty, memory pressure
 * when the system has no contiguous chunk. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
extern "C" volatile uint32_t native_adreno_pool_chunks[4];
static unsigned attribute_calls;
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out) {CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation)
{CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));++fixture.cache_calls;return kIOReturnSuccess;}
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *spans,size_t count,bool)
{++attribute_calls;return spans && count?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *,size_t,uint32_t *) {return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t,size_t,NativeAdrenoCacheOperation) {return kIOReturnSuccess;}
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
void NativeAdrenoNotePhase(unsigned,size_t) {}
extern "C" unsigned int native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
int main()
{
    /* A 1 MiB reserved chunk, write-combined once. */
    CHECK(NativeAdrenoMemory::reservePool(40,1u<<20)==kIOReturnSuccess);
    CHECK(NativeAdrenoMemory::writeCombinePool()==kIOReturnSuccess && attribute_calls==1);
    CHECK(NativeAdrenoMemory::poolBytes()==(1u<<20) && native_adreno_pool_chunks[0]==1);
    NativeAdrenoMemory a,b,c,d,e;
    CHECK(!a.allocate(512*1024,40,false,512*1024,true) && a.pooled() && a.spanCount()==1);
    CHECK(!b.allocate(512*1024,40,false,512*1024,true) && b.pooled());
    CHECK(NativeAdrenoMemory::poolContains(a.spans()[0].physical,4096) && !NativeAdrenoMemory::poolContains(1,4096));
    /* The reserved chunk is full: a 2 MiB chunk is made on demand and write-combined as it is made. */
    CHECK(!c.allocate(4096,40,false,4096,true) && c.pooled() && native_adreno_pool_chunks[0]==2 && native_adreno_pool_chunks[1]==1);
    CHECK(attribute_calls==2 && NativeAdrenoMemory::poolBytes()==(3u<<20) && !NativeAdrenoMemory::poolPressure());
    /* A BO larger than 2 MiB gets a chunk of its own size. */
    CHECK(!d.allocate(3u<<20,40,false,3u<<20,true) && d.pooled() && native_adreno_pool_chunks[0]==3);
    /* No contiguous chunk to be had: pressure, and the BO falls back to scattered pages. */
    fixture.contiguous_limit=32*1024;
    CHECK(!e.allocate(4u<<20,40,false,4u<<20,true) && !e.pooled() && NativeAdrenoMemory::poolPressure());
    CHECK(native_adreno_pool_chunks[3]==1);
    fixture.contiguous_limit=0;
    /* An empty on-demand chunk goes back to the system (attribute restored); the reserved chunk stays. */
    CHECK(!d.release() && native_adreno_pool_chunks[0]==2 && native_adreno_pool_chunks[2]==1 && attribute_calls==4);
    CHECK(!c.release() && native_adreno_pool_chunks[0]==1 && native_adreno_pool_chunks[2]==2);
    CHECK(!a.release() && !b.release() && native_adreno_pool_chunks[0]==1 && NativeAdrenoMemory::poolBytes()==(1u<<20));
    /* An allocation the pool serves again ends the pressure. */
    NativeAdrenoMemory f;
    CHECK(!f.allocate(1u<<20,40,false,1u<<20,true) && f.pooled() && !NativeAdrenoMemory::poolPressure());
    CHECK(!f.allocate(4096,40,false,4096,true)==false);   /* an allocated object refuses a second allocation */
    CHECK(!f.release() && !e.release());
    std::printf("PASS: BO pool with a reserved chunk, 2 chunks made on demand, 2 given back when empty, 1 pressure fallback and its end\n");
}
