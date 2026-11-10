/* SPDX-License-Identifier: MIT */
/* The pool of DRAM outside the kernel's, over host doubles: registration and its refusals, random allocation and release against a
 * model of every page, the zero fill of whatever a page held, runs that join again, the run limit, exhaustion and the physical
 * limit, and what becomes of a grant a client had a view of. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoPool.h"
#include <cstdio>
#include <map>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
extern "C" unsigned native_adreno_host_pool_drops;
static uint32_t pool_argument=2;   /* viewed pages are recycled, so the cache pass shows */
extern "C" boolean_t PE_parse_boot_argn(const char *name,void *out,int)
{if(std::strcmp(name,"gpu-pool"))return 0;*static_cast<uint32_t *>(out)=pool_argument;return 1;}
static const uint64_t A=UINT64_C(0xbab00000),ABytes=UINT64_C(64)<<20,B=UINT64_C(0xe0000000),BBytes=UINT64_C(16)<<20;
static std::map<uint64_t,int> owner;   /* page -> the grant that holds it */
static uint64_t rng=88172645463325252ull;
static uint64_t next() {rng^=rng<<13;rng^=rng>>7;rng^=rng<<17;return rng;}
static void claim(const NativeAdrenoPoolGrant *grant,int id,size_t bytes,uint64_t limit)
{
    uint64_t total=0;
    CHECK(grant->count && grant->count<=NativeAdrenoPoolRunLimit(bytes/4096) && grant->bytes==bytes && grant->references==1);
    for(uint32_t i=0;i<grant->count;++i) {
        const NativeAdrenoPoolRun &run=grant->runs[i];
        CHECK(run.bytes && !((run.physical|run.bytes)&4095) && run.physical+run.bytes<=limit && !run.viewed);
        CHECK((run.physical>=A && run.physical+run.bytes<=A+ABytes) || (run.physical>=B && run.physical+run.bytes<=B+BBytes));
        for(uint64_t page=run.physical;page<run.physical+run.bytes;page+=4096) {
            CHECK(!owner.count(page));owner[page]=id;
            const unsigned char *bytes_at=static_cast<const unsigned char *>(run.mapping)+(page-run.physical);
            for(size_t k=0;k<4096;k+=512)CHECK(!bytes_at[k]);   /* zero-filled, whatever it held */
            std::memset(run.mapping,0x5a+id%7,4096);            /* the owner's data, for the next owner not to see */
        }
        total+=run.bytes;
    }
    CHECK(total==bytes);
}
static void disown(const NativeAdrenoPoolGrant *grant)
{
    for(uint32_t i=0;i<grant->count;++i)
        for(uint64_t page=grant->runs[i].physical;page<grant->runs[i].physical+grant->runs[i].bytes;page+=4096)CHECK(owner.erase(page)==1);
}
int main()
{
    gPhysBase=UINT64_C(0x100000000);gPhysSize=UINT64_C(6)<<30;
    NativeAdrenoPoolGrant *grant=nullptr;
    /* No range: not usable, and the refusal is counted. */
    CHECK(!NativeAdrenoPoolUsable() && NativeAdrenoPoolAllocate(4096,UINT64_C(1)<<40,true,&grant)==kIOReturnUnsupported && !grant);
    CHECK(native_adreno_pool_fallbacks[NativeAdrenoPoolNoRange]==1 && !NativeAdrenoPoolContains(A,4096));
    /* Registration: whole pages, outside the kernel's DRAM (inside, straddling either end), disjoint. */
    CHECK(native_adreno_pool_add(A+1,4096)==kIOReturnBadArgument && native_adreno_pool_add(A,0)==kIOReturnBadArgument);
    CHECK(native_adreno_pool_add(gPhysBase,4096)==kIOReturnNotPermitted && native_adreno_pool_add(gPhysBase-4096,8192)==kIOReturnNotPermitted);
    CHECK(native_adreno_pool_add(gPhysBase+gPhysSize-4096,8192)==kIOReturnNotPermitted && native_adreno_pool_add(gPhysBase+(1u<<20),4096)==kIOReturnNotPermitted);
    CHECK(!native_adreno_pool_add(A,ABytes) && native_adreno_pool_add(A+ABytes-4096,8192)==kIOReturnBusy && !native_adreno_pool_add(B,BBytes));
    CHECK(!native_adreno_pool_add(gPhysBase+gPhysSize,4096));   /* just past the kernel's DRAM is outside it */
    CHECK(native_adreno_pool_state[0]==3 && native_adreno_pool_state[1]==((ABytes+BBytes)>>10)+4 && native_adreno_pool_state[11]==7);
    CHECK(NativeAdrenoPoolUsable() && NativeAdrenoPoolContains(A+4096,4096) && NativeAdrenoPoolContains(B-4096,8192) && !NativeAdrenoPoolContains(B-4096,4096));
    const uint64_t limit=UINT64_C(1)<<32;   /* the range above 4 GiB is out of reach of this client */
    const uint32_t reachable=uint32_t((ABytes+BBytes)>>10);
    /* A BO that is not write-combined never comes from the pool. */
    CHECK(NativeAdrenoPoolAllocate(4096,limit,false,&grant)==kIOReturnUnsupported && native_adreno_pool_fallbacks[NativeAdrenoPoolKind]==1);
    /* One run that fits best: the 16 MiB range takes a 16 MiB BO whole and leaves the long range alone. */
    CHECK(!NativeAdrenoPoolAllocate(BBytes,limit,true,&grant) && grant->count==1 && grant->runs[0].physical==B);
    claim(grant,1,BBytes,limit);disown(grant);NativeAdrenoPoolRelease(grant);
    /* Random allocation and release against the model: no page twice, every page zero, all of it back at the end. */
    std::vector<NativeAdrenoPoolGrant *> live;std::vector<size_t> sizes;
    int id=10;uint32_t served=0;
    for(unsigned step=0;step<6000;++step) {
        if(live.empty() || next()%5<3) {
            const size_t bytes=size_t(1+next()%(next()%8?64:2048))*4096;
            const IOReturn status=NativeAdrenoPoolAllocate(bytes,limit,true,&grant);
            if(status) {CHECK(status==kIOReturnNoResources && !grant);continue;}
            claim(grant,++id,bytes,limit);live.push_back(grant);sizes.push_back(bytes);++served;
        } else {
            const size_t at=next()%live.size();
            disown(live[at]);NativeAdrenoPoolRelease(live[at]);
            live[at]=live.back();live.pop_back();sizes[at]=sizes.back();sizes.pop_back();
        }
        CHECK(native_adreno_pool_state[2]==reachable+4-uint32_t(owner.size()*4) && native_adreno_pool_state[4]==live.size());
    }
    CHECK(served>1000 && native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted]+native_adreno_pool_fallbacks[NativeAdrenoPoolFragmented]>0);
    for(auto *held:live) {disown(held);NativeAdrenoPoolRelease(held);}
    CHECK(owner.empty() && native_adreno_pool_state[2]==reachable+4 && native_adreno_pool_state[4]==0 && native_adreno_pool_state[5]==0);
    CHECK(native_adreno_pool_state[3]<reachable && native_adreno_pool_state[6]==served+1 && native_adreno_pool_state[7]==served+1);
    CHECK(native_adreno_pool_zero[1]>0 && !native_adreno_host_pool_drops);
    /* Freed neighbours are one run again: the whole 64 MiB range as a single run. */
    CHECK(!NativeAdrenoPoolAllocate(ABytes,limit,true,&grant) && grant->count==1 && grant->runs[0].physical==A);
    NativeAdrenoPoolRelease(grant);
    /* Fragmentation: every other 64 KiB of the long range held. 1 MiB then takes sixteen runs, and 2 MiB as well; with only single
     * pages free between held ones a 1 MiB BO would take 256 runs and is refused, though the pages are there. */
    std::vector<NativeAdrenoPoolGrant *> comb;
    CHECK(!NativeAdrenoPoolAllocate(BBytes,limit,true,&grant));NativeAdrenoPoolGrant *const b=grant;   /* the short range out of the way */
    for(unsigned i=0;i<1024;++i) {CHECK(!NativeAdrenoPoolAllocate(65536,limit,true,&grant) && grant->count==1);comb.push_back(grant);}
    for(unsigned i=0;i<1024;i+=2) {NativeAdrenoPoolRelease(comb[i]);comb[i]=nullptr;}
    CHECK(!NativeAdrenoPoolAllocate(1u<<20,limit,true,&grant) && grant->count==16);
    NativeAdrenoPoolGrant *const sixteen=grant;
    CHECK(!NativeAdrenoPoolAllocate(2u<<20,limit,true,&grant) && grant->count==32);
    NativeAdrenoPoolRelease(grant);NativeAdrenoPoolRelease(sixteen);
    for(auto *held:comb)if(held)NativeAdrenoPoolRelease(held);
    comb.clear();
    for(unsigned i=0;i<4096;++i) {CHECK(!NativeAdrenoPoolAllocate(4096,limit,true,&grant));comb.push_back(grant);}
    for(unsigned i=0;i<4096;i+=2) {NativeAdrenoPoolRelease(comb[i]);comb[i]=nullptr;}
    NativeAdrenoPoolGrant *rest=nullptr;   /* everything behind the comb */
    CHECK(!NativeAdrenoPoolAllocate(ABytes-(16u<<20),limit,true,&rest) && rest->count==1);
    const uint32_t fragmented=native_adreno_pool_fallbacks[NativeAdrenoPoolFragmented],freeBefore=native_adreno_pool_state[2];
    CHECK(NativeAdrenoPoolAllocate(1u<<20,limit,true,&grant)==kIOReturnNoResources && !grant);
    CHECK(native_adreno_pool_fallbacks[NativeAdrenoPoolFragmented]==fragmented+1 && native_adreno_pool_state[2]==freeBefore);
    CHECK(!NativeAdrenoPoolAllocate(65536,limit,true,&grant) && grant->count==16);   /* a small BO may be single pages */
    NativeAdrenoPoolRelease(grant);
    /* Exhausted, too large for the reachable ranges, and a limit below every range. */
    const uint32_t exhausted=native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted];
    CHECK(NativeAdrenoPoolAllocate(32u<<20,limit,true,&grant)==kIOReturnNoResources && native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted]==exhausted+1);
    CHECK(NativeAdrenoPoolAllocate(ABytes+BBytes+4096,limit,true,&grant)==kIOReturnNoResources && native_adreno_pool_fallbacks[NativeAdrenoPoolTooLarge]==1);
    CHECK(NativeAdrenoPoolAllocate(4096,UINT64_C(1)<<31,true,&grant)==kIOReturnNoResources && native_adreno_pool_fallbacks[NativeAdrenoPoolLimit]==1);
    for(auto *held:comb)if(held)NativeAdrenoPoolRelease(held);
    NativeAdrenoPoolRelease(rest);NativeAdrenoPoolRelease(b);
    /* A wider client reaches the third range too: everything, as three runs, the longest first. */
    CHECK(!NativeAdrenoPoolAllocate(ABytes+BBytes+4096,UINT64_C(1)<<36,true,&grant) && grant->count==3);
    CHECK(grant->runs[0].physical==A && grant->runs[1].physical==B && grant->runs[2].physical==gPhysBase+gPhysSize && !native_adreno_pool_state[2]);
    NativeAdrenoPoolRelease(grant);
    CHECK(native_adreno_pool_state[2]==reachable+4 && native_adreno_pool_state[4]==0);
    /* A grant a client had a view of lives until the view lets go, and with gpu-pool=2 its pages come back through the cache pass:
     * the next owner of any of them has their lines dropped before the zero fill, once. */
    CHECK(!NativeAdrenoPoolAllocate(BBytes,limit,true,&grant) && grant->runs[0].physical==B);
    NativeAdrenoPoolRetain(grant);NativeAdrenoPoolNoteView(grant);
    NativeAdrenoPoolRelease(grant);                                   /* the BO goes, the view remains */
    CHECK(native_adreno_pool_state[4]==1 && native_adreno_pool_state[2]==reachable+4-uint32_t(BBytes>>10));
    std::memset(grant->runs[0].mapping,0x77,4096);
    NativeAdrenoPoolRelease(grant);                                   /* the view goes */
    CHECK(native_adreno_pool_state[4]==0 && native_adreno_pool_state[2]==reachable+4 && !native_adreno_pool_state[8]);
    CHECK(!NativeAdrenoPoolAllocate(8192,limit,true,&grant) && grant->runs[0].physical==B);
    CHECK(native_adreno_host_pool_drops==1 && !static_cast<unsigned char *>(grant->runs[0].mapping)[0] && native_adreno_pool_zero[3]==8);
    const uint64_t again=grant->runs[0].physical;
    NativeAdrenoPoolRelease(grant);
    CHECK(!NativeAdrenoPoolAllocate(8192,limit,true,&grant) && grant->runs[0].physical==again && native_adreno_host_pool_drops==1);
    /* A retired grant's pages never come back. */
    NativeAdrenoPoolRetire(grant);NativeAdrenoPoolRelease(grant);
    CHECK(native_adreno_pool_state[8]==1 && native_adreno_pool_state[9]==8 && native_adreno_pool_state[2]==reachable+4-8);
    std::printf("pool: %u served, %u exhausted, %u fragmented, lowest free %u KiB\n",native_adreno_pool_state[6],
        native_adreno_pool_fallbacks[NativeAdrenoPoolExhausted],native_adreno_pool_fallbacks[NativeAdrenoPoolFragmented],native_adreno_pool_state[3]);
    return 0;
}
