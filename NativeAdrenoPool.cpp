/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoPool.h"
#include <IOKit/IOLib.h>
extern "C" boolean_t PE_parse_boot_argn(const char *,void *,int);
extern "C" uint64_t gPhysBase,gPhysSize;   /* the kernel's own DRAM: what phystokv covers and the VM manages */
extern "C" {volatile uint32_t native_adreno_pool_state[12]={0,0,0,0,0,0,0,0,0,0,~0u,0},native_adreno_pool_fallbacks[8];
    volatile uint64_t native_adreno_pool_zero[4];volatile uint32_t native_adreno_pool_views[5];}
namespace {
/* One registered range: a bit a page in `used` (taken by a grant) and in `viewed` (free now, but a client could map it while it was
 * taken: its cache lines are dropped before it is zero-filled again). Ranges are appended under the gate and never removed. */
struct Range {
    Range *next;
    uint64_t physical;
    size_t pages,free_pages;
    uint64_t *used,*viewed;
    unsigned char *mapping;
    void *mapping_owner;
};
Range *ranges;
IOLock *poolGate,*registration;
uint64_t freePages,lowestFree,alivePages,retiredPages;
/* A lock made on first use: the board may register before anything else of the driver ran. */
IOLock *lockOnce(IOLock **slot)
{
    if(IOLock *made=__atomic_load_n(slot,__ATOMIC_ACQUIRE))return made;
    IOLock *fresh=IOLockAlloc();
    if(!fresh)return nullptr;
    IOLock *expected=nullptr;
    if(__atomic_compare_exchange_n(slot,&expected,fresh,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return fresh;
    IOLockFree(fresh);
    return expected;
}
uint32_t kib(uint64_t pages) {return pages>(UINT32_MAX>>2)?UINT32_MAX:(uint32_t)(pages<<2);}
void noteFallback(unsigned why) {__atomic_fetch_add(&native_adreno_pool_fallbacks[why&7],1,__ATOMIC_RELAXED);}
/* The first bit equal to value in [from, end); end when there is none. */
size_t scan(const uint64_t *bits,size_t from,size_t end,bool value)
{
    while(from<end) {
        uint64_t word=value?bits[from/64]:~bits[from/64];
        word&=~UINT64_C(0)<<(from%64);
        if(word) {
            const size_t at=(from&~(size_t)63)+(size_t)__builtin_ctzll(word);
            return at<end?at:end;
        }
        from=(from&~(size_t)63)+64;
    }
    return end;
}
void fill(uint64_t *bits,size_t from,size_t count,bool value)
{
    for(size_t at=from,end=from+count;at<end;) {
        const size_t bit=at%64,span=end-at<64-bit?end-at:64-bit;
        const uint64_t mask=(span==64?~UINT64_C(0):(UINT64_C(1)<<span)-1)<<bit;
        if(value)bits[at/64]|=mask;else bits[at/64]&=~mask;
        at+=span;
    }
}
/* The next run of free pages of the range at or after from. */
bool nextFree(const Range &range,size_t from,size_t *start,size_t *length)
{
    const size_t first=scan(range.used,from,range.pages,false);
    if(first>=range.pages)return false;
    *start=first;*length=scan(range.used,first,range.pages,true)-first;
    return true;
}
bool below(const Range &range,uint64_t limit) {return range.physical+(uint64_t)range.pages*4096<=limit;}
void noteFree()
{
    if(freePages<lowestFree)lowestFree=freePages;
    native_adreno_pool_state[2]=kib(freePages);native_adreno_pool_state[3]=kib(lowestFree);
}
/* count pages of the range from start become the grant's next run. Under the gate. */
void take(Range &range,size_t start,size_t count,NativeAdrenoPoolGrant *grant)
{
    const bool viewed=scan(range.viewed,start,start+count,true)<start+count;
    fill(range.used,start,count,true);
    if(viewed)fill(range.viewed,start,count,false);
    range.free_pages-=count;freePages-=count;
    grant->runs[grant->count++]={range.physical+(uint64_t)start*4096,(uint64_t)count*4096,range.mapping+start*4096,viewed?UINT64_C(1):0};
}
Range *rangeOf(uint64_t physical)
{
    for(Range *range=ranges;range;range=range->next)
        if(physical>=range->physical && physical-range->physical<(uint64_t)range->pages*4096)return range;
    return nullptr;
}
/* The grant's runs go back to their ranges; `viewed` marks them for the cache pass of their next allocation. Under the gate. */
void giveBack(NativeAdrenoPoolGrant *grant,bool viewed)
{
    for(uint32_t i=0;i<grant->count;++i) {
        const NativeAdrenoPoolRun &run=grant->runs[i];
        Range *range=rangeOf(run.physical);
        if(!range)continue;   /* a run always lies in the range it was taken from */
        const size_t start=(size_t)((run.physical-range->physical)/4096),count=(size_t)(run.bytes/4096);
        fill(range->used,start,count,false);
        if(viewed || run.viewed)fill(range->viewed,start,count,true);
        range->free_pages+=count;freePages+=count;
    }
    grant->count=0;
}
enum : unsigned {Served=100,NeedsRuns=101};
/* The pages of a grant, under the gate: one free run when one is long enough (the shortest such, so long runs stay whole), else the
 * longest runs first. Served, NeedsRuns (no single run fits and the grant has room for one run only) or the reason it cannot be. */
unsigned gather(size_t pages,uint64_t limit,NativeAdrenoPoolGrant *grant)
{
    uint64_t total=0,spare=0;
    for(const Range *range=ranges;range;range=range->next)if(below(*range,limit)) {total+=range->pages;spare+=range->free_pages;}
    if(!ranges)return NativeAdrenoPoolNoRange;
    if(!total)return NativeAdrenoPoolLimit;
    if(pages>total)return NativeAdrenoPoolTooLarge;
    if(pages>spare)return NativeAdrenoPoolExhausted;
    Range *best=nullptr;
    size_t bestStart=0,bestLength=SIZE_MAX,longest=0;
    for(Range *range=ranges;range && bestLength!=pages;range=range->next) {
        if(!below(*range,limit) || !range->free_pages)continue;
        for(size_t from=0,start=0,length=0;nextFree(*range,from,&start,&length);from=start+length) {
            if(length>longest)longest=length;
            if(length>=pages && length<bestLength) {best=range;bestStart=start;bestLength=length;if(length==pages)break;}
        }
    }
    if(best) {take(*best,bestStart,pages,grant);noteFree();return Served;}
    const size_t most=NativeAdrenoPoolRunLimit(pages);
    if(grant->capacity<most)return NeedsRuns;
    /* No run holds it: every free run at least half as long as the longest, then half of that, until the pages are found. All but
     * the last are taken whole, so the pool's short runs are used up rather than cut further. */
    size_t remaining=pages;
    for(size_t floor=longest>1?longest/2:1;;floor=floor>1?floor/2:1) {
        for(Range *range=ranges;range && remaining;range=range->next) {
            if(!below(*range,limit) || !range->free_pages)continue;
            for(size_t from=0,start=0,length=0;remaining && nextFree(*range,from,&start,&length);from=start+length) {
                if(length<floor)continue;
                if(grant->count==most) {giveBack(grant,false);return NativeAdrenoPoolFragmented;}
                take(*range,start,length<remaining?length:remaining,grant);
                remaining-=length<remaining?length:remaining;
            }
        }
        if(!remaining) {noteFree();return Served;}
        if(floor==1) {giveBack(grant,false);return NativeAdrenoPoolExhausted;}   /* the free count said otherwise */
    }
}
size_t grantBytes(size_t capacity) {return sizeof(NativeAdrenoPoolGrant)+(capacity-1)*sizeof(NativeAdrenoPoolRun);}
NativeAdrenoPoolGrant *newGrant(size_t capacity)
{
    if(!capacity || capacity>UINT32_MAX)return nullptr;
    auto *grant=static_cast<NativeAdrenoPoolGrant *>(IOMallocZero(grantBytes(capacity)));
    if(grant)grant->capacity=(uint32_t)capacity;
    return grant;
}
bool wanted()
{
    if(native_adreno_pool_state[10]==~0u) {
        uint32_t on=1;PE_parse_boot_argn("gpu-pool",&on,sizeof on);
        native_adreno_pool_state[10]=on>2?1:on;
    }
    return native_adreno_pool_state[10]!=0;
}
/* Whether the pages of a grant a client could map go back to the pool (gpu-pool=2). */
bool recyclesViewed() {return wanted() && native_adreno_pool_state[10]==2;}
}
extern "C" IOReturn native_adreno_pool_add(uint64_t physical,uint64_t bytes)
{
    IOReturn status=kIOReturnSuccess;
    if(!physical || !bytes || ((physical|bytes)&4095) || physical>UINT64_MAX-bytes || bytes/4096>SIZE_MAX/2)status=kIOReturnBadArgument;
    /* Memory the VM manages is never the pool's: the VM would hand the same page to someone else. */
    else if(!gPhysSize || !(physical+bytes<=gPhysBase || (physical>=gPhysBase && physical-gPhysBase>=gPhysSize)))status=kIOReturnNotPermitted;
    IOLock *const adding=status?nullptr:lockOnce(&registration),*const lock=status?nullptr:lockOnce(&poolGate);
    if(!status && (!adding || !lock))status=kIOReturnNoMemory;
    if(status) {__atomic_fetch_add(&native_adreno_pool_state[11],1,__ATOMIC_RELAXED);return status;}
    IOLockLock(adding);
    IOLockLock(lock);
    for(const Range *range=ranges;range;range=range->next)
        if(physical<range->physical+(uint64_t)range->pages*4096 && range->physical<physical+bytes)status=kIOReturnBusy;
    IOLockUnlock(lock);
    const size_t pages=(size_t)(bytes/4096),words=(pages+63)/64;
    Range *range=nullptr;
    if(!status) {
        range=static_cast<Range *>(IOMallocZero(sizeof(Range)));
        if(range) {
            range->physical=physical;range->pages=range->free_pages=pages;
            range->used=static_cast<uint64_t *>(IOMallocZero(words*8));
            range->viewed=static_cast<uint64_t *>(IOMallocZero(words*8));
            if(range->used && range->viewed)
                range->mapping=static_cast<unsigned char *>(NativeAdrenoPoolMapRange(physical,(size_t)bytes,&range->mapping_owner));
        }
        if(!range || !range->mapping) {
            if(range && range->used)IOFree(range->used,words*8);
            if(range && range->viewed)IOFree(range->viewed,words*8);
            if(range)IOFree(range,sizeof(Range));
            status=kIOReturnNoMemory;
        }
    }
    if(!status) {
        IOLockLock(lock);
        Range **tail=&ranges;while(*tail)tail=&(*tail)->next;
        if(!ranges)lowestFree=UINT64_MAX;
        /* Published whole: NativeAdrenoPoolContains walks the list without the gate. */
        __atomic_store_n(tail,range,__ATOMIC_RELEASE);
        freePages+=pages;
        native_adreno_pool_state[1]=native_adreno_pool_state[1]+kib(pages);
        noteFree();
        /* Last: an allocation that reads the count without the gate finds the range in place. */
        __atomic_fetch_add(&native_adreno_pool_state[0],1,__ATOMIC_RELEASE);
        IOLockUnlock(lock);
    } else __atomic_fetch_add(&native_adreno_pool_state[11],1,__ATOMIC_RELAXED);
    IOLockUnlock(adding);
    return status;
}
bool NativeAdrenoPoolUsable() {return __atomic_load_n(&native_adreno_pool_state[0],__ATOMIC_ACQUIRE) && wanted();}
IOReturn NativeAdrenoPoolAllocate(size_t bytes,uint64_t limit,bool write_combined,NativeAdrenoPoolGrant **out)
{
    if(!out)return kIOReturnBadArgument;
    *out=nullptr;
    if(!bytes || (bytes&4095))return kIOReturnBadArgument;
    if(!NativeAdrenoPoolUsable()) {noteFallback(NativeAdrenoPoolNoRange);return kIOReturnUnsupported;}
    /* A BO with the default attribute is cleaned through the kernel's cacheable physical aperture, which has no pool page. */
    if(!write_combined) {noteFallback(NativeAdrenoPoolKind);return kIOReturnUnsupported;}
    const size_t pages=bytes/4096;
    /* Nearly every BO is one run; the room for more is made only when the free memory asks for it, outside the gate. */
    NativeAdrenoPoolGrant *grant=newGrant(1);
    unsigned result=NativeAdrenoPoolNoMetadata;
    if(grant) {
        IOLockLock(poolGate);
        result=gather(pages,limit,grant);
        IOLockUnlock(poolGate);
    }
    if(result==NeedsRuns) {
        IOFree(grant,grantBytes(1));
        grant=newGrant(NativeAdrenoPoolRunLimit(pages));
        result=NativeAdrenoPoolNoMetadata;
        if(grant) {
            IOLockLock(poolGate);
            result=gather(pages,limit,grant);
            IOLockUnlock(poolGate);
        }
    }
    if(result!=Served) {
        if(grant)IOFree(grant,grantBytes(grant->capacity));
        noteFallback(result);
        return result==NativeAdrenoPoolNoMetadata?kIOReturnNoMemory:kIOReturnNoResources;
    }
    /* The pages are the grant's alone now; nothing else can reach them until it is handed out, zero-filled. */
    for(uint32_t i=0;i<grant->count;++i) {
        NativeAdrenoPoolRun &run=grant->runs[i];
        const uint64_t began=mach_absolute_time();
        if(NativeAdrenoPoolScrub(run.mapping,(size_t)run.bytes,run.viewed!=0)!=kIOReturnSuccess) {
            /* Not handed out: the runs go back, this one and those after it still marked for the cache pass. */
            IOLockLock(poolGate);
            giveBack(grant,false);
            noteFree();
            IOLockUnlock(poolGate);
            IOFree(grant,grantBytes(grant->capacity));
            noteFallback(NativeAdrenoPoolScrubFailed);
            return kIOReturnIOError;
        }
        const uint64_t ticks=mach_absolute_time()-began;
        __atomic_fetch_add(&native_adreno_pool_zero[0],ticks,__ATOMIC_RELAXED);
        __atomic_fetch_add(&native_adreno_pool_zero[1],run.bytes>>10,__ATOMIC_RELAXED);
        if(run.viewed) {
            /* The pass over the lines is part of the same call: its share is not told apart, the bytes it covered are. */
            __atomic_fetch_add(&native_adreno_pool_zero[2],ticks,__ATOMIC_RELAXED);
            __atomic_fetch_add(&native_adreno_pool_zero[3],run.bytes>>10,__ATOMIC_RELAXED);
            run.viewed=0;
        }
    }
    grant->references=1;grant->bytes=bytes;
    IOLockLock(poolGate);
    alivePages+=pages;
    native_adreno_pool_state[4]=native_adreno_pool_state[4]+1;native_adreno_pool_state[5]=kib(alivePages);
    native_adreno_pool_state[6]=native_adreno_pool_state[6]+1;
    IOLockUnlock(poolGate);
    *out=grant;
    return kIOReturnSuccess;
}
void NativeAdrenoPoolRetain(NativeAdrenoPoolGrant *grant)
{
    if(grant)__atomic_fetch_add(&grant->references,1,__ATOMIC_RELAXED);
}
void NativeAdrenoPoolRelease(NativeAdrenoPoolGrant *grant)
{
    if(!grant || __atomic_sub_fetch(&grant->references,1,__ATOMIC_ACQ_REL))return;
    const uint64_t pages=grant->bytes/4096;
    IOLockLock(poolGate);
    alivePages-=pages<alivePages?pages:alivePages;
    native_adreno_pool_state[4]=native_adreno_pool_state[4]?native_adreno_pool_state[4]-1:0;
    native_adreno_pool_state[5]=kib(alivePages);
    /* A grant a client had a view of: IOKit has freed every mapping it made of that view by now (the view held a reference until
     * then), but a copy of such a mapping made in the VM (mach_vm_remap, a memory entry) keeps the view's VM object, and nothing here
     * can see or revoke it. Such pages stay out of the pool unless the board chose gpu-pool=2. */
    const bool viewed=__atomic_load_n(&grant->viewed,__ATOMIC_RELAXED)!=0;
    const bool untracked=__atomic_load_n(&grant->untracked,__ATOMIC_RELAXED)!=0;
    /* Every reference is gone: the BO's, and each view's, a tracked view's only once the kernel reported that nothing maps it. */
    if(__atomic_load_n(&grant->retired,__ATOMIC_RELAXED) || (untracked && !recyclesViewed())) {
        retiredPages+=pages;
        native_adreno_pool_state[8]=native_adreno_pool_state[8]+1;native_adreno_pool_state[9]=kib(retiredPages);
    } else {
        giveBack(grant,viewed);
        native_adreno_pool_state[7]=native_adreno_pool_state[7]+1;
        if(viewed && !untracked)native_adreno_pool_views[3]=native_adreno_pool_views[3]+1;
        noteFree();
    }
    IOLockUnlock(poolGate);
    IOFree(grant,grantBytes(grant->capacity));
}
void NativeAdrenoPoolNoteView(NativeAdrenoPoolGrant *grant,bool tracked)
{
    if(!grant)return;
    __atomic_store_n(&grant->viewed,(uint8_t)1,__ATOMIC_RELAXED);
    if(!tracked)__atomic_store_n(&grant->untracked,(uint8_t)1,__ATOMIC_RELAXED);
    __atomic_fetch_add(&native_adreno_pool_views[tracked?0:1],1,__ATOMIC_RELAXED);
}
extern "C" void NativeAdrenoPoolViewClosed(void *grant)
{
    /* From the descriptor's free or from the VM's teardown of the last mapping, possibly under IOKit's memory lock: the release takes
     * only the allocator's gate (a leaf) and frees the grant's own memory. */
    __atomic_fetch_add(&native_adreno_pool_views[2],1,__ATOMIC_RELAXED);
    NativeAdrenoPoolRelease(static_cast<NativeAdrenoPoolGrant *>(grant));
}
void NativeAdrenoPoolRetire(NativeAdrenoPoolGrant *grant)
{
    if(grant)__atomic_store_n(&grant->retired,(uint8_t)1,__ATOMIC_RELAXED);
}
bool NativeAdrenoPoolContains(uint64_t physical,uint64_t bytes)
{
    /* Without the gate (an import asks for each of its pages): a range's place and size never change once it is in the list, and the
     * list only grows at its tail. */
    if(!bytes)return false;
    for(const Range *range=__atomic_load_n(&ranges,__ATOMIC_ACQUIRE);range;range=__atomic_load_n(&range->next,__ATOMIC_ACQUIRE))
        if(physical<range->physical+(uint64_t)range->pages*4096 && range->physical<physical+bytes)return true;
    return false;
}
