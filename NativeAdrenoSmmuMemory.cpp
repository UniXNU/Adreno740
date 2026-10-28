/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoSmmuMemory.h"
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/task.h>
#include <string.h>
/* The GPU SMMU walks its tables coherently (sm8550.dtsi: dma-coherent; Linux then cleans no descriptor): once the board proved it
 * (GpuPowerProbe coherenceProbe), a changed table page needs its stores ordered before the walk, not cleaned. [0] proved, [1] pages
 * published by ordering alone, [2] pages cleaned. */
extern "C" {volatile uint32_t native_adreno_walk_coherent[3];}
/* Table pages of every GPU address space: [0] live, [1] taken, [2] freed, [3] refused by the system. */
extern "C" {volatile uint32_t native_adreno_table_pages[4];}
unsigned NativeAdrenoSmmuInputBits(uint32_t id2)
{
    static const uint8_t bits[5]={32,36,40,42,44};
    const unsigned code=(id2>>8)&15;
    return code<5?bits[code]:48;
}
size_t NativeAdrenoTablePages::lowerBound(uint64_t physical) const
{
    size_t low=0,high=count_;
    while(low<high) {const size_t mid=low+(high-low)/2;if(pages_[mid].physical<physical)low=mid+1;else high=mid;}
    return low;
}
IOReturn NativeAdrenoTablePages::begin(unsigned output_bits)
{
    if(count_ || pages_ || quarantined_)return kIOReturnBusy;
    if(output_bits<32 || output_bits>48)return kIOReturnBadArgument;
    limit_=UINT64_C(1)<<output_bits;
    provider_={this,allocatePage,findPage,overlapsPages};
    return kIOReturnSuccess;
}
int NativeAdrenoTablePages::allocatePage(void *cookie,uint64_t *physical,volatile uint64_t **mapping)
{
    auto *self=static_cast<NativeAdrenoTablePages *>(cookie);
    if(!self || !physical || !mapping || !self->limit_ || self->quarantined_)return 1;
    if(self->count_==self->capacity_) {
        const size_t size=self->capacity_?self->capacity_*2:64;
        Page *made=IONewData(Page,size);
        if(!made) {native_adreno_table_pages[3]=native_adreno_table_pages[3]+1;return 1;}
        if(self->count_)memcpy(made,self->pages_,sizeof(Page)*self->count_);
        if(self->pages_)IODeleteData(self->pages_,Page,self->capacity_);
        self->pages_=made;self->capacity_=size;
    }
    /* Cacheable (the walk is coherent, and the kit reads descriptors on every map and unmap: int-153), wired, one page. */
    IOBufferMemoryDescriptor *page=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|kIOMemoryPhysicallyContiguous,
        4096,(self->limit_-1)&~UINT64_C(4095));
    if(!page) {native_adreno_table_pages[3]=native_adreno_table_pages[3]+1;return 1;}
    if(page->prepare()!=kIOReturnSuccess) {page->release();native_adreno_table_pages[3]=native_adreno_table_pages[3]+1;return 1;}
    IOByteCount length=0;
    const addr64_t at=page->getPhysicalSegment(0,&length,kIOMemoryMapperNone);
    void *virt=page->getBytesNoCopy();
    if(!at || (at&4095) || at>=self->limit_ || length<4096 || !virt || ((uintptr_t)virt&4095) || findPage(self,at)) {
        (void)page->complete();page->release();native_adreno_table_pages[3]=native_adreno_table_pages[3]+1;return 1;
    }
    memset(virt,0,4096);
    const size_t i=self->lowerBound(at);
    memmove(self->pages_+i+1,self->pages_+i,sizeof(Page)*(self->count_-i));
    self->pages_[i]={at,static_cast<volatile uint64_t *>(virt),page};++self->count_;
    native_adreno_table_pages[0]=native_adreno_table_pages[0]+1;native_adreno_table_pages[1]=native_adreno_table_pages[1]+1;
    *physical=at;*mapping=static_cast<volatile uint64_t *>(virt);
    return 0;
}
volatile uint64_t *NativeAdrenoTablePages::findPage(void *cookie,uint64_t physical)
{
    auto *self=static_cast<NativeAdrenoTablePages *>(cookie);
    if(!self)return nullptr;
    const size_t i=self->lowerBound(physical);
    return i<self->count_ && self->pages_[i].physical==physical?self->pages_[i].mapping:nullptr;
}
int NativeAdrenoTablePages::overlapsPages(void *cookie,uint64_t physical,uint64_t bytes)
{
    auto *self=static_cast<NativeAdrenoTablePages *>(cookie);
    return self && self->overlaps(physical,bytes);
}
bool NativeAdrenoTablePages::overlaps(uint64_t physical,uint64_t bytes) const
{
    if(!bytes || !count_ || physical>UINT64_MAX-bytes)return !!bytes && !!count_;
    /* The pages are sorted and disjoint: only the first one ending after physical can be the lowest that overlaps. */
    const size_t i=lowerBound(physical>4095?physical-4095:0);
    return i<count_ && pages_[i].physical<physical+bytes;
}
bool NativeAdrenoTablePages::contains(const volatile void *mapping) const
{
    for(size_t i=0;i<count_;++i)if(pages_[i].mapping==mapping)return true;
    return false;
}
IOReturn NativeAdrenoTablePages::release()
{
    if(quarantined_)return kIOReturnBusy;
    for(size_t i=0;i<count_;++i) {
        IOBufferMemoryDescriptor *page=pages_[i].descriptor;
        if(page->complete()!=kIOReturnSuccess) {quarantined_=true;return kIOReturnIOError;}
        page->release();pages_[i].descriptor=nullptr;
        native_adreno_table_pages[0]=native_adreno_table_pages[0]-1;native_adreno_table_pages[2]=native_adreno_table_pages[2]+1;
    }
    if(pages_)IODeleteData(pages_,Page,capacity_);
    pages_=nullptr;count_=capacity_=0;limit_=0;provider_={};
    return kIOReturnSuccess;
}
static int publishTable(void *cookie,volatile uint64_t *page,size_t bytes)
{
    if(native_adreno_walk_coherent[0]) {
        __asm__ volatile("dsb st":::"memory");
        native_adreno_walk_coherent[1]=native_adreno_walk_coherent[1]+1;
        return 0;
    }
    native_adreno_walk_coherent[2]=native_adreno_walk_coherent[2]+1;
    auto *pages=static_cast<NativeAdrenoTablePages *>(cookie);
    if(!pages || !page || bytes!=4096 || ((uintptr_t)page&4095) || !pages->contains(page))return 1;
    NativeAdrenoNotePhase(NativeAdrenoPhaseTable,bytes);
    return NativeAdrenoMaintainOwnedPages(const_cast<uint64_t *>(page),bytes,NativeAdrenoClean)==kIOReturnSuccess?0:1;
}
IOReturn NativeAdrenoBindSmmuVisibility(NativeAdrenoTablePages *pages,ArmSmmu4KLiveVisibility *visibility)
{
    if(!visibility)return kIOReturnBadArgument;*visibility={};
    if(!pages || pages->quarantined() || !pages->provider() || !pages->pageCount())return kIOReturnBadArgument;
    *visibility={pages,publishTable};return kIOReturnSuccess;
}
IOReturn NativeAdrenoPrepareSmmuMemory(NativeAdrenoTablePages *pages,unsigned input_bits,
    unsigned output_bits,uint64_t iova,uint64_t bytes,unsigned upper,size_t budget,
    ArmSmmu4KLiveArena *arena,ArmSmmu4KLiveState *state,ArmSmmu4KLiveVisibility *visibility)
{
    if(!pages || !arena || !state || !visibility)return kIOReturnBadArgument;
    *visibility={};
    if(state->prepared || state->poisoned || pages->quarantined() || pages->pageCount())return kIOReturnBusy;
    IOReturn result=pages->begin(output_bits);
    if(result!=kIOReturnSuccess)return result;
    const ArmSmmu4KTablePages *provider=pages->provider();
    uint64_t root=0;volatile uint64_t *mapping=nullptr;
    if(provider->allocate(provider->cookie,&root,&mapping))return kIOReturnNoMemory;
    *arena={{input_bits,output_bits,root,budget},iova,bytes,upper,provider,nullptr,0};
    ArmSmmu4KStatus status=ArmSmmu4KLivePrepare(arena,state);
    if(status!=ArmSmmu4KSuccess)return status==ArmSmmu4KCapacity?kIOReturnNoResources:kIOReturnBadArgument;
    ArmSmmu4KLiveResult audit;
    status=ArmSmmu4KLiveAudit(arena,state,nullptr,0,&audit);
    if(status!=ArmSmmu4KSuccess) {state->poisoned=1;return kIOReturnIOError;}
    result=NativeAdrenoBindSmmuVisibility(pages,visibility);
    if(result==kIOReturnSuccess && visibility->publish_table_page(visibility->cookie,mapping,4096))result=kIOReturnIOError;
    if(result!=kIOReturnSuccess) {*visibility={};state->poisoned=1;}
    return result;
}
