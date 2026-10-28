/* SPDX-License-Identifier: MIT */
/* Prepare/step-ordering tests over host doubles; no SMMU is programmed. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoSmmuContext.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *state)
{CHECK(state);*state={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{CHECK(mapping && bytes && op<=NativeAdrenoInvalidate);return kIOReturnSuccess;}
void NativeAdrenoNotePhase(unsigned,size_t) {}
int main()
{
    const uint64_t base=UINT64_C(0xffffffff00000000)+0x10000;
    NativeAdrenoTablePages tables;
    NativeAdrenoSmmuContext context;
    CHECK(context.program(nullptr)==kIOReturnBadArgument);
    CHECK(context.attach(nullptr,0)==kIOReturnBadArgument);
    CHECK(context.observe()==kIOReturnBadArgument);
    CHECK(context.disable()==kIOReturnBadArgument);
    CHECK(context.prepare(&tables,1,base,0x4000,8,2,1,1000,40,32)==kIOReturnSuccess);
    CHECK(context.prepared() && !context.programmed() && !context.attached());
    CHECK(context.configuration()->upper!=nullptr && context.configuration()->lower==nullptr);
    CHECK(context.configuration()->asid==1 && context.configuration()->coherent_walk==1);
    CHECK(context.prepare(&tables,1,base,0x4000,8,2,1,1000,40,32)==kIOReturnBusy);
    CHECK(context.program(nullptr)==kIOReturnBadArgument);
    CHECK(context.attach(nullptr,1)==kIOReturnBadArgument);
    CHECK(context.observe()==kIOReturnBadArgument);
    CHECK(context.disable()==kIOReturnBadArgument);
    /* A refused pool or bad bank/ASID/budget never prepares. */
    NativeAdrenoSmmuContext bad;
    CHECK(bad.prepare(nullptr,1,base,0x4000,8,2,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base+1,0x4000,8,2,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0,8,2,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,0,2,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,8,64,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,8,2,0x10000,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,8,2,1,0,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,2,base,0x4000,8,2,1,1000,40,32)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,8,2,1,1000,40,31)==kIOReturnBadArgument);
    CHECK(bad.prepare(&tables,1,base,0x4000,8,2,1,1000,40,49)==kIOReturnBadArgument);
    CHECK(!bad.prepared());
    /* The GMU's lower region: TTBR0 only. */
    NativeAdrenoTablePages low_tables;
    NativeAdrenoSmmuContext low;
    CHECK(low.prepare(&low_tables,0,0x60000000,0x8000,8,1,1,1000,40,32)==kIOReturnSuccess);
    CHECK(low.configuration()->lower!=nullptr && low.configuration()->upper==nullptr);
    CHECK(low.configuration()->lower_tables!=nullptr && low.configuration()->upper_tables==nullptr);
    CHECK(low_tables.release()==kIOReturnSuccess);
    CHECK(context.configuration()->upper_tables!=nullptr);
    CHECK(tables.release()==kIOReturnSuccess);
    CHECK(fixture.live.empty() && fixture.metadata.empty());
    std::printf("PASS: 1 prepared upper and 1 prepared lower SMMU context, 6 step-order refusals and 10 prepare refusals over host doubles\n");
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
