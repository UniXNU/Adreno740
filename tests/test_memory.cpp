/* SPDX-License-Identifier: MIT */
/* Real memory-management implementation linked to host-only IOKit doubles.
 * Production CPU/cache assembly is separately cross-built, never run here. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{CHECK(out);*out={};return fixture.cpu_status;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));
    bool owned=false;
    for(auto *d:fixture.live)if((uintptr_t)mapping>=(uintptr_t)d->raw &&
        (uintptr_t)mapping-(uintptr_t)d->raw<=d->bytes &&
        bytes<=d->bytes-((uintptr_t)mapping-(uintptr_t)d->raw))owned=true;
    CHECK(owned);++fixture.cache_calls;fixture.cache_operations.push_back(op);
    return fixture.cache_calls==fixture.cache_fail_call?kIOReturnIOError:kIOReturnSuccess;
}
static void reset()
{CHECK(fixture.live.empty() && fixture.metadata.empty());fixture=MemoryFixture{};}
/* Host fixture teardown only. This is not a production reset/release path. */
static void discardFixtureAfterQuarantine()
{
    while(!fixture.live.empty())fixture.live.back()->release();
    while(!fixture.metadata.empty())fixtureMetadataFree(fixture.metadata.back());
    reset();
}
static unsigned successes()
{
    unsigned profiles=0,outputs[]={32,36,40,42,44,48};
    for(unsigned output:outputs)for(unsigned mode:{Contiguous,Scattered,Paired}) {
        reset();fixture.mode=static_cast<SegmentMode>(mode);NativeAdrenoMemory memory;
        CHECK(memory.allocate(16384,output,mode==Contiguous,16384)==kIOReturnSuccess);
        CHECK(memory.bytes()==16384 && memory.mapping() && memory.descriptor() && !memory.quarantined());
        size_t count=mode==Contiguous?1:mode==Scattered?4:2;
        CHECK(memory.spanCount()==count && fixture.prepared==1 && fixture.segment_calls==4);
        for(size_t i=0;i<16384;++i)CHECK(static_cast<unsigned char *>(memory.mapping())[i]==0);
        uint64_t bytes=0;
        for(size_t i=0;i<count;++i) {
            const auto &span=memory.spans()[i];CHECK(span.offset==bytes);
            CHECK(span.physical==0x30000000+span.offset+(mode==Scattered?span.offset:mode==Paired?(span.offset/8192)*4096:0));
            bytes+=span.bytes;
        }
        CHECK(bytes==16384 && fixture.cache_calls==1 && fixture.cache_operations[0]==NativeAdrenoClean);
        CHECK(memory.holdForDevice()==kIOReturnSuccess && memory.holdForDevice()==kIOReturnSuccess);
        CHECK(memory.deviceReferences()==2 && memory.release()==kIOReturnBusy && !fixture.completed);
        CHECK(memory.publishForDevice(4096,4096,false)==kIOReturnSuccess);
        CHECK(fixture.cache_operations.back()==NativeAdrenoClean);
        CHECK(memory.publishForDevice(0,16384,true)==kIOReturnSuccess);
        CHECK(fixture.cache_operations.back()==NativeAdrenoCleanInvalidate);
        CHECK(memory.acquireDeviceWrites(0,16384)==kIOReturnSuccess);
        CHECK(fixture.cache_operations.back()==NativeAdrenoInvalidate);
        CHECK(memory.retireAfterQuiescence()==kIOReturnSuccess);
        CHECK(memory.release()==kIOReturnBusy && !fixture.completed);
        CHECK(memory.retireAfterQuiescence()==kIOReturnSuccess && !memory.deviceReferences());
        CHECK(memory.retireAfterQuiescence()==kIOReturnBadArgument);
        CHECK(memory.release()==kIOReturnSuccess && !memory.bytes() && !memory.descriptor() && !memory.spans());
        CHECK(fixture.completed==1 && fixture.released==1 && fixture.live.empty());
        CHECK(memory.release()==kIOReturnSuccess && memory.holdForDevice()==kIOReturnNotReady);++profiles;
    }
    return profiles;
}
static unsigned failures()
{
    unsigned controls=0;
    for(unsigned failure=0;failure<12;++failure) {
        reset();NativeAdrenoMemory memory;IOReturn expected=kIOReturnUnsupported;
        if(failure==0) {fixture.allocation_fail=true;expected=kIOReturnNoMemory;}
        if(failure==1) {fixture.prepare_fail=true;expected=kIOReturnIOError;}
        if(failure==2)fixture.null_mapping=true;
        if(failure==3)fixture.unaligned_mapping=true;
        if(failure==4)fixture.wrong_length=true;
        if(failure==5) {fixture.metadata_fail=true;expected=kIOReturnNoMemory;}
        if(failure>=6 && failure<=9)fixture.mode=static_cast<SegmentMode>(ZeroPhysical+(failure-6));
        if(failure==10)fixture.mode=Scattered;
        if(failure==11) {fixture.cache_fail_call=1;expected=kIOReturnIOError;}
        CHECK(memory.allocate(16384,32,true,16384)==expected);
        CHECK(!memory.descriptor() && !memory.spans() && !memory.quarantined() && fixture.live.empty());
        CHECK(fixture.released==fixture.created);
        CHECK(fixture.completed==(failure<2?0:1));++controls;
    }
    for(unsigned cleanup_failure=0;cleanup_failure<3;++cleanup_failure) {
        reset();NativeAdrenoMemory memory;
        if(cleanup_failure==1)fixture.mode=ZeroPhysical;
        if(cleanup_failure==2)fixture.cache_fail_call=1;
        fixture.complete_fail=true;
        IOReturn allocated=memory.allocate(16384,32,false,16384);
        if(!cleanup_failure) {
            CHECK(allocated==kIOReturnSuccess);
            CHECK(memory.release()==kIOReturnIOError);
        } else CHECK(allocated==(cleanup_failure==1?kIOReturnUnsupported:kIOReturnIOError));
        CHECK(memory.quarantined() && memory.descriptor() && memory.bytes()==16384);
        unsigned completed=fixture.completed;
        CHECK(memory.release()==kIOReturnBusy && fixture.completed==completed && !fixture.released);
        CHECK(memory.holdForDevice()==kIOReturnNotReady);
        CHECK(memory.publishForDevice(0,4096,false)==kIOReturnNotReady);
        discardFixtureAfterQuarantine();++controls;
    }
    reset();NativeAdrenoMemory memory;
    CHECK(memory.allocate(16384,32,false,16384)==kIOReturnSuccess);
    CHECK(memory.holdForDevice()==kIOReturnSuccess);memory.quarantine();
    CHECK(memory.retireAfterQuiescence()==kIOReturnNotReady && memory.deviceReferences()==1);
    CHECK(memory.release()==kIOReturnBusy && !fixture.completed && !fixture.released);
    CHECK(memory.acquireDeviceWrites(0,4096)==kIOReturnNotReady);
    discardFixtureAfterQuarantine();++controls;
    return controls;
}
static unsigned arguments()
{
    reset();NativeAdrenoMemory memory;unsigned controls=0;
    CHECK(memory.allocate(0,32,false,16384)==kIOReturnBadArgument);++controls;
    CHECK(memory.allocate(4097,32,false,16384)==kIOReturnBadArgument);++controls;
    CHECK(memory.allocate(4096,33,false,16384)==kIOReturnBadArgument);++controls;
    CHECK(memory.allocate(4096,32,false,0)==kIOReturnBadArgument);++controls;
    CHECK(memory.allocate(16384,32,false,4096)==kIOReturnNoResources);++controls;
    CHECK(!fixture.created && !fixture.cache_calls);
    fixture.cpu_status=kIOReturnUnsupported;
    CHECK(memory.allocate(4096,32,false,4096)==kIOReturnUnsupported);++controls;
    CHECK(!fixture.created);fixture.cpu_status=kIOReturnSuccess;
    CHECK(memory.allocate(16384,32,false,16384)==kIOReturnSuccess);
    CHECK(memory.allocate(16384,32,false,16384)==kIOReturnBusy);++controls;
    for(auto range:std::vector<std::pair<size_t,size_t>>{{0,0},{1,4096},{0,4097},{16384,4096},{SIZE_MAX-4095,4096},{4096,SIZE_MAX-4095}}) {
        CHECK(memory.publishForDevice(range.first,range.second,false)==kIOReturnBadArgument);
        CHECK(memory.acquireDeviceWrites(range.first,range.second)==kIOReturnBadArgument);controls+=2;
    }
    CHECK(fixture.cache_calls==1);CHECK(memory.release()==kIOReturnSuccess);
    return controls;
}
int main()
{
    auto profiles=successes(),failure_controls=failures(),argument_controls=arguments();reset();
    std::printf("PASS: %u allocation profiles, %u allocation/cache/lifetime failure controls, %u argument/CPU refusal controls\n",profiles,failure_controls,argument_controls);
    std::puts("Host IOKit doubles only; no real kernel allocation, physical provenance, cache/DMA, SMMU, GPU or fence qualification");
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
