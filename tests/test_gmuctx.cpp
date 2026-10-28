/* SPDX-License-Identifier: MIT */
/* The whole GMU context sequence from an unprogrammed SMMU over a software register peer: the real
 * allocator, table pool, context-bank programming, stream attach, VM and payload mapping. ID0/ID1 are
 * the values read on sheng (int-005); ID2 was not read there, so the peer's value is an assumption. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoGmuContext.h"
#include <cstdio>
#include <cstring>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *state)
{CHECK(state);*state={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{CHECK(mapping && bytes && op<=NativeAdrenoInvalidate);return kIOReturnSuccess;}
#define SMMU_BYTES 0x40000
alignas(8) static uint32_t mmio[SMMU_BYTES/4];
static struct {unsigned ignore_writes_from;size_t frozen;unsigned tbi;} peer;
static void reset()
{
    std::memset(mmio,0,sizeof(mmio));peer={};
    mmio[0]=0x002f0406;mmio[0x20/4]=0x4c017e18;mmio[0x24/4]=0x40000016;mmio[0x28/4]=0x1433;
    for(unsigned i=0;i<24;++i)mmio[(0xc00+4*i)/4]=0x20000;   /* every S2CR FAULT, every SMR invalid */
}
static int read32(void *,size_t at,uint32_t *v){CHECK(at%4==0 && at<=SMMU_BYTES-4);*v=mmio[at/4];return 0;}
static int read64(void *,size_t at,uint64_t *v){CHECK(at%8==0 && at<=SMMU_BYTES-8);std::memcpy(v,mmio+at/4,8);return 0;}
static int write32(void *,size_t at,uint32_t v)
{
    CHECK(at%4==0 && at<=SMMU_BYTES-4);
    if(peer.frozen && at==peer.frozen)return 0;   /* accepted on the bus, not stored */
    if((at&0xfff)==0x58 && at>=0x20000) {mmio[at/4]&=~v;return 0;}   /* context FSR is write-one-to-clear */
    if(peer.tbi && (at&0xfff)==0x10 && at>=0x20000)v|=0x60;   /* int-008: TCR2 reads TBI0/TBI1 as 1 */
    if(peer.tbi && (at&0xfff)==0 && at>=0x20000)v&=~0x1000u;   /* int-009: SCTLR.ASIDPNE reads 0 */
    mmio[at/4]=v;return 0;
}
static int write64(void *,size_t at,uint64_t v)
{CHECK(at%8==0 && at<=SMMU_BYTES-8);if(peer.frozen && at==peer.frozen)return 0;std::memcpy(mmio+at/4,&v,8);return 0;}
static int barrier(void *){return 0;}
static unsigned steps[16],step_count;
static void progress(uint32_t step){CHECK(step_count<16);steps[step_count++]=step;}
int main()
{
    const ArmSmmu4KIo io{&peer,SMMU_BYTES,read32,read64,write32,write64,barrier};
    const size_t cb=0x20000+1*0x1000;   /* NUMPAGE 32 x 4 KiB global space, bank 1 */
    {
        reset();step_count=0;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,progress,&report)==kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Ready && report.status==kIOReturnSuccess && context.ready());
        CHECK(step_count==7);
        for(unsigned i=0;i<7;++i)CHECK(steps[i]==i+1);
        CHECK(report.program_enabled && report.program_readback_valid && report.routes_verified==1);
        CHECK(report.mismatch==0);
        CHECK(report.id1==0x40000016 && report.spans==5);
        CHECK(report.id2==0x1433 && report.output_bits==40 && report.program_status==0 && report.route_status==0);
        /* Route: S2CR translate to bank 1, then exact SMR for SID 5. */
        CHECK(report.s2cr==mmio[(0xc00+8)/4] && (report.s2cr&0xff)==1 && ((report.s2cr>>16)&3)==0);
        CHECK(report.smr==((1u<<31)|5));
        CHECK(report.cbar==mmio[(0x1000+4)/4]);
        CHECK(report.sctlr==mmio[cb/4] && (report.sctlr&1));
        /* TTBR0 holds the root table page and ASID 1; TTBR1 is unused. */
        CHECK(report.ttbr0==((uint64_t(1)<<48)|context.allocator().tables().pagePhysical(0)));
        uint64_t ttbr1;std::memcpy(&ttbr1,mmio+(cb+0x28)/4,8);
        CHECK((ttbr1&0x0000fffffffff000)==0);
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)==kIOReturnBusy);   /* one attempt per object */
    }
    /* A S2CR that reads back other than FAULT (the apps SMMU stored BYPASS writes as FAULT; here the reverse
     * case) is refused at attach, after the bank was programmed; the SMR stays invalid. */
    {
        reset();
        mmio[(0xc00+8)/4]=0x10000;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)!=kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Attach && report.program_enabled && !report.routes_verified);
        CHECK(!(mmio[(0x800+8)/4]&(1u<<31)) && !context.ready());
    }
    /* A context-bank write the peer ignores (a hypervisor that drops it) is caught by the program readback. */
    {
        reset();peer.frozen=cb+0x30;   /* TCR */
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)!=kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Program && !report.program_enabled);
        CHECK(report.mismatch==(1u<<2) && report.image.tcr==0);   /* only TCR differs; the banner names it */
        CHECK(!(mmio[cb/4]&1) && !(mmio[(0x800+8)/4]&(1u<<31)));
    }
    /* An already enabled bank is refused before any write. */
    {
        reset();mmio[cb/4]=1;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)!=kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Program && report.program_writes==0);
    }
    /* A 36-bit OAS gives a 36-bit context (TCR2.PASize 1) and allocations below 2^36. */
    {
        reset();mmio[0x28/4]=0x1413;   /* OAS code 1 */
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)==kIOReturnSuccess && report.output_bits==36);
        CHECK((mmio[(cb+0x10)/4]&7)==1);
        for(size_t i=0;i<context.allocator().tables().pageCount();++i)
            CHECK(context.allocator().tables().pagePhysical(i)+0x1000<=(UINT64_C(1)<<36));
    }
    /* An OAS code the kit cannot encode is refused before any allocation or write. */
    {
        reset();mmio[0x28/4]=0x14f3;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)==kIOReturnUnsupported);
        CHECK(report.step==NativeAdrenoGmuContext::Allocate && !context.allocator().ready() && !(mmio[cb/4]&1));
    }
    /* No AArch64 4 KiB granule (ID2.PTFS_4K clear): the kit refuses in phase 1 with no write, the case int-006
     * may have hit; program_status names it. */
    {
        reset();mmio[0x28/4]=0x0433;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)!=kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Program && report.program_phase==1 && report.program_writes==0);
        CHECK(report.program_status==ArmSmmu4KUnsupported);
    }
    /* The int-007..009 hardware as observed: ID2 0x5511 (OAS 36), TCR2 reading 0x38071 for 0x38011 and SCTLR
     * reading 0x167 for 0x1167. */
    {
        reset();mmio[0x28/4]=0x5511;peer.tbi=1;
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,&report)==kIOReturnSuccess && report.step==NativeAdrenoGmuContext::Ready);
        CHECK(report.output_bits==36 && mmio[(cb+0x10)/4]==0x38071 && report.mismatch==((1u<<3)|(1u<<8)));
        CHECK(mmio[cb/4]==0x167);
        CHECK(report.smr==((1u<<31)|5) && (report.s2cr&0xff)==1 && (report.sctlr&1));
    }
    /* Argument refusals touch nothing. */
    {
        reset();
        NativeAdrenoGmuContext context;
        NativeAdrenoGmuContextReport report;
        CHECK(context.bringUp(nullptr,1,1,2,5,nullptr,&report)==kIOReturnBadArgument);
        CHECK(context.bringUp(&io,64,1,2,5,nullptr,&report)==kIOReturnBadArgument);
        CHECK(context.bringUp(&io,1,0,2,5,nullptr,&report)==kIOReturnBadArgument);
        CHECK(context.bringUp(&io,1,1,128,5,nullptr,&report)==kIOReturnBadArgument);
        CHECK(context.bringUp(&io,1,1,2,5,nullptr,nullptr)==kIOReturnBadArgument);
        CHECK(report.step==0);
    }
    /* The board entry over the kit's direct MMIO adapter on a plain array. A plain array has no
     * write-one-to-clear FSR, so the cleared fault status reads back set: the enable is refused as a
     * fault and the bank left disabled. Then its single-use guard. */
    {
        reset();
        NativeAdrenoGmuContextReport report;
        CHECK(NativeAdrenoGmuContextBringup(reinterpret_cast<volatile uint8_t *>(mmio),SMMU_BYTES,1,1,2,5,
            nullptr,&report)!=kIOReturnSuccess);
        CHECK(report.step==NativeAdrenoGmuContext::Program && !report.program_enabled && !(mmio[cb/4]&1));
        CHECK(NativeAdrenoGmuContextBringup(reinterpret_cast<volatile uint8_t *>(mmio),SMMU_BYTES,1,1,2,5,
            nullptr,&report)==kIOReturnBadArgument);
        NativeAdrenoGmuPayload payloads[5];   /* that context never reached Ready */
        CHECK(NativeAdrenoGmuContextPayloads(payloads)==kIOReturnNotReady && !payloads[4].bytes);
        CHECK(NativeAdrenoGmuContextPayloads(nullptr)==kIOReturnBadArgument);
    }
    std::printf("PASS: 1 complete GMU context bring-up with exact routes and TTBR0, 1 non-FAULT S2CR, 1 ignored "
        "context write, 1 enabled-bank refusal, 1 36-bit OAS context, 1 sheng-observed ID2/TCR2 context, 2 ID2 refusals, 5 argument refusals and 1 board entry with a stuck fault status over a software SMMU peer\n");
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
