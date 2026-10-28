/* SPDX-License-Identifier: MIT */
/* Userspace client and kernel dispatch over software SMMU/CP peers. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoClient.h"
#include "NativeAdrenoUserClient.h"
#include <cstdio>
#include <cstring>
#include <vector>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
#define SMMU_BYTES 0x40000
#define CB 0x22000
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{CHECK(out);*out={};return kIOReturnSuccess;}
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));
    CHECK(op==NativeAdrenoClean || op==NativeAdrenoCleanInvalidate || op==NativeAdrenoInvalidate);
    ++fixture.cache_calls;
    return fixture.cache_calls==fixture.cache_fail_call?kIOReturnIOError:kIOReturnSuccess;
}
static uint32_t mmio[SMMU_BYTES/4];
static struct {unsigned calls,fail_call,after;} mf;
static ArmSmmu4KIo smmuio;
static ArmSmmu4KStreamState streams;
static ArmSmmu4KStream routes[2]={{2,0},{4,1}};
static ArmSmmu4KStreamResult route_result;
static ArmSmmu4KContextConfig context;
static ArmSmmu4KContextImage context_image;
static int mevent(void){return ++mf.calls==mf.fail_call;}
static int mread32(void *c,size_t at,uint32_t *v)
{CHECK(c==&mf && at%4==0 && at<=SMMU_BYTES-4);if(mevent())return 1;*v=mmio[at/4];return 0;}
static int mread64(void *c,size_t at,uint64_t *v)
{CHECK(c==&mf && at%8==0 && at<=SMMU_BYTES-8);if(mevent())return 1;std::memcpy(v,mmio+at/4,8);return 0;}
static int mwrite32(void *c,size_t at,uint32_t v)
{CHECK(c==&mf && at%4==0 && at<=SMMU_BYTES-4);int f=mevent();if(f && !mf.after)return 1;mmio[at/4]=v;return f;}
static int mwrite64(void *c,size_t at,uint64_t v)
{CHECK(c==&mf && at%8==0 && at<=SMMU_BYTES-8);int f=mevent();if(f && !mf.after)return 1;std::memcpy(mmio+at/4,&v,8);return f;}
static int mbarrier(void *c){CHECK(c==&mf);return mevent();}
struct CpPeer {
    std::vector<uint32_t> regs,writes;
    unsigned publish=0,acquire=0,barriers=0,fail_publish=0;
    CpPeer():regs(0x10000,0){}
};
static CpPeer cp;
static uint64_t clock_us=0;
static int cwrite(void *c,uint32_t reg,uint32_t v)
{CHECK(c==&cp && reg<cp.regs.size());cp.regs[reg]=v;cp.writes.push_back(reg);return 0;}
static int cread(void *c,uint32_t reg,uint32_t *v)
{CHECK(c==&cp && v && reg<cp.regs.size());*v=cp.regs[reg];return 0;}
static int cbarrier(void *c){CHECK(c==&cp);++cp.barriers;return 0;}
static int cpublish(void *c,const volatile uint32_t *w,size_t bytes)
{CHECK(c==&cp && w && bytes==32768);++cp.publish;return cp.publish==cp.fail_publish;}
static int cacquire(void *c,volatile uint32_t *w,size_t bytes)
{CHECK(c==&cp && w && bytes==4096);++cp.acquire;return 0;}
static uint64_t cnow(void *c){CHECK(c==&cp);return clock_us;}
static int cidle(void *c){CHECK(c==&cp);clock_us+=1000;return 0;}
struct Path {
    NativeAdrenoTablePages tables;
    ArmSmmu4KLiveArena arena;
    ArmSmmu4KLiveState state{};
    ArmSmmu4KLiveVisibility visibility;
    NativeAdrenoVm vm;
    NativeAdrenoRing ring;
    NativeAdrenoSubmit execbuf;
    NativeAdrenoUserClient client;
};
static uint32_t ringmem[8192],fencemem[1024];
static NativeAdrenoRingIo ringio{&cp,cwrite,cread,cbarrier,nullptr,nullptr};
static NativeAdrenoRingCache ringcache{&cp,cpublish,cacquire};
static NativeAdrenoRingClock ringclock{&cp,cnow,cidle};
static int timeread(void *c,uint32_t address,uint32_t *value)
{CHECK(c==&cp && value && address<cp.regs.size());*value=cp.regs[address];return 0;}
static NativeAdrenoTimeIo timeio{&cp,timeread};
static void smmuReset(void)
{
    std::memset(mmio,0,sizeof(mmio));mf={};streams=(ArmSmmu4KStreamState){};
    smmuio=(ArmSmmu4KIo){&mf,SMMU_BYTES,mread32,mread64,mwrite32,mwrite64,mbarrier};
    mmio[0x20/4]=(1u<<30)|(1u<<27)|(4u<<9)|8;
    mmio[0x24/4]=0x40000010;mmio[0x28/4]=0x1433;mmio[0]=0x436;
    for(unsigned i=0;i<8;++i)mmio[(0xc00+4*i)/4]=0x20000;
    mmio[0x800/4]=1u<<31|9;mmio[0xc00/4]=3;
}
static int exposed(Path &p)
{
    smmuReset();cp=CpPeer{};clock_us=0;
    std::memset(ringmem,0xef,sizeof(ringmem));std::memset(fencemem,0,sizeof(fencemem));
    if(NativeAdrenoPrepareSmmuMemory(&p.tables,32,40,0x800000,65536,0,16,&p.arena,&p.state,&p.visibility))
        return kIOReturnBadArgument;
    context=(ArmSmmu4KContextConfig){.lower=&p.arena.config,.lower_tables=&p.state.tables,
        .id0=mmio[0x20/4],.id2=mmio[0x28/4],.asid=1,.coherent_walk=1,
        .stall_on_fault=0,.continue_after_fault=1};
    ArmSmmu4KContextConfig actual=context;
    CHECK(ArmSmmu4KEncodeContext(&actual,&context_image)==ArmSmmu4KSuccess);
    mmio[0x1008/4]=context_image.cbar;mmio[0x1808/4]=context_image.cba2r;
    mmio[CB/4]=context_image.sctlr;mmio[(CB+0x10)/4]=context_image.tcr2;mmio[(CB+0x30)/4]=context_image.tcr;
    std::memcpy(mmio+(CB+0x20)/4,&context_image.ttbr0,8);
    std::memcpy(mmio+(CB+0x28)/4,&context_image.ttbr1,8);
    mmio[(CB+0x38)/4]=context_image.mair0;
    if(ArmSmmu4KAttachStreams(&smmuio,&streams,routes,2,2,&context,&route_result))return kIOReturnBadArgument;
    NativeAdrenoVmRegion lower={&p.tables,&p.arena,&p.state};
    if(p.vm.attach(&smmuio,&context,&streams,&lower,nullptr,32,1000))return kIOReturnBadArgument;
    if(p.ring.attach(&ringio,&ringcache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000))
        return kIOReturnBadArgument;
    if(p.execbuf.attach(&p.vm,&p.ring))return kIOReturnBadArgument;
    return p.client.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,32,40);
}

/* The transport is exactly the kernel dispatch entry, so a successful
 * round-trip proves both sides agree on every wire record. */
struct Wire {
    NativeAdrenoUserClient *core;
    const NativeAdrenoRingClock *clock;
    unsigned calls=0;
    unsigned corrupt_input=0,corrupt_version=0;
};
static int call(void *cookie,uint32_t method,const void *input,size_t input_size,
    void *output,size_t output_size)
{
    Wire *w=static_cast<Wire *>(cookie);CHECK(w && w->core);++w->calls;
    size_t size=input_size;
    if(w->corrupt_input && size)size-=1;
    const IOReturn status=w->core->dispatch(method,input,size,output,output_size,w->clock);
    if(!status && w->corrupt_version && method==NativeAdrenoMethodQuery &&
        output && output_size==sizeof(NativeAdrenoAbiParams))
        static_cast<NativeAdrenoAbiParams *>(output)->abi_version=NativeAdrenoAbiVersion+1;
    return status;
}
static NativeAdrenoTransport transport(Wire *w){return NativeAdrenoTransport{w,call};}
int main()
{
    /* Full userspace round-trip through the kernel dispatch core. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoStatus counters;CHECK(p.client.attachStatus(&counters)==kIOReturnSuccess);
        NativeAdrenoTimer timer;CHECK(timer.attach(&timeio,NativeAdrenoTimeCp)==kIOReturnSuccess);
        CHECK(p.client.attachTimer(&timer)==kIOReturnSuccess);
        Wire wire{&p.client,&ringclock};
        NativeAdrenoTransport t=transport(&wire);
        NativeAdrenoClient client;
        CHECK(client.open(&t)==kIOReturnSuccess && client.isOpen());
        CHECK(client.open(&t)==kIOReturnBusy);
        NativeAdrenoAbiParams info{};
        CHECK(client.query(&info)==kIOReturnSuccess && info.input_bits==32 && info.output_bits==40);
        uint64_t handle=0,mapped=0;
        CHECK(client.allocBo(8192,12,true,&handle)==kIOReturnSuccess && handle);
        uint32_t token=0;
        uint64_t mapbytes=0;
        CHECK(client.cpuMap(handle,&token,&mapbytes)==kIOReturnSuccess && token && mapbytes==8192);
        CHECK(client.freeBo(handle)==kIOReturnBusy); /* user mapping still leased */
        CHECK(client.cpuUnmap(handle,token)==kIOReturnSuccess);
        CHECK(client.cpuUnmap(handle,token)==kIOReturnBusy);
        CHECK(client.mapBo(handle,0x800000,3,0,2,&mapped)==kIOReturnSuccess && mapped==0x800000);
        const char name[]="wire";
        CHECK(client.setMetadata(handle,name,4)==kIOReturnSuccess);
        char read[8]{};
        uint32_t length=0;
        CHECK(client.getMetadata(handle,read,8,&length)==kIOReturnSuccess);
        CHECK(length==4 && !std::memcmp(read,"wire",4));
        NativeAdrenoAbiSubmitBuffer buffer{handle,0x800000,1024};
        uint64_t fence=0;
        CHECK(client.submit(&buffer,1,0,0,&fence)==kIOReturnSuccess && fence==1);
        fencemem[1]=1;
        CHECK(client.waitFence(fence,0)==kIOReturnSuccess);
        CHECK(client.submit(&buffer,1,fence,0,&fence)==kIOReturnSuccess && fence==2);
        CHECK(client.waitFence(fence,0)==kIOReturnTimeout);
        fencemem[1]=2;
        CHECK(client.waitFence(fence,UINT64_C(1000000))==kIOReturnSuccess);
        cp.regs[0x0980]=0x42;cp.regs[0x0981]=7;
        uint64_t ticks=0;
        CHECK(client.timestamp(&ticks)==kIOReturnSuccess && ticks==((UINT64_C(7)<<32)|0x42));
        counters.noteFault();
        uint64_t faults=0,suspends=0;
        CHECK(client.status(&faults,&suspends)==kIOReturnSuccess && faults==1 && !suspends);
        CHECK(client.waitFence(fence+5,0)==kIOReturnBadArgument);
        CHECK(client.getMetadata(handle,read,2,&length)==kIOReturnBadArgument);
        uint64_t queue=0;
        CHECK(client.createQueue(&queue)==kIOReturnSuccess && queue==1);
        NativeAdrenoAbiSubmitBuffer queued{handle,0x800000,256};
        uint64_t qfence=0;
        CHECK(client.submit(&queued,1,0,0,&qfence,queue)==kIOReturnSuccess);
        CHECK(client.closeQueue(queue)==kIOReturnBusy);
        fencemem[1]=qfence;
        CHECK(client.waitFence(qfence,0)==kIOReturnSuccess);
        CHECK(client.closeQueue(queue)==kIOReturnSuccess);
        CHECK(client.submit(&queued,1,0,0,&qfence,queue)==kIOReturnBadArgument);
        CHECK(client.freeBo(handle)==kIOReturnSuccess);
        CHECK(client.freeBo(handle)==kIOReturnBadArgument);
        CHECK(client.close()==kIOReturnSuccess && !client.isOpen());
        CHECK(client.close()==kIOReturnNotReady);
        CHECK(p.execbuf.outstanding()==0);
    }
    /* A transport that perturbs the version is refused at open. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        Wire wire{&p.client,&ringclock};wire.corrupt_version=1;
        NativeAdrenoTransport t=transport(&wire);
        NativeAdrenoClient client;
        CHECK(client.open(&t)==kIOReturnUnsupported && !client.isOpen());
        CHECK(client.query(nullptr)==kIOReturnBadArgument);
        CHECK(client.close()==kIOReturnNotReady);
    }
    /* Altered record sizes reach the kernel and are refused without effects. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        Wire wire{&p.client,&ringclock};wire.corrupt_input=1;
        NativeAdrenoTransport t=transport(&wire);
        NativeAdrenoClient client;
        CHECK(client.open(&t)==kIOReturnSuccess);
        uint64_t handle=0;
        CHECK(client.allocBo(4096,12,false,&handle)==kIOReturnBadArgument && !handle);
        unsigned calls=wire.calls;
        CHECK(client.mapBo(1,0x800000,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(wire.calls==calls+1);
        CHECK(client.submit(nullptr,0,0,0,&handle)==kIOReturnBadArgument);
        CHECK(client.setMetadata(0,nullptr,0)==kIOReturnBadArgument);
        CHECK(client.close()==kIOReturnSuccess);
    }
    /* Client-side refusals never call the transport. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        Wire wire{&p.client,&ringclock};
        NativeAdrenoTransport t=transport(&wire);
        NativeAdrenoClient client;
        uint64_t handle=0;
        /* Before open every method reports not-ready, not a transport error. */
        CHECK(client.allocBo(4096,12,false,&handle)==kIOReturnNotReady);
        CHECK(client.submit(nullptr,1,0,0,&handle)==kIOReturnNotReady);
        CHECK(client.freeBo(0)==kIOReturnNotReady);
        CHECK(client.setMetadata(1,nullptr,65)==kIOReturnNotReady);
        CHECK(client.open(nullptr)==kIOReturnBadArgument);
        NativeAdrenoTransport broken{&wire,nullptr};
        CHECK(client.open(&broken)==kIOReturnBadArgument);
        CHECK(wire.calls==0);
        CHECK(client.open(&t)==kIOReturnSuccess && client.isOpen());
        const unsigned calls=wire.calls;
        /* Argument refusals are decided here and never reach the kernel. */
        CHECK(client.allocBo(4096,12,false,nullptr)==kIOReturnBadArgument);
        CHECK(client.allocBo(4096,11,false,&handle)==kIOReturnBadArgument);
        CHECK(client.allocBo((UINT64_C(1)<<32)+4096,12,false,&handle)==kIOReturnBadArgument);   /* past the GPU address window */
        CHECK(client.freeBo(0)==kIOReturnBadArgument);
        CHECK(client.setMetadata(1,nullptr,65)==kIOReturnBadArgument);
        uint32_t length=0;
        CHECK(client.getMetadata(1,nullptr,65,&length)==kIOReturnBadArgument);
        NativeAdrenoAbiSubmitBuffer many[NativeAdrenoAbiMaxBuffers+1]{};
        CHECK(client.submit(many,NativeAdrenoAbiMaxBuffers+1,0,0,&handle)==kIOReturnBadArgument);
        CHECK(client.mapBo(0,0x800000,3,0,2,&handle)==kIOReturnBadArgument);
        CHECK(wire.calls==calls);
        CHECK(client.close()==kIOReturnSuccess);
    }
    std::printf("PASS: 1 userspace-to-kernel round trip including CPU mapping leases and queue lifecycle, 1 version mismatch, 1 altered-size rejection and 8 client-side refusals over software SMMU/CP peers\n");
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
