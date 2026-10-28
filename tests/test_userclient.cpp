/* SPDX-License-Identifier: MIT */
/* Real user-client body over software SMMU/CP register peers. No IOKit. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoUserClient.h"
#include <cstdio>
#include <cstring>
#include <vector>
extern "C" volatile uint32_t native_adreno_io_coherent;
extern "C" volatile uint32_t native_adreno_coherent_skips[4];
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
int main()
{
    /* Full client transaction: query, allocate, map, submit, wait, free, close. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiParams params{};
        CHECK(p.client.query(&params)==kIOReturnSuccess && params.abi_version==NativeAdrenoAbiVersion);
        CHECK(params.input_bits==32 && params.output_bits==40);
        CHECK(params.max_bo_bytes==(UINT64_C(1)<<32));   /* the GPU address window: the whole 32-bit input range here */
        NativeAdrenoAbiTimestamp no_timer{};
        CHECK(p.client.timestamp(&no_timer)==kIOReturnUnsupported);
        NativeAdrenoAbiStatus no_status{};
        CHECK(p.client.status(&no_status)==kIOReturnUnsupported);
        CHECK(p.client.close()==kIOReturnSuccess); /* empty client closes */
        CHECK(p.client.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,32,40)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{8192,12,1};
        NativeAdrenoAbiBoAllocReply allocated{};
        NativeAdrenoTimer timer;
        CHECK(timer.attach(&timeio,NativeAdrenoTimeCp)==kIOReturnSuccess);
        cp.regs[0x0980]=0x12345678u;cp.regs[0x0981]=0x00000009u;
        CHECK(p.client.attachTimer(&timer)==kIOReturnSuccess);
        NativeAdrenoAbiTimestamp ticks{};
        CHECK(p.client.timestamp(&ticks)==kIOReturnSuccess && ticks.ticks==UINT64_C(0x0000000912345678));
        CHECK(p.client.attachTimer(&timer)==kIOReturnBadArgument);
        NativeAdrenoStatus counters;
        CHECK(p.client.attachStatus(&counters)==kIOReturnSuccess);
        counters.noteFault();counters.noteFault();counters.noteSuspend();
        NativeAdrenoAbiStatus queried{};
        CHECK(p.client.status(&queried)==kIOReturnSuccess && queried.faults==2 && queried.suspends==1);
        counters.noteSuspend();
        CHECK(p.client.status(&queried)==kIOReturnSuccess && queried.faults==2 && queried.suspends==2);
        CHECK(p.client.attachStatus(&counters)==kIOReturnBadArgument);
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess && allocated.handle);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess && mapped.iova==0x800000);
        CHECK(p.client.mapped(allocated.handle) && p.client.liveObjects()==1);
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count=1;
        submit.buffers[0]={allocated.handle,0x800000,1024};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess && reply.fence==1);
        /* The client's first submission carries the 26-word context entry (ending in the SRAM-init wait) before the IB. */
        CHECK(cp.regs[0x0807]==72 && (ringmem[37]>>16&0x7fu)==0x3f && (ringmem[9]>>16&0x7fu)==0x1f && (ringmem[21]>>16&0x7fu)==0x3c);
        CHECK(ringmem[38]==0x800000 && ringmem[39]==0 && ringmem[40]==256);
        CHECK(p.execbuf.stats().references_held==1);
        NativeAdrenoAbiBoFree released{allocated.handle};
        CHECK(p.client.freeBo(&released)==kIOReturnBusy); /* job outstanding */
        NativeAdrenoAbiWaitFence wait{1,0};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnTimeout);
        CHECK(p.execbuf.outstanding()==1);
        fencemem[1]=1;
        wait.timeout_ns=UINT64_C(1000000);
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(p.execbuf.outstanding()==0);
        NativeAdrenoAbiMetadata meta{};
        meta.handle=allocated.handle;meta.bytes=8;
        std::memcpy(meta.text,"ring-buf",8);
        CHECK(p.client.setMetadata(&meta)==kIOReturnSuccess);
        NativeAdrenoAbiMetadata got{};
        got.handle=allocated.handle;got.bytes=16;
        CHECK(p.client.getMetadata(&got,&got)==kIOReturnSuccess);
        CHECK(got.bytes==8 && !std::memcmp(got.text,"ring-buf",8));
        got.bytes=4;CHECK(p.client.getMetadata(&got,&got)==kIOReturnBadArgument);
        meta.bytes=65;CHECK(p.client.setMetadata(&meta)==kIOReturnBadArgument);
        meta.bytes=8;meta.handle=allocated.handle+9;
        CHECK(p.client.setMetadata(&meta)==kIOReturnBadArgument);
        NativeAdrenoAbiSubmit second{};
        second.buffer_count=1;second.wait_fence=1;
        second.buffers[0]={allocated.handle,0x800000,1024};
        CHECK(p.client.submit(&second,&reply,&ringclock)==kIOReturnSuccess && reply.fence==2);
        const uint32_t wait_wptr=cp.regs[0x0807];
        NativeAdrenoAbiSubmit blocked{};
        blocked.buffer_count=1;blocked.wait_fence=2;
        blocked.buffers[0]={allocated.handle,0x800000,1024};
        CHECK(p.client.submit(&blocked,&reply,&ringclock)==kIOReturnTimeout);
        CHECK(cp.regs[0x0807]==wait_wptr && p.execbuf.outstanding()==1);
        fencemem[1]=2;
        wait.fence=2;wait.timeout_ns=UINT64_C(1000000);
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(p.client.freeBo(&released)==kIOReturnSuccess);
        CHECK(p.client.liveObjects()==0 && !p.client.mapped(allocated.handle));
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* Host import: a caller's pages become a BO with scattered spans, map into the GPU, refuse a CPU-map token and the task-less
     * dispatch path, and unpin on free. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiParams params{};
        CHECK(p.client.query(&params)==kIOReturnSuccess && (params.flags&NativeAdrenoParamsHostImport));
        void *task=reinterpret_cast<void *>(0x77);
        NativeAdrenoAbiBoAllocReply imported{};
        NativeAdrenoAbiBoImport bad{0x10000001,8192};
        CHECK(p.client.importBo(&bad,task,&imported)==kIOReturnBadArgument && !imported.handle);
        bad={0x10000000,0};CHECK(p.client.importBo(&bad,task,&imported)==kIOReturnBadArgument);
        NativeAdrenoAbiBoImport request{0x10000000,4*4096};
        CHECK(p.client.importBo(&request,nullptr,&imported)==kIOReturnBadArgument);
        const unsigned before=fixture.imports_live;
        CHECK(p.client.importBo(&request,task,&imported)==kIOReturnSuccess && imported.handle);
        CHECK(fixture.imports_live==before+1);
        NativeAdrenoMemory *memory=p.client.memoryOf(imported.handle);
        CHECK(memory && memory->imported() && memory->spanCount()==4 && !memory->mapping());
        NativeAdrenoAbiBoCpuMap cpu{imported.handle};
        NativeAdrenoAbiBoCpuMapReply token{};
        CHECK(p.client.cpuMap(&cpu,&token)==kIOReturnUnsupported && !token.token);
        NativeAdrenoAbiBoAllocReply untouched{};
        CHECK(p.client.dispatch(NativeAdrenoMethodBoImport,&request,sizeof(request),&untouched,sizeof(untouched),&ringclock)==kIOReturnUnsupported);
        CHECK(!untouched.handle);
        NativeAdrenoAbiBoMap map{imported.handle,0x808000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        const unsigned physical=fixture.physical_calls;
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess && mapped.iova==0x808000);
        CHECK(fixture.physical_calls==physical+4);   /* published at map, one call per scattered span */
        /* A submission maintains the imported pages before it starts and again when its job is released after completion. */
        NativeAdrenoAbiBoAlloc ib_alloc{4096,12,1};NativeAdrenoAbiBoAllocReply ib{};
        CHECK(p.client.allocBo(&ib_alloc,&ib)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap ib_map{ib.handle,0x800000,3,0,2,0};NativeAdrenoAbiBoMapReply ib_mapped{};
        CHECK(p.client.mapBo(&ib_map,&ib_mapped)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit submit{};submit.buffer_count=1;submit.buffers[0]={ib.handle,0x800000,256};
        NativeAdrenoAbiSubmitReply reply{};
        const size_t bytes=fixture.physical_bytes;
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess);
        CHECK(fixture.physical_bytes==bytes+4*4096);
        fencemem[1]=reply.fence;
        NativeAdrenoAbiWaitFence wait{reply.fence,UINT64_C(1000000)};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(fixture.physical_bytes==bytes+2*4*4096);
        NativeAdrenoAbiBoFree ib_free{ib.handle};
        CHECK(p.client.freeBo(&ib_free)==kIOReturnSuccess);
        NativeAdrenoAbiBoFree released{imported.handle};
        CHECK(p.client.freeBo(&released)==kIOReturnSuccess);
        CHECK(fixture.imports_live==before && p.client.liveObjects()==0);
        /* Memory the owner maps read-only cannot be pinned for writing (KERN_PROTECTION_FAILURE): it is pinned for reading, reported read-only,
         * and the GPU mapping the client asks for read/write comes out read-only. */
        fixture.write_protected_imports=true;
        const unsigned prepared=fixture.prepared;
        NativeAdrenoAbiBoAllocReply ro{};
        NativeAdrenoAbiBoImport ro_request{0x20000000,2*4096};
        CHECK(p.client.importBo(&ro_request,task,&ro)==kIOReturnSuccess && ro.handle);
        CHECK(fixture.prepared==prepared+2);   /* the writable attempt, then the read-only one */
        NativeAdrenoMemory *ro_memory=p.client.memoryOf(ro.handle);
        CHECK(ro_memory && ro_memory->readOnly() && ro_memory->imported());
        NativeAdrenoAbiBoMap ro_map{ro.handle,0x80a000,3,0,2,0};NativeAdrenoAbiBoMapReply ro_mapped{};
        CHECK(p.client.mapBo(&ro_map,&ro_mapped)==kIOReturnSuccess && ro_mapped.iova==0x80a000);
        NativeAdrenoAbiBoFree ro_free{ro.handle};
        CHECK(p.client.freeBo(&ro_free)==kIOReturnSuccess);
        fixture.write_protected_imports=false;
        /* Any other prepare failure is still a refusal. */
        fixture.prepare_fail=true;
        CHECK(p.client.importBo(&ro_request,task,&ro)==kIOReturnIOError);
        fixture.prepare_fail=false;
        CHECK(fixture.imports_live==before && p.client.liveObjects()==0);
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* With IO coherence proved, an import the GPU only reads maps write-back inner-shareable whatever the client asked (a GPU-written one
     * stays non-coherent: int-288..291), and neither its map, its submission, its completion nor its free maintains its pages; the GPU's
     * own BOs keep the attributes asked for. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        native_adreno_io_coherent=1;
        const uint32_t skips[4]={native_adreno_coherent_skips[0],native_adreno_coherent_skips[1],native_adreno_coherent_skips[2],
                                 native_adreno_coherent_skips[3]};
        void *task=reinterpret_cast<void *>(0x77);
        NativeAdrenoAbiBoImport request{0x10000000,4*4096};NativeAdrenoAbiBoAllocReply imported{};
        CHECK(p.client.importBo(&request,task,&imported)==kIOReturnSuccess && imported.handle);
        const unsigned physical=fixture.physical_calls;const size_t bytes=fixture.physical_bytes;
        NativeAdrenoAbiBoMap map{imported.handle,0x808000,1,0,2,0};NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess && mapped.iova==0x808000);
        CHECK(fixture.physical_calls==physical && native_adreno_coherent_skips[0]==skips[0]+1);
        NativeAdrenoAbiBoAlloc ib_alloc{4096,12,1};NativeAdrenoAbiBoAllocReply ib{};
        CHECK(p.client.allocBo(&ib_alloc,&ib)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap ib_map{ib.handle,0x800000,3,0,2,0};NativeAdrenoAbiBoMapReply ib_mapped{};
        CHECK(p.client.mapBo(&ib_map,&ib_mapped)==kIOReturnSuccess);
        CHECK(native_adreno_coherent_skips[0]==skips[0]+1);   /* the GPU's own BO is not coherent */
        NativeAdrenoAbiSubmit submit{};submit.buffer_count=1;submit.buffers[0]={ib.handle,0x800000,256};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess);
        CHECK(fixture.physical_bytes==bytes && native_adreno_coherent_skips[2]>skips[2]);
        fencemem[1]=reply.fence;
        NativeAdrenoAbiWaitFence wait{reply.fence,UINT64_C(1000000)};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(fixture.physical_bytes==bytes);   /* completion maintains only GPU-written imports */
        NativeAdrenoAbiBoFree ib_free{ib.handle};CHECK(p.client.freeBo(&ib_free)==kIOReturnSuccess);
        NativeAdrenoAbiBoFree released{imported.handle};CHECK(p.client.freeBo(&released)==kIOReturnSuccess);
        CHECK(fixture.physical_bytes==bytes && native_adreno_coherent_skips[1]==skips[1]+1 && p.client.liveObjects()==0);
        native_adreno_io_coherent=0;
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* The external-method entry enforces exact record sizes and copies only
     * on success. This is the entire surface the IOKit wrapper may expose. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiParams params{};
        CHECK(p.client.dispatch(NativeAdrenoMethodQuery,nullptr,0,&params,sizeof(params),&ringclock)==kIOReturnSuccess);
        CHECK(params.abi_version==NativeAdrenoAbiVersion);
        unsigned char untouched[sizeof(NativeAdrenoAbiParams)];
        std::memset(untouched,0x5a,sizeof(untouched));
        CHECK(p.client.dispatch(NativeAdrenoMethodQuery,nullptr,1,untouched,sizeof(untouched),&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.dispatch(NativeAdrenoMethodQuery,nullptr,0,untouched,sizeof(untouched)-1,&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.dispatch(NativeAdrenoMethodQuery,nullptr,0,nullptr,0,&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.dispatch(99,nullptr,0,nullptr,0,&ringclock)==kIOReturnUnsupported);
        CHECK(untouched[0]==0x5a && untouched[sizeof(untouched)-1]==0x5a);
        NativeAdrenoAbiBoAlloc alloc{8192,12,1};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.dispatch(NativeAdrenoMethodBoAlloc,&alloc,sizeof(alloc),&allocated,sizeof(allocated),&ringclock)==kIOReturnSuccess);
        CHECK(allocated.handle);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.dispatch(NativeAdrenoMethodBoMap,&map,sizeof(map),&mapped,sizeof(mapped),&ringclock)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count=1;submit.buffers[0]={allocated.handle,0x800000,2048};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.dispatch(NativeAdrenoMethodSubmit,&submit,sizeof(submit),&reply,sizeof(reply),&ringclock)==kIOReturnSuccess);
        CHECK(reply.fence==1 && cp.regs[0x0807]==72);
        NativeAdrenoAbiWaitFence wait{reply.fence,0};
        CHECK(p.client.dispatch(NativeAdrenoMethodWaitFence,&wait,sizeof(wait),nullptr,0,&ringclock)==kIOReturnTimeout);
        fencemem[1]=1;
        wait.timeout_ns=UINT64_C(1000000);
        CHECK(p.client.dispatch(NativeAdrenoMethodWaitFence,&wait,sizeof(wait),nullptr,0,&ringclock)==kIOReturnSuccess);
        NativeAdrenoAbiBoFree released{allocated.handle};
        CHECK(p.client.dispatch(NativeAdrenoMethodBoFree,&released,sizeof(released),nullptr,0,&ringclock)==kIOReturnSuccess);
        CHECK(p.client.dispatch(NativeAdrenoMethodBoFree,&released,sizeof(released),nullptr,0,&ringclock)==kIOReturnBadArgument);
        NativeAdrenoTimer timer;CHECK(timer.attach(&timeio,NativeAdrenoTimeCp)==kIOReturnSuccess);
        CHECK(p.client.attachTimer(&timer)==kIOReturnSuccess);
        cp.regs[0x0980]=0x00000011u;cp.regs[0x0981]=0x00000022u;
        NativeAdrenoAbiTimestamp ticks{};
        CHECK(p.client.dispatch(NativeAdrenoMethodGetTimestamp,nullptr,0,&ticks,sizeof(ticks),&ringclock)==kIOReturnSuccess);
        CHECK(ticks.ticks==UINT64_C(0x0000002200000011));
        NativeAdrenoStatus counters;
        CHECK(p.client.attachStatus(&counters)==kIOReturnSuccess);
        counters.noteFault();
        NativeAdrenoAbiStatus status{};
        CHECK(p.client.dispatch(NativeAdrenoMethodGetStatus,nullptr,0,&status,sizeof(status),&ringclock)==kIOReturnSuccess);
        CHECK(status.faults==1 && status.suspends==0);
        unsigned char small[4]{};
        CHECK(p.client.dispatch(NativeAdrenoMethodGetStatus,nullptr,0,small,sizeof(small),&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* Refusals never reach a device reference, ring write or unmap. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        const uint32_t wptr=cp.regs[0x0807];
        NativeAdrenoAbiBoAllocReply allocated{};
        NativeAdrenoAbiBoAlloc bad[]={{0,12,0},{4097,12,0},{8192,11,0},
            {8192,25,0},{8192,12,2},{0x10000001,12,0}};
        for(const auto &request:bad)CHECK(p.client.allocBo(&request,&allocated)==kIOReturnBadArgument);
        CHECK(!allocated.handle);
        NativeAdrenoAbiBoAlloc good{4096,12,0};
        CHECK(p.client.allocBo(&good,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap wrong[]={{0,0x800000,3,0,2,0},{allocated.handle,0x800000,2,0,2,0},
            {allocated.handle,0x800001,3,0,2,0},{allocated.handle,0x800000,3,2,2,0},
            {allocated.handle,0x800000,3,0,1,0},{allocated.handle,0x800000,3,0,2,1},
            {allocated.handle+9,0x800000,3,0,2,0}};
        NativeAdrenoAbiBoMapReply mapped{};
        for(const auto &request:wrong)CHECK(p.client.mapBo(&request,&mapped)==kIOReturnBadArgument);
        CHECK(!mapped.iova && !p.client.mapped(allocated.handle));
        NativeAdrenoAbiBoMap request{allocated.handle,0x800000,3,0,2,0};
        CHECK(p.client.mapBo(&request,&mapped)==kIOReturnSuccess);
        CHECK(p.client.mapBo(&request,&mapped)==kIOReturnBusy);
        NativeAdrenoAbiSubmit empty{};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&empty,&reply,&ringclock)==kIOReturnBadArgument);
        empty.buffer_count=17;CHECK(p.client.submit(&empty,&reply,&ringclock)==kIOReturnBadArgument);
        empty.buffer_count=1;empty.reserved=1;CHECK(p.client.submit(&empty,&reply,&ringclock)==kIOReturnBadArgument);
        empty.reserved=0;empty.buffers[0]={allocated.handle+5,0x800000,1024};
        CHECK(p.client.submit(&empty,&reply,&ringclock)==kIOReturnBadArgument);
        empty.buffers[0]={allocated.handle,0x800000,1026};
        CHECK(p.client.submit(&empty,&reply,&ringclock)==kIOReturnBadArgument);
        CHECK(cp.regs[0x0807]==wptr && p.execbuf.outstanding()==0);
        NativeAdrenoAbiWaitFence wait{0,0};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnBadArgument);
        wait.fence=7;
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnBadArgument);
        NativeAdrenoAbiBoFree unknown{allocated.handle+5};
        CHECK(p.client.freeBo(&unknown)==kIOReturnBadArgument);
        CHECK(p.client.close()==kIOReturnBusy); /* object still live */
        NativeAdrenoAbiBoFree final_free{allocated.handle};
        CHECK(p.client.freeBo(&final_free)==kIOReturnSuccess);
        CHECK(p.client.close()==kIOReturnSuccess);
        CHECK(p.client.query(nullptr)==kIOReturnBadArgument);
    }
    /* Connection teardown: an in-flight job, a cpu-map lease and a second queue are all drained by releaseAll. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess);
        NativeAdrenoAbiBoCpuMap cm{allocated.handle};
        NativeAdrenoAbiBoCpuMapReply cr{};
        CHECK(p.client.cpuMap(&cm,&cr)==kIOReturnSuccess && cr.bytes==4096);
        CHECK(p.client.memoryForToken(cr.token)==p.client.memoryOf(allocated.handle));
        CHECK(p.client.memoryForToken(0)==nullptr && p.client.memoryForToken(cr.token+1)==nullptr);
        NativeAdrenoAbiQueueReply queue{};
        CHECK(p.client.createQueue(&queue)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count=1;submit.buffers[0]={allocated.handle,0x800000,1024};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess);
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnTimeout); /* the CP has not retired the job */
        CHECK(p.client.liveObjects()==1);
        fencemem[1]=reply.fence;
        CHECK(p.client.releaseAll(nullptr,1000)==kIOReturnBadArgument);
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.client.liveObjects()==0 && p.client.memoryForToken(cr.token)==nullptr);
        CHECK(p.client.query(nullptr)==kIOReturnBadArgument && p.client.close()==kIOReturnNotReady);
        CHECK(p.client.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,31,40)==kIOReturnSuccess);
    }
    /* More than the old 64-object limit: 300 unmapped objects live at once, then drained by releaseAll. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        for(unsigned i=0;i<300;++i) {
            NativeAdrenoAbiBoAllocReply allocated{};
            CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess && allocated.handle);
        }
        CHECK(p.client.liveObjects()==300);
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnSuccess && p.client.liveObjects()==0);
    }
    /* Constant device facts: unsupported until attached, refused when malformed, then returned verbatim through dispatch. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiDeviceInfo got{};
        CHECK(p.client.deviceInfo(&got)==kIOReturnUnsupported);
        NativeAdrenoAbiDeviceInfo bad{};
        CHECK(p.client.attachDeviceInfo(&bad)==kIOReturnBadArgument);
        static const NativeAdrenoAbiDeviceInfo info={0x43050a01u,3u<<20,0x1000000ull,0x1fffffffff000ull,0x20000000ull,
            0x60000000ull,16,6,1,4,NativeAdrenoDeviceClientIova,0};
        CHECK(p.client.attachDeviceInfo(&info)==kIOReturnSuccess && p.client.attachDeviceInfo(&info)==kIOReturnBadArgument);
        CHECK(p.client.dispatch(NativeAdrenoMethodGetDeviceInfo,nullptr,0,&got,sizeof(got),&ringclock)==kIOReturnSuccess);
        CHECK(got.chip_id==0x43050a01u && got.va_start==0x20000000ull && got.highest_bank_bit==16 && got.ubwc_version==4);
        CHECK(p.client.dispatch(NativeAdrenoMethodGetDeviceInfo,nullptr,0,&got,sizeof(got)-1,&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.close()==kIOReturnSuccess);
        CHECK(p.client.deviceInfo(&got)==kIOReturnNotReady);
    }
    /* Two clients on one VM, submit object and ring (one per process): each maps only inside its own window and reports it as its VA,
     * their queues stay apart, and one tears down while the other's job is still outstanding. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        static const NativeAdrenoAbiDeviceInfo info={0x43050a01u,3u<<20,0x1000000ull,0x1fffffffff000ull,0x20000000ull,
            0x60000000ull,16,6,1,4,NativeAdrenoDeviceClientIova,0};
        NativeAdrenoUserClient other;
        CHECK(other.setScope(NativeAdrenoQueueRange,0x808000,0x8000)==kIOReturnNotReady);
        CHECK(other.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,32,40)==kIOReturnSuccess);
        CHECK(p.client.attachDeviceInfo(&info)==kIOReturnSuccess && other.attachDeviceInfo(&info)==kIOReturnSuccess);
        CHECK(p.client.setScope(0,0x800000,0x8000)==kIOReturnSuccess);
        CHECK(other.setScope(3,0x808000,0x8000)==kIOReturnBadArgument);
        CHECK(other.setScope(NativeAdrenoQueueRange,0x808001,0x8000)==kIOReturnBadArgument);
        CHECK(other.setScope(NativeAdrenoQueueRange,0xfffff000,0x2000)==kIOReturnBadArgument);   /* past the 32-bit input range */
        CHECK(other.setScope(NativeAdrenoQueueRange,0x808000,0x8000)==kIOReturnSuccess);
        NativeAdrenoAbiDeviceInfo got{};
        CHECK(other.deviceInfo(&got)==kIOReturnSuccess && got.va_start==0x808000 && got.va_size==0x8000 && got.chip_id==0x43050a01u);
        CHECK(p.client.deviceInfo(&got)==kIOReturnSuccess && got.va_start==0x800000 && got.va_size==0x8000);
        NativeAdrenoAbiBoAlloc alloc{8192,12,0};
        NativeAdrenoAbiBoAllocReply mine{},theirs{};
        CHECK(p.client.allocBo(&alloc,&mine)==kIOReturnSuccess && other.allocBo(&alloc,&theirs)==kIOReturnSuccess);
        NativeAdrenoAbiBoMapReply mapped{};
        NativeAdrenoAbiBoMap trespass{mine.handle,0x808000,3,0,2,0},straddle{theirs.handle,0x80f000,3,0,2,0},below{theirs.handle,0x800000,3,0,2,0};
        CHECK(p.client.mapBo(&trespass,&mapped)==kIOReturnBadArgument && other.mapBo(&straddle,&mapped)==kIOReturnBadArgument);
        CHECK(other.mapBo(&below,&mapped)==kIOReturnBadArgument && !other.windowPeak());
        NativeAdrenoAbiBoMap ours{mine.handle,0x800000,3,0,2,0},yours{theirs.handle,0x80e000,3,0,2,0};
        CHECK(p.client.mapBo(&ours,&mapped)==kIOReturnSuccess && other.mapBo(&yours,&mapped)==kIOReturnSuccess);
        CHECK(p.client.windowPeak()==0x802000 && other.windowPeak()==0x810000);
        CHECK(p.client.setScope(0,0x800000,0x8000)==kIOReturnBusy);   /* objects are live */
        NativeAdrenoAbiSubmit submit{};
        NativeAdrenoAbiSubmitReply reply{};
        submit.buffer_count=1;submit.buffers[0]={mine.handle,0x800000,1024};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess && reply.fence==1);
        submit.buffers[0]={theirs.handle,0x80e000,1024};
        CHECK(other.submit(&submit,&reply,&ringclock)==kIOReturnSuccess && reply.fence==2);
        CHECK(p.execbuf.queueOutstanding(0) && p.execbuf.queueOutstanding(NativeAdrenoQueueRange));
        fencemem[1]=1;
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnSuccess && p.client.liveObjects()==0);
        CHECK(p.execbuf.outstanding()==1 && other.liveObjects()==1);
        CHECK(other.close()==kIOReturnBusy);
        CHECK(other.releaseAll(&ringclock,1000)==kIOReturnTimeout && other.liveObjects()==1);
        fencemem[1]=2;
        CHECK(other.releaseAll(&ringclock,1000)==kIOReturnSuccess && p.execbuf.outstanding()==0);
        CHECK(other.deviceInfo(&got)==kIOReturnNotReady);
    }
    /* int-200: a client opened while another one has a job in flight. Before its first setScope a client owns no queue range (its queue_base_ is the
     * default 0, the first client's), so neither the first scope nor the teardown of a client that never got one may be refused or held up by
     * the first client's jobs. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        static const NativeAdrenoAbiDeviceInfo info={0x43050a01u,3u<<20,0x1000000ull,0x1fffffffff000ull,0x20000000ull,
            0x60000000ull,16,6,1,4,NativeAdrenoDeviceClientIova,0};
        CHECK(p.client.attachDeviceInfo(&info)==kIOReturnSuccess && p.client.setScope(0,0x800000,0x8000)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{8192,12,0};
        NativeAdrenoAbiBoAllocReply mine{};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.allocBo(&alloc,&mine)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap ours{mine.handle,0x800000,3,0,2,0};
        CHECK(p.client.mapBo(&ours,&mapped)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit submit{};
        NativeAdrenoAbiSubmitReply reply{};
        submit.buffer_count=1;submit.buffers[0]={mine.handle,0x800000,1024};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess && p.execbuf.queueOutstanding(0));
        NativeAdrenoUserClient late,never;
        CHECK(late.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,32,40)==kIOReturnSuccess);
        CHECK(late.setScope(NativeAdrenoQueueRange,0x808000,0x8000)==kIOReturnSuccess);
        CHECK(late.ownRangeOutstanding()==false && late.close()==kIOReturnSuccess);
        CHECK(never.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,32,40)==kIOReturnSuccess);
        CHECK(never.ownRangeOutstanding()==false);
        CHECK(never.releaseAll(&ringclock,0)==kIOReturnSuccess);   /* the mount's cleanup after a refused scope */
        CHECK(p.execbuf.queueOutstanding(0) && p.client.ownRangeOutstanding());
        fencemem[1]=1;
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnSuccess);
    }
    /* Completed work never pins a buffer: no wait() call, the CP only advances its fence, and BoFree still succeeds. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count=1;submit.buffers[0]={allocated.handle,0x800000,1024};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnSuccess);
        NativeAdrenoAbiBoFree request{allocated.handle};
        CHECK(p.client.freeBo(&request)!=kIOReturnSuccess); /* the job is still in flight */
        fencemem[1]=reply.fence;
        CHECK(p.client.freeBo(&request)==kIOReturnSuccess && p.client.liveObjects()==0);
        NativeAdrenoAbiWaitFence wait{reply.fence,0};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess); /* retired job, fence already passed */
        NativeAdrenoAbiWaitFence future{reply.fence+5,0};
        CHECK(p.client.waitFence(&future,&ringclock)==kIOReturnBadArgument);
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* Large objects try one contiguous run first and fall back to scattered pages when that fails (and a refused retry). */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc big{1u<<20,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        fixture.allocation_fail_count=1;
        CHECK(p.client.allocBo(&big,&allocated)==kIOReturnSuccess && allocated.handle && fixture.allocation_fail_count==0);
        NativeAdrenoAbiBoAllocReply second{};
        fixture.allocation_fail_count=2; /* contiguous and scattered both fail */
        CHECK(p.client.allocBo(&big,&second)==kIOReturnNoMemory && !second.handle && fixture.allocation_fail_count==0);
        NativeAdrenoAbiBoAlloc demanded{1u<<20,12,1};
        fixture.allocation_fail_count=1; /* a demanded contiguous run is not retried scattered */
        CHECK(p.client.allocBo(&demanded,&second)==kIOReturnNoMemory);
        CHECK(p.client.releaseAll(&ringclock,1000)==kIOReturnSuccess);
    }
    /* Version and mount refusals, then a commit that poisons everything. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoUserClient client;
        CHECK(client.open(NativeAdrenoAbiVersion+1,&p.vm,&p.execbuf,&p.ring,32,40)==kIOReturnUnsupported);
        CHECK(client.open(NativeAdrenoAbiVersion,nullptr,&p.execbuf,&p.ring,32,40)==kIOReturnBadArgument);
        CHECK(client.open(NativeAdrenoAbiVersion,&p.vm,nullptr,&p.ring,32,40)==kIOReturnBadArgument);
        CHECK(client.open(NativeAdrenoAbiVersion,&p.vm,&p.execbuf,&p.ring,30,40)==kIOReturnBadArgument);
        CHECK(client.close()==kIOReturnNotReady && client.query(nullptr)==kIOReturnBadArgument);
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess);
        cp.fail_publish=cp.publish+1;
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count=1;submit.buffers[0]={allocated.handle,0x800000,4096};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&submit,&reply,&ringclock)==kIOReturnIOError && !reply.fence);
        CHECK(p.execbuf.poisoned() && p.execbuf.outstanding()==1);
        NativeAdrenoAbiBoFree retained{allocated.handle};
        CHECK(p.client.freeBo(&retained)==kIOReturnBusy);
        CHECK(p.client.close()==kIOReturnBusy);
    }
    /* CPU mapping leases keep the backing object alive for the user mapping. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoCpuMap map{allocated.handle};
        NativeAdrenoAbiBoCpuMapReply mapped{};
        CHECK(p.client.cpuMap(&map,&mapped)==kIOReturnSuccess);
        CHECK(mapped.token && mapped.bytes==4096);
        const uint32_t token=mapped.token;
        CHECK(p.client.cpuMap(&map,&mapped)==kIOReturnSuccess && mapped.token==token);
        NativeAdrenoAbiBoFree released{allocated.handle};
        CHECK(p.client.freeBo(&released)==kIOReturnBusy);
        CHECK(p.client.close()==kIOReturnBusy);
        NativeAdrenoAbiBoCpuUnmap wrong{allocated.handle,token+1,0};
        CHECK(p.client.cpuUnmap(&wrong)==kIOReturnBadArgument);
        NativeAdrenoAbiBoCpuUnmap unmap{allocated.handle,token,0};
        CHECK(p.client.cpuUnmap(&unmap)==kIOReturnSuccess);
        CHECK(p.client.freeBo(&released)==kIOReturnBusy); /* one lease left */
        CHECK(p.client.cpuUnmap(&unmap)==kIOReturnSuccess);
        CHECK(p.client.cpuUnmap(&unmap)==kIOReturnBusy);
        CHECK(p.client.freeBo(&released)==kIOReturnSuccess);
        CHECK(p.client.cpuMap(&map,&mapped)==kIOReturnBadArgument);
        CHECK(p.client.close()==kIOReturnSuccess);
    }
    /* Queue objects bind submissions and must be drained before close. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        for(unsigned i=1;i<8;++i) {
            NativeAdrenoAbiQueueReply reply{};
            CHECK(p.client.createQueue(&reply)==kIOReturnSuccess && reply.queue==i);
        }
        NativeAdrenoAbiQueueReply extra{};
        CHECK(p.client.createQueue(&extra)==kIOReturnSuccess && extra.queue==8);   /* queues grow as msm's submitqueues do */
        NativeAdrenoAbiBoAlloc alloc{4096,12,0};
        NativeAdrenoAbiBoAllocReply allocated{};
        CHECK(p.client.allocBo(&alloc,&allocated)==kIOReturnSuccess);
        NativeAdrenoAbiBoMap map{allocated.handle,0x800000,3,0,2,0};
        NativeAdrenoAbiBoMapReply mapped{};
        CHECK(p.client.mapBo(&map,&mapped)==kIOReturnSuccess);
        NativeAdrenoAbiSubmit first{};
        first.buffer_count=1;first.queue=1;first.buffers[0]={allocated.handle,0x800000,512};
        NativeAdrenoAbiSubmitReply reply{};
        CHECK(p.client.submit(&first,&reply,&ringclock)==kIOReturnSuccess && reply.fence==1);
        NativeAdrenoAbiQueue close{1};
        CHECK(p.client.closeQueue(&close)==kIOReturnBusy);
        NativeAdrenoAbiSubmit second{};
        second.buffer_count=1;second.queue=2;second.buffers[0]={allocated.handle,0x800000,512};
        CHECK(p.client.submit(&second,&reply,&ringclock)==kIOReturnSuccess && reply.fence==2);
        fencemem[1]=1;
        NativeAdrenoAbiWaitFence wait{1,0};
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(p.client.closeQueue(&close)==kIOReturnSuccess);
        fencemem[1]=2;wait.fence=2;
        CHECK(p.client.waitFence(&wait,&ringclock)==kIOReturnSuccess);
        CHECK(p.client.closeQueue(&close)==kIOReturnBadArgument);
        CHECK(p.client.submit(&first,&reply,&ringclock)==kIOReturnBadArgument);
        NativeAdrenoAbiQueue none{0};
        CHECK(p.client.closeQueue(&none)==kIOReturnBadArgument);
        NativeAdrenoAbiQueue two{2};
        CHECK(p.client.close()==kIOReturnBusy); /* queue 2 still open */
        CHECK(p.client.closeQueue(&two)==kIOReturnSuccess);
        for(uint64_t id=3;id<=8;++id) {
            NativeAdrenoAbiQueue rest{id};
            CHECK(p.client.closeQueue(&rest)==kIOReturnSuccess);
        }
        NativeAdrenoAbiBoFree released{allocated.handle};
        CHECK(p.client.freeBo(&released)==kIOReturnSuccess);
        CHECK(p.client.close()==kIOReturnSuccess);
        NativeAdrenoAbiQueueReply after{};
        CHECK(p.client.createQueue(&after)==kIOReturnNotReady);
    }
    std::printf("PASS: 1 complete user-client transaction with pre-submit wait, 1 CPU mapping lease block, 1 queue lifecycle block, 3 metadata refusals, 1 dispatched transaction, 6 dispatch size/argument refusals, 6 allocation refusals, 7 map refusals and 1 poisoned-commit retention case, 1 two-client shared-ring block with 8 scope/window refusals, 1 late-client first-scope block over software SMMU/CP peers\n");
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
