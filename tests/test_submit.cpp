/* SPDX-License-Identifier: MIT */
/* Real VM, ring and execbuf code over two software register peers. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoSubmit.h"
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
/* SMMUv2 register peer. */
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
/* CP register peer. */
struct CpPeer {
    std::vector<uint32_t> regs;
    std::vector<uint32_t> writes;
    unsigned publish=0,acquire=0,barriers=0,fail_publish=0,fail_acquire=0,fail_barrier=0;
    CpPeer():regs(0x10000,0){}
};
static CpPeer cp;
static uint64_t clock_us=0;
static int cwrite(void *c,uint32_t reg,uint32_t v)
{CHECK(c==&cp && reg<cp.regs.size());cp.regs[reg]=v;cp.writes.push_back(reg);return 0;}
static int cread(void *c,uint32_t reg,uint32_t *v)
{CHECK(c==&cp && v && reg<cp.regs.size());*v=cp.regs[reg];return 0;}
static int cbarrier(void *c){CHECK(c==&cp);++cp.barriers;return cp.barriers==cp.fail_barrier;}
static int cpublish(void *c,const volatile uint32_t *w,size_t bytes)
{CHECK(c==&cp && w && (bytes==32768 || bytes==4096));++cp.publish;return cp.publish==cp.fail_publish;}
static int cacquire(void *c,volatile uint32_t *w,size_t bytes)
{CHECK(c==&cp && w && bytes==4096);++cp.acquire;return cp.acquire==cp.fail_acquire;}
static uint64_t cnow(void *c){CHECK(c==&cp);return clock_us;}
static int cidle(void *c){CHECK(c==&cp);clock_us+=1000;return 0;}
struct Path {
    NativeAdrenoTablePages tables;
    NativeAdrenoMemory object;
    ArmSmmu4KLiveArena arena;
    ArmSmmu4KLiveState state{};
    ArmSmmu4KLiveVisibility visibility;
    NativeAdrenoVm vm;
    NativeAdrenoRing ring;
    NativeAdrenoSubmit submit;
    uint64_t handle=0;
};
static uint32_t ringmem[8192],fencemem[1024];
static std::vector<uint64_t> dram_reads;
static NativeAdrenoRingIo ringio{&cp,cwrite,cread,cbarrier,nullptr,nullptr};
static NativeAdrenoRingCache ringcache{&cp,cpublish,cacquire};
static NativeAdrenoRingClock ringclock{&cp,cnow,cidle};
static void smmuReset(void)
{
    std::memset(mmio,0,sizeof(mmio));mf={};streams=(ArmSmmu4KStreamState){};
    smmuio=(ArmSmmu4KIo){&mf,SMMU_BYTES,mread32,mread64,mwrite32,mwrite64,mbarrier};
    mmio[0x20/4]=(1u<<30)|(1u<<27)|(4u<<9)|8;
    mmio[0x24/4]=0x40000010;mmio[0x28/4]=0x1433;mmio[0]=0x436;
    for(unsigned i=0;i<8;++i)mmio[(0xc00+4*i)/4]=0x20000;
    mmio[0x800/4]=1u<<31|9;mmio[0xc00/4]=3;
}
static int exposed(Path &p,bool write_combined=false)
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
    if(p.object.allocate(8192,40,false,8192))return kIOReturnNoMemory;
    if(write_combined && p.object.makeWriteCombined())return kIOReturnNoMemory;
    if(p.vm.map(&p.object,0,0x800000,3,0,2,&p.handle))return kIOReturnBadArgument;
    if(p.ring.attach(&ringio,&ringcache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000))
        return kIOReturnBadArgument;
    return p.submit.attach(&p.vm,&p.ring);
}
void NativeAdrenoNotePhase(unsigned,size_t) {}
int main()
{
    /* One validated indirect-buffer submission with exact ring bytes. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        uint32_t fence=0;
        /* Before the ring: clean+invalidate of the one mapped BO and a coherence sample of the IB's first word; no TLBIALL (a map fills only
         * invalid leaves and an unmap drops its own pages). */
        static_cast<volatile uint32_t *>(p.object.mapping())[0]=0x70970001u;
        const unsigned cache_calls=fixture.cache_calls;
        mmio[(CB+0x618)/4]=0xffffffffu;
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==1);
        CHECK(fixture.cache_calls==cache_calls+1 && mmio[(CB+0x618)/4]==0xffffffffu);
        CHECK(native_adreno_coherence[0]==1 && native_adreno_coherence[1]==0x70970001u &&
            native_adreno_coherence[2]==~0u && native_adreno_coherence[3]==~0u);   /* no board DRAM reader installed */
        CHECK(cp.writes.back()==0x0807 && cp.regs[0x0807]==11+4+31);
        CHECK((ringmem[11]>>16&0x7fu)==0x3f && (ringmem[11]&0x3fffu)==3);
        CHECK(ringmem[12]==0x800000 && ringmem[13]==0 && ringmem[14]==1024);
        CHECK(p.object.deviceReferences()==2); /* mapping + job */
        CHECK(p.submit.outstanding()==1 && p.submit.stats().references_held==1);
        CHECK(p.submit.detach()==kIOReturnBusy);
        uint32_t observed=0;
        CHECK(p.ring.wait(1,&ringclock,1000,&observed)==NativeAdrenoRingFenceTimeout);
        CHECK(p.submit.wait(1,&ringclock,1000)==kIOReturnTimeout && p.submit.stats().timeouts==1);
        fencemem[1]=1;
        CHECK(p.submit.wait(1,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.outstanding()==0 && p.submit.stats().completed==1);
        CHECK(p.object.deviceReferences()==1 && p.submit.stats().references_retired==1);
        CHECK(p.submit.wait(1,&ringclock,1000)==kIOReturnSuccess); /* retired job: the ring says the CP passed it */
        CHECK(p.submit.wait(2,&ringclock,1000)==kIOReturnBadArgument); /* never submitted */
        /* The coherence sample runs on every 64th submission only (the first was sampled above). */
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==2 && native_adreno_coherence[0]==1);
        fencemem[1]=2;
        CHECK(p.submit.wait(2,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.detach()==kIOReturnSuccess);
    }
    /* int-100: a write-combined BO is not maintained while the IB audit finds no dirty line; a dirty line or a failed audit ends the
     * trust, and that submission already maintains every BO through its physical pages. */
    {
        Path p;CHECK(exposed(p,true)==kIOReturnSuccess);
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        static std::vector<uint64_t> audited;static uint32_t dirty_lines,fail;
        audited.clear();dirty_lines=0;fail=0;
        native_adreno_line_audit=[](uint64_t physical,size_t bytes,uint32_t *dirty,uint32_t *stale) {
            audited.push_back(physical);audited.push_back(bytes);
            if(fail)return 1;
            *dirty+=dirty_lines;*stale+=1;return 0;};
        const uint32_t submissions=native_adreno_wc_audit[0],lines=native_adreno_wc_audit[1];
        const unsigned physical=fixture.physical_calls,owned=fixture.cache_calls;
        uint32_t fence=0;
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==1);
        CHECK(audited.size()==2 && audited[0]==p.object.spans()[0].physical && audited[1]==4096);
        CHECK(native_adreno_wc_audit[0]==submissions+1 && native_adreno_wc_audit[1]==lines+64 && native_adreno_wc_distrusted==0);
        CHECK(fixture.physical_calls==physical && fixture.cache_calls==owned && p.vm.writeCombinedTrusted());   /* no maintenance */
        fencemem[1]=1;CHECK(p.submit.wait(1,&ringclock,1000)==kIOReturnSuccess);
        dirty_lines=2;
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==2);
        CHECK(!p.vm.writeCombinedTrusted() && native_adreno_wc_distrusted);
        CHECK(fixture.physical_calls>physical);   /* this submission maintained the write-combined BO */
        const size_t calls=audited.size();
        fencemem[1]=2;CHECK(p.submit.wait(2,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && audited.size()==calls);   /* no audit once distrusted */
        fencemem[1]=3;CHECK(p.submit.wait(3,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.detach()==kIOReturnSuccess);
    }
    /* The audit compares only the first 16 submissions after attach; the re-assertion runs at each. */
    {
        Path p;CHECK(exposed(p,true)==kIOReturnSuccess);
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        static unsigned calls;calls=0;
        native_adreno_line_audit=[](uint64_t,size_t,uint32_t *,uint32_t *stale) {++calls;*stale+=0;return 0;};
        const uint32_t submissions=native_adreno_wc_audit[0];
        for(uint32_t n=1;n<=24;++n) {
            uint32_t fence=0;
            CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==n);
            fencemem[1]=n;CHECK(p.submit.wait(n,&ringclock,1000)==kIOReturnSuccess);
        }
        CHECK(calls==16 && native_adreno_wc_audit[0]==submissions+16 && p.vm.writeCombinedTrusted());
        native_adreno_line_audit=nullptr;
        CHECK(p.submit.detach()==kIOReturnSuccess);
    }
    {
        Path p;CHECK(exposed(p,true)==kIOReturnSuccess);
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        native_adreno_line_audit=[](uint64_t,size_t,uint32_t *,uint32_t *) {return 1;};
        uint32_t fence=0;
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==1);
        CHECK(!p.vm.writeCombinedTrusted() && native_adreno_wc_audit[4]==kIOReturnIOError);
        native_adreno_line_audit=nullptr;native_adreno_wc_distrusted=0;
        fencemem[1]=1;CHECK(p.submit.wait(1,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.detach()==kIOReturnSuccess);
    }
    /* Refusals precede references, ring bytes and WPTR updates. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        const uint32_t wptr=cp.regs[0x0807];
        uint32_t fence=0;
        CHECK(p.submit.submit(nullptr,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer zero[1]={{p.handle,0x800000,0}};
        CHECK(p.submit.submit(zero,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer unaligned[1]={{p.handle,0x800000,4098}};
        CHECK(p.submit.submit(unaligned,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer wrong[1]={{p.handle,0x802000,4096}};
        CHECK(p.submit.submit(wrong,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer overrun[1]={{p.handle,0x800000,8192+4}};
        CHECK(p.submit.submit(overrun,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer unknown[1]={{p.handle+7,0x800000,4096}};
        CHECK(p.submit.submit(unknown,1,0,&fence)==kIOReturnBadArgument);
        NativeAdrenoSubmitBuffer many[NativeAdrenoSubmitMaxBuffers+1];
        for(auto &b:many)b={p.handle,0x800000,4096};
        CHECK(p.submit.submit(many,NativeAdrenoSubmitMaxBuffers+1,0,&fence)==kIOReturnBadArgument);
        CHECK(p.object.deviceReferences()==1 && cp.regs[0x0807]==wptr && p.submit.outstanding()==0);
        /* Duplicate buffers share one device reference. */
        NativeAdrenoSubmitBuffer twice[2]={{p.handle,0x800000,4096},{p.handle,0x800000,4096}};
        CHECK(p.submit.submit(twice,2,0,&fence)==kIOReturnSuccess);
        CHECK(p.object.deviceReferences()==2 && cp.regs[0x0807]==11+8+31);
        fencemem[1]=1;
        CHECK(p.submit.wait(1,&ringclock,100)==kIOReturnSuccess && p.object.deviceReferences()==1);
        CHECK(p.submit.detach()==kIOReturnSuccess);
    }
    /* A commit that poisons the ring retains every reference and disables execbuf. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        cp.fail_publish=cp.publish+1;
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        uint32_t fence=0;
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnIOError);
        CHECK(!fence && p.submit.poisoned() && p.ring.poisoned());
        CHECK(p.submit.outstanding()==1 && p.submit.stats().poisoned_jobs==1);
        CHECK(p.object.deviceReferences()==2); /* never retired */
        CHECK(p.submit.detach()==kIOReturnBusy && p.submit.wait(1,&ringclock,10)==kIOReturnNotReady);
        CHECK(p.ring.detach()==kIOReturnBusy && p.vm.detach()==kIOReturnBusy);
        CHECK(p.object.release()==kIOReturnBusy);
    }
    /* A refused attach leaves both peers untouched. */
    {
        Path p;NativeAdrenoRing ring;NativeAdrenoSubmit submit;
        CHECK(submit.attach(nullptr,&ring)==kIOReturnBadArgument);
        CHECK(submit.attach(&p.vm,nullptr)==kIOReturnBadArgument);
        CHECK(submit.detach()==kIOReturnNotReady);
        CHECK(submit.submit(nullptr,1,0,nullptr)==kIOReturnBadArgument);
    }
    /* msm hangcheck: no fence progress with work pending is a hang once the CP state stops moving or after three progress retries; recovery
     * marks the guilty job's VM, moves the fence past it, restarts the ring and replays the later jobs with their own fences. */
    {
        Path p;CHECK(exposed(p)==kIOReturnSuccess);
        NativeAdrenoSubmitBuffer buffer{p.handle,0x800000,4096};
        uint32_t fence=0;
        for(uint32_t n=1;n<=3;++n)CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnSuccess && fence==n);
        bool hung=true;
        for(uint32_t tick=1;tick<=3;++tick) {
            cp.regs[0x0928]=0x1000*tick;   /* CP_IB1_BASE moves: progress */
            CHECK(p.submit.hangCheck(&hung)==kIOReturnSuccess && !hung);
        }
        cp.regs[0x0928]=0x9000;
        CHECK(p.submit.hangCheck(&hung)==kIOReturnSuccess && hung);   /* three retries spent */
        CHECK(p.submit.hangCheck(&hung)==kIOReturnSuccess && !hung);   /* progress retries start again: one retry for the changed state */
        static uint32_t restarted;restarted=~0u;
        auto restart=[](void *,NativeAdrenoRing *ring,uint32_t completed)->IOReturn {restarted=completed;return ring->restart(completed);};
        NativeAdrenoRecovery result{};
        const uint32_t refs=p.object.deviceReferences();
        CHECK(p.submit.recover(restart,nullptr,&ringclock,&result)==kIOReturnSuccess);
        CHECK(restarted==1 && result.fence==1 && result.replayed==2 && result.guilty==&p.vm && p.vm.faults()==1 && !p.vm.unusable());
        CHECK(fencemem[1]==1 && p.ring.lastSubmitted()==3 && p.submit.outstanding()==2 && p.object.deviceReferences()==refs-1);
        CHECK(p.submit.stats().restarts==1);
        fencemem[1]=3;
        CHECK(p.submit.wait(3,&ringclock,1000)==kIOReturnSuccess && p.submit.wait(2,&ringclock,1000)==kIOReturnSuccess);
        CHECK(p.submit.outstanding()==0);
        /* A VM a hang made unusable submits nothing more. */
        p.vm.noteHang(true);
        CHECK(p.submit.submit(&buffer,1,0,&fence)==kIOReturnNotPermitted);
        /* A restart that fails leaves the submit object poisoned. */
        CHECK(p.submit.recover([](void *,NativeAdrenoRing *,uint32_t)->IOReturn {return kIOReturnIOError;},nullptr,&ringclock,&result)==kIOReturnIOError);
        CHECK(p.submit.poisoned());
    }
    std::printf("PASS: 1 execbuf submission with exact IB packet and fence, 8 pre-reference refusals, 1 duplicate-buffer reference case and 1 poisoned-commit retention case over software SMMU/CP peers; 2 write-combine audit cases (trusted, dirty and failed); 1 hangcheck and 1 recovery with replay, 1 unusable-VM refusal and 1 failed restart\n");
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
