/* SPDX-License-Identifier: MIT */
/* The passes over a VM's mappings (map, unmap, submission, completion, re-assertion) over the real VM and memory code and a software
 * SMMUv2 register peer: which mappings each pass maintains, which it leaves alone and how many it visits. Host cache doubles only
 * count their calls; no CPU cache, DRAM or GPU is involved. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoVm.h"
#include <cstdio>
#include <vector>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
#define BYTES 0x40000
#define CB 0x22000
extern "C" volatile uint32_t native_adreno_io_coherent,native_adreno_coherent_skips[4];
IOReturn NativeAdrenoQueryCpu4K(NativeAdrenoCpuState *out)
{CHECK(out);*out={};return kIOReturnSuccess;}
/* Maintenance through a kernel mapping, counted for the buffers under test only (a map also cleans its new table pages this way):
 * their number and the operations in order. */
static std::vector<void *> tracked;
static std::vector<unsigned> operations;
static unsigned tracked_calls;
IOReturn NativeAdrenoMaintainOwnedPages(void *mapping,size_t bytes,NativeAdrenoCacheOperation op)
{
    CHECK(mapping && bytes && !(((uintptr_t)mapping|bytes)&4095));
    ++fixture.cache_calls;
    for(void *base:tracked)if(base==mapping) {++tracked_calls;operations.push_back(op);}
    return kIOReturnSuccess;
}
/* A buffer with a kernel mapping, allocated and tracked. */
static IOReturn buffer(NativeAdrenoMemory &memory,size_t bytes)
{
    const IOReturn status=memory.allocate(bytes,40,true,bytes);
    if(!status)tracked.push_back(memory.mapping());
    return status;
}
static uint32_t memory[BYTES/4];
static ArmSmmu4KIo io;
static ArmSmmu4KStreamState streams;
static ArmSmmu4KStream routes[2]={{2,0},{4,1}};
static ArmSmmu4KStreamResult route_result;
static ArmSmmu4KContextConfig context;
static ArmSmmu4KContextImage image;
static int read32(void *,size_t at,uint32_t *v){CHECK(at%4==0 && at<=BYTES-4);*v=memory[at/4];return 0;}
static int read64(void *,size_t at,uint64_t *v){CHECK(at%8==0 && at<=BYTES-8);std::memcpy(v,memory+at/4,8);return 0;}
static int write32(void *,size_t at,uint32_t v){CHECK(at%4==0 && at<=BYTES-4);memory[at/4]=v;return 0;}
static int write64(void *,size_t at,uint64_t v){CHECK(at%8==0 && at<=BYTES-8);std::memcpy(memory+at/4,&v,8);return 0;}
static int barrier(void *){return 0;}
struct Space {
    NativeAdrenoTablePages tables;
    ArmSmmu4KLiveArena arena;
    ArmSmmu4KLiveState state{};
    ArmSmmu4KLiveVisibility visibility;
    NativeAdrenoVm vm;
};
/* A prepared arena of 4096 pages at 0x800000, an enabled context image and exact stream routes. */
static IOReturn exposed(Space &s)
{
    std::memset(memory,0,sizeof(memory));streams=(ArmSmmu4KStreamState){};
    io=(ArmSmmu4KIo){&memory,BYTES,read32,read64,write32,write64,barrier};
    memory[0x20/4]=(1u<<30)|(1u<<27)|(4u<<9)|8;
    memory[0x24/4]=0x40000010;memory[0x28/4]=0x1433;memory[0]=0x436;
    for(unsigned i=0;i<8;++i)memory[(0xc00+4*i)/4]=0x20000;
    memory[0x800/4]=1u<<31|9;memory[0xc00/4]=3;
    IOReturn smmu=NativeAdrenoPrepareSmmuMemory(&s.tables,32,40,0x800000,4096*4096,0,4096,&s.arena,&s.state,&s.visibility);
    if(smmu)return smmu;
    context=(ArmSmmu4KContextConfig){.lower=&s.arena.config,.lower_tables=&s.state.tables,
        .id0=memory[0x20/4],.id2=memory[0x28/4],.asid=1,.coherent_walk=1,
        .stall_on_fault=0,.continue_after_fault=1};
    ArmSmmu4KContextConfig actual=context;
    CHECK(ArmSmmu4KEncodeContext(&actual,&image)==ArmSmmu4KSuccess);
    memory[0x1008/4]=image.cbar;memory[0x1808/4]=image.cba2r;
    memory[CB/4]=image.sctlr;memory[(CB+0x10)/4]=image.tcr2;memory[(CB+0x30)/4]=image.tcr;
    std::memcpy(memory+(CB+0x20)/4,&image.ttbr0,8);
    std::memcpy(memory+(CB+0x28)/4,&image.ttbr1,8);
    memory[(CB+0x38)/4]=image.mair0;
    if(ArmSmmu4KAttachStreams(&io,&streams,routes,2,2,&context,&route_result)!=ArmSmmu4KSuccess)
        return kIOReturnBadArgument;
    NativeAdrenoVmRegion lower={&s.tables,&s.arena,&s.state};
    return s.vm.attach(&io,&context,&streams,&lower,nullptr,32,1000);
}
static void teardown(Space &s)
{
    CHECK(s.vm.detach()==kIOReturnSuccess && s.tables.release()==kIOReturnSuccess);
    tracked.clear();
    CHECK(fixture.live.empty() && fixture.metadata.empty() && !fixture.imports_live);
}
struct Counters {
    uint64_t kind[NativeAdrenoKindCount][3],walk[8];
    uint32_t skips[2],coherent[4];
    unsigned owned,physical;
    Counters() {
        for(unsigned k=0;k<NativeAdrenoKindCount;++k)for(unsigned i=0;i<3;++i)kind[k][i]=native_adreno_maint_kind[k][i];
        for(unsigned i=0;i<8;++i)walk[i]=native_adreno_maint_walk[i];
        skips[0]=native_adreno_sync_skips[0];skips[1]=native_adreno_sync_skips[1];
        for(unsigned i=0;i<4;++i)coherent[i]=native_adreno_coherent_skips[i];
        owned=tracked_calls;physical=fixture.physical_calls;
    }
};
int main()
{
    native_adreno_maint_all=0;   /* no boot argument on the host */
    /* A synchronized driver buffer: cleaned and dropped once when declared, then left alone by a map, a submission's pass and an
     * unmap, while memory nobody declared is cleaned and invalidated by each of them. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory driver,plain;
        CHECK(buffer(driver,8192)==kIOReturnSuccess && buffer(plain,4096)==kIOReturnSuccess);
        CHECK(!driver.synchronized());
        operations.clear();
        CHECK(driver.declareSynchronized()==kIOReturnSuccess && driver.synchronized());
        CHECK(operations.size()==1 && operations[0]==NativeAdrenoCleanInvalidate);
        Counters a;
        uint64_t d=0,p=0;
        CHECK(s.vm.map(&driver,0,0x800000,3,0,2,&d)==kIOReturnSuccess);
        CHECK(tracked_calls==a.owned && native_adreno_sync_skips[0]==a.skips[0]+1);
        CHECK(s.vm.map(&plain,0,0x810000,3,0,2,&p)==kIOReturnSuccess);
        CHECK(tracked_calls==a.owned+1 && operations.back()==NativeAdrenoCleanInvalidate);
        CHECK(driver.declareSynchronized()==kIOReturnNotReady);   /* only before any mapping */
        Counters b;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess);
        CHECK(tracked_calls==b.owned+1);   /* the undeclared buffer alone */
        CHECK(native_adreno_maint_kind[NativeAdrenoKindKernel][0]==b.kind[NativeAdrenoKindKernel][0]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindKernel][1]==b.kind[NativeAdrenoKindKernel][1]+4);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindDriver][0]==b.kind[NativeAdrenoKindDriver][0] &&
            native_adreno_maint_kind[NativeAdrenoKindDriver][2]==b.kind[NativeAdrenoKindDriver][2]+1);
        CHECK(native_adreno_maint_walk[0]==b.walk[0]+1 && native_adreno_maint_walk[1]==b.walk[1]+1 && native_adreno_maint_walk[2]==b.walk[2]+2);
        Counters c;
        CHECK(s.vm.unmap(d)==kIOReturnSuccess);
        CHECK(tracked_calls==c.owned && native_adreno_sync_skips[1]==c.skips[1]+1);
        CHECK(s.vm.unmap(p)==kIOReturnSuccess);
        CHECK(tracked_calls==c.owned+1 && operations.back()==NativeAdrenoInvalidate);
        /* A writer's own clean of what it stored is done whatever the buffer was declared. */
        CHECK(driver.publishForDevice(0,4096,false,true,NativeAdrenoPhaseWrite)==kIOReturnSuccess && operations.back()==NativeAdrenoClean);
        CHECK(driver.release()==kIOReturnSuccess && !driver.synchronized() && plain.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* gpu-maint-all=1 restores the passes over everything: the synchronized buffer is maintained at its map, at every submission and
     * at its unmap, and each pass visits every mapping. Turning it off again classifies the mappings again. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory driver;
        CHECK(buffer(driver,8192)==kIOReturnSuccess && driver.declareSynchronized()==kIOReturnSuccess);
        native_adreno_maint_all=1;
        Counters a;
        uint64_t d=0;
        CHECK(s.vm.map(&driver,0,0x800000,3,0,2,&d)==kIOReturnSuccess);
        CHECK(tracked_calls==a.owned+1 && native_adreno_sync_skips[0]==a.skips[0]);
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==a.owned+2);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindDriver][0]==a.kind[NativeAdrenoKindDriver][0]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindDriver][1]==a.kind[NativeAdrenoKindDriver][1]+8);
        native_adreno_maint_all=0;
        Counters b;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==b.owned);
        CHECK(native_adreno_maint_walk[5]==b.walk[5]+1 && native_adreno_maint_walk[1]==b.walk[1]);   /* classified again, nothing to visit */
        native_adreno_maint_all=1;
        CHECK(s.vm.unmap(d)==kIOReturnSuccess && tracked_calls==b.owned+1 && operations.back()==NativeAdrenoInvalidate);
        native_adreno_maint_all=0;
        CHECK(driver.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* A dirty line found by the audit ends the trust in the synchronized buffers (native_adreno_sync_audit[4]): every VM maintains
     * them again from its next pass on. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory driver;
        CHECK(buffer(driver,8192)==kIOReturnSuccess && driver.declareSynchronized()==kIOReturnSuccess);
        uint64_t d=0;
        CHECK(s.vm.map(&driver,0,0x800000,3,0,2,&d)==kIOReturnSuccess);
        Counters a;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==a.owned);
        native_adreno_sync_audit[4]=3;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==a.owned+1 && operations.back()==NativeAdrenoCleanInvalidate);
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==a.owned+2);
        CHECK(native_adreno_maint_walk[5]==a.walk[5]+1);
        CHECK(s.vm.unmap(d)==kIOReturnSuccess && tracked_calls==a.owned+3);
        native_adreno_sync_audit[4]=0;
        CHECK(driver.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* The pass of a submission visits the mappings it has work for, not every mapping: 300 write-combined user BOs (more than the
     * first mapping table holds), one synchronized buffer and three undeclared ones. Unmapping from the middle of the visited ones
     * keeps the rest. Once a write-combined view was found caching, every BO is visited and maintained. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        static NativeAdrenoMemory combined[300],plain[3],driver;
        uint64_t handles[300]={},p[3]={},d=0;
        for(unsigned i=0;i<300;++i) {
            CHECK(combined[i].allocateSystem(4096,40,4096,true)==kIOReturnSuccess);
            CHECK(s.vm.map(&combined[i],0,0x800000+0x1000ull*i,3,0,2,&handles[i])==kIOReturnSuccess);
            if(i==100)for(unsigned k=0;k<3;++k) {
                CHECK(buffer(plain[k],4096)==kIOReturnSuccess);
                CHECK(s.vm.map(&plain[k],0,0xa00000+0x1000ull*k,3,0,2,&p[k])==kIOReturnSuccess);
            }
        }
        CHECK(buffer(driver,8192)==kIOReturnSuccess && driver.declareSynchronized()==kIOReturnSuccess);
        CHECK(s.vm.map(&driver,0,0xb00000,3,0,2,&d)==kIOReturnSuccess);
        CHECK(s.vm.objectCount()==304);
        Counters a;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess);
        CHECK(tracked_calls==a.owned+3 && fixture.physical_calls==a.physical);
        CHECK(native_adreno_maint_walk[1]==a.walk[1]+3 && native_adreno_maint_walk[2]==a.walk[2]+304);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindKernel][0]==a.kind[NativeAdrenoKindKernel][0]+3 &&
            native_adreno_maint_kind[NativeAdrenoKindSystemWc][2]==a.kind[NativeAdrenoKindSystemWc][2]+300 &&
            native_adreno_maint_kind[NativeAdrenoKindDriver][2]==a.kind[NativeAdrenoKindDriver][2]+1);
        uint32_t reset=0;
        CHECK(s.vm.reassertWriteCombined(&reset)==kIOReturnSuccess && native_adreno_maint_walk[6]==a.walk[6]+3);
        CHECK(s.vm.acquireImported()==kIOReturnSuccess && native_adreno_maint_walk[4]==a.walk[4]+3);
        CHECK(s.vm.unmap(p[1])==kIOReturnSuccess && plain[1].release()==kIOReturnSuccess);
        Counters b;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==b.owned+2 && native_adreno_maint_walk[1]==b.walk[1]+2);
        s.vm.distrustWriteCombined();
        Counters c;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess);
        CHECK(tracked_calls==c.owned+2 && fixture.physical_calls==c.physical+300);   /* write-combined pages by physical address */
        CHECK(native_adreno_maint_walk[5]==c.walk[5]+1 && native_adreno_maint_walk[1]==c.walk[1]+302);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindSystemWc][0]==c.kind[NativeAdrenoKindSystemWc][0]+300 &&
            native_adreno_maint_kind[NativeAdrenoKindSystemWc][1]==c.kind[NativeAdrenoKindSystemWc][1]+1200);
        for(unsigned i=0;i<300;++i)CHECK(s.vm.unmap(handles[i])==kIOReturnSuccess && combined[i].release()==kIOReturnSuccess);
        Counters e;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==e.owned+2 && native_adreno_maint_walk[1]==e.walk[1]+2);
        CHECK(s.vm.unmap(p[0])==kIOReturnSuccess && s.vm.unmap(p[2])==kIOReturnSuccess && s.vm.unmap(d)==kIOReturnSuccess);
        Counters g;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && tracked_calls==g.owned && native_adreno_maint_walk[1]==g.walk[1]);
        CHECK(plain[0].release()==kIOReturnSuccess && plain[2].release()==kIOReturnSuccess && driver.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* System memory: a write-combined user BO is in no pass; one with the default attribute is cleaned at every submission. A
     * write-combined BO with a kernel mapping (a contiguous one) is visited for its pages' attribute only. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory wc,cached,contiguous;
        CHECK(wc.allocateSystem(8192,40,8192,true)==kIOReturnSuccess && cached.allocateSystem(8192,40,8192,false)==kIOReturnSuccess);
        CHECK(buffer(contiguous,4096)==kIOReturnSuccess && contiguous.makeWriteCombined()==kIOReturnSuccess);
        uint64_t w=0,c=0,k=0;
        CHECK(s.vm.map(&wc,0,0x800000,3,0,2,&w)==kIOReturnSuccess && s.vm.map(&cached,0,0x810000,3,0,2,&c)==kIOReturnSuccess &&
            s.vm.map(&contiguous,0,0x820000,3,0,2,&k)==kIOReturnSuccess);
        Counters a;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess);
        CHECK(fixture.physical_calls==a.physical+1 && tracked_calls==a.owned && native_adreno_maint_walk[1]==a.walk[1]+2);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindSystemCached][0]==a.kind[NativeAdrenoKindSystemCached][0]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindSystemCached][1]==a.kind[NativeAdrenoKindSystemCached][1]+8 &&
            native_adreno_maint_kind[NativeAdrenoKindSystemWc][2]==a.kind[NativeAdrenoKindSystemWc][2]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindKernel][2]==a.kind[NativeAdrenoKindKernel][2]+1);
        uint32_t reset=0;
        CHECK(s.vm.reassertWriteCombined(&reset)==kIOReturnSuccess && native_adreno_maint_walk[6]==a.walk[6]+2);
        CHECK(s.vm.unmap(w)==kIOReturnSuccess && s.vm.unmap(c)==kIOReturnSuccess && s.vm.unmap(k)==kIOReturnSuccess);
        CHECK(wc.release()==kIOReturnSuccess && cached.release()==kIOReturnSuccess && contiguous.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* Imports. Without IO coherence a submission cleans every import and a completion invalidates the GPU-written ones. Once the
     * board proved IO coherence the passes leave them alone and count them without a visit; the mappings are classified again when the
     * proof arrives. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory read,written;
        void *task=reinterpret_cast<void *>(0x77);
        CHECK(read.importTask(task,0x10000000,2*4096,40,2*4096)==kIOReturnSuccess);
        CHECK(written.importTask(task,0x20000000,2*4096,40,2*4096)==kIOReturnSuccess);
        uint64_t r=0,w=0;
        CHECK(s.vm.map(&read,0,0x800000,1,0,2,&r)==kIOReturnSuccess && s.vm.map(&written,0,0x810000,3,0,2,&w)==kIOReturnSuccess);
        Counters a;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && fixture.physical_calls==a.physical+4);   /* two scattered spans each */
        CHECK(native_adreno_maint_kind[NativeAdrenoKindImportRead][0]==a.kind[NativeAdrenoKindImportRead][0]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindImportWritten][0]==a.kind[NativeAdrenoKindImportWritten][0]+1);
        CHECK(s.vm.acquireImported()==kIOReturnSuccess && fixture.physical_calls==a.physical+6);
        CHECK(native_adreno_maint_walk[3]==a.walk[3]+1 && native_adreno_maint_walk[4]==a.walk[4]+2);
        native_adreno_io_coherent=1;
        Counters b;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && s.vm.acquireImported()==kIOReturnSuccess);
        CHECK(fixture.physical_calls==b.physical && native_adreno_maint_walk[5]==b.walk[5]+1);
        CHECK(native_adreno_maint_walk[1]==b.walk[1] && native_adreno_maint_walk[4]==b.walk[4]);
        CHECK(native_adreno_coherent_skips[2]==b.coherent[2]+2 && native_adreno_coherent_skips[3]==b.coherent[3]+1);
        CHECK(native_adreno_maint_kind[NativeAdrenoKindImportRead][2]==b.kind[NativeAdrenoKindImportRead][2]+1 &&
            native_adreno_maint_kind[NativeAdrenoKindImportWritten][2]==b.kind[NativeAdrenoKindImportWritten][2]+1);
        /* The comparison's passes count the same. */
        native_adreno_maint_all=1;
        Counters c;
        CHECK(s.vm.publishMapped()==kIOReturnSuccess && s.vm.acquireImported()==kIOReturnSuccess);
        CHECK(fixture.physical_calls==c.physical);
        CHECK(native_adreno_coherent_skips[2]==c.coherent[2]+2 && native_adreno_coherent_skips[3]==c.coherent[3]+1);
        CHECK(native_adreno_maint_walk[1]==c.walk[1]+2 && native_adreno_maint_walk[4]==c.walk[4]+2);
        native_adreno_maint_all=0;native_adreno_io_coherent=0;
        CHECK(s.vm.unmap(r)==kIOReturnSuccess && s.vm.unmap(w)==kIOReturnSuccess);
        CHECK(read.release()==kIOReturnSuccess && written.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* The audit of the synchronized buffers: the first 32 KiB of each, a page a call, and every page's lines dropped again; other
     * mappings are not looked at. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory small,large,plain;
        CHECK(buffer(small,8192)==kIOReturnSuccess && small.declareSynchronized()==kIOReturnSuccess);
        CHECK(buffer(large,65536)==kIOReturnSuccess && large.declareSynchronized()==kIOReturnSuccess);
        CHECK(buffer(plain,4096)==kIOReturnSuccess);
        uint64_t a=0,b=0,c=0;
        CHECK(s.vm.map(&small,0,0x800000,3,0,2,&a)==kIOReturnSuccess && s.vm.map(&large,0,0x810000,3,0,2,&b)==kIOReturnSuccess &&
            s.vm.map(&plain,0,0x830000,3,0,2,&c)==kIOReturnSuccess);
        uint32_t lines=0,dirty=0,stale=0;
        CHECK(s.vm.auditSynchronized(&lines,&dirty,&stale)==kIOReturnUnsupported);   /* no board hook */
        static std::vector<uint64_t> pages;pages.clear();
        native_adreno_line_audit=[](uint64_t physical,size_t bytes,uint32_t *found,uint32_t *) {
            CHECK(bytes==4096 && !(physical&4095));pages.push_back(physical);*found+=1;return 0;};
        const unsigned physical=fixture.physical_calls;
        CHECK(s.vm.auditSynchronized(&lines,&dirty,&stale)==kIOReturnSuccess);
        CHECK(pages.size()==10 && lines==640 && dirty==10 && fixture.physical_calls==physical+10);
        CHECK(pages[0]==small.spans()[0].physical && pages[1]==pages[0]+4096 && pages[2]==large.spans()[0].physical &&
            pages[9]==pages[2]+7*4096);
        native_adreno_line_audit=[](uint64_t,size_t,uint32_t *,uint32_t *) {return 1;};
        CHECK(s.vm.auditSynchronized(&lines,&dirty,&stale)==kIOReturnIOError);
        native_adreno_line_audit=nullptr;
        CHECK(s.vm.unmap(a)==kIOReturnSuccess && s.vm.unmap(b)==kIOReturnSuccess && s.vm.unmap(c)==kIOReturnSuccess);
        CHECK(small.release()==kIOReturnSuccess && large.release()==kIOReturnSuccess && plain.release()==kIOReturnSuccess);
        teardown(s);
    }
    /* The submission log's words: through the kernel's mapping where the memory has one; for system memory through the board's
     * cacheable reader, and through the uncached one only when that is all the board gave. */
    {
        Space s;CHECK(exposed(s)==kIOReturnSuccess);
        NativeAdrenoMemory mapped,system;
        fixture.mode=ScatterUnlessContiguous;   /* system memory arrives a page at a time */
        CHECK(buffer(mapped,8192)==kIOReturnSuccess && system.allocateSystem(8192,40,8192,true)==kIOReturnSuccess);
        fixture.mode=Contiguous;
        CHECK(system.spanCount()==2);
        uint64_t m=0,y=0;
        CHECK(s.vm.map(&mapped,0,0x800000,3,0,2,&m)==kIOReturnSuccess && s.vm.map(&system,0,0x810000,3,0,2,&y)==kIOReturnSuccess);
        static_cast<uint32_t *>(mapped.mapping())[1024]=0x70970001u;static_cast<uint32_t *>(mapped.mapping())[1025]=3;
        static_cast<uint32_t *>(mapped.mapping())[1027]=0x1234;
        uint32_t words[4]={9,9,9,9};
        CHECK(s.vm.headWords(m,0x801000,words,4)==kIOReturnSuccess && words[0]==0x70970001u && words[1]==3 && words[2]==0 && words[3]==0x1234);
        CHECK(s.vm.headWords(m,0x801ff8,words,4)==kIOReturnSuccess && words[2]==0 && words[3]==0);   /* the object's end */
        CHECK(s.vm.headWords(m,0x810000,words,4)==kIOReturnBadArgument && s.vm.headWords(m+7,0x800000,words,4)==kIOReturnBadArgument);
        CHECK(s.vm.headWords(y,0x810000,words,4)==kIOReturnUnsupported);   /* no reader */
        static std::vector<uint64_t> cached,dram;cached.clear();dram.clear();
        native_adreno_dram_read32=[](uint64_t physical,uint32_t *value) {dram.push_back(physical);*value=0xd0000000u;return 0;};
        CHECK(s.vm.headWords(y,0x810ff8,words,4)==kIOReturnSuccess && dram.size()==4 && words[3]==0xd0000000u);
        /* The words cross from the first page into the second, which is elsewhere in DRAM. */
        CHECK(dram[0]==system.spans()[0].physical+0xff8 && dram[1]==dram[0]+4 && dram[2]==system.spans()[1].physical && dram[3]==dram[2]+4);
        native_adreno_cached_read32=[](uint64_t physical,uint32_t *value) {cached.push_back(physical);*value=0xc0000000u;return 0;};
        dram.clear();
        CHECK(s.vm.headWords(y,0x810000,words,4)==kIOReturnSuccess && cached.size()==4 && dram.empty() && words[0]==0xc0000000u);
        native_adreno_dram_read32=nullptr;native_adreno_cached_read32=nullptr;
        CHECK(s.vm.unmap(m)==kIOReturnSuccess && s.vm.unmap(y)==kIOReturnSuccess);
        CHECK(mapped.release()==kIOReturnSuccess && system.release()==kIOReturnSuccess);
        teardown(s);
    }
    std::printf("PASS: synchronized driver buffers left out of map, submission and unmap; gpu-maint-all and the audit's distrust restore "
        "their maintenance; passes visit only mappings with work (304 objects, 3 visited); system memory by attribute; imports with and "
        "without IO coherence; the synchronized audit's pages; the submission log's readers\n");
}
/* Physical-page maintenance (imports, system memory): record it; no host page is touched. */
IOReturn NativeAdrenoSetPagesCacheMode(const NativeAdrenoPhysicalSpan *spans,size_t count,bool){return spans && count?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoReassertWriteCombined(const NativeAdrenoPhysicalSpan *spans,size_t count,uint32_t *reset)
{return spans && count && reset?kIOReturnSuccess:kIOReturnBadArgument;}
IOReturn NativeAdrenoMaintainPhysicalPages(uint64_t physical,size_t bytes,NativeAdrenoCacheOperation op)
{
    if(!physical || !bytes || ((physical|bytes)&4095) || op>NativeAdrenoInvalidate)std::abort();
    ++fixture.physical_calls;fixture.physical_bytes+=bytes;return kIOReturnSuccess;
}
void NativeAdrenoNotePhase(unsigned,size_t) {}
bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
extern "C" unsigned int native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
