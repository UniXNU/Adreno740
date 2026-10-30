/* SPDX-License-Identifier: MIT */
/* Real ring/fence code over a software CP register peer. No GPU access. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoRing.h"
#include <cstdio>
#include <cstring>
#include <vector>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
struct Peer {
    std::vector<uint32_t> regs;
    std::vector<std::pair<uint32_t,uint32_t>> writes;
    unsigned read_calls=0,publish=0,acquire=0,barriers=0;
    unsigned fail_write=0,fail_read=0,fail_barrier=0,fail_publish=0,fail_acquire=0;
    bool log_order=false;std::vector<const char *> order;
    Peer():regs(0x10000,0){}
};
static Peer peer;
static uint64_t clock_us=0;
static int write32(void *cookie,uint32_t reg,uint32_t value)
{
    CHECK(cookie==&peer && reg<peer.regs.size());
    if(peer.writes.size()+1==peer.fail_write)return 1;
    peer.regs[reg]=value;peer.writes.emplace_back(reg,value);
    if(peer.log_order)peer.order.push_back("write");
    return 0;
}
static int read32(void *cookie,uint32_t reg,uint32_t *value)
{
    CHECK(cookie==&peer && value && reg<peer.regs.size());++peer.read_calls;
    if(peer.read_calls==peer.fail_read)return 1;
    *value=peer.regs[reg];return 0;
}
static int barrier(void *cookie){CHECK(cookie==&peer);++peer.barriers;
    if(peer.barriers==peer.fail_barrier)return 1;
    if(peer.log_order)peer.order.push_back("barrier");return 0;}
static int publish(void *cookie,const volatile uint32_t *words,size_t bytes)
{
    CHECK(cookie==&peer && words && bytes==32768);++peer.publish;
    if(peer.log_order)peer.order.push_back("publish");
    return peer.publish==peer.fail_publish;
}
static int acquire(void *cookie,volatile uint32_t *words,size_t bytes)
{
    CHECK(cookie==&peer && words && bytes==4096);++peer.acquire;
    return peer.acquire==peer.fail_acquire;
}
static uint64_t now(void *cookie){CHECK(cookie==&peer);return clock_us;}
static int idle(void *cookie){CHECK(cookie==&peer);clock_us+=1000;return 0;}
static uint32_t ring[8192];static uint32_t fence[1024];
static NativeAdrenoRingIo io{&peer,write32,read32,barrier,nullptr,nullptr};
/* The GMU's AHB fence: the next `dropped` WPTR writes are flagged as dropped; a clear resets the flag. */
static unsigned dropped=0,fence_reads=0,fence_clears=0;static bool flagged=false;
static int fence_status(void *cookie,int clear,uint32_t *status)
{
    CHECK(cookie==&peer && status);
    if(clear){++fence_clears;flagged=false;return 0;}
    ++fence_reads;
    if(!flagged && dropped){--dropped;flagged=true;}
    *status=flagged?1u:0u;
    if(flagged && !dropped)flagged=false;   /* the next read after the last drop sees the write land */
    return 0;
}
static NativeAdrenoRingCache cache{&peer,publish,acquire};
static NativeAdrenoRingClock clock_{&peer,now,idle};
static void reset(void)
{
    peer=Peer{};clock_us=0;
    for(auto &v:ring)v=0xdeadbeef;
    for(auto &v:fence)v=0xa5a5a5a5;
}
static int attach(NativeAdrenoRing &r)
{return r.attach(&io,&cache,ring,sizeof(ring),0x100000,fence,sizeof(fence),0x108000);}
int main()
{
    /* Exact register programming and one complete submission. */
    {
        reset();NativeAdrenoRing r;
        CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t expected[][2]={
            {0x0800,0x100000},{0x0801,0},{0x0802,(12u)|(2u<<8)},
            {0x0804,0x108000},{0x0805,0},{0x0807,0},{0x0885,0}};
        CHECK(peer.writes.size()==sizeof(expected)/sizeof(expected[0]));
        for(size_t i=0;i<peer.writes.size();++i)
            CHECK(peer.writes[i].first==expected[i][0] && peer.writes[i].second==expected[i][1]);
        CHECK(peer.read_calls==1 && r.readPointer()==0 && r.outstanding()==0);
        const uint32_t words[4]={0x11111111,0x22222222,0x33333333,0x44444444};
        uint32_t end=0;
        peer.log_order=true;
        CHECK(r.submitFramed(words,4,&end)==kIOReturnSuccess && end==1 && r.outstanding()==1);
        CHECK(peer.writes.back().first==0x0807 && peer.writes.back().second==46);
        /* a7xx_submit() framing: BR sync, the BR-only GPU cache flush/invalidate (CACHE_FLUSH7, CACHE_INVALIDATE7, a one-word
         * CP_WAIT_FOR_IDLE), both threads and IFPC off around the user words, then the fence epilogue. */
        auto op=[](uint32_t header) {return (header>>28)==7u?(header>>16)&0x7fu:0xffu;};
        CHECK(op(ring[0])==0x17 && ring[1]==0x80000001u);
        CHECK(op(ring[2])==0x46 && ring[3]==0x32 && op(ring[4])==0x46 && ring[5]==0x33 && op(ring[6])==0x26 && (ring[6]&0x3fffu)==0);
        CHECK(op(ring[7])==0x17 && ring[8]==3 && op(ring[9])==0x65 && ring[10]==0x101);
        for(unsigned i=0;i<4;++i)CHECK(ring[11+i]==words[i]);
        CHECK((ring[15]>>8&0x3ffffu)==0x885 && (ring[15]&0x3fu)==1 && ring[16]==1);
        CHECK(op(ring[17])==0x17 && ring[18]==1 && op(ring[19])==0x46 && ring[20]==0x18 && op(ring[21])==0x46 && ring[22]==0x19);
        CHECK(op(ring[23])==0x17 && ring[24]==2);
        CHECK(op(ring[25])==0x46 && ring[26]==(0x31u|(1u<<27)) && ring[27]==0x10800c && ring[28]==0 && ring[29]==1);   /* BV timestamp */
        CHECK(op(ring[30])==0x17 && ring[31]==1);
        CHECK(op(ring[32])==0x14 && ring[33]==0 && ring[34]==0x10800c && ring[35]==0 && ring[36]==1);   /* BR waits for BV */
        CHECK(op(ring[37])==0x46 && ring[38]==(0x31u|(1u<<27)|(1u<<31)));
        CHECK(ring[39]==0x108004 && ring[40]==0 && ring[41]==1);   /* msm_rbmemptrs.fence, where wait() reads */
        CHECK(op(ring[42])==0x17 && ring[43]==3 && op(ring[44])==0x65 && ring[45]==0x100);
        CHECK(peer.publish==1);
        /* Cache publication precedes the barrier, which precedes WPTR. */
        CHECK(peer.order.size()==3 && !std::strcmp(peer.order[0],"publish") &&
            !std::strcmp(peer.order[1],"barrier") && !std::strcmp(peer.order[2],"write"));
        uint32_t observed=0;
        CHECK(r.wait(1,&clock_,5000,&observed)==NativeAdrenoRingFenceTimeout && !observed);
        fence[1]=1;
        CHECK(r.wait(1,&clock_,5000,&observed)==NativeAdrenoRingFenceComplete &&
            observed==1 && r.outstanding()==0 && r.completedFence()==1);
        CHECK(r.wait(2,&clock_,5000,&observed)==NativeAdrenoRingFenceArgument);
        CHECK(r.detach()==kIOReturnSuccess);
    }
    /* Timeline stamps: the CP's always-on counter is read into the fence page before the user words and after them, and read back by fence. */
    {
        reset();NativeAdrenoRing r;
        CHECK(attach(r)==kIOReturnSuccess);
        r.setTimeline(true);
        const uint32_t words[4]={0x11111111,0x22222222,0x33333333,0x44444444};
        uint32_t end=0;
        CHECK(r.submitFramed(words,4,&end)==kIOReturnSuccess && end==1);
        CHECK(peer.writes.back().first==0x0807 && peer.writes.back().second==46+8);
        auto op=[](uint32_t header) {return (header>>28)==7u?(header>>16)&0x7fu:0xffu;};
        CHECK(op(ring[6])==0x26 && op(ring[7])==0x3e && ring[8]==(0x980u|(2u<<18)|(1u<<30)) && ring[9]==0x108000u+512+16 && ring[10]==0);
        CHECK(op(ring[11])==0x17 && ring[12]==3 && op(ring[13])==0x65 && ring[14]==0x101);
        for(unsigned i=0;i<4;++i)CHECK(ring[15+i]==words[i]);
        CHECK(op(ring[21])==0x17 && ring[22]==1 && op(ring[23])==0x3e && ring[24]==(0x980u|(2u<<18)|(1u<<30)) && ring[25]==0x108000u+512+16+8 && ring[26]==0);
        CHECK(op(ring[27])==0x46 && ring[28]==0x18);
        uint64_t start=0,stop=0;
        fence[132]=1000;fence[133]=0;fence[134]=1700;fence[135]=0;
        CHECK(r.timeline(1,&start,&stop)==31 && start==1000 && stop==1700);
        fence[134]=0;fence[135]=0;
        CHECK(r.timeline(1,&start,&stop)==31 && start==1000 && stop==0);
        NativeAdrenoRing off;
        CHECK(attach(off)==kIOReturnSuccess && off.timeline(1,&start,&stop)==5);
    }
    /* A requested context reset puts a6xx_set_pagetable()'s A7XX context entry into the next framed prologue, once. */
    {
        reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t words[1]={0x11111111};
        uint32_t end=0;
        r.requestContextReset();
        CHECK(r.submitFramed(words,1,&end)==kIOReturnSuccess && end==1);
        CHECK(peer.writes.back().first==0x0807 && peer.writes.back().second==11+26+1+31);
        auto op=[](uint32_t header) {return (header>>28)==7u?(header>>16)&0x7fu:0xffu;};
        CHECK(op(ring[0])==0x17 && ring[1]==0x80000001u);
        CHECK(op(ring[2])==0x14 && ring[3]==0 && ring[4]==0x108004 && ring[5]==0 && ring[6]==0);   /* wait for the last fence */
        CHECK(op(ring[7])==0x17 && ring[8]==0x80000003u && op(ring[9])==0x1f && ring[10]==0xf);
        CHECK(op(ring[11])==0x17 && ring[12]==0x80000003u && op(ring[13])==0x46 && ring[14]==0x26);
        CHECK(op(ring[15])==0x17 && ring[16]==0x80000001u && (ring[17]>>8&0x3ffffu)==0x50e && ring[18]==1);
        CHECK(op(ring[19])==0x46 && ring[20]==0x31);
        CHECK(op(ring[21])==0x3c && ring[22]==3 && ring[23]==0x50f && ring[24]==0 && ring[25]==1 && ring[26]==1 && ring[27]==0);
        CHECK(op(ring[28])==0x46 && ring[29]==0x32 && op(ring[30])==0x46 && ring[31]==0x33 && op(ring[32])==0x26);
        CHECK(op(ring[33])==0x17 && ring[34]==3 && op(ring[35])==0x65 && ring[36]==0x101 && ring[37]==words[0]);
        fence[1]=1;
        CHECK(r.submitFramed(words,1,&end)==kIOReturnSuccess && end==2);
        CHECK(peer.writes.back().second==69+11+1+31 && op(ring[69+2])==0x46 && op(ring[69+7])==0x17 && ring[69+8]==3);
    }
    /* A change of submitting client is a context switch: the next framed prologue carries the context entry, the same client's
     * next one does not. */
    {
        reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t words[1]={0x11111111};
        uint32_t end=0;
        r.useContext(1);
        CHECK(r.submitFramed(words,1,&end)==kIOReturnSuccess && peer.writes.back().second==11+26+1+31);
        fence[1]=1;r.useContext(1);
        CHECK(r.submitFramed(words,1,&end)==kIOReturnSuccess && peer.writes.back().second==69+11+1+31);
        fence[1]=2;r.useContext(2);
        CHECK(r.submitFramed(words,1,&end)==kIOReturnSuccess && peer.writes.back().second==112+11+26+1+31);
    }
    /* WPTR is a fenced write: a dropped write is given again until the GMU reports it landed. */
    {
        reset();NativeAdrenoRing r;
        NativeAdrenoRingIo fenced=io;fenced.fence_cookie=&peer;fenced.fence_status=fence_status;
        CHECK(r.attach(&fenced,&cache,ring,sizeof(ring),0x100000,fence,sizeof(fence),0x108000)==kIOReturnSuccess);
        const uint32_t one[1]={0xabc};uint32_t end=0;
        size_t before=0;for(auto &w:peer.writes)before+=w.first==0x0807;
        dropped=3;fence_reads=0;
        CHECK(r.submit(one,1,&end)==kIOReturnSuccess);
        size_t after=0;for(auto &w:peer.writes)after+=w.first==0x0807;
        CHECK(after-before==4 && fence_clears==3 && fence_reads==4);   /* the first write and three repeats */
        dropped=0;CHECK(r.kick()==kIOReturnSuccess);
    }
    /* Wrap-around emission keeps packets contiguous modulo the ring. */
    {
        reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t big[1]={0xabc};
        uint32_t end=0;
        CHECK(r.submit(big,1,&end)==kIOReturnSuccess);
        CHECK(r.submit(nullptr,0,&end)==kIOReturnBadArgument);
        /* Re-attach is refused while the object is live. */
        CHECK(attach(r)==kIOReturnBusy);
        CHECK(r.detach()==kIOReturnBusy); /* one unobserved submission */
    }
    {
        reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
        uint32_t words[sizeof(ring)/4-53];
        for(auto &w:words)w=0x5a5a5a5a;
        uint32_t end=0;
        CHECK(r.submitFramed(words,sizeof(words)/sizeof(words[0]),&end)==kIOReturnSuccess);
        /* Consume the whole submission, then a second one wraps past the end. */
        CHECK(end==1 && peer.writes.back().second==8181);
        peer.regs[0x0806]=8181;
        CHECK(r.submitFramed(words,4,&end)==kIOReturnSuccess);
        CHECK(peer.writes.back().first==0x0807 && peer.writes.back().second==35);
        CHECK(ring[8181]>=0x70000000u && ring[8191]==0x101);
        CHECK(ring[0]==words[0] && ring[1]==words[1] && ring[2]==words[2] && ring[3]==words[3]);
        CHECK((ring[4]>>8&0x3ffffu)==0x885 && ring[5]==2);
        CHECK(ring[28]==0x108004 && ring[30]==2 && ring[34]==0x100);
        CHECK(r.detach()==kIOReturnBusy); /* unobserved submissions */
    }
    /* Argument refusals never touch registers or memory. */
    {
        reset();NativeAdrenoRing r;
        CHECK(r.attach(nullptr,&cache,ring,sizeof(ring),0x100000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,nullptr,ring,sizeof(ring),0x100000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,nullptr,sizeof(ring),0x100000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,ring,4096,0x100000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,ring,16384+4096,0x100000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,ring,sizeof(ring),0x104000,fence,sizeof(fence),0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,ring,sizeof(ring),0x100000,fence,32,0x108000)==kIOReturnBadArgument);
        CHECK(r.attach(&io,&cache,ring,sizeof(ring),0x100000,fence,sizeof(fence),0x108004)==kIOReturnBadArgument);
        CHECK(peer.writes.empty() && peer.read_calls==0);
        CHECK(r.submit(nullptr,1,nullptr)==kIOReturnBadArgument);
        uint32_t end=0;CHECK(r.submit(nullptr,1,&end)==kIOReturnNotReady);
        CHECK(r.detach()==kIOReturnNotReady);
        CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t one=1;
        CHECK(r.submit(nullptr,1,&end)==kIOReturnBadArgument);
        CHECK(r.submit(&one,0,&end)==kIOReturnBadArgument);
        uint32_t huge[sizeof(ring)/4];CHECK(r.submit(huge,sizeof(huge)/sizeof(huge[0]),&end)==kIOReturnNoResources);
        CHECK(r.submit(&one,1,&end)==kIOReturnSuccess && end==1);
        CHECK(r.detach()==kIOReturnBusy);
        fence[1]=1;uint32_t observed=0;
        CHECK(r.wait(1,&clock_,1000,&observed)==NativeAdrenoRingFenceComplete);
        CHECK(r.detach()==kIOReturnSuccess);
    }
    /* Every attach register-write failure leaves the ring unusable and owned. */
    {
        unsigned cases=0;
        for(unsigned failure=1;failure<=7;++failure) {
            reset();NativeAdrenoRing r;peer.fail_write=failure;
            CHECK(attach(r)==kIOReturnIOError && r.poisoned());
            CHECK(r.detach()==kIOReturnBusy);
            uint32_t end=0;CHECK(r.submit(&failure,1,&end)==kIOReturnNotReady);
            CHECK(peer.writes.size()+1==failure);
            ++cases;
        }
        reset();NativeAdrenoRing broken;peer.fail_read=1;
        CHECK(attach(broken)==kIOReturnIOError && broken.poisoned() && broken.detach()==kIOReturnBusy);++cases;
        /* Publication, barrier and WPTR failures poison after partial commit. */
        for(unsigned kind=0;kind<3;++kind) {
            reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
            if(kind==0)peer.fail_publish=1;
            if(kind==1)peer.fail_barrier=1;
            if(kind==2)peer.fail_write=peer.writes.size()+1;
            const uint32_t one=0x1234;uint32_t end=0;
            CHECK(r.submit(&one,1,&end)==kIOReturnIOError && !end && r.poisoned());
            CHECK(r.outstanding()==0 && r.detach()==kIOReturnBusy);
            CHECK(fence[1]==0); /* never completed */
            ++cases;
        }
        reset();NativeAdrenoRing r;CHECK(attach(r)==kIOReturnSuccess);
        const uint32_t one=1;uint32_t end=0;
        CHECK(r.submit(&one,1,&end)==kIOReturnSuccess);
        peer.fail_acquire=peer.acquire+1;
        uint32_t observed=0;
        CHECK(r.wait(1,&clock_,10,&observed)==NativeAdrenoRingFencePoisoned && r.poisoned());++cases;
        std::printf("PASS: 1 exact ring programming and submission, 1 wrap submission, 1 refused re-attach, 7 write/1 read attach failures and 4 post-commit poison cases over a software CP register peer (%u failure cases)\n",
            cases);
    }
}
