/* SPDX-License-Identifier: MIT */
/* Real CP init sequence over a software CP register peer. No GPU start. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoCpInit.h"
#include <cstdio>
#include <cstring>
#include <vector>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
struct Peer {
    uint32_t regs[0x10000]{};
    std::vector<uint32_t> writes;
    unsigned publish=0,acquire=0,barriers=0,fail_write=0,fail_publish=0;
};
static Peer peer;
static uint32_t ringmem[8192],fencemem[1024];
static int write32(void *c,uint32_t reg,uint32_t v)
{
    CHECK(c==&peer && reg<0x10000);
    if(peer.writes.size()+1==peer.fail_write)return 1;
    peer.regs[reg]=v;peer.writes.push_back(reg);return 0;
}
static int read32(void *c,uint32_t reg,uint32_t *v)
{CHECK(c==&peer && v);*v=peer.regs[reg];return 0;}
static int barrier(void *c){CHECK(c==&peer);++peer.barriers;return 0;}
static int publish(void *c,const volatile uint32_t *w,size_t bytes)
{CHECK(c==&peer && w && bytes==32768);++peer.publish;return peer.publish==peer.fail_publish;}
static int acquire(void *c,volatile uint32_t *w,size_t bytes)
{CHECK(c==&peer && w && bytes==4096);++peer.acquire;return 0;}
static NativeAdrenoRingIo io{&peer,write32,read32,barrier};
static NativeAdrenoRingCache cache{&peer,publish,acquire};
static NativeAdrenoRingClock clock_{&peer,nullptr,nullptr};
static void reset(void)
{
    peer=Peer{};
    std::memset(ringmem,0xee,sizeof(ringmem));std::memset(fencemem,0,sizeof(fencemem));
}
int main()
{
    /* Exact SQE base and CP_ME_INIT sequence through the ring. */
    {
        reset();NativeAdrenoRing ring;CHECK(ring.attach(&io,&cache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000)==kIOReturnSuccess);
        NativeAdrenoCpInit init;
        NativeAdrenoCpInitConfig config{0x200000,0x300000,0};
        uint32_t fence=0;
        CHECK(init.run(&io,&ring,&config,&fence)==kIOReturnSuccess && fence==1 && init.completed());
        bool paired=false;
        for(size_t i=0;i+1<peer.writes.size();++i)
            if(peer.writes[i]==0x0830 && peer.writes[i+1]==0x0831)paired=true;
        CHECK(paired && peer.regs[0x0830]==0x200000 && peer.regs[0x0831]==0);
        CHECK((ringmem[0]>>16&0x7fu)==0x17 && (ringmem[0]&0x3fffu)==1);
        CHECK(ringmem[1]==(1u<<27));
        CHECK((ringmem[2]>>16&0x7fu)==0x48 && (ringmem[2]&0x3fffu)==7);
        CHECK(ringmem[3]==NativeAdrenoCpInit::meInitMask() && ringmem[3]==0x14b);
        CHECK(ringmem[4]==3 && ringmem[5]==0x20000000 && ringmem[6]==2);
        CHECK(ringmem[7]==0x300000 && ringmem[8]==0 && ringmem[9]==(1u<<31));
        CHECK(peer.regs[0x0807]==17); /* ten init words plus the seven-word fence */
        CHECK(init.run(&io,&ring,&config,&fence)==kIOReturnNotReady);
        fencemem[1]=1;
        CHECK(ring.wait(1,&clock_,1000,nullptr)==NativeAdrenoRingFenceComplete);
        CHECK(ring.detach()==kIOReturnSuccess);
    }
    /* A6XX (a6xx_cp_init): CP_ME_INIT alone, eight words, no CP_THREAD_CONTROL and no power-up list required. */
    {
        reset();NativeAdrenoRing ring;CHECK(ring.setGeneration(6)==kIOReturnSuccess);
        CHECK(ring.attach(&io,&cache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000)==kIOReturnSuccess);
        NativeAdrenoCpInit init;
        NativeAdrenoCpInitConfig config{0x200000,0,1,6};
        peer.regs[0x0830]=0x200000;peer.regs[0x0831]=0;
        uint32_t fence=0;
        CHECK(init.run(&io,&ring,&config,&fence)==kIOReturnSuccess && fence==1 && init.completed());
        CHECK((ringmem[0]>>16&0x7fu)==0x48 && (ringmem[0]&0x3fffu)==8);
        CHECK(ringmem[1]==0x2f && ringmem[2]==3 && ringmem[3]==0x20000000);
        for(unsigned i=4;i<9;++i)CHECK(ringmem[i]==0);
        CHECK((ringmem[9]>>8&0x3ffffu)==0x885 && ringmem[10]==1);
        CHECK((ringmem[11]>>16&0x7fu)==0x46 && ringmem[12]==(0x04u|(1u<<31)));    /* the A6XX CACHE_FLUSH_TS fence */
        CHECK(peer.regs[0x0807]==9+7);
        NativeAdrenoCpInit other;NativeAdrenoCpInitConfig bad{0x200000,0,1,5};
        CHECK(other.run(&io,&ring,&bad,&fence)==kIOReturnBadArgument);
    }
    /* Refusals precede MMIO and ring writes. */
    {
        reset();NativeAdrenoRing ring;CHECK(ring.attach(&io,&cache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000)==kIOReturnSuccess);
        NativeAdrenoCpInit init;uint32_t fence=0;
        const size_t attached_writes=peer.writes.size();
        NativeAdrenoCpInitConfig good{0x200000,0x300000,0},zero{0,0x300000,0},
            unaligned{0x200001,0x300000,0},high{1ull<<48,0x300000,0},list{0x200000,0x300002,0};
        CHECK(init.run(nullptr,&ring,&good,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,nullptr,&good,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,nullptr,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,&good,nullptr)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,&zero,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,&unaligned,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,&high,&fence)==kIOReturnBadArgument);
        CHECK(init.run(&io,&ring,&list,&fence)==kIOReturnBadArgument);
        CHECK(peer.writes.size()==attached_writes && peer.regs[0x0807]==0 && !init.completed());
        /* A failed SQE write poisons without submitting the ring. */
        peer.fail_write=peer.writes.size()+2; /* second SQE write */
        CHECK(init.run(&io,&ring,&good,&fence)==kIOReturnIOError && init.poisoned() && !fence);
        CHECK(peer.regs[0x0807]==0 && peer.writes.size()==8);
        CHECK(init.run(&io,&ring,&good,&fence)==kIOReturnNotReady);
        CHECK(ring.detach()==kIOReturnSuccess); /* no submission was committed */
    }
    /* A ring commit failure poisons the initializer instead of claiming start. */
    {
        reset();NativeAdrenoRing ring;CHECK(ring.attach(&io,&cache,ringmem,sizeof(ringmem),0x100000,fencemem,sizeof(fencemem),0x108000)==kIOReturnSuccess);
        NativeAdrenoCpInit init;NativeAdrenoCpInitConfig config{0x200000,0x300000,0};uint32_t fence=0;
        peer.fail_publish=1;
        CHECK(init.run(&io,&ring,&config,&fence)==kIOReturnIOError && init.poisoned());
        CHECK(ring.poisoned() && ring.detach()==kIOReturnBusy);
    }
    std::printf("PASS: 1 exact CP_ME_INIT sequence with SQE base, 8 pre-MMIO refusals, 1 SQE write failure and 1 ring commit failure over a software CP register peer\n");
}
