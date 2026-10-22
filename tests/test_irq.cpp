/* SPDX-License-Identifier: MIT */
/* Real interrupt decode over a software register peer. No interrupt taken. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoIrq.h"
#include <cstdio>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
struct Peer {
    uint32_t regs[0x10000]{};
    unsigned reads=0,writes=0,barriers=0,fail_read=0,fail_write=0,fail_barrier=0;
    uint32_t cleared=0xffffffff,last_write_reg=0;
};
static Peer peer;
static int write32(void *c,uint32_t reg,uint32_t v)
{
    CHECK(c==&peer && reg<0x10000);++peer.writes;
    if(peer.writes==peer.fail_write)return 1;
    peer.regs[reg]=v;peer.last_write_reg=reg;
    if(reg==0x0037)peer.cleared=v;
    return 0;
}
static int read32(void *c,uint32_t reg,uint32_t *v)
{
    CHECK(c==&peer && v && reg<0x10000);++peer.reads;
    if(peer.reads==peer.fail_read)return 1;
    *v=peer.regs[reg];return 0;
}
static int barrier(void *c){CHECK(c==&peer);++peer.barriers;return peer.barriers==peer.fail_barrier;}
static NativeAdrenoRingIo io{&peer,write32,read32,barrier};
static void reset(void)
{
    peer=Peer{};peer.cleared=0xffffffff;
    for(auto &v:peer.regs)v=0;
}
int main()
{
    /* Mask programming matches the pinned A7XX enable set exactly. */
    {
        reset();NativeAdrenoIrq irq;CHECK(irq.attach(&io)==kIOReturnSuccess);
        uint32_t programmed=0;
        CHECK(irq.programMask(&programmed)==kIOReturnSuccess);
        CHECK(programmed==NativeAdrenoIrq::mask());
        CHECK(programmed==((1u<<1)|(1u<<6)|(1u<<7)|(1u<<8)|(1u<<9)|(1u<<15)|(1u<<17)|
            (1u<<20)|(1u<<22)|(1u<<23)|(1u<<24)|(1u<<25)|(1u<<28)|(1u<<29)));
        CHECK(peer.regs[0x0038]==programmed && irq.enabled()==programmed);
        CHECK(irq.detach()==kIOReturnSuccess && irq.detach()==kIOReturnNotReady);
    }
    /* A6XX (Adreno 610): A6XX_INT_MASK 0x03d0e242, and a CACHE_FLUSH_TS status still decodes as a fence. */
    {
        reset();NativeAdrenoIrq irq;CHECK(irq.attach(&io)==kIOReturnSuccess);
        CHECK(irq.setGeneration(8)==kIOReturnBadArgument && irq.setGeneration(6)==kIOReturnSuccess);
        uint32_t programmed=0;
        CHECK(irq.programMask(&programmed)==kIOReturnSuccess && programmed==NativeAdrenoIrq::maskA6xx());
        CHECK(programmed==0x03d0e242u && peer.regs[0x0038]==programmed);
        CHECK(irq.setGeneration(7)==kIOReturnBusy);
        peer.regs[0x0201]=1u<<20;
        NativeAdrenoIrqResult result;
        CHECK(irq.handle(&result)==kIOReturnSuccess && result.flags==NativeAdrenoIrqFence);
        CHECK(irq.detach()==kIOReturnSuccess);
    }
    /* Each class is decoded, acknowledged and classified. */
    {
        reset();NativeAdrenoIrq irq;CHECK(irq.attach(&io)==kIOReturnSuccess);
        NativeAdrenoIrqResult result;
        peer.regs[0x0201]=1u<<20;
        CHECK(irq.handle(&result)==kIOReturnSuccess);
        CHECK(peer.cleared==(1u<<20) && peer.last_write_reg==0x0037);
        CHECK(result.flags==NativeAdrenoIrqFence && !(result.flags&NativeAdrenoIrqFatal));
        peer.regs[0x0201]=1u<<0;
        CHECK(irq.handle(&result)==kIOReturnSuccess && result.flags==NativeAdrenoIrqIdle);
        peer.regs[0x0201]=1u<<8;
        CHECK(irq.handle(&result)==kIOReturnSuccess && result.flags==NativeAdrenoIrqPreempt);
        peer.regs[0x0201]=1u<<23;
        CHECK(irq.handle(&result)==kIOReturnSuccess &&
            result.flags==(NativeAdrenoIrqHang|NativeAdrenoIrqFatal));
        peer.regs[0x0201]=1u<<24;
        CHECK(irq.handle(&result)==kIOReturnSuccess &&
            result.flags==(NativeAdrenoIrqUche|NativeAdrenoIrqFatal));
        peer.regs[0x0201]=(1u<<1)|(1u<<7)|(1u<<22);
        CHECK(irq.handle(&result)==kIOReturnSuccess &&
            result.flags==(NativeAdrenoIrqBusError|NativeAdrenoIrqFatal));
        peer.regs[0x0201]=(1u<<28)|(1u<<29);
        CHECK(irq.handle(&result)==kIOReturnSuccess &&
            result.flags==(NativeAdrenoIrqSecurity|NativeAdrenoIrqFatal));
        /* CP_HW_ERROR without a CP status bit is not a CP fault class. */
        peer.regs[0x0201]=1u<<9;peer.regs[0x0823]=0;
        CHECK(irq.handle(&result)==kIOReturnSuccess && !result.flags && !result.cp_status);
        peer.regs[0x0201]=1u<<9;peer.regs[0x0823]=0xf7;peer.regs[0x0824]=0x00100003;
        CHECK(irq.handle(&result)==kIOReturnSuccess);
        CHECK(result.flags==(NativeAdrenoIrqCpError|NativeAdrenoIrqFatal));
        CHECK(result.cp_status==0xf7 && result.protect_status==0x00100003);
        /* An unprogrammed source must never be silently ignored. */
        peer.regs[0x0201]=1u<<3;
        CHECK(irq.handle(&result)==kIOReturnSuccess &&
            result.flags==(NativeAdrenoIrqUnknown|NativeAdrenoIrqFatal) && result.unknown==(1u<<3));
        peer.regs[0x0201]=0;
        CHECK(irq.handle(&result)==kIOReturnSuccess && !result.flags && peer.cleared==0);
        CHECK(irq.detach()==kIOReturnSuccess);
    }
    /* Failures poison because the source may still be armed. */
    {
        reset();NativeAdrenoIrq irq;CHECK(irq.attach(&io)==kIOReturnSuccess);
        peer.fail_read=1;CHECK(irq.programMask(nullptr)==kIOReturnIOError && irq.poisoned());
        CHECK(irq.detach()==kIOReturnNotReady);
        reset();NativeAdrenoIrq second;CHECK(second.attach(&io)==kIOReturnSuccess);
        peer.fail_write=1;CHECK(second.programMask(nullptr)==kIOReturnIOError && second.poisoned());
        reset();NativeAdrenoIrq third;CHECK(third.attach(&io)==kIOReturnSuccess);
        peer.regs[0x0201]=1u<<23;
        peer.fail_write=1; /* clear write failure after a successful status read */
        NativeAdrenoIrqResult result;
        CHECK(third.handle(&result)==kIOReturnIOError && third.poisoned());
        reset();NativeAdrenoIrq fourth;CHECK(fourth.attach(&io)==kIOReturnSuccess);
        peer.regs[0x0201]=1u<<9;peer.regs[0x0823]=1;
        peer.fail_read=2; /* CP status read failure */
        CHECK(fourth.handle(&result)==kIOReturnIOError && fourth.poisoned());
        reset();NativeAdrenoIrq fifth;CHECK(fifth.attach(&io)==kIOReturnSuccess);
        peer.fail_barrier=1;
        CHECK(fifth.handle(&result)==kIOReturnIOError && fifth.poisoned());
        CHECK(fifth.detach()==kIOReturnNotReady);
        CHECK(irq.detach()==kIOReturnNotReady && second.detach()==kIOReturnNotReady);
    }
    std::printf("PASS: 1 exact A7XX mask programming, 10 decoded interrupt/status cases, 5 poison-on-unacknowledged cases over a software register peer\n");
}
