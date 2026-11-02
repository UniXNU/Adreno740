/* SPDX-License-Identifier: MIT */
/* Real ring/CP/IRQ/timer sequencing over software register peers; the board
 * power/GMU/SMMU steps are fakes that only record their own calls. */
#include "XnuMemoryFixture.h"
#include "NativeAdrenoGpu.h"
#include <cstdio>
#include <cstring>
#include <vector>
#define CHECK(x) do {if(!(x)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
struct Board {
    std::vector<const char *> order;
    unsigned fail_call=0,quiesce_fail=0,quiesce_calls=0;
    unsigned calls=0;
};
static Board board;
static int step(const char *name)
{
    ++board.calls;board.order.push_back(name);
    return board.calls==board.fail_call?kIOReturnIOError:0;
}
static int reserve(void *c){CHECK(c==&board);return step("reserve");}
static int bootGmu(void *c){CHECK(c==&board);return step("gmu");}
static int programSmmu(void *c){CHECK(c==&board);return step("smmu");}
static int quiesce(void *c)
{
    CHECK(c==&board);++board.quiesce_calls;board.order.push_back("quiesce");
    return board.quiesce_fail;
}
static NativeAdrenoGpuBoard ops{&board,reserve,bootGmu,programSmmu,quiesce};
struct Registers {
    std::vector<uint32_t> values;
    std::vector<uint32_t> writes;
    unsigned publish=0,acquire=0,barriers=0;
    Registers():values(0x10000,0){}
};
static Registers regs;
static uint32_t ringmem[8192],fencemem[1024];
static int write32(void *c,uint32_t reg,uint32_t value)
{CHECK(c==&regs && reg<regs.values.size());regs.values[reg]=value;regs.writes.push_back(reg);return 0;}
static int read32(void *c,uint32_t reg,uint32_t *value)
{CHECK(c==&regs && value && reg<regs.values.size());*value=regs.values[reg];return 0;}
static int barrier(void *c){CHECK(c==&regs);++regs.barriers;return 0;}
static int publish(void *c,const volatile uint32_t *words,size_t bytes)
{CHECK(c==&regs && words && bytes==sizeof(ringmem));++regs.publish;return 0;}
static int acquire(void *c,volatile uint32_t *words,size_t bytes)
{CHECK(c==&regs && words && bytes==sizeof(fencemem));++regs.acquire;return 0;}
static int timeRead(void *c,uint32_t address,uint32_t *value)
{CHECK(c==&regs && value && address<regs.values.size());*value=regs.values[address];return 0;}
static unsigned start_pending=0;
static uint64_t clock_us=0;
static uint64_t now(void *c){CHECK(c==&board);return clock_us;}
static int idle(void *c)
{
    CHECK(c==&board);clock_us+=1000;
    if(start_pending) {fencemem[1]=start_pending;start_pending=0;}
    return 0;
}
static NativeAdrenoGpuConfig config()
{
    NativeAdrenoGpuConfig c{};
    c.mmio={&regs,write32,read32,barrier,nullptr,nullptr};
    c.cache={&regs,publish,acquire};
    c.time={&regs,timeRead};
    c.clock={&board,now,idle};
    c.start_timeout_us=5000;
    c.ring=ringmem;c.ring_bytes=sizeof(ringmem);c.ring_iova=0x200000;
    c.fence=fencemem;c.fence_bytes=sizeof(fencemem);c.fence_iova=0x208000;
    c.sqe_iova=0x300000;c.pwrup_reglist_iova=0x400000;
    return c;
}
static void reset(void)
{
    board=Board{};regs=Registers{};
    std::memset(ringmem,0xee,sizeof(ringmem));std::memset(fencemem,0,sizeof(fencemem));
    start_pending=1;clock_us=0;
}
int main()
{
    /* Happy path: fixed order, real ring/CP/IRQ programming, ready. */
    {
        reset();NativeAdrenoGpu gpu;const NativeAdrenoGpuConfig c=config();
        CHECK(gpu.start(&ops,&c)==kIOReturnSuccess);
        CHECK(gpu.ready() && gpu.stepsCompleted()==NativeAdrenoGpuSteps);
        CHECK(gpu.startFence()==1 && !gpu.quiesced());
        CHECK(fencemem[1]==1); /* the CP init fence was observed */
        CHECK(board.order.size()==3 && !std::strcmp(board.order[0],"reserve") &&
            !std::strcmp(board.order[1],"gmu") && !std::strcmp(board.order[2],"smmu"));
        CHECK(gpu.ring() && gpu.interrupts() && gpu.timer());
        CHECK(regs.values[0x0800]==0x200000 && regs.values[0x0804]==0x208000);
        CHECK(regs.values[0x0802]==((12u)|(2u<<8)));
        CHECK((ringmem[2]>>16&0x7fu)==0x48 && ringmem[3]==0x14b);
        CHECK(regs.values[0x0830]==0x300000 && regs.values[0x0038]==NativeAdrenoIrq::mask());
        CHECK(regs.values[0x0807]==17);
        CHECK(gpu.start(&ops,&c)==kIOReturnBusy);
        CHECK(gpu.stop()==kIOReturnSuccess && !gpu.ready());
        CHECK(board.quiesce_calls==1); /* only the explicit stop */
        CHECK(gpu.stop()==kIOReturnNotReady);
    }
    /* Each board step failure stops the sequence and quiesces exactly once. */
    {
        for(unsigned failure=1;failure<=3;++failure) {
            reset();board.fail_call=failure;
            NativeAdrenoGpu gpu;const NativeAdrenoGpuConfig c=config();
            CHECK(gpu.start(&ops,&c)==kIOReturnIOError);
            CHECK(gpu.failed() && !gpu.ready() && gpu.stepsCompleted()==failure-1);
            CHECK(board.quiesce_calls==(failure==1?0:1));
            CHECK(board.order.size()==(failure==1?1:failure+1));
            CHECK(!gpu.ring() && !gpu.interrupts() && gpu.stop()==kIOReturnNotReady);
            if(failure>1)CHECK(!std::strcmp(board.order.back(),"quiesce"));
            CHECK(regs.writes.empty());
        }
    }
    /* Real component failures quarantine and never retry. */
    {
        reset();board.fail_call=0;NativeAdrenoGpu gpu;NativeAdrenoGpuConfig c=config();
        c.ring_iova=0x201000; /* not ring-aligned */
        CHECK(gpu.start(&ops,&c)==kIOReturnBadArgument);
        CHECK(gpu.failed() && gpu.quiesced() && board.quiesce_calls==1);
        CHECK(!gpu.ready() && board.order.size()==4 && !std::strcmp(board.order[3],"quiesce"));
        const NativeAdrenoGpuConfig again=config();
        CHECK(gpu.start(&ops,&again)==kIOReturnBusy);
    }
    {
        reset();NativeAdrenoGpu gpu;NativeAdrenoGpuConfig c=config();
        c.sqe_iova=0x300001; /* CP init refuses an unaligned SQE base */
        CHECK(gpu.start(&ops,&c)==kIOReturnBadArgument);
        CHECK(gpu.failed() && gpu.quiesced() && board.order.size()==4);
        CHECK(regs.values[0x0800]==0x200000); /* ring was attached before failing */
    }
    {
        reset();board.quiesce_fail=1;NativeAdrenoGpu gpu;NativeAdrenoGpuConfig c=config();
        c.ring_bytes=4096; /* refused by the ring before CP start */
        CHECK(gpu.start(&ops,&c)==kIOReturnBadArgument);
        CHECK(gpu.failed() && !gpu.quiesced() && board.quiesce_calls==1);
        CHECK(!gpu.ready());
        CHECK(board.order.size()==4 && !std::strcmp(board.order[3],"quiesce"));
    }
    /* A CP start fence that never completes is a timeout, not readiness. */
    {
        reset();start_pending=0;board.fail_call=0;
        NativeAdrenoGpu gpu;const NativeAdrenoGpuConfig c=config();
        CHECK(gpu.start(&ops,&c)==kIOReturnTimeout);
        CHECK(gpu.failed() && gpu.quiesced() && board.quiesce_calls==1);
        CHECK(!gpu.ready() && gpu.startFence()==1); /* issued but never observed */
        CHECK(regs.values[0x0807]==17); /* ME_INIT was submitted */
    }
    /* Argument refusals precede reserve and never touch the board. */
    {
        reset();NativeAdrenoGpu gpu;const NativeAdrenoGpuConfig c=config();
        CHECK(gpu.start(nullptr,&c)==kIOReturnBadArgument);
        CHECK(gpu.start(&ops,nullptr)==kIOReturnBadArgument);
        NativeAdrenoGpuBoard broken=ops;broken.quiesce=nullptr;
        CHECK(gpu.start(&broken,&c)==kIOReturnBadArgument);
        NativeAdrenoGpuConfig bad=c;bad.cache.publish=nullptr;
        CHECK(gpu.start(&ops,&bad)==kIOReturnBadArgument);
        bad=c;bad.ring=nullptr;
        CHECK(gpu.start(&ops,&bad)==kIOReturnBadArgument);
        CHECK(board.calls==0 && board.quiesce_calls==0 && !gpu.failed());
        /* Validation failures do not consume the one attempt. */
        CHECK(gpu.start(&ops,&c)==kIOReturnSuccess && gpu.ready());
    }
    /* Optional timer absent, stop refuses while a submission is outstanding. */
    {
        reset();NativeAdrenoGpu gpu;NativeAdrenoGpuConfig c=config();c.time.read32=nullptr;
        CHECK(gpu.start(&ops,&c)==kIOReturnSuccess && gpu.timer()==nullptr);
        const uint32_t one=1;uint32_t fence=0;
        CHECK(gpu.ring()->submit(&one,1,&fence)==kIOReturnSuccess);
        CHECK(gpu.stop()==kIOReturnBusy);
        fencemem[1]=fence;
        CHECK(gpu.ring()->wait(fence,nullptr,0,nullptr)==NativeAdrenoRingFenceComplete);
        CHECK(gpu.stop()==kIOReturnSuccess);
    }
    std::printf("PASS: 1 full lifecycle start/stop, 3 board-step failures, 3 real component failures and 6 argument refusals over software GPU registers and fake board power\n");
}
