/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoPreempt.h"
#include <IOKit/IOLib.h>

/* Pinned definitions, all from registers/adreno/a6xx.xml (dword register indices) and a6xx_preempt.c:
 *   CP_CONTEXT_SWITCH_CNTL 0x08a0, CP_CONTEXT_SWITCH_SMMU_INFO 0x08a1, CP_CONTEXT_SWITCH_PRIV_NON_SECURE_RESTORE_ADDR 0x08a3,
 *   RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE 0x8e50.
 * The control word: STOP at bit 0, LEVEL at bits 6..7, USES_GMEM at bit 8, SKIP_SAVE_RESTORE at bit 9 (a6xx_cp_context_switch_cntl).
 * a6xx_preempt_trigger writes LEVEL(1) | USES_GMEM | SKIP_SAVE_RESTORE | STOP with the incoming ring's SMMU info and record
 * already programmed, and keeps the GPU awake with GMU_PWR_COL_PREEMPT_KEEPALIVE until a6xx_preempt_irq finds STOP cleared.
 * A record's magic is A6XX_PREEMPT_RECORD_MAGIC 0xAE399D6E (a6xx_gpu.h) and the SMMU info's GEN7_CP_SMMU_INFO_MAGIC 0x241350d5. */
enum : uint32_t {
    PreemptCntl = 0x08a0,
    PreemptSmmuInfo = 0x08a1,
    PreemptRestoreAddr = 0x08a3,
    PreemptGmemSaveRestore = 0x8e50,
    CntlStop = 1u << 0,
    CntlLevel1 = 1u << 6,
    CntlUsesGmem = 1u << 8,
    CntlSkipSaveRestore = 1u << 9,
    RecordMagic = 0xAE399D6Eu,
    SmmuInfoMagic = 0x241350d5u,
    GmuPreemptKeepalive = 0x1f8c4,   /* a6xx_gmu.xml GMU_PWR_COL_PREEMPT_KEEPALIVE (dword offset in the GMU's window) */
    RecordCntlOffset = 16,      /* magic, info, errno, data, cntl */
    RecordRptrOffset = 20,
    RecordWptrOffset = 24,
    RecordRptrAddrOffset = 32,
    RecordRbaseOffset = 40,
    RecordCounterOffset = 48,
    RecordBvRptrAddrOffset = 56
};
static void put32(uint8_t *at,uint32_t value)
{
    at[0]=(uint8_t)value;at[1]=(uint8_t)(value>>8);at[2]=(uint8_t)(value>>16);at[3]=(uint8_t)(value>>24);
}
static void put64(uint8_t *at,uint64_t value)
{
    put32(at,(uint32_t)value);put32(at+4,(uint32_t)(value>>32));
}
static uint32_t get32(const uint8_t *at)
{
    return uint32_t(at[0])|uint32_t(at[1])<<8|uint32_t(at[2])<<16|uint32_t(at[3])<<24;
}
bool NativeAdrenoPreempt::writeRecord(uint8_t *record,uint64_t rbase,uint64_t rptr_addr,uint64_t bv_rptr_addr,uint32_t cntl,uint32_t wptr)
{
    if(!record || !rbase || !rptr_addr)return false;
    put32(record+0,RecordMagic);
    put32(record+4,0);              /* info: written by the CP */
    put32(record+8,0);              /* errno: written by the CP */
    put32(record+12,0);             /* data: the YIELD and SET_MARKER payload, written by the CP */
    put32(record+RecordCntlOffset,cntl);
    put32(record+RecordRptrOffset,0);
    put32(record+RecordWptrOffset,wptr);
    put32(record+28,0);
    put64(record+RecordRptrAddrOffset,rptr_addr);
    put64(record+RecordRbaseOffset,rbase);
    put64(record+RecordCounterOffset,0);   /* the preemption counters: 0 disables the block, as msm's A7XX rings do */
    put64(record+RecordBvRptrAddrOffset,bv_rptr_addr);
    return true;
}
uint32_t NativeAdrenoPreempt::recordWptr(const uint8_t *record)
{
    return record?get32(record+RecordWptrOffset):0;
}
bool NativeAdrenoPreempt::writeSmmuInfo(uint8_t *page,uint64_t ttbr0,uint32_t asid,uint32_t context_idr)
{
    if(!page || !ttbr0)return false;
    put32(page+0,SmmuInfoMagic);
    put32(page+4,0);
    put64(page+8,ttbr0);
    put32(page+16,asid);
    put32(page+20,context_idr);
    put32(page+24,0);               /* context_bank: the GPU's bank is 0 */
    return true;
}
IOReturn NativeAdrenoPreempt::attach(const NativeAdrenoPreemptIo *io,const NativeAdrenoPreemptRings &rings,
    uint32_t loaded,uint64_t (*now)(void *),void *clock)
{
    if(attached_)return kIOReturnBusy;
    if(!io || !io->write32 || !io->read32 || !io->barrier)return kIOReturnBadArgument;
    if(loaded>=NativeAdrenoPreemptRingCount)return kIOReturnBadArgument;
    for(uint32_t i=0;i<NativeAdrenoPreemptRingCount;++i)
        if(!rings.record[i] || !rings.smmu_info[i])return kIOReturnBadArgument;
    io_=*io;rings_=rings;loaded_=loaded;next_=loaded;armed_=false;
    now_us_=now;clock_cookie_=clock;
    /* a6xx_preempt_hw_init: the GMEM save and restore on, the SMMU info zeroed (no switch in flight). */
    if(io_.write32(io_.cookie,PreemptGmemSaveRestore,1) ||
        io_.write32(io_.cookie,PreemptSmmuInfo,0) || io_.write32(io_.cookie,PreemptSmmuInfo+1,0) ||
        io_.barrier(io_.cookie))return kIOReturnIOError;
    attached_=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoPreempt::detach()
{
    if(!attached_)return kIOReturnNotReady;
    if(armed_)return kIOReturnBusy;
    attached_=false;io_={};return kIOReturnSuccess;
}
bool NativeAdrenoPreempt::trigger(uint32_t ring,bool uses_gmem)
{
    if(!attached_ || armed_ || ring>=NativeAdrenoPreemptRingCount || ring==loaded_)return false;
    /* The incoming ring's state first, then the write that starts the switch (a6xx_preempt_trigger's order). */
    const uint64_t info=rings_.smmu_info[ring],record=rings_.record[ring];
    if(io_.write32(io_.cookie,PreemptSmmuInfo,(uint32_t)info) ||
        io_.write32(io_.cookie,PreemptSmmuInfo+1,(uint32_t)(info>>32)) ||
        io_.write32(io_.cookie,PreemptRestoreAddr,(uint32_t)record) ||
        io_.write32(io_.cookie,PreemptRestoreAddr+1,(uint32_t)(record>>32)))
        return false;
    /* Keep the GPU on until the switch is done, as a6xx_preempt_keepalive_vote does. */
    if(io_.gmu_write32 && io_.gmu_write32(io_.gmu_cookie,GmuPreemptKeepalive,1))return false;
    if(io_.barrier(io_.cookie))return false;
    const uint32_t cntl=CntlStop|CntlLevel1|CntlSkipSaveRestore|(uses_gmem?CntlUsesGmem:0);
    if(io_.write32(io_.cookie,PreemptCntl,cntl))return false;
    next_=ring;armed_=true;
    trigger_us_=now_us_?now_us_(clock_cookie_):0;
    stats_.triggers=stats_.triggers+1;
    return true;
}
bool NativeAdrenoPreempt::complete()
{
    if(!attached_ || !armed_)return false;
    uint32_t cntl=0;
    if(io_.read32(io_.cookie,PreemptCntl,&cntl)) {stats_.faulted=stats_.faulted+1;return false;}
    if(cntl&CntlStop) {stats_.faulted=stats_.faulted+1;return false;}   /* the hardware never finished it */
    /* The GPU's keepalive vote ends once the switch is complete, as a6xx_preempt_irq does. */
    if(io_.gmu_write32)(void)io_.gmu_write32(io_.gmu_cookie,GmuPreemptKeepalive,0);
    loaded_=next_;armed_=false;
    if(trigger_us_ && now_us_) {
        const uint64_t us=now_us_(clock_cookie_)-trigger_us_;
        stats_.last_us=uint32_t(us>UINT32_MAX?UINT32_MAX:us);
    }
    stats_.completed=stats_.completed+1;
    return true;
}
