/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoGpuContext.h"
#include "ArmSmmu4KMmio.h"
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/task.h>
#include "AdrenoPM4.h"

extern "C" volatile uint32_t native_adreno_pool_mib;
extern "C" volatile uint32_t native_adreno_pool_stats[8],native_adreno_pool_status;
extern "C" volatile uint32_t native_adreno_wc_enabled,native_adreno_pool_wc,native_adreno_owned_coherent,native_adreno_io_coherent;
static const uint64_t kIovas[GpuBufferCount]={GpuSqeIova,GpuRingIova,GpuMemptrsIova,GpuPwrupIova,GpuScratchIova};
static const size_t kBytes[GpuBufferCount]={GpuSqeBytes,GpuRingBytes,GpuPageBytes,GpuPageBytes,GpuPageBytes};
IOReturn NativeAdrenoGpuContext::bringUp(const ArmSmmu4KIo *io,unsigned bank,unsigned asid,const unsigned slots[2],
    const uint16_t sids[2],void (*progress)(uint32_t),NativeAdrenoGmuContextReport *report)
{
    if(!report)return kIOReturnBadArgument;
    *report={};
    if(used_)return kIOReturnBusy;
    if(!io || !slots || !sids || bank>63 || !asid || asid>0xffff || slots[0]>127 || slots[1]>127 ||
        slots[0]==slots[1])return kIOReturnBadArgument;
    used_=true;kernel_asid_=asid;
    IOReturn status=kIOReturnSuccess;
    auto step=[&](Step s) {report->step=s;if(progress)progress(s);};
    step(Allocate);
    static const uint8_t oas_bits[16]={32,36,40,42,44,48};
    if(io->read32(io->cookie,0x28,&report->id2))status=kIOReturnIOError;
    else {
        const unsigned code=(report->id2>>4)&15;
        report->output_bits=code<6?oas_bits[code]:0;
        if(report->output_bits>40)report->output_bits=40;
        if(!report->output_bits)status=kIOReturnUnsupported;
        /* Stage-1 input size: smmu->va_size from ID2.UBS (arm_smmu_init_domain_context: ias = smmu->va_size for AArch64 stage 1). */
        input_bits_=NativeAdrenoSmmuInputBits(report->id2);
    }
    /* The BO pool: its first chunk is taken first, while the VM still has long contiguous runs (the board sets the size from sheng-gpu-pool;
     * 0 = none). A chunk that cannot be had at one size is tried at the next smaller; none is not an error: the pool still grows on demand. */
    if(!status) {
        IOReturn pooled=kIOReturnNoMemory;
        for(uint32_t mib=native_adreno_pool_mib;mib>=16 && pooled;mib>>=1) {
            pooled=NativeAdrenoMemory::reservePool(report->output_bits,(size_t)mib<<20);
            native_adreno_pool_stats[0]=pooled==kIOReturnSuccess?mib:0;native_adreno_pool_status=(uint32_t)pooled;
        }
        if(pooled)native_adreno_pool_status=(uint32_t)NativeAdrenoMemory::reservePool(report->output_bits,0);
        /* Not fatal: without it each pooled BO changes its pages' attribute itself, as a scattered one does. */
        if(native_adreno_wc_enabled && !native_adreno_owned_coherent)native_adreno_pool_wc=(uint32_t)NativeAdrenoMemory::writeCombinePool()|1u<<31;
    }
    for(unsigned i=0;!status && i<GpuBufferCount;++i)
        status=buffers_[i].allocate(kBytes[i],report->output_bits,true,kBytes[i]);
    if(!status) {
        step(Prepare);
        const uint64_t aperture=UINT64_C(1)<<input_bits_;
        status=smmu_.prepare(&tables_,0,0,aperture,size_t(aperture/4096),bank,asid,1000,report->output_bits,input_bits_);
    }
    if(!status) {
        step(Program);
        status=smmu_.program(io);
        const ArmSmmu4KProgramResult *p=smmu_.programResult();
        report->program_phase=p->phase;report->program_writes=p->writes;
        report->program_readback_valid=p->readback_valid;report->program_enabled=p->enabled;
        report->id1=p->id1;report->program_status=smmu_.programStatus();
        report->image=p->readback;
    }
    if(!status) {
        step(Attach);
        const ArmSmmu4KStream routes[2]={{slots[0],sids[0]},{slots[1],sids[1]}};
        status=smmu_.attach(routes,2);
        const ArmSmmu4KStreamResult *s=smmu_.streamResult();
        report->route_status=smmu_.attachStatus();
        report->route_phase=s->phase;report->route_writes=s->writes;report->routes_verified=s->streams_verified;
    }
    if(!status) {
        step(VmAttach);
        NativeAdrenoVmRegion lower={&tables_,smmu_.arenaHandle(),smmu_.stateHandle()};
        /* 65536 spans: 256 MiB of BOs even when every page is its own span. The Metal runtime pools its memory in 64 MiB blocks,
         * which XNU rarely finds physically contiguous; one scattered block is 16384 spans (int-070 ran out at 16384). */
        status=vm_.attach(io,smmu_.configHandle(),smmu_.streamsHandle(),&lower,nullptr,1048576,262144);
    }
    if(!status) {
        step(Map);
        /* adreno_fw_create_bo maps the SQE MSM_BO_GPU_READONLY without NOEXEC; every other kernel BO read/write. */
        for(unsigned i=0;!status && i<GpuBufferCount;++i)
            status=vm_.map(&buffers_[i],0,kIovas[i],i?3u:5u,0,2,&handles_[i]);
        report->spans=uint32_t(vm_.spanCount());
    }
    if(!status) {
        step(Ready);
        ready_=true;
    }
    report->status=status;
    return status;
}
extern "C" volatile uint32_t native_adreno_table_switches[8];
IOReturn NativeAdrenoGpuContext::enableSwitching()
{
    if(!ready_)return kIOReturnNotReady;
    if(!smmu_.stateHandle()->context_offset)return kIOReturnNotReady;
    smmu_.configHandle()->ttbr0_switched=1;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::writeTtbr0(uint64_t ttbr0)
{
    if(!ready_ || !smmu_.configHandle()->ttbr0_switched)return kIOReturnNotReady;
    ArmSmmu4KProgramResult r;
    return ArmSmmu4KWriteContextTtbr0(smmu_.ioHandle(),smmu_.stateHandle()->context_offset,ttbr0,&r)==ArmSmmu4KSuccess?
        kIOReturnSuccess:kIOReturnIOError;
}
IOReturn NativeAdrenoGpuContext::createVm(NativeAdrenoProcessVm **out)
{
    if(!out)return kIOReturnBadArgument;*out=nullptr;
    if(!ready_ || !smmu_.configHandle()->ttbr0_switched)return kIOReturnNotReady;
    unsigned asid=0;
    for(unsigned a=16;a<65536;++a)if(a!=kernel_asid_ && !(asids_[a>>6]>>(a&63)&1)) {asid=a;break;}
    if(!asid) {native_adreno_table_switches[6]=uint32_t(kIOReturnNoResources);return kIOReturnNoResources;}
    NativeAdrenoProcessVm *p=new NativeAdrenoProcessVm;
    if(!p)return kIOReturnNoMemory;
    const unsigned bits=uint32_t(smmu_.arena()->config.output_bits);
    const uint64_t aperture=UINT64_C(1)<<input_bits_;
    native_adreno_table_switches[7]=2;
    IOReturn status=NativeAdrenoPrepareSmmuMemory(&p->tables_,input_bits_,bits,0,aperture,0,size_t(aperture/4096),
        &p->arena_,&p->state_,&p->visibility_);
    if(!status) {
        native_adreno_table_switches[7]=3;
        /* The kernel table's context image with this table as the lower region and its own ASID. */
        p->config_=*smmu_.configHandle();
        p->config_.lower=&p->arena_.config;p->config_.lower_tables=&p->state_.tables;p->config_.asid=asid;p->config_.ttbr0_switched=1;
        p->state_.detached=1;p->state_.context_offset=smmu_.stateHandle()->context_offset;
        NativeAdrenoVmRegion lower={&p->tables_,&p->arena_,&p->state_};
        p->vm_.keepStreamsOnFailure();
        status=p->vm_.attach(smmu_.ioHandle(),&p->config_,smmu_.streamsHandle(),&lower,nullptr,1048576,262144);
        if(status)native_adreno_table_switches[7]=0x30|uint32_t(p->vm_.lastResult().operation)<<8;
        if(!status)p->attached_=true;
    }
    for(unsigned i=0;!status && i<GpuBufferCount;++i) {
        native_adreno_table_switches[7]=4+i;
        status=p->vm_.map(&buffers_[i],0,kIovas[i],i?3u:5u,0,2,&p->handles_[i]);
    }
    if(status) {
        native_adreno_table_switches[6]=uint32_t(status);
        /* Nothing ran on the table: undo what can be undone; a poisoned VM keeps its memory. */
        if(!p->vm_.poisoned()) {
            bool clean=true;
            for(auto &h:p->handles_)if(h) {if(p->vm_.unmap(h))clean=false;else h=0;}
            if(clean && p->attached_ && p->vm_.detach())clean=false;
            if(clean && !p->tables_.release()) {delete p;return status;}
        }
        return status;
    }
    native_adreno_table_switches[7]=0;
    p->asid_=asid;asids_[asid>>6]|=1ull<<(asid&63);
    ++live_vms_;
    native_adreno_table_switches[2]=native_adreno_table_switches[2]+1;
    native_adreno_table_switches[4]=live_vms_;
    if(live_vms_>native_adreno_table_switches[5])native_adreno_table_switches[5]=live_vms_;
    *out=p;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::destroyVm(NativeAdrenoProcessVm *p)
{
    if(!p || !p->attached_)return kIOReturnBadArgument;
    IOReturn status=kIOReturnSuccess;
    for(auto &h:p->handles_)if(h) {status=p->vm_.unmap(h);if(status)break;h=0;}
    if(!status)status=p->vm_.invalidateAsid();
    if(!status)status=p->vm_.detach();
    if(!status) {
        p->attached_=false;
        /* No TLB entry of the ASID is left and the table is not installed: its pages go back to the system. */
        status=p->tables_.release();
    }
    if(status) {native_adreno_table_switches[6]=uint32_t(status)|0x80000000u;return status;}
    asids_[p->asid_>>6]&=~(1ull<<(p->asid_&63));
    --live_vms_;
    native_adreno_table_switches[3]=native_adreno_table_switches[3]+1;
    native_adreno_table_switches[4]=live_vms_;
    delete p;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::observeContext(uint32_t *kit_status,uint32_t out[8]) const
{
    if(!kit_status || !out)return kIOReturnBadArgument;
    if(!ready_)return kIOReturnNotReady;
    NativeAdrenoSmmuContext &smmu=const_cast<NativeAdrenoSmmuContext &>(smmu_);
    ArmSmmu4KProgramResult r{};
    const ArmSmmu4KStatus status=ArmSmmu4KObserveOwnedContext(smmu.ioHandle(),smmu.streamsHandle()->bank,smmu.configHandle(),&r);
    *kit_status=uint32_t(status);
    const ArmSmmu4KContextImage &v=r.readback;
    out[0]=v.sctlr;out[1]=v.tcr;out[2]=v.tcr2;out[3]=v.mair0;out[4]=uint32_t(v.ttbr1);out[5]=uint32_t(v.ttbr1>>32);out[6]=v.cbar;
    out[7]=r.fault_status;
    return status==ArmSmmu4KSuccess?kIOReturnSuccess:kIOReturnIOError;
}
IOReturn NativeAdrenoGpuContext::clearFault(uint32_t fsr,uint32_t far[3])
{
    if(!far || !ready_)return kIOReturnBadArgument;
    const ArmSmmu4KIo *io=smmu_.ioHandle();
    const uint64_t offset=smmu_.stateHandle()->context_offset;
    if(!offset)return kIOReturnNotReady;
    if(io->read32(io->cookie,(size_t)offset+0x60,&far[0]) || io->read32(io->cookie,(size_t)offset+0x64,&far[1]) ||
        io->read32(io->cookie,(size_t)offset+0x68,&far[2]))return kIOReturnIOError;
    if(io->write32(io->cookie,(size_t)offset+0x58,fsr) || io->barrier(io->cookie))return kIOReturnIOError;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::endFault(uint32_t out[4])
{
    if(!out || !ready_)return kIOReturnBadArgument;
    out[0]=out[1]=out[2]=out[3]=0;
    const ArmSmmu4KIo *io=smmu_.ioHandle();
    const uint64_t offset=smmu_.stateHandle()->context_offset;
    if(!offset)return kIOReturnNotReady;
    if(io->read32(io->cookie,(size_t)offset+0x58,&out[0]))return kIOReturnIOError;
    /* FSR bits 9-10 are the context's format (0x400: AArch64), not a fault: only the fault bits latch one. */
    out[0]&=~UINT32_C(0x600);
    if(!out[0])return kIOReturnSuccess;
    if(io->read32(io->cookie,(size_t)offset+0x60,&out[1]) || io->read32(io->cookie,(size_t)offset+0x64,&out[2]) ||
        io->read32(io->cookie,(size_t)offset+0x68,&out[3]))return kIOReturnIOError;
    /* CB_RESUME (0x08) bit 0 TnR: terminate the stalled transaction; the GPU sees an error response instead of waiting forever. */
    if((out[0]&(1u<<30)) && io->write32(io->cookie,(size_t)offset+0x08,1))return kIOReturnIOError;
    if(io->write32(io->cookie,(size_t)offset+0x58,out[0]) || io->barrier(io->cookie))return kIOReturnIOError;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::readTtbr0(uint64_t *out) const
{
    if(!out)return kIOReturnBadArgument;*out=0;
    if(!ready_)return kIOReturnNotReady;
    NativeAdrenoSmmuContext &smmu=const_cast<NativeAdrenoSmmuContext &>(smmu_);
    const ArmSmmu4KIo *io=smmu.ioHandle();
    const uint64_t offset=smmu.stateHandle()->context_offset;
    if(!offset || !io->read64)return kIOReturnNotReady;
    return io->read64(io->cookie,(size_t)offset+0x20,out)?kIOReturnIOError:kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuContext::probeBuffer(NativeAdrenoProcessVm *p,uint64_t *handle,uint64_t *iova,size_t *bytes)
{
    if(!p || !p->attached_ || !handle || !iova || !bytes)return kIOReturnBadArgument;
    volatile uint32_t *scratch=static_cast<volatile uint32_t *>(buffers_[4].mapping());
    if(!scratch)return kIOReturnNotReady;
    uint32_t nop=0;
    if(AdrenoPM4Type7(0x10,0,&nop)!=AdrenoPM4Success)return kIOReturnInternalError;   /* CP_NOP */
    scratch[0xF00/4]=nop;
    *handle=p->handles_[4];*iova=GpuScratchIova+0xF00;*bytes=4;
    return kIOReturnSuccess;
}
static NativeAdrenoGpuContext *board_gpu_context;
extern "C" IOReturn NativeAdrenoGpuContextBringup(volatile uint8_t *smmu,size_t bytes,unsigned bank,unsigned asid,
    unsigned slot0,unsigned sid0,unsigned slot1,unsigned sid1,void (*progress)(uint32_t),
    NativeAdrenoGmuContextReport *report)
{
    static ArmSmmu4KMmio mmio;
    static ArmSmmu4KIo io;
    if(!report || board_gpu_context || !smmu || !bytes || sid0>0xffff || sid1>0xffff)return kIOReturnBadArgument;
    mmio={reinterpret_cast<volatile unsigned char *>(smmu),bytes};
    if(ArmSmmu4KBindMmio(&mmio,&io)!=ArmSmmu4KSuccess)return kIOReturnBadArgument;
    board_gpu_context=new NativeAdrenoGpuContext;
    if(!board_gpu_context)return kIOReturnNoMemory;
    const unsigned slots[2]={slot0,slot1};
    const uint16_t sids[2]={uint16_t(sid0),uint16_t(sid1)};
    return board_gpu_context->bringUp(&io,bank,asid,slots,sids,progress,report);
}
extern "C" IOReturn NativeAdrenoGpuContextBuffers(NativeAdrenoGpuBuffer out[5])
{
    if(!out)return kIOReturnBadArgument;
    for(unsigned i=0;i<GpuBufferCount;++i)out[i]={};
    if(!board_gpu_context || !board_gpu_context->ready())return kIOReturnNotReady;
    for(unsigned i=0;i<GpuBufferCount;++i) {
        const NativeAdrenoMemory &m=board_gpu_context->buffer(i);
        if(m.spanCount()!=1 || m.spans()[0].bytes!=m.bytes())return kIOReturnUnsupported;
        out[i]={m.spans()[0].physical,m.bytes(),kIovas[i]};
    }
    return kIOReturnSuccess;
}
extern "C" unsigned NativeAdrenoGpuContextInputBits()
{
    return board_gpu_context && board_gpu_context->ready()?board_gpu_context->inputBits():0;
}
extern "C" NativeAdrenoVm *NativeAdrenoGpuContextVm()
{
    return board_gpu_context?board_gpu_context->vm():nullptr;
}
extern "C" NativeAdrenoGpuContext *NativeAdrenoGpuContextObject()
{
    return board_gpu_context && board_gpu_context->ready()?board_gpu_context:nullptr;
}
