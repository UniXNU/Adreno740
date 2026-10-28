/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoGmuContext.h"
#include "ArmSmmu4KMmio.h"
#include <IOKit/IOLib.h>

static void readBack(const ArmSmmu4KIo *io,const ArmSmmu4KProgramResult *program,unsigned bank,
    unsigned slot,NativeAdrenoGmuContextReport *r)
{
    /* Only registers the program step itself has already read; skipped if that step never ran. */
    if(!program->context_offset || !program->id1)return;
    const size_t page=(program->id1&(1u<<31))?0x10000:0x1000;
    io->read32(io->cookie,0x800+4*slot,&r->smr);
    io->read32(io->cookie,0xc00+4*slot,&r->s2cr);
    io->read32(io->cookie,page+4*bank,&r->cbar);
    io->read32(io->cookie,size_t(program->context_offset),&r->sctlr);
    io->read64(io->cookie,size_t(program->context_offset)+0x20,&r->ttbr0);
}
IOReturn NativeAdrenoGmuContext::bringUp(const ArmSmmu4KIo *io,unsigned bank,unsigned asid,
    unsigned slot,uint16_t sid,void (*progress)(uint32_t),NativeAdrenoGmuContextReport *report)
{
    if(!report)return kIOReturnBadArgument;
    *report={};
    if(used_)return kIOReturnBusy;
    if(!io || bank>63 || !asid || asid>0xffff || slot>127)return kIOReturnBadArgument;
    used_=true;
    IOReturn status=kIOReturnSuccess;
    auto step=[&](Step s) {report->step=s;if(progress)progress(s);};
    step(Allocate);
    /* Stage-1 output size is ID2.OAS, as arm_smmu_init_domain_context uses smmu->ipa_size; tables and
     * payloads must then lie below it. 40 bits is this driver's own allocation ceiling. */
    static const uint8_t oas_bits[16]={32,36,40,42,44,48};
    if(io->read32(io->cookie,0x28,&report->id2))status=kIOReturnIOError;
    else {
        const unsigned code=(report->id2>>4)&15;
        report->output_bits=code<6?oas_bits[code]:0;
        if(report->output_bits>40)report->output_bits=40;
        if(!report->output_bits)status=kIOReturnUnsupported;
    }
    if(!status)status=gmu_.allocate(NativeAdrenoGmuDefaultSizes(),report->output_bits);
    if(!status) {
        step(Prepare);
        /* The GMU's own space is 0..2 GiB (a6xx_gmu_memory_probe: msm_gem_vm_create(drm, mmu, "gmu", 0x0, 0x80000000, true)), a 32-bit input. */
        status=smmu_.prepare(&gmu_.tables(),0,0,GmuApertureBytes,GmuApertureBytes/4096,
            bank,asid,1000,report->output_bits,32);
    }
    if(!status) {
        step(Program);
        status=smmu_.program(io);
        const ArmSmmu4KProgramResult *p=smmu_.programResult();
        report->program_phase=p->phase;report->program_writes=p->writes;
        report->program_readback_valid=p->readback_valid;report->program_enabled=p->enabled;
        report->id1=p->id1;report->program_status=smmu_.programStatus();
        report->image=p->readback;
        ArmSmmu4KContextConfig actual=*smmu_.configHandle();
        actual.id0=p->id0;actual.id2=p->id2;
        ArmSmmu4KContextImage want;
        if(p->phase>=3 && ArmSmmu4KEncodeContext(&actual,&want)==ArmSmmu4KSuccess) {
            const ArmSmmu4KContextImage &got=p->readback;
            const uint32_t sctlr_want=p->enabled?want.sctlr:0;
            report->mismatch=(got.cbar!=want.cbar)|(got.cba2r!=want.cba2r)<<1|(got.tcr!=want.tcr)<<2|
                (got.tcr2!=want.tcr2)<<3|(got.mair0!=want.mair0)<<4|(got.mair1!=want.mair1)<<5|
                (got.ttbr0!=want.ttbr0)<<6|(got.ttbr1!=want.ttbr1)<<7|(got.sctlr!=sctlr_want)<<8;
        }
    }
    if(!status) {
        step(Attach);
        const ArmSmmu4KStream route{slot,sid};
        status=smmu_.attach(&route,1);
        const ArmSmmu4KStreamResult *s=smmu_.streamResult();
        report->route_status=smmu_.attachStatus();
        report->route_phase=s->phase;report->route_writes=s->writes;report->routes_verified=s->streams_verified;
    }
    if(!status) {
        step(VmAttach);
        NativeAdrenoVmRegion lower={&gmu_.tables(),smmu_.arenaHandle(),smmu_.stateHandle()};
        status=vm_.attach(io,smmu_.configHandle(),smmu_.streamsHandle(),&lower,nullptr,64,4000);
    }
    if(!status) {
        step(Map);
        status=gmu_.mapInto(&vm_);
        report->spans=uint32_t(vm_.spanCount());
    }
    if(!status) {
        step(Ready);
        ready_=true;
    }
    report->status=status;
    if(report->step>=Program)readBack(io,smmu_.programResult(),bank,slot,report);
    return status;
}
/* Lives for the rest of the boot: the context bank points at its tables. */
static NativeAdrenoGmuContext *board_context;
extern "C" IOReturn NativeAdrenoGmuContextBringup(volatile uint8_t *smmu,size_t bytes,
    unsigned bank,unsigned asid,unsigned slot,unsigned sid,void (*progress)(uint32_t),
    NativeAdrenoGmuContextReport *report)
{
    static ArmSmmu4KMmio mmio;
    static ArmSmmu4KIo io;
    NativeAdrenoGmuContext *&context=board_context;
    if(!report || context || !smmu || !bytes || sid>0xffff)return kIOReturnBadArgument;
    mmio={reinterpret_cast<volatile unsigned char *>(smmu),bytes};
    if(ArmSmmu4KBindMmio(&mmio,&io)!=ArmSmmu4KSuccess)return kIOReturnBadArgument;
    context=new NativeAdrenoGmuContext;
    if(!context)return kIOReturnNoMemory;
    return context->bringUp(&io,bank,asid,slot,uint16_t(sid),progress,report);
}
extern "C" IOReturn NativeAdrenoGmuContextPayloads(NativeAdrenoGmuPayload out[5])
{
    if(!out)return kIOReturnBadArgument;
    for(unsigned i=0;i<5;++i)out[i]={};
    if(!board_context || !board_context->ready())return kIOReturnNotReady;
    NativeAdrenoGmuAllocator &gmu=board_context->allocator();
    const NativeAdrenoGmuMemoryLayout &layout=gmu.layout();
    const NativeAdrenoMemory *objects[5]={&gmu.icache(),&gmu.dummy(),&gmu.debug(),&gmu.log(),&gmu.hfi()};
    const uint64_t iovas[5]={layout.icache_iova,layout.dummy_iova,layout.debug_iova,layout.log_iova,layout.hfi_iova};
    for(unsigned i=0;i<5;++i) {
        const NativeAdrenoMemory *m=objects[i];
        if(m->spanCount()!=1 || m->spans()[0].bytes!=m->bytes())return kIOReturnUnsupported;
        out[i]={m->spans()[0].physical,m->bytes(),iovas[i]};
    }
    return kIOReturnSuccess;
}
