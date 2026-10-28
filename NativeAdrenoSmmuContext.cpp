/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoSmmuContext.h"
#include <IOKit/IOLib.h>
IOReturn NativeAdrenoSmmuContext::prepare(NativeAdrenoTablePages *tables,unsigned upper,uint64_t iova,
    uint64_t bytes,size_t leaf_budget,unsigned bank,unsigned asid,unsigned poll_budget,unsigned output_bits,unsigned input_bits)
{
    if(prepared_ || poisoned_)return kIOReturnBusy;
    if(!tables || upper>1 || !bytes || (iova&4095) || !leaf_budget || input_bits<32 || input_bits>48 ||
        bank>63 || asid>0xffff || !poll_budget || poll_budget>4096)
        return kIOReturnBadArgument;
    const IOReturn prepared=NativeAdrenoPrepareSmmuMemory(tables,input_bits,output_bits,iova,bytes,upper,leaf_budget,
        &arena_,&state_,&visibility_);
    if(prepared!=kIOReturnSuccess)return prepared==kIOReturnNoResources?prepared:kIOReturnBadArgument;
    /* The adreno SMMU node is dma-coherent at the pinned tree, so table walks are inner write-back. */
    config_=(ArmSmmu4KContextConfig){.lower=upper?nullptr:&arena_.config,.upper=upper?&arena_.config:nullptr,
        .lower_tables=upper?nullptr:&state_.tables,.upper_tables=upper?&state_.tables:nullptr,.id0=0,.id2=0,
        .asid=asid,.coherent_walk=1,.stall_on_fault=0,.continue_after_fault=1};
    tables_=tables;bank_=bank;poll_budget_=poll_budget;prepared_=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSmmuContext::program(const ArmSmmu4KIo *io)
{
    if(!prepared_ || programmed_ || poisoned_ || !io)return kIOReturnBadArgument;
    const ArmSmmu4KStatus status=ArmSmmu4KProgramDisabledContext(io,bank_,&config_,
        poll_budget_,&program_);
    program_status_=status;
    if(status!=ArmSmmu4KSuccess) {poisoned_=true;return kIOReturnIOError;}
    io_=*io;programmed_=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSmmuContext::attach(const ArmSmmu4KStream *routes,size_t count)
{
    if(!programmed_ || attached_ || poisoned_ || !routes || !count || count>8)
        return kIOReturnBadArgument;
    const ArmSmmu4KStatus status=ArmSmmu4KAttachStreams(&io_,&streams_,routes,count,
        bank_,&config_,&routes_);
    attach_status_=status;
    if(status!=ArmSmmu4KSuccess) {poisoned_=true;return kIOReturnIOError;}
    attached_=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSmmuContext::observe() const
{
    if(!programmed_ || poisoned_)return kIOReturnBadArgument;
    ArmSmmu4KProgramResult result{};
    return ArmSmmu4KObserveOwnedContext(&io_,bank_,&config_,&result)==ArmSmmu4KSuccess?
        kIOReturnSuccess:kIOReturnIOError;
}
IOReturn NativeAdrenoSmmuContext::disable()
{
    if(!attached_ || poisoned_)return kIOReturnBadArgument;
    ArmSmmu4KProgramResult result{};
    if(ArmSmmu4KDisableOwnedContext(&io_,bank_,&config_,poll_budget_,&result)!=ArmSmmu4KSuccess) {
        poisoned_=true;
        return kIOReturnIOError;
    }
    attached_=false;programmed_=false;
    return kIOReturnSuccess;
}
