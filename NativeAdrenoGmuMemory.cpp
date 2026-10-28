/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoGmuMemory.h"
#include <IOKit/IOLib.h>
enum : size_t { PageBytes=0x1000 };
NativeAdrenoGmuSizes NativeAdrenoGmuDefaultSizes()
{
    return {0xffc000,0x4000,0x7000,0x4000,0x4000};
}
IOReturn NativeAdrenoGmuAllocator::allocate(const NativeAdrenoGmuSizes &sizes,unsigned address_bits)
{
    if(ready_ || icache_.bytes())return kIOReturnBusy;
    const uint64_t requested[]={sizes.icache,sizes.dummy,sizes.debug,sizes.log,sizes.hfi};
    for(uint64_t value:requested)
        if(!value || (value&(PageBytes-1)) || value>uint64_t(SIZE_MAX))
            return kIOReturnBadArgument;
    /* The fixed addresses of a6xx_gmu.c at the pinned tree: icache at 0x4000 below the uncached range, dummy and
     * debug at 0x60000000/0x60400000. Log and HFI have no fixed address there; drm_mm gives them the lowest free
     * pages after the dummy, which is where they are put here. */
    if(sizes.icache>GmuUncachedBase-GmuIcacheIova ||
        sizes.dummy+sizes.log+sizes.hfi>GmuDebugIova-GmuUncachedBase ||
        sizes.debug>GmuApertureBytes-GmuDebugIova)return kIOReturnBadArgument;
    struct Selection {NativeAdrenoMemory *memory;uint64_t bytes;uint64_t *iova;uint64_t at;};
    const Selection selections[]={
        {&icache_,sizes.icache,&layout_.icache_iova,GmuIcacheIova},
        {&dummy_,sizes.dummy,&layout_.dummy_iova,GmuUncachedBase},
        {&debug_,sizes.debug,&layout_.debug_iova,GmuDebugIova},
        {&log_,sizes.log,&layout_.log_iova,GmuUncachedBase+sizes.dummy},
        {&hfi_,sizes.hfi,&layout_.hfi_iova,GmuUncachedBase+sizes.dummy+sizes.log}};
    for(const Selection &selection:selections) {
        const IOReturn status=selection.memory->allocate(static_cast<size_t>(selection.bytes),
            address_bits,true,static_cast<size_t>(selection.bytes));
        if(status) {
            (void)releaseAll();
            return status;
        }
        *selection.iova=selection.at;
    }
    layout_.end_iova=GmuDebugIova+sizes.debug;
    ready_=true;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGmuAllocator::mapInto(NativeAdrenoVm *vm)
{
    if(!ready_ || !vm)return kIOReturnBadArgument;
    for(uint64_t handle:handles_)if(handle)return kIOReturnBusy;
    /* The table pages themselves are never mapped: they are the translation
     * storage and the VM refuses such an overlap. */
    NativeAdrenoMemory *objects[]={&icache_,&dummy_,&debug_,&log_,&hfi_};
    const uint64_t iovas[]={layout_.icache_iova,layout_.dummy_iova,layout_.debug_iova,
        layout_.log_iova,layout_.hfi_iova};
    for(size_t index=0;index<5;++index) {
        /* a6xx_gmu.c maps the icache MAP_PRIV and every buffer without NOEXEC; here only the icache, which
         * the GMU executes from, is executable, and the data buffers stay execute-never. */
        const unsigned permissions=index?3u:15u;
        const IOReturn status=vm->map(objects[index],0,iovas[index],permissions,0,2,&handles_[index]);
        if(status) {
            for(size_t previous=0;previous<index;++previous)
                if(handles_[previous]) {vm->unmap(handles_[previous]);handles_[previous]=0;}
            return status;
        }
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGmuAllocator::unmapFrom(NativeAdrenoVm *vm)
{
    if(!vm)return kIOReturnBadArgument;
    IOReturn status=kIOReturnSuccess;
    for(size_t index=5;index--;)if(handles_[index]) {
        const IOReturn unmapped=vm->unmap(handles_[index]);
        if(unmapped)status=unmapped;else handles_[index]=0;
    }
    return status;
}
IOReturn NativeAdrenoGmuAllocator::releaseAll()
{
    NativeAdrenoMemory *objects[]={&icache_,&dummy_,&debug_,&log_,&hfi_};
    IOReturn status=kIOReturnSuccess;
    for(NativeAdrenoMemory *object:objects)
        if(object->bytes() && object->release()!=kIOReturnSuccess)status=kIOReturnIOError;
    if(tables_.release()!=kIOReturnSuccess)status=kIOReturnIOError;
    return status;
}
IOReturn NativeAdrenoGmuAllocator::release()
{
    if(!ready_)return kIOReturnNotReady;
    const IOReturn status=releaseAll();
    if(status)return status;
    ready_=false;layout_={};
    return kIOReturnSuccess;
}
