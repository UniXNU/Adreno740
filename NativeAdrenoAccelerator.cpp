// SPDX-License-Identifier: MIT
// The registry-visible accelerator for the original Metal discovery (IOServiceMatching("IOAcceleratorES") ->
// MetalPluginName -> /System/Library/Extensions/<name>.bundle). It derives from the original IOAcceleratorES base exactly as
// qualified in drivers/AdrenoMetal/OriginalAcceleratorABI.h: the original base has no operational overrides, so the only
// virtuals this class adds are newUserClient and the property reads below (diagnostic counting only). Its static initializer
// is staged out of __mod_init_func and run by the board probe after the original IOGPUFamily kext has loaded, because the
// parent metaclass does not exist before that. The class is named after the Apple accelerator class on purpose, so original
// sandbox profiles that name the service class accept the Adreno plugin's open; the Apple AGX kext is never loaded here.
#include "../AdrenoMetal/OriginalAcceleratorABI.h"
#include <IOKit/IOUserClient.h>

extern "C" IOReturn native_adreno_new_user_client(IOService *,task_t,void *,UInt32,OSDictionary *,IOUserClient **,unsigned);
extern "C" boolean_t PE_parse_boot_argn(const char *,void *,int);
extern "C" void native_adreno_note_property_read(const char *);

class AGXAccelerator : public IOAcceleratorES {
    OSDeclareDefaultStructors(AGXAccelerator)
public:
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,OSDictionary *properties,
        IOUserClient **handler) APPLE_KEXT_OVERRIDE;
    OSObject *copyProperty(const OSSymbol *key) const APPLE_KEXT_OVERRIDE;
    OSObject *copyProperty(const OSString *key) const APPLE_KEXT_OVERRIDE;
    OSObject *copyProperty(const char *key) const APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(AGXAccelerator,IOAcceleratorES)

IOReturn AGXAccelerator::newUserClient(task_t owningTask,void *securityID,UInt32 type,OSDictionary *properties,
    IOUserClient **handler)
{
    unsigned limit=0;   /* sheng-gpu-metal-opens: connections granted to each process name (0 = unlimited, as on Apple GPUs) */
    PE_parse_boot_argn("sheng-gpu-metal-opens",&limit,sizeof(limit));
    return native_adreno_new_user_client(this,owningTask,securityID,type,properties,handler,limit);
}
OSObject *AGXAccelerator::copyProperty(const OSSymbol *key) const
{
    native_adreno_note_property_read(key?key->getCStringNoCopy():nullptr);
    return IORegistryEntry::copyProperty(key);
}
OSObject *AGXAccelerator::copyProperty(const OSString *key) const
{
    native_adreno_note_property_read(key?key->getCStringNoCopy():nullptr);
    return IORegistryEntry::copyProperty(key);
}
OSObject *AGXAccelerator::copyProperty(const char *key) const
{
    native_adreno_note_property_read(key);
    return IORegistryEntry::copyProperty(key);
}
extern "C" IOService *native_adreno_accelerator_allocate(void) {return new AGXAccelerator;}
extern "C" unsigned native_adreno_accelerator_instance_size(void) {return sizeof(AGXAccelerator);}
