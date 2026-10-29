/* SPDX-License-Identifier: MIT */
/* IOKit entry points the Turnip darwin backend uses, answered by the host copy of the kernel stack; every other service is the system's. Inserted with DYLD_INSERT_LIBRARIES
 * and redirected by dyld interposing. Never built for the tablet. */
#include <IOKit/IOKitLib.h>
#include <mach/mach.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <unistd.h>
static bool trace(){static int t=-1;if(t<0)t=getenv("HOSTSIM_TRACE")?1:0;return t;}

extern "C" {
int host_gpu_open();
int host_gpu_close();
int host_gpu_call(uint32_t,const void *,size_t,void *,size_t);
int host_gpu_map(uint32_t,void **,uint64_t *);
}

static kern_return_t translate(int fixture_status) {
    switch(fixture_status) {
    case 0:return KERN_SUCCESS;
    case 1:return kIOReturnBadArgument;
    case 2:return kIOReturnBusy;
    case 3:return kIOReturnNoResources;
    case 4:return kIOReturnNoMemory;
    case 5:return kIOReturnUnsupported;
    case 6:return kIOReturnNotReady;
    case 7:return kIOReturnIOError;
    case 8:return kIOReturnTimeout;
    default:return kIOReturnError;
    }
}

/* Two-level-namespace binding of the Turnip library to the real IOKit is redirected with dyld interposing (the flat-namespace route no longer
 * takes effect on current macOS). The service answers like the tablet: "IOAcceleratorES" needs open type 0x4147 and only the tunnel through one
 * admitted selector (kAdmitted, deliberately not 0), "NativeAdrenoGpuPublished" is the unrestricted direct form. */
static const uint32_t kAdmitted = 5, kTunnelMagic = 0x4e544e4c;
static uint32_t gOpenKind;   /* 1 direct, 2 tunnel only */
#define INTERPOSE(replacement, replacee) \
    __attribute__((used)) static const struct {const void *with; const void *what;} interpose_##replacee \
    __attribute__((section("__DATA,__interpose"))) = {reinterpret_cast<const void *>(&replacement), reinterpret_cast<const void *>(&replacee)}

extern "C" {
static CFMutableDictionaryRef shim_IOServiceMatching(const char *name) {
    if(trace())fprintf(stderr,"shim: IOServiceMatching(%s)\n",name);
    if(!strcmp(name,"NativeAdrenoGpuPublished"))return reinterpret_cast<CFMutableDictionaryRef>(1);
    if(!strcmp(name,"IOAcceleratorES"))return reinterpret_cast<CFMutableDictionaryRef>(2);
    return IOServiceMatching(name);   /* any other service is the system's (IOSurface for a CARenderer instrument) */
}
static io_service_t shim_IOServiceGetMatchingService(mach_port_t,CFDictionaryRef matching) {
    if(trace())fprintf(stderr,"shim: IOServiceGetMatchingService(%p)\n",(const void *)matching);
    const uintptr_t ours=reinterpret_cast<uintptr_t>(matching);
    if(ours==1 || ours==2)return static_cast<io_service_t>(ours);
    return IOServiceGetMatchingService(kIOMainPortDefault,matching);
}
static kern_return_t shim_IOServiceOpen(io_service_t service,task_port_t,uint32_t type,io_connect_t *connect) {
    if(trace())fprintf(stderr,"shim: IOServiceOpen(%u, 0x%x)\n",service,type);
    if(service!=1 && service!=2)return IOServiceOpen(service,mach_task_self(),type,connect);
    if(service==2 && type!=0x4147)return kIOReturnNotPermitted;
    const kern_return_t kr=translate(host_gpu_open());
    if(trace())fprintf(stderr,"shim: open -> 0x%x\n",kr);
    if(kr==KERN_SUCCESS) {*connect=7;gOpenKind=service;}
    return kr;
}
static kern_return_t shim_IOServiceClose(io_connect_t connect) {return connect==7?translate(host_gpu_close()):IOServiceClose(connect);}
static kern_return_t shim_IOObjectRelease(io_object_t object) {return object==1 || object==2 || object==7?KERN_SUCCESS:IOObjectRelease(object);}
static kern_return_t shim_IOConnectCallStructMethod(mach_port_t connect,uint32_t selector,const void *in,size_t in_size,void *out,size_t *out_size) {
    if(connect!=7)return IOConnectCallStructMethod(connect,selector,in,in_size,out,out_size);
    if(gOpenKind==2) {
        if(selector!=kAdmitted)return kIOReturnNotPermitted;
        const uint32_t *header=static_cast<const uint32_t *>(in);
        if(!in || in_size<8 || header[0]!=kTunnelMagic)return kIOReturnBadArgument;
        if(header[1]==0xffffffffu)return KERN_SUCCESS;
        /* HOSTSIM_STALL=<seconds>: the first BoAlloc (the first call vkCreateDevice makes) blocks that long, to exercise the plugin's watchdog. */
        static bool stalled=false;
        if(header[1]==2 && !stalled && getenv("HOSTSIM_STALL")) {stalled=true;sleep((unsigned)atoi(getenv("HOSTSIM_STALL")));}
        const int status=host_gpu_call(header[1],static_cast<const uint8_t *>(in)+8,in_size-8,out,out_size?*out_size:0);
        if(trace())fprintf(stderr,"shim: method %u first word 0x%llx -> %d\n",header[1],in_size>=16?(unsigned long long)*reinterpret_cast<const uint64_t *>(static_cast<const uint8_t *>(in)+8):0ull,status);
        if(trace() && status)fprintf(stderr,"shim: method %u (in %zu, out %zu) -> fixture status %d\n",header[1],in_size-8,out_size?*out_size:0,status);
        return translate(status);
    }
    return translate(host_gpu_call(selector,in,in_size,out,out_size?*out_size:0));
}
static kern_return_t shim_IOConnectMapMemory64(io_connect_t connect,uint32_t type,task_port_t task,mach_vm_address_t *address,mach_vm_size_t *size,
    IOOptionBits options) {
    if(connect!=7)return IOConnectMapMemory64(connect,type,task,address,size,options);
    void *mapping=nullptr;uint64_t bytes=0;
    const kern_return_t kr=translate(host_gpu_map(type,&mapping,&bytes));
    if(kr==KERN_SUCCESS) {*address=reinterpret_cast<mach_vm_address_t>(mapping);*size=bytes;}
    return kr;
}
static kern_return_t shim_IOConnectUnmapMemory64(io_connect_t connect,uint32_t type,task_port_t task,mach_vm_address_t address) {
    if(connect!=7)return IOConnectUnmapMemory64(connect,type,task,address);
    return getenv("HOSTSIM_FAIL_USER_UNMAP") ? kIOReturnIOError : KERN_SUCCESS;
}
/* HOSTSIM_PAGE_SIZE=4096: Turnip sizes and places BOs with the page size (os_get_page_size, getpagesize), 16 KiB on this Mac and 4 KiB on the
 * tablet, so a recording meant to match the tablet's GPU address layout reports the tablet's page size. */
static long pageSize(){static long p=-1;if(p<0)p=getenv("HOSTSIM_PAGE_SIZE")?atol(getenv("HOSTSIM_PAGE_SIZE")):0;return p;}
static int shim_getpagesize(void) {return pageSize()?int(pageSize()):getpagesize();}
static long shim_sysconf(int name) {return name==_SC_PAGESIZE && pageSize()?pageSize():sysconf(name);}
}
INTERPOSE(shim_getpagesize, getpagesize);
INTERPOSE(shim_sysconf, sysconf);
INTERPOSE(shim_IOServiceMatching, IOServiceMatching);
INTERPOSE(shim_IOServiceGetMatchingService, IOServiceGetMatchingService);
INTERPOSE(shim_IOServiceOpen, IOServiceOpen);
INTERPOSE(shim_IOServiceClose, IOServiceClose);
INTERPOSE(shim_IOObjectRelease, IOObjectRelease);
INTERPOSE(shim_IOConnectCallStructMethod, IOConnectCallStructMethod);
INTERPOSE(shim_IOConnectMapMemory64, IOConnectMapMemory64);
INTERPOSE(shim_IOConnectUnmapMemory64, IOConnectUnmapMemory64);
