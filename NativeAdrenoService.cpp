/* SPDX-License-Identifier: MIT */
/* IOKit surface for the native Adreno backend. This file is compiled against
 * the real source-kernel headers; it does not run on any host. start() parses
 * the provider's device-tree reg windows through NativeAdrenoBoard and maps the
 * first window, then stops: board power, GMU boot and SMMU context programming
 * are not implemented for Darwin yet, so the service refuses to publish rather
 * than expose a device that cannot execute. newUserClient and the external
 * method table are present because they are fixed by the core ABI; no client
 * can be created while the GPU is not ready. clientMemoryForType has no
 * descriptor to return until a client owns a leased BO, so it refuses. */
#include <IOKit/IODeviceMemory.h>
#include <string.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include "NativeAdrenoBoard.h"
#include "NativeAdrenoFirmware.h"
#include "NativeAdrenoGmuConfig.h"
#include "NativeAdrenoGmuContext.h"
#include "NativeAdrenoGmuPlatform.h"
#include "NativeAdrenoGmuService.h"
#include "NativeAdrenoSmmuContext.h"
#include "NativeAdrenoIrq.h"
#include "NativeAdrenoKernelTime.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoPwrup.h"
#include "NativeAdrenoRing.h"
#include "ShengGmuCommandDb.h"
#include "NativeAdrenoSmmuService.h"
#include <pexpert/pexpert.h>
#if __has_include("ShengGmuSource.h")
#include "ShengGmuSource.h"
#define NATIVE_ADRENO_HAVE_GMU_RULES 1
#endif
#if __has_include("ShengGmuVotesSource.h")
#include "ShengGmuVotesSource.h"
#define NATIVE_ADRENO_HAVE_GMU_VOTES 1
#endif
#if __has_include("ShengGmuCommandDbSource.h")
#include "ShengGmuCommandDbSource.h"
#define NATIVE_ADRENO_HAVE_COMMAND_DB 1
#endif
extern "C" const uint8_t native_adreno_firmware_image[] __attribute__((weak));
extern "C" const uint64_t native_adreno_firmware_image_size __attribute__((weak));
extern "C" const uint8_t native_adreno_sqe_firmware_image[] __attribute__((weak));
extern "C" const uint64_t native_adreno_sqe_firmware_image_size __attribute__((weak));
#include "NativeAdrenoUserClient.h"
/* Physical address of the fence page (the CP writes its timestamps there): the board log reads it from DRAM and through the CPU cache. */
extern "C" {volatile uint64_t native_adreno_fence_phys=0;}

class NativeAdrenoGpuClient;
static IOUserClient *createGpuClient(task_t,void *,UInt32,OSDictionary *,NativeAdrenoUserClient *);

struct NativeAdrenoGpuWindow {
    volatile uint8_t *base;
    size_t bytes;
};
class NativeAdrenoGpuService : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGpuService)
    OSSharedPtr<IOMemoryMap> maps_[2];
    NativeAdrenoGmuContext gmu_ctx_;
    NativeAdrenoGmuPlatform gmu_platform_;
    ShengGmuIo gmu_io_{};
    ShengGmuDevice gmu_device_{};
    ShengGmuStartupResult gmu_startup_{};
    NativeAdrenoFirmwareView firmware_{};
    bool gmu_io_bound_=false,firmware_valid_=false,ready_=false;
    /* Set only once a separate GPU-stream context exists; until then the CP path below is refused. */
    bool gpu_context_ready_=false;
    NativeAdrenoMemory ring_mem_,fence_mem_,sqe_mem_,pwrup_mem_;
    NativeAdrenoRing ring_;
    uint64_t ring_map_=0,fence_map_=0,sqe_map_=0,pwrup_map_=0;
    NativeAdrenoIrq irq_;
    NativeAdrenoSubmit *execbuf_=nullptr;
    NativeAdrenoUserClient *core_=nullptr;
    bool core_ready_=false;
    struct RingCache {NativeAdrenoMemory *ring;NativeAdrenoMemory *fence;};
    RingCache ring_cache_{};
    NativeAdrenoGpuWindow ring_window_{};   /* the ring keeps a pointer to it: it must outlive start() */
    static int gpu_read32(void *,uint32_t,uint32_t *);
    static int gmu_fence_status(void *,int clear,uint32_t *status);
    static int gpu_write32(void *,uint32_t,uint32_t);
    static int gpu_barrier(void *);
    static int ring_publish(void *,const volatile uint32_t *,size_t);
    static int fence_acquire(void *,volatile uint32_t *,size_t);
    static uint64_t ring_now(void *);
public:
    IOService *probe(IOService *provider,SInt32 *score) APPLE_KEXT_OVERRIDE;
    bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
    void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties,IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuService,IOService)

static bool advertisesAdreno(IOService *provider)
{
    OSObject *property=provider?provider->getProperty("compatible"):nullptr;
    OSData *data=OSDynamicCast(OSData,property);
    if(!data || !data->getBytesNoCopy() || !data->getLength())return false;
    const char *text=static_cast<const char *>(data->getBytesNoCopy());
    const size_t length=data->getLength();
    static const char needle[]="qcom,adreno";
    for(size_t at=0;at+sizeof(needle)-1<=length;++at) {
        size_t index=0;
        while(index<sizeof(needle)-1 && text[at+index]==needle[index])++index;
        if(index==sizeof(needle)-1)return true;
    }
    return false;
}
struct NativeAdrenoSmmuWindow {
    volatile uint8_t *base;
    size_t bytes;
};
static int smmu_read32(void *cookie,size_t at,uint32_t *value)
{
    NativeAdrenoSmmuWindow *window=static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if(!window || !value || !window->base || (at&3) || at+4>window->bytes)return 1;
    *value=*reinterpret_cast<volatile uint32_t *>(window->base+at);
    return 0;
}
static int smmu_read64(void *cookie,size_t at,uint64_t *value)
{
    NativeAdrenoSmmuWindow *window=static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if(!window || !value || !window->base || (at&7) || at+8>window->bytes)return 1;
    *value=*reinterpret_cast<volatile uint64_t *>(window->base+at);
    return 0;
}
static int smmu_write32(void *cookie,size_t at,uint32_t value)
{
    NativeAdrenoSmmuWindow *window=static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if(!window || !window->base || (at&3) || at+4>window->bytes)return 1;
    *reinterpret_cast<volatile uint32_t *>(window->base+at)=value;
    return 0;
}
static int smmu_write64(void *cookie,size_t at,uint64_t value)
{
    NativeAdrenoSmmuWindow *window=static_cast<NativeAdrenoSmmuWindow *>(cookie);
    if(!window || !window->base || (at&7) || at+8>window->bytes)return 1;
    *reinterpret_cast<volatile uint64_t *>(window->base+at)=value;
    return 0;
}
static int smmu_barrier(void *cookie)
{
    if(!cookie)return 1;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 0;
}
int NativeAdrenoGpuService::gpu_read32(void *cookie,uint32_t reg,uint32_t *value)
{
    NativeAdrenoGpuWindow *window=static_cast<NativeAdrenoGpuWindow *>(cookie);
    const size_t at=size_t(reg)*4;
    if(!window || !value || !window->base || at+4>window->bytes)return 1;
    *value=*reinterpret_cast<volatile uint32_t *>(window->base+at);
    return 0;
}
int NativeAdrenoGpuService::gpu_write32(void *cookie,uint32_t reg,uint32_t value)
{
    NativeAdrenoGpuWindow *window=static_cast<NativeAdrenoGpuWindow *>(cookie);
    const size_t at=size_t(reg)*4;
    if(!window || !window->base || at+4>window->bytes)return 1;
    *reinterpret_cast<volatile uint32_t *>(window->base+at)=value;
    return 0;
}
/* GMU_AHB_FENCE_STATUS (0x23b13) and GMU_AHB_FENCE_STATUS_CLR (0x23b14) in the GMU core window, whose byte offset is the GPU word
 * offset times 4 minus SHENG_GMU_CORE_GPU_DELTA (0x6a000). */
int NativeAdrenoGpuService::gmu_fence_status(void *cookie,int clear,uint32_t *status)
{
    const ShengGmuIo *io=static_cast<const ShengGmuIo *>(cookie);
    if(!io || !io->read32 || !status)return 1;
    if(clear)return io->write32(io->cookie,ShengGmuCore,0x23b14u*4u-0x6a000u,0x7u);
    return io->read32(io->cookie,ShengGmuCore,0x23b13u*4u-0x6a000u,status);
}
int NativeAdrenoGpuService::gpu_barrier(void *cookie)
{
    if(!cookie)return 1;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 0;
}
int NativeAdrenoGpuService::ring_publish(void *cookie,const volatile uint32_t *words,size_t bytes)
{
    NativeAdrenoGpuService::RingCache *cache=static_cast<RingCache *>(cookie);
    return cache && cache->ring && cache->ring->publishForDevice(0,bytes,false)==kIOReturnSuccess?0:1;
}
int NativeAdrenoGpuService::fence_acquire(void *cookie,volatile uint32_t *words,size_t bytes)
{
    NativeAdrenoGpuService::RingCache *cache=static_cast<RingCache *>(cookie);
    return cache && cache->fence && cache->fence->acquireDeviceWrites(0,bytes)==kIOReturnSuccess?0:1;
}
uint64_t NativeAdrenoGpuService::ring_now(void *)
{
    uint64_t microseconds=0;
    NativeAdrenoKernelTime::now_us(nullptr,&microseconds);
    return microseconds;
}
extern "C" uint64_t (*native_adreno_vm_now_us)(void);
static uint64_t vm_clock_us(void) {uint64_t microseconds=0;NativeAdrenoKernelTime::now_us(nullptr,&microseconds);return microseconds;}
IOService *NativeAdrenoGpuService::probe(IOService *provider,SInt32 *score)
{
    native_adreno_vm_now_us=vm_clock_us;
    if(!advertisesAdreno(provider))return nullptr;
    if(score)*score=1000;
    return this;
}
bool NativeAdrenoGpuService::start(IOService *provider)
{
    if(!IOService::start(provider))return false;
    OSData *reg=OSDynamicCast(OSData,provider->getProperty("reg"));
    OSData *address=OSDynamicCast(OSData,provider->getProperty("#address-cells"));
    OSData *size=OSDynamicCast(OSData,provider->getProperty("#size-cells"));
    NativeAdrenoWindow windows[NativeAdrenoMaxWindows]{};
    size_t count=0;
    uint32_t address_cells=2,size_cells=2;
    if(address && address->getLength()>=4)
        address_cells=*static_cast<const uint32_t *>(address->getBytesNoCopy());
    if(size && size->getLength()>=4)
        size_cells=*static_cast<const uint32_t *>(size->getBytesNoCopy());
    if(!reg || NativeAdrenoBoard::parseReg(
        static_cast<const uint8_t *>(reg->getBytesNoCopy()),reg->getLength(),
        address_cells,size_cells,windows,NativeAdrenoMaxWindows,&count)!=kIOReturnSuccess) {
        IOLog("NativeAdreno: provider reg windows are not usable\n");
        IOService::stop(provider);
        return false;
    }
    if(count<2) {
        IOLog("NativeAdreno: GPU node needs the register and CX windows\n");
        IOService::stop(provider);
        return false;
    }
    for(unsigned index=0;index<2;++index) {
        IODeviceMemory *memory=getDeviceMemoryWithIndex(index);
        if(!memory) {
            IOLog("NativeAdreno: provider window %u has no device memory\n",index);
            stop(provider);
            return false;
        }
        maps_[index]=OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere|kIOMapInhibitCache),
            OSNoRetain);
        if(!maps_[index]) {
            IOLog("NativeAdreno: GPU window %u did not map\n",index);
            stop(provider);
            return false;
        }
    }
    IOLog("NativeAdreno: %lu windows, GPU window at 0x%llx/0x%llx mapped\n",
        static_cast<unsigned long>(count),
        static_cast<unsigned long long>(windows[0].base),
        static_cast<unsigned long long>(windows[0].bytes));
    /* The GMU node publishes its bases so no DT phandle resolution is needed.
     * Discovery does not imply readiness: bring-up still needs board power,
     * firmware and the SMMU context before any register may be touched. */
    OSDictionary *matching=IOService::serviceMatching("NativeAdrenoGmuService");
    IOService *gmu=matching?IOService::copyMatchingService(matching):nullptr;
    if(matching)matching->release();
    if(gmu) {
        NativeAdrenoGmuService *node=OSDynamicCast(NativeAdrenoGmuService,gmu);
        const unsigned published=node?static_cast<unsigned>(node->windowCount()):0;
        IOLog("NativeAdreno: GMU node present with %u windows\n",published);
        if(node && published>=3) {
            NativeAdrenoGmuWindow gmu_windows[ShengGmuWindows]{};
            for(unsigned index=0;index<3;++index) {
                gmu_windows[index].base=node->windowAddress(index);
                gmu_windows[index].bytes=static_cast<size_t>(node->windowBytes(index));
            }
            gmu_windows[ShengGmuGpu].base=
                reinterpret_cast<volatile uint8_t *>(maps_[0]->getVirtualAddress());
            gmu_windows[ShengGmuGpu].bytes=static_cast<size_t>(windows[0].bytes);
            gmu_windows[ShengGmuCxMisc].base=
                reinterpret_cast<volatile uint8_t *>(maps_[1]->getVirtualAddress());
            gmu_windows[ShengGmuCxMisc].bytes=static_cast<size_t>(windows[1].bytes);
            if(gmu_platform_.bind(gmu_windows,&gmu_io_)==kIOReturnSuccess) {
                gmu_io_bound_=true;
                IOLog("NativeAdreno: GMU five-window I/O adapter bound\n");
            } else
                IOLog("NativeAdreno: GMU windows are not usable\n");
        }
        gmu->release();
    } else {
        IOLog("NativeAdreno: GMU node not published yet\n");
    }
    /* Stream IDs are this node's iommus; the specifier width is published by
     * the SMMU node. Nothing is assumed when either is missing. */
    OSDictionary *smmu_match=IOService::serviceMatching("NativeAdrenoSmmuService");
    IOService *smmu=smmu_match?IOService::copyMatchingService(smmu_match):nullptr;
    if(smmu_match)smmu_match->release();
    if(smmu) {
        OSNumber *cells=OSDynamicCast(OSNumber,smmu->getProperty("iommu-cells"));
        OSData *iommus=OSDynamicCast(OSData,provider->getProperty("iommus"));
        uint16_t sids[8]{};
        size_t sid_count=0;
        if(cells && iommus && NativeAdrenoBoard::parseIommus(
            static_cast<const uint8_t *>(iommus->getBytesNoCopy()),iommus->getLength(),
            cells->unsigned32BitValue(),sids,8,&sid_count)==kIOReturnSuccess)
            IOLog("NativeAdreno: %lu stream IDs from iommus, first %u\n",
                static_cast<unsigned long>(sid_count),sid_count?sids[0]:0);
        else
            IOLog("NativeAdreno: iommus are not usable\n");
        smmu->release();
    } else {
        IOLog("NativeAdreno: SMMU node not published yet\n");
    }
    /* Explicit opt-in (sheng-gpu-bringup=1). Without the bank/asid/sid/slot arguments the SMMU window is only read
     * (identity and fault registers). With all four arguments the sequence below PROGRAMS the SMMU: it prepares a context
     * bank, attaches the stream and maps the GMU payloads. Do not run it before the read-only power/clock observation
     * (sheng-gpu-power, docs/gpu-hardware-plan.md) shows the GPU SMMU powered and writable: the apps SMMU on this board is
     * hypervisor-managed (docs/native-ufs.md) and the adreno SMMU has not been shown to differ. */
    uint32_t bringup=0;
    if(PE_parse_boot_argn("sheng-gpu-bringup",&bringup,sizeof(bringup)) && bringup) {
        NativeAdrenoSmmuService *node=OSDynamicCast(NativeAdrenoSmmuService,smmu);
        if(node && node->windowCount() && node->windowAddress(0)) {
            volatile uint8_t *smmu_base=node->windowAddress(0);
            const size_t smmu_bytes=static_cast<size_t>(node->windowBytes(0));
            const uint32_t id0=*reinterpret_cast<volatile uint32_t *>(smmu_base+0x20);
            const uint32_t id1=*reinterpret_cast<volatile uint32_t *>(smmu_base+0x24);
            const uint32_t id2=*reinterpret_cast<volatile uint32_t *>(smmu_base+0x28);
            const uint32_t scr0=*reinterpret_cast<volatile uint32_t *>(smmu_base+0x0);
            const uint32_t fault=*reinterpret_cast<volatile uint32_t *>(smmu_base+0x48);
            IOLog("NativeAdreno: SMMU id0=%08x id1=%08x id2=%08x scr0=%08x fault=%08x window=%lu\n",
                id0,id1,id2,scr0,fault,static_cast<unsigned long>(smmu_bytes));
            uint32_t bank=0,asid=0,sid=0,slot=0;
            const bool configured=
                PE_parse_boot_argn("sheng-gpu-bank",&bank,sizeof(bank)) &&
                PE_parse_boot_argn("sheng-gpu-asid",&asid,sizeof(asid)) &&
                PE_parse_boot_argn("sheng-gpu-sid",&sid,sizeof(sid)) &&
                PE_parse_boot_argn("sheng-gpu-slot",&slot,sizeof(slot));
            if(!configured || bank>63 || !asid || asid>0xffff || sid>0xffff || slot>127) {
                IOLog("NativeAdreno: bring-up needs bank/asid/sid/slot arguments\n");
            } else {
                /* The GMU context at its fixed a6xx_gmu.c addresses (NativeAdrenoGmuContext). The ring, fence,
                 * SQE and power-up list are read by the CP through the GPU's own streams (iommus 0/1) and need
                 * the GPU context, which is not built yet; they are never placed in the GMU context. */
                [[maybe_unused]] const size_t ring_bytes=0x8000,fence_bytes=0x1000;
                [[maybe_unused]] const uint64_t ring_iova=0,fence_iova=0,sqe_iova=0,pwrup_iova=0;
                [[maybe_unused]] const uint64_t sqe_bytes=(reinterpret_cast<uintptr_t>(&native_adreno_sqe_firmware_image_size)&&
                    reinterpret_cast<uintptr_t>(native_adreno_sqe_firmware_image))?
                    ((native_adreno_sqe_firmware_image_size+0xfff)&~UINT64_C(0xfff)):0;
                NativeAdrenoSmmuWindow window{smmu_base,smmu_bytes};
                ArmSmmu4KIo smmu_io{&window,smmu_bytes,smmu_read32,smmu_read64,
                    smmu_write32,smmu_write64,smmu_barrier};
                NativeAdrenoGmuContextReport report{};
                IOReturn status=gmu_ctx_.bringUp(&smmu_io,bank,asid,slot,uint16_t(sid),nullptr,&report);
                [[maybe_unused]] NativeAdrenoGmuAllocator &gmu_=gmu_ctx_.allocator();
                [[maybe_unused]] NativeAdrenoVm &vm_=gmu_ctx_.vm();
                if(status) {
                    IOLog("NativeAdreno: GMU context stopped at step %u status 0x%x; resources retained\n",
                        report.step,status);
                } else {
                    IOLog("NativeAdreno: GMU payloads mapped, bank=%u asid=%u sid=%u slot=%u ttbr0=0x%llx\n",
                        bank,asid,sid,slot,static_cast<unsigned long long>(report.ttbr0));
#ifdef NATIVE_ADRENO_HAVE_GMU_RULES
                    if(reinterpret_cast<uintptr_t>(native_adreno_firmware_image) &&
                        reinterpret_cast<uintptr_t>(&native_adreno_firmware_image_size)) {
                        NativeAdrenoFirmwareRules rules{};
                        rules.bytes=SHENG_GMU_FIRMWARE_BYTES;
                        for(unsigned i=0;i<32;++i)rules.sha256[i]=sheng_gmu_firmware_sha256[i];
                        const IOReturn checked=NativeAdrenoFirmware::validate(
                            native_adreno_firmware_image,native_adreno_firmware_image_size,
                            &rules,&firmware_);
                        if(checked)
                            IOLog("NativeAdreno: embedded GMU firmware failed validation 0x%x\n",
                                checked);
                        else {
                            firmware_valid_=true;
                            IOLog("NativeAdreno: embedded GMU firmware validated (%lu bytes)\n",
                                static_cast<unsigned long>(firmware_.length));
                        }
                    } else {
                        IOLog("NativeAdreno: no embedded GMU firmware in this image\n");
                    }
#else
                    IOLog("NativeAdreno: GMU rules not generated for this build\n");
#endif
#ifdef NATIVE_ADRENO_HAVE_COMMAND_DB
                    uint32_t version=0;
                    if(!firmware_valid_ || !gmu_io_bound_) {
                        IOLog("NativeAdreno: GMU boot needs firmware and bound windows\n");
                    } else if(!PE_parse_boot_argn("sheng-gpu-cmddb-version",&version,
                        sizeof(version)) || !version) {
                        IOLog("NativeAdreno: GMU boot needs sheng-gpu-cmddb-version\n");
                    } else {
                        IODeviceMemory *db=IODeviceMemory::withRange(
                            SHENG_GMU_COMMAND_DB_BASE,SHENG_GMU_COMMAND_DB_BYTES);
                        if(!db) {
                            IOLog("NativeAdreno: command-db reservation did not map\n");
                        } else {
                            OSSharedPtr<IOMemoryMap> db_map(db->map(
                                kIOMapAnywhere|kIOMapInhibitCache),OSNoRetain);
                            db->release();
                            if(!db_map) {
                                IOLog("NativeAdreno: command-db mapping failed\n");
                            } else {
                                ShengGmuVoteInputs inputs{};
                                const ShengGmuDbStatus parsed=ShengGmuCommandDbInputs(
                                    reinterpret_cast<const uint8_t *>(static_cast<uintptr_t>(db_map->getVirtualAddress())),
                                    SHENG_GMU_COMMAND_DB_BYTES,version,&inputs);
                                if(parsed!=ShengGmuDbSuccess) {
                                    IOLog("NativeAdreno: command-db refused (%d)\n",parsed);
                                } else {
#ifdef NATIVE_ADRENO_HAVE_GMU_VOTES
                                    inputs.chip_id=SHENG_GMU_VOTES_CHIP_ID;
                                    inputs.enable_acd=1;
                                    /* IFPC stays off: the power-up list carries no IFPC restore registers and
                                     * WPTR writes are not fenced, so a collapse between frames loses CP state. */
                                    inputs.enable_ifpc=0;
                                    AdrenoHfiStartupConfig startup{};
                                    ShengGmuConfig config{};
                                    NativeAdrenoGmuTime time{nullptr,
                                        NativeAdrenoKernelTime::now_us,
                                        NativeAdrenoKernelTime::wait_us};
                                    ShengGmuServices services{};
                                    if(ShengGmuBuildStartup(&inputs,&startup)!=AdrenoHfiStartupSuccess)
                                        IOLog("NativeAdreno: startup config refused\n");
                                    else if(NativeAdrenoAssembleGmuConfig(&firmware_,&gmu_,
                                        SHENG_GMU_VOTES_CHIP_ID,1000,&config)!=kIOReturnSuccess)
                                        IOLog("NativeAdreno: GMU config assembly refused\n");
                                    else if(NativeAdrenoGmuPlatform::services(&time,&services))
                                        IOLog("NativeAdreno: GMU services refused\n");
                                    else {
                                        ShengGmuStartupLimits limits{4,100,1000,10000};
                                        const ShengGmuStatus started=ShengGmuColdStart(
                                            &gmu_device_,&config,&startup,&limits,&gmu_startup_);
                                        IOLog("NativeAdreno: ColdStart status=%d fw_ready=%u hfi_ready=%u stage=%u\n",
                                        started,gmu_startup_.load.firmware_ready_observed,
                                        gmu_startup_.load.hfi_ready_observed,
                                        gmu_startup_.protocol.stage);
                                    if(started==ShengGmuSuccess && !gpu_context_ready_) {
                                        IOLog("NativeAdreno: ring/SQE refused: the GPU context (iommus 0/1) is not built\n");
                                    } else if(started==ShengGmuSuccess) {
                                        IOReturn ring_status=ring_mem_.allocate(ring_bytes,40,true,ring_bytes);
                                        if(!ring_status)ring_status=fence_mem_.allocate(fence_bytes,40,true,fence_bytes);
                                        if(!ring_status && sqe_bytes)
                                            ring_status=sqe_mem_.allocate(size_t(sqe_bytes),40,true,size_t(sqe_bytes));
                                        if(!ring_status)ring_status=vm_.map(&ring_mem_,1,ring_iova,3,0,2,&ring_map_);
                                        if(!ring_status)ring_status=vm_.map(&fence_mem_,1,fence_iova,3,0,2,&fence_map_);
                                        if(!ring_status && fence_mem_.spanCount())native_adreno_fence_phys=fence_mem_.spans()[0].physical;
                                        if(!ring_status)ring_status=vm_.map(&sqe_mem_,1,sqe_iova,3,0,2,&sqe_map_);
                                        if(ring_status) {
                                            IOLog("NativeAdreno: ring/SQE setup stopped at 0x%x\n",ring_status);
                                        } else {
                                            ring_window_={
                                                reinterpret_cast<volatile uint8_t *>(maps_[0]->getVirtualAddress()),
                                                static_cast<size_t>(windows[0].bytes)};
                                            NativeAdrenoRingIo ring_io{&ring_window_,gpu_write32,gpu_read32,gpu_barrier,
                                                gmu_io_bound_?&gmu_io_:nullptr,gmu_io_bound_?gmu_fence_status:nullptr};
                                            ring_cache_={&ring_mem_,&fence_mem_};
                                            NativeAdrenoRingCache cache{&ring_cache_,ring_publish,fence_acquire};
                                            ring_status=ring_.attach(&ring_io,&cache,ring_mem_.mapping(),
                                                ring_bytes,ring_iova,fence_mem_.mapping(),fence_bytes,fence_iova);
                                            if(!ring_status && sqe_bytes) {
                                                memcpy(sqe_mem_.mapping(),native_adreno_sqe_firmware_image,
                                                    size_t(native_adreno_sqe_firmware_image_size));
                                                ring_status=sqe_mem_.publishForDevice(0,size_t(sqe_bytes),false);
                                            }
                                            if(ring_status)
                                                IOLog("NativeAdreno: ring attach stopped at 0x%x\n",ring_status);
                                            else
                                                IOLog("NativeAdreno: ring attached ring_iova=0x%llx fence_iova=0x%llx sqe_iova=0x%llx\n",
                                                    static_cast<unsigned long long>(ring_iova),
                                                    static_cast<unsigned long long>(fence_iova),
                                                    static_cast<unsigned long long>(sqe_iova));
                                            }
                                            if(!ring_status) {
                                                NativeAdrenoGpuWindow cp_window={
                                                    reinterpret_cast<volatile uint8_t *>(maps_[0]->getVirtualAddress()),
                                                    static_cast<size_t>(windows[0].bytes)};
                                                NativeAdrenoRingIo cp_io{&cp_window,gpu_write32,gpu_read32,gpu_barrier,nullptr,nullptr};
                                                ring_status=pwrup_mem_.allocate(0x1000,40,true,0x1000);
                                                if(!ring_status)ring_status=vm_.map(&pwrup_mem_,1,
                                                    pwrup_iova,3,0,2,&pwrup_map_);
                                                size_t list_bytes=0;
                                                if(!ring_status)ring_status=NativeAdrenoPwrupList::buildA740(
                                                    gpu_read32,&cp_window,
                                                    static_cast<uint8_t *>(pwrup_mem_.mapping()),
                                                    0x1000,&list_bytes);
                                                if(!ring_status)ring_status=pwrup_mem_.publishForDevice(0,0x1000,false);
                                                if(!ring_status) {
                                                    NativeAdrenoCpInit cp;
                                                    NativeAdrenoCpInitConfig cfg{sqe_iova,pwrup_iova,0};
                                                    uint32_t start_fence=0;
                                                    ring_status=cp.run(&cp_io,&ring_,&cfg,&start_fence);
                                                    if(!ring_status) {
                                                        NativeAdrenoRingClock clock{nullptr,ring_now,nullptr};
                                                        ring_status=ring_.wait(start_fence,&clock,2000,nullptr)
                                                            ==NativeAdrenoRingFenceComplete?
                                                            kIOReturnSuccess:kIOReturnTimeout;
                                                    }
                                                    if(!ring_status) {
                                                        ring_status=irq_.attach(&cp_io);
                                                        if(!ring_status)ring_status=irq_.programMask(nullptr);
                                                    }
                                                    if(ring_status)
                                                        IOLog("NativeAdreno: CP start stopped at 0x%x\n",ring_status);
                                                    else
                                                    {
                                                        execbuf_=new NativeAdrenoSubmit;
                                                        core_=new NativeAdrenoUserClient;
                                                        if(execbuf_ && core_ &&
                                                            execbuf_->attach(&vm_,&ring_)==kIOReturnSuccess &&
                                                            core_->open(NativeAdrenoAbiVersion,&vm_,execbuf_,
                                                                &ring_,32,40)==kIOReturnSuccess)
                                                            core_ready_=true;
                                                        ready_=true;
                                                        IOLog("NativeAdreno: CP started, fence=%u, interrupts masked=%08x\n",
                                                            start_fence,NativeAdrenoIrq::mask());
                                                    }
                                                }
                                            }
                                    }
                                    }
#else
                                    IOLog("NativeAdreno: vote rules not generated for this build\n");
#endif
                                }
                            }
                        }
                    }
#else
                    IOLog("NativeAdreno: command-db reservation not generated for this build\n");
#endif
                }
            }
        }
    }
    if(ready_) {
        IOLog("NativeAdreno: GPU ready, publishing service\n");
        registerService();
        return true;
    }
    IOLog("NativeAdreno: refusing to publish without a started CP\n");
    IOService::stop(provider);
    return false;
}
void NativeAdrenoGpuService::stop(IOService *provider)
{
    for(auto &map:maps_)map.reset();
    IOService::stop(provider);
}
IOReturn NativeAdrenoGpuService::newUserClient(task_t owningTask,void *securityID,
    UInt32 type,OSDictionary *properties,IOUserClient **handler)
{
    if(handler)*handler=nullptr;
    if(!core_ready_)return kIOReturnNotReady;
    IOUserClient *client=createGpuClient(owningTask,securityID,type,properties,core_);
    if(!client)return kIOReturnNoResources;
    if(handler)*handler=client;
    return kIOReturnSuccess;
}

class NativeAdrenoGpuClient : public IOUserClient {
    OSDeclareDefaultStructors(NativeAdrenoGpuClient)
    NativeAdrenoUserClient *core_=nullptr;
    task_t task_=TASK_NULL;
    static IOReturn action(OSObject *target,void *reference,
        IOExternalMethodArguments *arguments);
public:
    void setCore(NativeAdrenoUserClient *core) {core_=core;}
    bool initWithTask(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties) APPLE_KEXT_OVERRIDE;
    IOReturn clientClose() APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type,IOOptionBits *options,
        IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
        IOExternalMethodDispatch *dispatch,OSObject *target,
        void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuClient,IOUserClient)

bool NativeAdrenoGpuClient::initWithTask(task_t owningTask,void *securityID,
    UInt32 type,OSDictionary *properties)
{
    if(!IOUserClient::initWithTask(owningTask,securityID,type,properties))return false;
    task_=owningTask;
    return true;
}
IOReturn NativeAdrenoGpuClient::clientClose()
{
    core_=nullptr;task_=TASK_NULL;
    terminate();
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoGpuClient::clientMemoryForType(UInt32 type,IOOptionBits *options,
    IOMemoryDescriptor **memory)
{
    if(memory)*memory=nullptr;
    if(options)*options=kIOMapAnywhere;
    /* The type is a BO mapping token; no client can hold one before the GPU
     * and its core client exist, and the descriptor handoff is not written. */
    return kIOReturnUnsupported;
}
IOReturn NativeAdrenoGpuClient::action(OSObject *target,void *reference,
    IOExternalMethodArguments *arguments)
{
    NativeAdrenoGpuClient *client=OSDynamicCast(NativeAdrenoGpuClient,target);
    if(!client || !arguments)return kIOReturnBadArgument;
    if(!client->core_)return kIOReturnNotReady;
    (void)reference;
    /* Waits need a real time base, supplied by the service once bring-up exists. */
    return client->core_->dispatch(arguments->selector,arguments->structureInput,
        arguments->structureInputSize,arguments->structureOutput,
        arguments->structureOutputSize,nullptr);
}
IOReturn NativeAdrenoGpuClient::externalMethod(uint32_t selector,
    IOExternalMethodArguments *arguments,IOExternalMethodDispatch *dispatch,
    OSObject *target,void *reference)
{
    /* One entry per ABI method; sizes are the exact wire records. */
    static const IOExternalMethodDispatch table[]={
        {&action,0,0,0,sizeof(NativeAdrenoAbiParams)},                    /* Query */
        {&action,0,sizeof(NativeAdrenoAbiBoAlloc),0,sizeof(NativeAdrenoAbiBoAllocReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoMap),0,sizeof(NativeAdrenoAbiBoMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoFree),0,0},
        {&action,0,sizeof(NativeAdrenoAbiSubmit),0,sizeof(NativeAdrenoAbiSubmitReply)},
        {&action,0,sizeof(NativeAdrenoAbiWaitFence),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiTimestamp)},
        {&action,0,0,0,sizeof(NativeAdrenoAbiStatus)},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,0},
        {&action,0,sizeof(NativeAdrenoAbiMetadata),0,sizeof(NativeAdrenoAbiMetadata)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,sizeof(NativeAdrenoAbiQueueReply)},
        {&action,0,sizeof(NativeAdrenoAbiQueue),0,0},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuMap),0,sizeof(NativeAdrenoAbiBoCpuMapReply)},
        {&action,0,sizeof(NativeAdrenoAbiBoCpuUnmap),0,0}};
    if(selector<1 || selector>sizeof(table)/sizeof(table[0]))return kIOReturnUnsupported;
    return IOUserClient::externalMethod(selector,arguments,
        const_cast<IOExternalMethodDispatch *>(&table[selector-1]),target?target:this,reference);
}

static IOUserClient *createGpuClient(task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,NativeAdrenoUserClient *core)
{
    NativeAdrenoGpuClient *client=new NativeAdrenoGpuClient;
    if(!client)return nullptr;
    if(!client->initWithTask(owningTask,securityID,type,properties)) {
        client->release();
        return nullptr;
    }
    client->setCore(core);
    return client;
}
