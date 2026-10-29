/* SPDX-License-Identifier: MIT */
/* IOKit surface for an already-running GPU context. The board code (kits/sheng) brings the GPU up, then hands this file
 * the live VM, submit object, ring, clock and optional timer; one IOService is published and each IOServiceOpen creates a
 * connection that mounts its own NativeAdrenoUserClient core. Every connection of a user process gets its own GPU address space
 * (NativeAdrenoProcessVm: its own page table and ASID, switched into the GPU's context bank for its submissions) and its own queue
 * range on the one ring; there is no fixed number of connections. Closing a connection waits for its own jobs, unmaps and frees
 * everything it created and frees its address space. The board's kernel connection uses the kernel table. A GPU hang is found by msm's
 * hangcheck and restarted through the board's hook; only the client whose submission hung sees its device lost. This is not the Metal
 * or Turnip ABI; it carries NativeAdrenoABI.h unchanged.
 * BO CPU mappings are handed out as uncached physical-range mappings to match the GPU's non-cacheable PTEs.
 * The connection class is named AGXDeviceUserClient on purpose: the original sandbox profiles of the processes that use Metal
 * allow IOServiceOpen for that user-client class name, and nothing else in this system can create one (the Apple AGX kext is
 * never loaded without an Apple GPU), so the name only decides whether the Adreno Metal plugin may open its own driver. */
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <pexpert/pexpert.h>
#include <IOKit/IOUserClient.h>
#include <string.h>
#include <kern/thread.h>
#include "NativeAdrenoUserClient.h"
#include "NativeAdrenoGpuContext.h"

extern "C" void proc_selfname(char *, int);
extern "C" volatile uint32_t native_adreno_table_switches[8];
/* The plugin's telemetry table (rows of text the Metal bridge keeps), sent as markers: 0xC000|row starts a row, 0x8000|(a<<7)|b appends two characters. */
extern "C" {char native_adreno_metal_rows[48][96];volatile uint32_t native_adreno_metal_rows_seq;}

extern "C" {
/* What the processes that reached this driver did, for the boot diagnostic page. Plain counters and short strings. */
struct NativeAdrenoMetalDiag {
    uint32_t opens, busy, bad, closes, reports, property_reads, foreign_opens, foreign_type, name_reads, calls, last_selector, last_status, ext_entries, ext_selector, ext_status, ext_in, ext_out, tunnel_calls, probes, sel_bits[8], mb_maps, mb_calls, marks_total, marks[16], chk_macf, chk_res, chk_type, type_pass_lo, type_pass_hi, attach_status, inflight, inflight_since_calls, forced_unmaps;
    char opener[3][17];      /* process names of the latest connections, newest first */
    char reader[3][17];      /* process names that read a registry property of the accelerator, newest first */
    char name_reader[17];    /* latest process that read MetalPluginName specifically */
    char starter[3][17];     /* processes that sent plugin marker 1 (the plugin's initWithAcceleratorPort: started), newest first */
    char report[64];         /* the plugin's own result line (SetMetadata text), latest */
    char report2[64];        /* the plugin's Turnip-phase line (text starting "T " or "W "), latest */
    uint32_t grants;         /* opens granted to the render server by this driver in place of the sandbox verdict (the view in GpuPowerProbe.cpp ends before this) */
} native_adreno_metal_diag;
/* BO CPU views (clientMemoryForType for a BO token): [0] calls, [1] new descriptors, [2] served from the view cache, [3] refused (not a BO / out of
 * resources), [4] last refusal's IOReturn, [5] threads inside now. int-115 showed a backboardd thread in IOConnectMapMemory64 ~650 times a second. */
extern "C" {volatile uint32_t native_adreno_view_diag[6],native_adreno_view_kept;}   /* kept: BoCpuUnmap calls that left the view for another lease */
}

/* The open type the Adreno plugin passes to IOServiceOpen ('AG'); it lets the driver tell the plugin from the original IOGPU library
 * (type 1 plus an API value) and from any other client of the accelerator. */
enum : UInt32 {NativeAdrenoPluginOpenType=0x4147,NativeAdrenoTunnelMagic=0x4e544e4c /* 'NTNL' */};
/* The display hand-off (boot-arg sheng-fb-gpu=1, drivers/GopFramebuffer) reaches the plugin through this already admitted connection: the
 * original sandbox profiles admit this class but not the presenter's own service. Methods 0x7000 + the presenter's NGP_METHOD numbers (mode,
 * wait for a frame, complete it) are forwarded unchanged, without the GPU lock (a wait sleeps); memory type NativeAdrenoScanoutType is the
 * writable scanout. Nothing here touches the GPU. */
enum : UInt32 {NativeAdrenoHandoffBase=0x7000,NativeAdrenoHandoffLast=0x7003,NativeAdrenoScanoutType=0x5343,
               NativeAdrenoHandoffOpenType=0x4148};
/* Open type 0x4148 is a connection that serves only the hand-off: it does not mount a GPU core, so it does not need (or take) a
 * GPU connection slot (the Vulkan driver holds one for as long as the Metal device lives). Same class, so the same sandbox admission. */
extern "C" IOReturn native_gop_handoff_call(uint32_t op,task_t task,const void *in,size_t inSize,void *out,size_t outSize);
/* Export (int-147): method NativeAdrenoExportMethod hands one in-order piece of a blob to the board's UFS log (kits/sheng/BootLog.c), which writes
 * the newest complete blob to its export area. The render server sends its pipeline caches this way, so a later image can be built with them.
 * Input: {generation, total, offset, bytes} then the bytes. No GPU state is touched; any connection of the render server may send it. */
enum : UInt32 {NativeAdrenoExportMethod=0x7100,NativeAdrenoExportMaxPiece=3968};
extern "C" int boot_log_export_put(uint32_t generation,uint32_t total,uint32_t offset,const void *data,uint32_t size) __attribute__((weak_import));
extern "C" {volatile uint32_t native_adreno_export_stats[4];}   /* [0] pieces taken, [1] refused, [2] last generation, [3] its total bytes */
/* Import: method NativeAdrenoImportMethod reads the board's import region (the read-only part of the userdata reserve that the host writes,
 * UniXNU-sheng drivers/ShengUfs native_ufs_import_read), which holds the pipeline caches compiled ahead on the host. Input {u64 byte offset},
 * a multiple of 4096; output: whole 4096-byte blocks from there, at most NativeAdrenoImportMaxBytes. A zero-length output answers the
 * region's size (u64). Only the render server may read it; no GPU state is touched. */
enum : UInt32 {NativeAdrenoImportMethod=0x7101,NativeAdrenoImportMaxBytes=4096};
extern "C" uint64_t native_ufs_import_blocks(void) __attribute__((weak_import));
extern "C" int native_ufs_import_read(uint64_t,uint32_t,void *) __attribute__((weak_import));
extern "C" {volatile uint64_t native_adreno_import_stats[3];}   /* [0] reads served, [1] refused, [2] bytes served */
extern "C" IOMemoryDescriptor *native_gop_handoff_memory(void);

/* Mailbox: a last-resort command channel for a sandbox that admits no external-method selector at all. It is one shared page (memory
 * type NativeAdrenoMailboxType through IOConnectMapMemory64, which the selector filter does not cover) and a kernel poller that runs
 * whatever the live connection writes into it. Layout (little endian dwords): +0 magic, +4 request seq, +8 response seq, +12 method,
 * +16 status, +20 input bytes, +24 output bytes, input at +0x40 (max 0x700), output at +0x800 (max 0x700). */
enum : UInt32 {NativeAdrenoMailboxType=0xC0DE,NativeAdrenoMailboxMagic=0x4d41494c /* 'MAIL' */};
namespace {
IOBufferMemoryDescriptor *gMailbox=nullptr;
volatile uint32_t *gMailboxWords=nullptr;
volatile bool gPollerStarted=false;
}
class AGXDeviceUserClient;
/* The render server's connection serves the mailbox; it is retained while set (under g.lock), so the poller never runs on a freed object. */
static AGXDeviceUserClient *gMailboxOwner=nullptr;
void nativeAdrenoMailboxPoller(void *,wait_result_t);
static IOReturn runCore(AGXDeviceUserClient *,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);

/* The two sandbox gates IOServiceOpen applies after newUserClient (XNU IOUserClient.cpp is_io_service_open_extended): the MACF open check on
 * the client object and the sandbox's filter resolver. They are run here for the caller too, only to record which one refuses it. */
extern "C" int mac_iokit_check_open(void *cred,void *client,unsigned type);
extern "C" void *kauth_cred_get(void);
extern const struct io_filter_callbacks *gIOUCFilterCallbacks;

extern "C" volatile uint32_t native_adreno_trace_stage;
extern "C" {
/* One entry per IOServiceOpen of the accelerator (not markers): who, which type, what the two sandbox gates said. */
struct NativeAdrenoOpenRecord {char name[20]; uint32_t type, macf, res, attach, foreign;};
struct NativeAdrenoOpenRecord native_adreno_open_log[8];
volatile uint32_t native_adreno_open_log_count;
/* One row per process name that opened the plugin type (first eight names): opens granted, refused (sandbox, or no address space),
 * closes, the board log row of its live connection (4 the shared row, 0xff none), the highest GPU address its mappings reached (MiB) and its last refusal. */
struct NativeAdrenoClientRecord {char name[20]; uint32_t opens, refused, closes, slot, peak_mib, last_refusal;};
struct NativeAdrenoClientRecord native_adreno_client_log[8];
/* The last text line (the plugin's failure reason) each process other than the render server streamed: process name and a 64-byte line. */
struct NativeAdrenoClientText {char name[20]; char text[64];};
struct NativeAdrenoClientText native_adreno_client_text[8];
/* Why the latest mount of a connection failed: [0] 1 no address space, 2 the core's open, 3 the window scope; [1] its IOReturn; [2] the queue base; [3] failures so far. */
volatile uint32_t native_adreno_mount_fail[4];
/* Processes that read MetalPluginName (the Metal discovery), first twelve names, with counts. */
struct NativeAdrenoAskerRecord {char name[20]; uint32_t reads;};
struct NativeAdrenoAskerRecord native_adreno_metal_askers[12];
}
/* Every user connection's GPU addresses: its own address space from GpuClientVaStart (msm ADRENO_VM_START) to the aperture's end, above
 * the kernel buffers' fixed addresses and the GMEM window that each address space also maps, as msm's per-process VM
 * (adreno_private_vm_size: BIT(ias) - ADRENO_VM_START). A kernel connection (the board's self-test) maps where it likes. */
static uint64_t clientWindowBytes(uint32_t input_bits)
{return input_bits>32 && input_bits<=48?(UINT64_C(1)<<input_bits)-GpuClientVaStart:0;}
/* The board's qcom_scm_set_gpu_smmu_aperture result ([0] availability, [1] the call's x0: 0 is success); [2] the CP switch check's
 * status, [3] the table mode in use. */
extern "C" {volatile uint32_t native_adreno_cp_aperture[4];
/* The CP switch check: the bank's image (NativeAdrenoGpuContext::observeContext) before the switch [0..7] and after it [8..15], and the
 * kit status of each [16], [17]; a fault the switch left latched: FAR lo, hi, FSYNR0 [18..20] and the status after clearing it [21]. */
volatile uint32_t native_adreno_cp_image[22];}
/* Table switching (NativeAdrenoSubmit): 2 the CP switches in the ring (the default once the board opened the CP's SMMU aperture), 1 the
 * CPU writes TTBR0 with the GPU idle. sheng-gpu-ttbr=1 or 2 forces one. */
static unsigned tableMode()
{
    unsigned mode=native_adreno_cp_aperture[0] && !(native_adreno_cp_aperture[0]>>31) && !native_adreno_cp_aperture[1]?2:1;
    PE_parse_boot_argn("sheng-gpu-ttbr",&mode,sizeof mode);
    return mode==2?2:1;
}

namespace {
void shiftName(char names[3][17]) {
    memcpy(names[2], names[1], 17);
    memcpy(names[1], names[0], 17);
    proc_selfname(names[0], 17);
}
struct Published {
    bool published=false;
    NativeAdrenoVm *vm=nullptr;
    NativeAdrenoSubmit *submit=nullptr;
    NativeAdrenoRing *ring=nullptr;
    NativeAdrenoRingClock clock{};
    const NativeAdrenoTimer *timer=nullptr;
    const NativeAdrenoAbiDeviceInfo *info=nullptr;
    IOLock *lock=nullptr;
    IOService *service=nullptr;
    NativeAdrenoGpuContext *context=nullptr;
    bool rows[4]{};   /* the board log's client rows (NativeAdrenoUserClient::setStatsSlot): 0 the render server, 1..3 the next processes */
    uint32_t next_client=1;   /* queue range n (NativeAdrenoQueueRange) per user connection; range 0 is the kernel connection's */
    bool kernel_client=false;
    uint32_t input_bits=0,output_bits=0;
    uint32_t opened=0,closed=0;
    /* Global fault and restart counts every connection reports (NativeAdrenoUserClient::status adds its own VM's faults). */
    NativeAdrenoStatus status;
    NativeAdrenoRestart restart=nullptr;
    void *restart_cookie=nullptr;
} g;
}
/* GPU hangs (msm hangcheck, NativeAdrenoSubmit::hangCheck) and restarts: [0] hangs, [1] restarts, [2] failed restarts, [3] the last hung
 * fence, [4] the last restart's IOReturn, [5] submissions replayed so far, [6] hangs with a guilty job, [7] hangs of the kernel's own VM. */
extern "C" {volatile uint32_t native_adreno_recovery[8];}
/* SMMU context faults ended before a restart (NativeAdrenoGpuContext::endFault): [0] how many, then the last one's FSR, FAR lo, FAR hi,
 * FSYNR0. A latched FSR fails every later table operation's proof of the bank, so a restart that left it would poison the tables. */
extern "C" {volatile uint32_t native_adreno_recovery_fault[5];}
/* The board's GPU restart (a6xx_recover's reset and hw_init), installed before or after publication. */
extern "C" void native_adreno_set_restart(NativeAdrenoRestart restart,void *cookie)
{
    if(g.lock)IOLockLock(g.lock);
    g.restart=restart;g.restart_cookie=cookie;
    if(g.lock)IOLockUnlock(g.lock);
}
/* msm's hangcheck timer: every DRM_MSM_HANGCHECK_DEFAULT_PERIOD (500 ms, msm_gpu.h) the ring is checked for progress; a hang restarts the
 * GPU through the board's hook and replays the other submissions (recover_worker). Without a hook a hang is only counted. */
static void nativeAdrenoHangcheck(void *,wait_result_t)
{
    for(;;) {
        IOSleep(500);
        IOLockLock(g.lock);
        bool hung=false;
        if(g.submit && !g.submit->hangCheck(&hung) && hung) {
            native_adreno_recovery[0]=native_adreno_recovery[0]+1;
            if(g.restart) {
                uint32_t fault[4]={};
                if(g.context && !g.context->endFault(fault) && fault[0]) {
                    native_adreno_recovery_fault[0]=native_adreno_recovery_fault[0]+1;
                    for(unsigned i=0;i<4;++i)native_adreno_recovery_fault[1+i]=fault[i];
                }
                NativeAdrenoRecovery result{};
                const IOReturn status=g.submit->recover(g.restart,g.restart_cookie,&g.clock,&result);
                native_adreno_recovery[3]=result.fence;native_adreno_recovery[4]=uint32_t(status?status:result.status);
                native_adreno_recovery[5]=native_adreno_recovery[5]+result.replayed;
                if(result.guilty)native_adreno_recovery[6]=native_adreno_recovery[6]+1;
                if(result.guilty==g.vm)native_adreno_recovery[7]=native_adreno_recovery[7]+1;
                if(status)native_adreno_recovery[2]=native_adreno_recovery[2]+1;
                else native_adreno_recovery[1]=native_adreno_recovery[1]+1;
                /* Only a GPU that did not come back is everyone's loss; after a restart every client goes on, as Metal's do. */
                if(status)g.status.noteFault();
                if(!status)g.status.noteSuspend();
            }
        }
        IOLockUnlock(g.lock);
    }
}

class NativeAdrenoGpuPublished : public IOService {
    OSDeclareDefaultStructors(NativeAdrenoGpuPublished)
public:
    IOReturn newUserClient(task_t owningTask,void *securityID,UInt32 type,
        OSDictionary *properties,IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(NativeAdrenoGpuPublished,IOService)

class AGXDeviceUserClient : public IOUserClient {
    OSDeclareDefaultStructors(AGXDeviceUserClient)
    bool live_=false;
    bool handoffOnly_=false;
    bool kernel_=false;
    NativeAdrenoUserClient *core_=nullptr;   /* own core; deleted in free(), so a call that raced the close sees live_ false, never freed memory */
    NativeAdrenoProcessVm *space_=nullptr;   /* a user connection's own address space */
    unsigned row_=4;
    char name_[20]{};
    /* One user-view descriptor per BO token, reused by every clientMemoryForType call. IOConnectUnmapMemory64 asks clientMemoryForType for
     * the descriptor again and unmaps only a mapping that belongs to THAT descriptor (kIOMapReference); a fresh descriptor per call made the
     * unmap a silent no-op, the user PTEs outlived BoCpuUnmap, and BoFree handed pages with a live user mapping to the VM:
     * pmap_assert_free panicked the freeing core and the panic screen parked it (the "BoFree hang", dead touch). */
    struct View {uint32_t token;IOMemoryDescriptor *descriptor;};
    /* One view per live BO token. The cache holds every BO the client has (it grows with the tokens): a BO whose view is not cached makes
     * IOConnectUnmapMemory64 (which asks clientMemoryForType again) a no-op, the user mapping outlives BoFree and the freed pages panic
     * pmap_assert_free. A cache of 64 held until the render server's hundreds of mapped BOs (int-087). */
    /* A token is its BO's entry index plus one, so it names its own slot (a walk over every slot ran on each CPU map and unmap). */
    View *viewSlot(uint32_t token) {return views_ && token && token<=view_capacity_?&views_[token-1]:nullptr;}
    View *views_=nullptr;   /* on the heap, grown to the highest token (growViews) */
    size_t view_capacity_=0;
    bool growViews(uint32_t token);
    void dropViews(uint32_t token);
    static IOReturn action(OSObject *target,void *reference,IOExternalMethodArguments *arguments);
    IOReturn run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size);
public:
    void setHandoffOnly() {handoffOnly_=true;}
    bool mount(bool kernel);
    friend IOReturn runCore(AGXDeviceUserClient *,uint32_t,const void *,size_t,void *,size_t);
    friend void nativeAdrenoMailboxPoller(void *,wait_result_t);
    IOReturn clientClose() APPLE_KEXT_OVERRIDE;
    void free() APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
        IOExternalMethodDispatch *dispatch,OSObject *target,void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(AGXDeviceUserClient,IOUserClient)

void nativeAdrenoMailboxPoller(void *,wait_result_t)
{
    thread_set_thread_name(current_thread(),"sheng-gpu-mailbox");
    uint32_t last=0;
    for(;;) {
        IOSleep(1);
        if(!gMailboxOwner || !gMailboxWords)continue;
        const uint32_t request=gMailboxWords[1];
        if(request==last || request==gMailboxWords[2])continue;
        last=request;
        const uint32_t method=gMailboxWords[3],in_size=gMailboxWords[5],out_size=gMailboxWords[6];
        IOReturn status=kIOReturnBadArgument;
        if(in_size<=0x700 && out_size<=0x700) {
            uint8_t input[0x700],output[0x700];
            memcpy(input,const_cast<uint32_t *>(gMailboxWords)+0x10,in_size);
            IOLockLock(g.lock);
            AGXDeviceUserClient *owner=gMailboxOwner;
            if(owner)owner->retain();
            IOLockUnlock(g.lock);
            status=owner?runCore(owner,method,input,in_size,output,out_size):kIOReturnNotReady;
            if(owner)owner->release();
            if(!status)memcpy(const_cast<uint32_t *>(gMailboxWords)+0x200,output,out_size);
        }
        native_adreno_metal_diag.mb_calls=native_adreno_metal_diag.mb_calls+1;
        gMailboxWords[4]=uint32_t(status);
        __asm__ volatile("dmb sy" ::: "memory");
        gMailboxWords[2]=request;
        __asm__ volatile("dmb sy" ::: "memory");
    }
}

/* The client log row of name (created for the first eight names); null when the table is full. Under g.lock. */
static NativeAdrenoClientRecord *clientRecord(const char *name)
{
    for(auto &record:native_adreno_client_log) {
        if(!record.name[0]) {strlcpy(record.name,name,sizeof record.name);record.slot=0xff;return &record;}
        if(!strncmp(record.name,name,sizeof record.name))return &record;
    }
    return nullptr;
}
bool AGXDeviceUserClient::mount(bool kernel)
{
    proc_selfname(name_,sizeof name_);
    NativeAdrenoUserClient *core=new NativeAdrenoUserClient;
    if(!core)return false;
    IOLockLock(g.lock);
    bool ok=false;
    const bool server=!kernel && !strncmp(name_,"backboardd",sizeof name_);
    NativeAdrenoProcessVm *space=nullptr;
    IOReturn opened=kIOReturnSuccess;
    if(kernel)opened=g.kernel_client?kIOReturnBusy:kIOReturnSuccess;
    else opened=g.context?g.context->createVm(&space):kIOReturnNotReady;
    NativeAdrenoVm *vm=kernel?g.vm:space?space->vm():nullptr;
    if(!opened && !vm)opened=kIOReturnNotReady;
    if(!opened)opened=core->open(NativeAdrenoAbiVersion,vm,g.submit,g.ring,g.input_bits,g.output_bits);
    if(opened!=kIOReturnSuccess) {native_adreno_mount_fail[0]=space?2:1;native_adreno_mount_fail[1]=uint32_t(opened);native_adreno_mount_fail[2]=0;native_adreno_mount_fail[3]=native_adreno_mount_fail[3]+1;}
    if(opened==kIOReturnSuccess) {
        if(g.timer)core->attachTimer(g.timer);
        if(g.info)core->attachDeviceInfo(g.info);
        core->attachStatus(&g.status);
        /* Queue ranges: the kernel connection keeps range 0; each user connection a range of its own, never reused while the system runs. */
        uint64_t base=0;
        IOReturn scoped=kIOReturnSuccess;
        if(!kernel) {
            base=(uint64_t)g.next_client*NativeAdrenoQueueRange;
            const uint64_t window=clientWindowBytes(g.input_bits);
            scoped=!g.next_client || !window?kIOReturnNoResources:core->setScope(base,GpuClientVaStart,window);
        } else scoped=core->setScope(0,0,0);
        if(scoped==kIOReturnSuccess) {
            if(!kernel)++g.next_client;
            unsigned row=4;
            if(server && !g.rows[0])row=0;
            for(unsigned i=1;!server && row==4 && i<4;++i)if(!g.rows[i])row=i;
            if(row<4)g.rows[row]=true;
            core->setStatsSlot(row);
            core_=core;core=nullptr;space_=space;space=nullptr;row_=row;kernel_=kernel;live_=true;ok=true;
            if(kernel)g.kernel_client=true;
            ++g.opened;
            if(NativeAdrenoClientRecord *record=clientRecord(name_))record->slot=row;
        } else {
            native_adreno_mount_fail[0]=3;native_adreno_mount_fail[1]=uint32_t(scoped);native_adreno_mount_fail[2]=uint32_t(base>>32);native_adreno_mount_fail[3]=native_adreno_mount_fail[3]+1;
            core->releaseAll(&g.clock,0);
        }
    }
    /* Not used by any core (a failed open or scope): nothing of it reached the GPU. */
    if(space && g.context->destroyVm(space)) {}
    /* The mailbox page and its poller serve the render server's connection only. */
    if(ok && server && row_==0) {
        if(!gMailbox) {
            gMailbox=IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,kIODirectionInOut,4096,4096);
            if(gMailbox) {gMailboxWords=static_cast<volatile uint32_t *>(gMailbox->getBytesNoCopy());memset(const_cast<uint32_t *>(gMailboxWords),0,4096);}
        }
        if(gMailboxWords) {
            memset(const_cast<uint32_t *>(gMailboxWords),0,4096);
            gMailboxWords[0]=NativeAdrenoMailboxMagic;
            if(!gPollerStarted) {
                thread_t thread=nullptr;
                if(kernel_thread_start(nativeAdrenoMailboxPoller,nullptr,&thread)==KERN_SUCCESS) {thread_deallocate(thread);gPollerStarted=true;}
            }
        }
        if(gMailboxOwner)gMailboxOwner->release();
        retain();gMailboxOwner=this;
    }
    IOLockUnlock(g.lock);
    if(core)delete core;
    return ok;
}
/* Removes the cached view(s) of token (0 = all) and tears down every user mapping of them, so the BO's pages are never freed while a user
 * PTE still points at them, whether or not the client unmapped. */
void AGXDeviceUserClient::dropViews(uint32_t token)
{
    for(;;) {
        IOMemoryDescriptor *view=nullptr;
        IOLockLock(g.lock);
        if(token) {View *v=viewSlot(token);if(v && v->descriptor && v->token==token) {view=v->descriptor;v->descriptor=nullptr;v->token=0;}}
        else if(views_)for(size_t i=0;i<view_capacity_;++i) {View &v=views_[i];if(v.descriptor) {view=v.descriptor;v.descriptor=nullptr;v.token=0;break;}}
        IOLockUnlock(g.lock);
        if(!view)return;
        /* The returned reference is the last one outside the descriptor's own list: dropping it frees the map, which unmaps the user range. */
        while(removeMappingForDescriptor(view))native_adreno_metal_diag.forced_unmaps=native_adreno_metal_diag.forced_unmaps+1;
        view->release();
    }
}
/* Room for token's view slot (tokens are entry indices plus one, so they stay below the client's object count). */
bool AGXDeviceUserClient::growViews(uint32_t token)
{
    if(!token)return true;
    if(token<=view_capacity_)return true;
    size_t size=view_capacity_?view_capacity_:64;
    while(size<token)size*=2;
    View *made=IONewZero(View,size);
    if(!made)return false;
    for(size_t i=0;i<view_capacity_;++i)made[i]=views_[i];
    if(views_)IODelete(views_,View,view_capacity_);
    views_=made;view_capacity_=size;
    return true;
}
IOReturn AGXDeviceUserClient::clientClose()
{
    dropViews(0);
    IOLockLock(g.lock);
    IOReturn status=kIOReturnSuccess;
    AGXDeviceUserClient *mailbox=nullptr;
    if(gMailboxOwner==this) {mailbox=gMailboxOwner;gMailboxOwner=nullptr;}
    if(live_) {
        const uint64_t peak=core_->windowPeak();
        status=core_->releaseAll(&g.clock,2000000);
        /* The address space goes once the GPU no longer runs it: idle, and the kernel table back in the bank if it was installed. */
        if(!status && space_)status=g.submit->forgetVm(space_->vm(),&g.clock,2000000);
        if(!status && space_) {status=g.context->destroyVm(space_);if(!status)space_=nullptr;}
        /* A failed teardown leaves the objects (and the address space) live: nothing is forced or reused. */
        if(!status) {
            live_=false;
            if(kernel_)g.kernel_client=false;
            if(row_<4)g.rows[row_]=false;
            row_=4;
            ++g.closed;native_adreno_metal_diag.closes=native_adreno_metal_diag.closes+1;
            if(NativeAdrenoClientRecord *record=clientRecord(name_)) {
                record->closes=record->closes+1;record->slot=0xff;
                if(peak>>20>record->peak_mib)record->peak_mib=uint32_t(peak>>20);
            }
        }
    }
    IOLockUnlock(g.lock);
    if(mailbox)mailbox->release();
    if(!status)terminate();
    return status;
}
void AGXDeviceUserClient::free()
{
    dropViews(0);
    if(views_) {IODelete(views_,View,view_capacity_);views_=nullptr;view_capacity_=0;}
    /* Still live only when a teardown failed: its objects stay referenced by the VM and the ring, so the core is kept, not freed. */
    if(core_ && !live_) {delete core_;core_=nullptr;}
    IOUserClient::free();
}
IOReturn AGXDeviceUserClient::clientMemoryForType(UInt32 type,IOOptionBits *options,IOMemoryDescriptor **memory)
{
    if(memory)*memory=nullptr;
    if(!memory || !options)return kIOReturnBadArgument;
    IOReturn status=kIOReturnNotFound;
    if(type==NativeAdrenoScanoutType) {
        if(!live_ && !handoffOnly_)return kIOReturnNotReady;
        IOMemoryDescriptor *scanout=native_gop_handoff_memory();
        if(!scanout)return kIOReturnNotReady;
        *memory=scanout;                        /* already retained for the caller */
        *options=kIOMapAnywhere|kIOMapInhibitCache;   /* firmware framebuffer: device memory, never cached by the CPU */
        return kIOReturnSuccess;
    }
    if(type==NativeAdrenoMailboxType) {
        if(live_ && gMailbox && gMailboxOwner==this) {
            gMailbox->retain();
            *memory=gMailbox;
            *options=kIOMapAnywhere;   /* ordinary cached memory: both sides are CPUs, so it is coherent */
            native_adreno_metal_diag.mb_maps=native_adreno_metal_diag.mb_maps+1;
            return kIOReturnSuccess;
        }
        return kIOReturnNotReady;
    }
    native_adreno_view_diag[0]=native_adreno_view_diag[0]+1;
    /* [5]: threads inside this function right now (a thread stuck here, on the gate or in a mapping, shows as a non-zero count that stays). */
    struct Inside {Inside() {__atomic_fetch_add(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);} ~Inside() {__atomic_fetch_sub(&native_adreno_view_diag[5],1,__ATOMIC_RELAXED);}} inside;
    IOLockLock(g.lock);
    if(!growViews(type)) {IOLockUnlock(g.lock);return kIOReturnNoMemory;}
    View *v=viewSlot(type);
    if(v && v->descriptor && v->token==type) {
        v->descriptor->retain();*memory=v->descriptor;*options=kIOMapAnywhere|kIOMapInhibitCache;
        native_adreno_view_diag[2]=native_adreno_view_diag[2]+1;
        IOLockUnlock(g.lock);
        return kIOReturnSuccess;
    }
    NativeAdrenoMemory *backing=live_?core_->memoryForToken(type):nullptr;
    if(backing) {
        /* The BO's pages are cached in the kernel and the GPU's PTEs are non-cacheable, so the client needs an uncached
         * view. A mapping of the buffer descriptor with a different cache mode is not reliable, so describe the same raw
         * physical spans (the ones the SMMU maps) and ask for kIOMapInhibitCache, exactly how the board code maps the
         * ring and the GMU windows. The descriptor does not retain the pages: the client must unmap before BoCpuUnmap/BoFree. */
        /* A BO's pages can come scattered, one span per page (a 64 MiB BO is 16384 of them): the descriptor takes them all, as many as the GPU's
         * own table accepts. A fixed 64-entry array made every scattered BO above 256 KiB fail to map (VK_ERROR_MEMORY_MAP_FAILED, int-084),
         * only when memory happened to be fragmented. The descriptor copies the ranges. */
        enum {MaxSpans=65536};
        const size_t count=backing->spanCount();
        if(!count || count>MaxSpans)status=kIOReturnNoResources;
        else {
            IOAddressRange *ranges=IONewData(IOAddressRange,count);
            if(!ranges)status=kIOReturnNoMemory;
            else {
                for(size_t i=0;i<count;++i)ranges[i]={backing->spans()[i].physical,backing->spans()[i].bytes};
                IOMemoryDescriptor *descriptor=IOMemoryDescriptor::withAddressRanges(ranges,(UInt32)count,
                    static_cast<IOOptionBits>(kIODirectionInOut)|static_cast<IOOptionBits>(kIOMemoryTypePhysical64),nullptr);
                IODeleteData(ranges,IOAddressRange,count);
                if(!descriptor)status=kIOReturnNoMemory;
                else {
                    bool cached=false;
                    if(v && !v->descriptor) {v->token=type;v->descriptor=descriptor;descriptor->retain();cached=true;}
                    if(!cached) {
                        /* Never hand out a view that cannot be torn down again. */
                        descriptor->release();
                        status=kIOReturnNoResources;
                    } else {
                        *memory=descriptor;
                        *options=kIOMapAnywhere|kIOMapInhibitCache;
                        native_adreno_view_diag[1]=native_adreno_view_diag[1]+1;
                        status=kIOReturnSuccess;
                    }
                }
            }
        }
    }
    IOLockUnlock(g.lock);
    if(status){native_adreno_view_diag[3]=native_adreno_view_diag[3]+1;native_adreno_view_diag[4]=(uint32_t)status;}
    return status;
}
/* Fence waits as the callers see them (int-147; method 6's own statistics count every poll): [0] waits, [1] us waited in total, [2] pauses
 * taken outside the connection lock, [3] the longest wait in us. */
extern "C" {volatile uint64_t native_adreno_wait_outer[4];}
static IOReturn runCore(AGXDeviceUserClient *connection,uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(method==NativeAdrenoMethodSetMetadata && input && input_size==sizeof(NativeAdrenoAbiMetadata)) {
        const NativeAdrenoAbiMetadata *meta=static_cast<const NativeAdrenoAbiMetadata *>(input);
        /* The raw test's line and the Turnip phase's line ("T ..." result or "W ..." watchdog sample) are kept apart, so neither hides the other. */
        const bool turnip=meta->bytes>=2 && (meta->text[0]=='T' || meta->text[0]=='W') && meta->text[1]==' ';
        char *line=turnip?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
        uint32_t n=meta->bytes<sizeof(native_adreno_metal_diag.report)-1?meta->bytes:uint32_t(sizeof(native_adreno_metal_diag.report)-1);
        memset(line,0,sizeof(native_adreno_metal_diag.report));
        memcpy(line,meta->text,n);
        native_adreno_metal_diag.reports=native_adreno_metal_diag.reports+1;
    }
    /* A call that is still running when the boot page is photographed shows as inflight = its method number + 1 (0 = none), so a hang in the
     * kernel is attributable to one ABI method. */
    native_adreno_metal_diag.inflight=method+1;
    native_adreno_metal_diag.inflight_since_calls=native_adreno_metal_diag.calls;
    IOReturn status;
    if(method==NativeAdrenoMethodWaitFence && connection->live_ && input && input_size==sizeof(NativeAdrenoAbiWaitFence) && g.clock.now_us) {
        /* int-147: a fence wait polls under the lock but pauses outside it. int-146 slept inside the lock (up to the caller's 1 ms slice per
         * call), so every other thread's submit, map or allocation waited for the GPU to finish unrelated work: a frame could not be queued
         * while the previous one ran. Each poll is the core's own zero-timeout wait (same checks, retirement and statistics); the pause is the
         * board's (until the next GPU interrupt, at most 100 us). */
        NativeAdrenoAbiWaitFence poll{};
        memcpy(&poll,input,sizeof(poll));
        const uint64_t timeout_us=poll.timeout_ns/1000,began=g.clock.now_us(g.clock.cookie);
        poll.timeout_ns=0;
        uint64_t waited=0;
        for(;;) {
            /* The connection's own core while it is live; a close that ran between two polls ends the wait. */
            IOLockLock(g.lock);
            status=connection->live_?connection->core_->dispatch(method,&poll,sizeof(poll),output,output_size,&g.clock):kIOReturnNotReady;
            IOLockUnlock(g.lock);
            waited=g.clock.now_us(g.clock.cookie)-began;
            if(status!=kIOReturnTimeout || waited>=timeout_us)break;
            native_adreno_wait_outer[2]=native_adreno_wait_outer[2]+1;
            if(!g.clock.idle || g.clock.idle(g.clock.cookie))break;
        }
        native_adreno_wait_outer[0]=native_adreno_wait_outer[0]+1;native_adreno_wait_outer[1]=native_adreno_wait_outer[1]+waited;
        if(waited>native_adreno_wait_outer[3])native_adreno_wait_outer[3]=waited;
    } else {
        IOLockLock(g.lock);
        status=connection->live_?connection->core_->dispatch(method,input,input_size,output,output_size,&g.clock):kIOReturnNotReady;
        if(method==NativeAdrenoMethodBoMap && !status && connection->live_ && !connection->kernel_)
            if(NativeAdrenoClientRecord *record=clientRecord(connection->name_))
                if(connection->core_->windowPeak()>>20>record->peak_mib)record->peak_mib=uint32_t(connection->core_->windowPeak()>>20);
        IOLockUnlock(g.lock);
    }
    native_adreno_metal_diag.inflight=0;
    native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
    native_adreno_metal_diag.last_selector=method;
    native_adreno_metal_diag.last_status=uint32_t(status);
    return status;
}
IOReturn AGXDeviceUserClient::run(uint32_t method,const void *input,size_t input_size,void *output,size_t output_size)
{
    if(method>=NativeAdrenoHandoffBase && method<=NativeAdrenoHandoffLast)
        return native_gop_handoff_call(method-NativeAdrenoHandoffBase,current_task(),input,input_size,output,output_size);
    if(method==NativeAdrenoExportMethod) {
        struct Piece {uint32_t generation,total,offset,bytes;};
        char caller[20]={};proc_selfname(caller,sizeof caller);
        Piece piece{};
        if(input && input_size>=sizeof(piece))memcpy(&piece,input,sizeof(piece));
        const bool valid=!output_size && input && input_size>=sizeof(piece) && piece.bytes && piece.bytes<=NativeAdrenoExportMaxPiece &&
            input_size==sizeof(piece)+piece.bytes && !strncmp(caller,"backboardd",sizeof caller);
        const int stored=valid && boot_log_export_put?boot_log_export_put(piece.generation,piece.total,piece.offset,
            static_cast<const uint8_t *>(input)+sizeof(piece),piece.bytes):-1;
        if(stored) {native_adreno_export_stats[1]=native_adreno_export_stats[1]+1;return kIOReturnBadArgument;}
        native_adreno_export_stats[0]=native_adreno_export_stats[0]+1;
        native_adreno_export_stats[2]=piece.generation;native_adreno_export_stats[3]=piece.total;
        return kIOReturnSuccess;
    }
    if(method==NativeAdrenoImportMethod) {
        char caller[20]={};proc_selfname(caller,sizeof caller);
        uint64_t offset=0;
        if(input && input_size==sizeof(offset))memcpy(&offset,input,sizeof(offset));
        const uint64_t blocks=&native_ufs_import_blocks && &native_ufs_import_read?native_ufs_import_blocks():0;
        IOReturn status=kIOReturnBadArgument;
        if(!blocks)status=kIOReturnNotReady;
        else if(input_size==sizeof(offset) && !strncmp(caller,"backboardd",sizeof caller) && !(offset&4095)) {
            if(output_size==sizeof(uint64_t) && output) {
                /* The size query: an output of exactly eight bytes answers the region's bytes instead of data. */
                const uint64_t bytes=blocks*4096;memcpy(output,&bytes,sizeof(bytes));status=kIOReturnSuccess;
            } else if(output && output_size && output_size<=NativeAdrenoImportMaxBytes && !(output_size&4095) &&
                offset/4096+output_size/4096<=blocks)
                status=native_ufs_import_read(offset/4096,uint32_t(output_size/4096),output)?kIOReturnSuccess:kIOReturnIOError;
        }
        if(status) {native_adreno_import_stats[1]=native_adreno_import_stats[1]+1;return status;}
        native_adreno_import_stats[0]=native_adreno_import_stats[0]+1;
        native_adreno_import_stats[2]=native_adreno_import_stats[2]+output_size;
        return kIOReturnSuccess;
    }
    if(method==NativeAdrenoMethodBoCpuUnmap && input && input_size==sizeof(NativeAdrenoAbiBoCpuUnmap)) {
        /* The view is shared by every CPU mapping of the BO, and dropViews removes all of them. A BO mapped twice (the view cache served 207
         * second requests in int-177) lost both mappings at the first unmap, and the other holder's next store hit an unmapped page: the
         * render server's crash in the command stream writes of vkCmdCopyBuffer. Only the last lease takes the view down; the earlier
         * ones leave their mapping until it, or the client unmaps it itself (IOConnectUnmapMemory64 works on the cached view). */
        const uint32_t token=static_cast<const NativeAdrenoAbiBoCpuUnmap *>(input)->token;
        IOLockLock(g.lock);
        const uint32_t leases=live_?core_->mappingLeases(token):0;
        IOLockUnlock(g.lock);
        if(leases>1)native_adreno_view_kept=native_adreno_view_kept+1;
        else dropViews(token);
    }
    if(method==NativeAdrenoMethodBoImport) {
        /* The pages belong to the task making this call (an external method runs in the caller's context). */
        if(!input || input_size!=sizeof(NativeAdrenoAbiBoImport) || !output || output_size!=sizeof(NativeAdrenoAbiBoAllocReply))
            return kIOReturnBadArgument;
        NativeAdrenoAbiBoImport request{};
        memcpy(&request,input,sizeof(request));
        NativeAdrenoAbiBoAllocReply reply{};
        native_adreno_metal_diag.inflight=method+1;
        IOLockLock(g.lock);
        const IOReturn status=live_?core_->importBo(&request,current_task(),&reply):kIOReturnNotReady;
        IOLockUnlock(g.lock);
        native_adreno_metal_diag.inflight=0;
        native_adreno_metal_diag.calls=native_adreno_metal_diag.calls+1;
        native_adreno_metal_diag.last_selector=method;
        native_adreno_metal_diag.last_status=uint32_t(status);
        if(!status)memcpy(output,&reply,sizeof(reply));
        return status;
    }
    return runCore(this,method,input,input_size,output,output_size);
}
IOReturn AGXDeviceUserClient::action(OSObject *target,void *,IOExternalMethodArguments *arguments)
{
    AGXDeviceUserClient *connection=OSDynamicCast(AGXDeviceUserClient,target);
    if(!connection || !arguments)return kIOReturnBadArgument;
    return connection->run(arguments->selector,arguments->structureInput,arguments->structureInputSize,
        arguments->structureOutput,arguments->structureOutputSize);
}
IOReturn AGXDeviceUserClient::externalMethod(uint32_t selector,IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *,OSObject *target,void *reference)
{
    /* One entry per ABI method; sizes are the exact wire records. */
    static const IOExternalMethodDispatch table[]={
        {&action,0,0,0,sizeof(NativeAdrenoAbiParams)},
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
        {&action,0,sizeof(NativeAdrenoAbiBoCpuUnmap),0,0},
        {&action,0,0,0,sizeof(NativeAdrenoAbiDeviceInfo)},
        {&action,0,sizeof(NativeAdrenoAbiBoImport),0,sizeof(NativeAdrenoAbiBoAllocReply)}};
    native_adreno_metal_diag.ext_entries=native_adreno_metal_diag.ext_entries+1;
    if(selector<256)native_adreno_metal_diag.sel_bits[selector>>5]|=1u<<(selector&31);
    /* Tunnel. The sandbox resolves a filter policy for this class at open time and then admits only the selectors its profile lists,
     * so which selector numbers reach this object is decided outside the driver. A tunnel call carries the real ABI method in its
     * first eight bytes and may use ANY admitted selector: {magic, method, payload...} in, the method's exact output out. Method
     * 0xffffffff is a probe that only proves the selector was admitted. */
    if(arguments && arguments->structureInput && arguments->structureInputSize>=8 &&
        static_cast<const uint32_t *>(arguments->structureInput)[0]==NativeAdrenoTunnelMagic) {
        const uint32_t *header=static_cast<const uint32_t *>(arguments->structureInput);
        if(header[1]==0xffffffffu) {
            native_adreno_metal_diag.probes=native_adreno_metal_diag.probes+1;
            return kIOReturnSuccess;
        }
        native_adreno_metal_diag.tunnel_calls=native_adreno_metal_diag.tunnel_calls+1;
        const IOReturn tunneled=run(header[1],reinterpret_cast<const uint8_t *>(arguments->structureInput)+8,
            arguments->structureInputSize-8,arguments->structureOutput,arguments->structureOutputSize);
        native_adreno_metal_diag.ext_status=uint32_t(tunneled);
        return tunneled;
    }
    native_adreno_metal_diag.ext_selector=selector;
    native_adreno_metal_diag.ext_in=arguments?arguments->structureInputSize:0xffff;
    native_adreno_metal_diag.ext_out=arguments?arguments->structureOutputSize:0xffff;
    if(selector<1 || selector>sizeof(table)/sizeof(table[0])) {
        native_adreno_metal_diag.ext_status=uint32_t(kIOReturnUnsupported);
        return kIOReturnUnsupported;
    }
    const IOReturn result=IOUserClient::externalMethod(selector,arguments,const_cast<IOExternalMethodDispatch *>(&table[selector-1]),
        target?target:this,reference);
    native_adreno_metal_diag.ext_status=uint32_t(result);
    return result;
}
/* limit > 0 caps the connections ever granted: a plugin that crashes its host process on every launch (the process respawns and
 * loads it again) must not be able to keep a system process in a crash loop, so after the cap further opens are refused. */
extern "C" IOReturn native_adreno_new_user_client(IOService *provider,task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,IOUserClient **handler,unsigned limit)
{
    if(handler)*handler=nullptr;
    if(!handler || !g.published)return kIOReturnNotReady;
    /* Progress markers: open types 0x4147xxxx (xxxx nonzero) are not connections. The plugin sends one before and after each step so
     * that a sandbox that admits neither selectors nor mapping, or a crash, still leaves a trace of how far it got. */
    if((type>>16)==NativeAdrenoPluginOpenType && (type&0xffff)) {
        const uint32_t value=type&0xffff,slot=native_adreno_metal_diag.marks_total&15;
        /* Text stream into report2, one character per marker: 0x7fff starts a line, 0x70cc appends character cc. It needs no connection,
         * so the plugin's Turnip watchdog can report while Turnip itself holds its connection slot. Not counted as markers. */
        if(value>=0x8000) {
            static unsigned row=48,length;
            if(value>=0xC000) {
                row=value&0x3f;length=0;
                if(row<48)memset(native_adreno_metal_rows[row],0,sizeof(native_adreno_metal_rows[row]));
                native_adreno_metal_rows_seq=native_adreno_metal_rows_seq+1;
            } else if(row<48) {
                const char pair[2]={char((value>>7)&0x7f),char(value&0x7f)};
                for(unsigned i=0;i<2;++i)if(pair[i] && length<sizeof(native_adreno_metal_rows[row])-1)native_adreno_metal_rows[row][length++]=pair[i];
            }
            return kIOReturnUnsupported;
        }
        if(value>=0x7000 && value<0x8000) {
            /* 0x7ffe starts a line in report (GPU7) instead: the Metal bridge's result, after the raw test's line has been read. */
            static uint32_t length;
            static char *line=native_adreno_metal_diag.report2;
            static_assert(sizeof(native_adreno_metal_diag.report)==sizeof(native_adreno_metal_diag.report2),"stream lines");
            if(value==0x7fff || value==0x7ffe) {
                line=value==0x7fff?native_adreno_metal_diag.report2:native_adreno_metal_diag.report;
                /* A line from a process other than the render server goes to that process's own row (native_adreno_client_text): the render server
                 * rewrites report2 every ten seconds, so the reason another process's device failed would be gone from it. */
                char who[20]={};
                proc_selfname(who,sizeof who);
                if(value==0x7fff && strncmp(who,"backboardd",sizeof who)) {
                    for(auto &entry:native_adreno_client_text) {
                        if(!entry.name[0]) strlcpy(entry.name,who,sizeof entry.name);
                        if(!strncmp(entry.name,who,sizeof entry.name)) {line=entry.text;break;}
                    }
                }
                memset(line,0,sizeof(native_adreno_metal_diag.report2));length=0;
            }
            else if(value<0x7100 && length<sizeof(native_adreno_metal_diag.report2)-1) {line[length++]=char(value&0xff);line[length]=0;}
            return kIOReturnUnsupported;
        }
        native_adreno_metal_diag.marks[slot]=value;
        native_adreno_metal_diag.marks_total=native_adreno_metal_diag.marks_total+1;
        if(value==1) {
            memcpy(native_adreno_metal_diag.starter[2],native_adreno_metal_diag.starter[1],17);
            memcpy(native_adreno_metal_diag.starter[1],native_adreno_metal_diag.starter[0],17);
            proc_selfname(native_adreno_metal_diag.starter[0],17);
        }
        return kIOReturnUnsupported;
    }
    shiftName(native_adreno_metal_diag.opener);
    AGXDeviceUserClient *connection=new AGXDeviceUserClient;
    if(!connection)return kIOReturnNoMemory;
    if(!connection->initWithTask(owningTask,securityID,type,properties)) {
        connection->release();
        native_adreno_metal_diag.bad=native_adreno_metal_diag.bad+1;
        return kIOReturnNoResources;
    }
    /* IOService::newUserClient attaches and starts what it creates (a client without a provider chain is also a reason for the sandbox to
     * refuse). Then the two sandbox gates IOServiceOpen would apply afterwards are run here for every opener and logged with the
     * process name and open type, so the boot page shows which processes the sandbox lets reach this class and with which type. */
    unsigned attach_state=0;
    if(provider) {
        const bool attached=connection->attach(provider);
        attach_state=attached?1:2;
        if(attached && !connection->start(provider)) {
            connection->detach(provider);
            attach_state=3;
        }
    }
    native_adreno_metal_diag.attach_status=attach_state;
    const uint32_t macf=uint32_t(mac_iokit_check_open(kauth_cred_get(),connection,type));
    uint32_t resolver=0xfeedfeed;
    if(gIOUCFilterCallbacks && gIOUCFilterCallbacks->io_filter_resolver) {
        io_filter_policy_t policy=0;
        const IOReturn resolved=gIOUCFilterCallbacks->io_filter_resolver(owningTask,connection,type,&policy);
        resolver=uint32_t(resolved);
        if(resolved==kIOReturnSuccess && policy && gIOUCFilterCallbacks->io_filter_release)gIOUCFilterCallbacks->io_filter_release(policy);
    }
    native_adreno_metal_diag.chk_type=type;
    native_adreno_metal_diag.chk_macf=macf;
    native_adreno_metal_diag.chk_res=resolver;
    {
        NativeAdrenoOpenRecord &record=native_adreno_open_log[native_adreno_open_log_count&7];
        proc_selfname(record.name,sizeof(record.name));
        record.type=type;record.macf=macf;record.res=resolver;record.attach=attach_state;record.foreign=type!=NativeAdrenoPluginOpenType;
        native_adreno_open_log_count=native_adreno_open_log_count+1;
    }
    if(type==NativeAdrenoHandoffOpenType) {
        connection->setHandoffOnly();
        *handler=connection;
        return kIOReturnSuccess;
    }
    /* Only the Adreno plugin's open (type NativeAdrenoPluginOpenType) is served. The original IOGPU library opens the accelerator with type 1
     * (and an API value in the upper bits) for an IOGPUDeviceUserClient, which this driver is not; refuse it cleanly. */
    if(type!=NativeAdrenoPluginOpenType) {
        connection->release();
        native_adreno_metal_diag.foreign_opens=native_adreno_metal_diag.foreign_opens+1;
        native_adreno_metal_diag.foreign_type=type;
        return kIOReturnUnsupported;
    }
    /* Every process that runs the Adreno plugin opens with its type: on Apple hardware every Metal process opens the AGX user client, and the
     * original profiles here name no class of this driver for the render server (MACF 1 for the plugin's types at every boot since int-038).
     * The plugin's two open types are granted here, by this driver: the grant is a property of the client object that the kernel's open path
     * (patches/xnu/native-adreno-sandbox-grant.patch) honours in place of the sandbox verdict. The verdict is still taken above and logged.
     * Which processes run the plugin at all is the plugin's allow-list (NativeAdrenoMetal.clients). */
    const bool kernel=owningTask==kernel_task;
    char opener[20]={};
    proc_selfname(opener,sizeof opener);
    if(!kernel && (macf || resolver!=uint32_t(kIOReturnSuccess))) {
        connection->setProperty("NativeSandboxGrant",kOSBooleanTrue);
        native_adreno_metal_diag.grants=native_adreno_metal_diag.grants+1;
    }
    /* limit (boot-arg sheng-gpu-metal-opens, default 0 = none) caps the connections granted to each process name. No cap by default: a
     * client that crashed and restarts (iconservicesagent, i-39b429634) must get the GPU again, or it is left without one for the boot. */
    const unsigned cap=limit;
    IOLockLock(g.lock);
    NativeAdrenoClientRecord *record=kernel?nullptr:clientRecord(opener);
    const bool capped=cap && record && record->opens>=cap;
    IOLockUnlock(g.lock);
    IOReturn refusal=kIOReturnSuccess;
    if(capped)refusal=kIOReturnBusy;
    else if(!connection->mount(kernel))refusal=kIOReturnBusy;   /* no address space or queue range */
    if(refusal) {
        connection->release();
        native_adreno_metal_diag.busy=native_adreno_metal_diag.busy+1;
        IOLockLock(g.lock);
        if(record) {record->refused=record->refused+1;record->last_refusal=uint32_t(refusal);}
        IOLockUnlock(g.lock);
        return refusal;
    }
    IOLockLock(g.lock);
    if(record)record->opens=record->opens+1;
    IOLockUnlock(g.lock);
    native_adreno_metal_diag.opens=native_adreno_metal_diag.opens+1;
    *handler=connection;
    return kIOReturnSuccess;
}
extern "C" void native_adreno_note_property_read(const char *key)
{
    if(key && !strcmp(key,"MetalPluginName")) {
        proc_selfname(native_adreno_metal_diag.name_reader,17);
        native_adreno_metal_diag.name_reads=native_adreno_metal_diag.name_reads+1;
        char name[20]={};
        proc_selfname(name,sizeof name);
        /* Unlocked: the discovery runs at process start, a lost count is harmless and a row is claimed by its first byte. */
        for(auto &asker:native_adreno_metal_askers) {
            if(!asker.name[0]) {strlcpy(asker.name,name,sizeof asker.name);asker.reads=1;break;}
            if(!strncmp(asker.name,name,sizeof asker.name)) {asker.reads=asker.reads+1;break;}
        }
    }
    shiftName(native_adreno_metal_diag.reader);
    native_adreno_metal_diag.property_reads=native_adreno_metal_diag.property_reads+1;
}
IOReturn NativeAdrenoGpuPublished::newUserClient(task_t owningTask,void *securityID,UInt32 type,
    OSDictionary *properties,IOUserClient **handler)
{
    return native_adreno_new_user_client(this,owningTask,securityID,type,properties,handler,0);
}

static IOReturn checkCpSwitch()
{
    uint32_t kit=0,image[8]={};
    (void)g.context->observeContext(&kit,image);
    native_adreno_cp_image[16]=kit;for(unsigned i=0;i<8;++i)native_adreno_cp_image[i]=image[i];
    NativeAdrenoProcessVm *space=nullptr;
    IOReturn status=g.context->createVm(&space);
    if(status)return status;
    NativeAdrenoSubmitBuffer ib{};
    status=g.context->probeBuffer(space,&ib.handle,&ib.iova,&ib.bytes);
    uint32_t fence=0;
    if(!status)status=g.submit->submit(space->vm(),&ib,1,0,&fence,&g.clock);
    if(!status)status=g.submit->waitAll(&g.clock,2000000);
    uint64_t seen=0;
    if(!status)status=g.context->readTtbr0(&seen);
    if(!status && seen!=space->vm()->ttbr0())status=kIOReturnUnsupported;
    IOReturn freed=g.submit->forgetVm(space->vm(),&g.clock,2000000);
    if(!freed)freed=g.context->destroyVm(space);
    /* Every table operation proves the bank's image (TTBR0 aside): a CP switch that changed another register would fail the next one
     * and poison the kernel table. */
    const IOReturn observed=g.context->observeContext(&kit,image);
    native_adreno_cp_image[17]=kit;for(unsigned i=0;i<8;++i)native_adreno_cp_image[8+i]=image[i];
    if(observed && kit==uint32_t(ArmSmmu4KFault)) {
        uint32_t far[3]={};
        if(!g.context->clearFault(image[7],far)) {
            native_adreno_cp_image[18]=far[0];native_adreno_cp_image[19]=far[1];native_adreno_cp_image[20]=far[2];
            const IOReturn again=g.context->observeContext(&kit,image);
            native_adreno_cp_image[21]=kit|0x100;
            /* The GPU touched an address the probe's table does not map: the switch itself worked. */
            if(!again && !status && !freed)return kIOReturnSuccess;
        }
    }
    if(!status && !freed && observed)status=kIOReturnNotPermitted;
    return status?status:freed;
}
/* Publishes the service once. Everything passed in must outlive the system (the board probe keeps them in statics) and
 * the VM, submit object and ring must already be attached to the live GPU context. The core must not be open. */
extern "C" IOReturn native_adreno_publish(NativeAdrenoVm *vm,NativeAdrenoSubmit *submit,NativeAdrenoRing *ring,
    const NativeAdrenoRingClock *clock,const NativeAdrenoTimer *timer,const NativeAdrenoAbiDeviceInfo *info,
    uint32_t input_bits,uint32_t output_bits)
{
    if(g.published || !vm || !submit || !ring || !clock || !clock->now_us)return kIOReturnBadArgument;
    g.lock=IOLockAlloc();
    NativeAdrenoGpuPublished *service=new NativeAdrenoGpuPublished;
    if(!g.lock || !service || !service->init() || !service->attach(IOService::getPlatform())) {
        if(service)service->release();
        return kIOReturnNoMemory;
    }
    g.vm=vm;g.submit=submit;g.ring=ring;g.clock=*clock;g.timer=timer;g.info=info;g.input_bits=input_bits;g.output_bits=output_bits;
    /* Per-process address spaces: without table switching every user connection would be refused (createVm needs it). */
    g.context=NativeAdrenoGpuContextObject();
    IOReturn switching=g.context?g.context->enableSwitching():kIOReturnNotReady;
    if(!switching)switching=submit->setTableSwitching(tableMode(),
        [](void *cookie,uint64_t ttbr0)->IOReturn {return static_cast<NativeAdrenoGpuContext *>(cookie)->writeTtbr0(ttbr0);},g.context);
    native_adreno_table_switches[6]=uint32_t(switching);
    if(switching)g.context=nullptr;
    else if(submit->tableMode()==2) {
        /* The CP's switch must reach the bank: one NOP submission on a fresh address space, then the bank's TTBR0 is read back. A
         * switch that did not land falls back to the CPU's. */
        const IOReturn checked=checkCpSwitch();
        native_adreno_cp_aperture[2]=uint32_t(checked);
        if(checked && !submit->outstanding())(void)submit->setTableSwitching(1,
            [](void *cookie,uint64_t ttbr0)->IOReturn {return static_cast<NativeAdrenoGpuContext *>(cookie)->writeTtbr0(ttbr0);},g.context);
    }
    native_adreno_cp_aperture[3]=submit->tableMode();
    g.service=service;g.published=true;
    {
        thread_t thread=nullptr;
        if(kernel_thread_start(nativeAdrenoHangcheck,nullptr,&thread)==KERN_SUCCESS)thread_deallocate(thread);
    }
    service->setName("NativeAdrenoGpu");
    service->registerService();
    return kIOReturnSuccess;
}
extern "C" uint32_t native_adreno_publish_stats(uint32_t *closed)
{
    if(closed)*closed=g.closed;
    return g.opened;
}
extern "C" IOService *native_adreno_published_service()
{
    return g.service;
}
