/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_GPU_CONTEXT_H
#define NATIVE_ADRENO_GPU_CONTEXT_H
#include "NativeAdrenoGmuContext.h"

/* The GPU's own SMMU context, built like the GMU's (NativeAdrenoGmuContext): a native4K lower-region (TTBR0) table
 * pool for a 0..2 GiB GPU space, one context bank, the GPU's two streams routed to it, then the kernel buffers the
 * command processor needs at fixed GPU addresses below the A740 GMEM window (UCHE_GMEM_RANGE_MIN 16 MiB):
 *   sqe      the SQE microcode (a740_sqe.fw without its first dword, as adreno_fw_create_bo copies it), R+X
 *   ring     ring buffer 0, 32 KiB, aligned to its size, R+W
 *   memptrs  msm_rbmemptrs layout: +0 rptr, +4 fence, +8 bv_rptr, +12 bv_fence, R+W
 *   pwrup    the a7xx power-up register list (cpu_gpu_lock header + pairs), R+W
 *   scratch  one page the first submissions may write, R+W
 * All mappings are Normal non-cacheable (Linux maps kernel BOs write-combined) and outer shareable; the CPU must use
 * a matching non-cacheable view. Bank, ASID, SMR slots and stream IDs are caller inputs. Nothing is torn down. */
enum : uint64_t {
    GpuSqeIova=0x100000, GpuRingIova=0x200000, GpuMemptrsIova=0x300000, GpuPwrupIova=0x301000,
    GpuScratchIova=0x302000, GpuApertureBytes=0x80000000
};
enum : size_t {GpuSqeBytes=0x13000, GpuRingBytes=0x8000, GpuPageBytes=0x1000, GpuBufferCount=5};
struct NativeAdrenoGpuBuffer {uint64_t physical,bytes,iova;};
/* One GPU process's own address space: a table pool of the kernel table's shape (the same 0..2 GiB aperture and attributes) under its
 * own ASID, switched into the bank's TTBR0 while the GPU runs that process's work (NativeAdrenoSubmit table switching). The kernel
 * buffers are mapped at the same GPU addresses as in the kernel table, so the CP finds its ring, SQE and memptrs whatever table runs. */
class NativeAdrenoProcessVm final {
    friend class NativeAdrenoGpuContext;
    NativeAdrenoMemory tables_;
    int table_block_=-1;   /* the cacheable table block it borrowed (NativeAdrenoGpuContext::table_blocks_), -1: its tables came from elsewhere */
    ArmSmmu4KLiveArena arena_{};
    ArmSmmu4KLiveState state_{};
    ArmSmmu4KLiveVisibility visibility_{};
    ArmSmmu4KContextConfig config_{};
    NativeAdrenoVm vm_;
    uint64_t handles_[GpuBufferCount]{};
    unsigned asid_=0;
    bool attached_=false;
public:
    NativeAdrenoProcessVm()=default;
    NativeAdrenoProcessVm(const NativeAdrenoProcessVm &)=delete;
    NativeAdrenoProcessVm &operator=(const NativeAdrenoProcessVm &)=delete;
    NativeAdrenoVm *vm() {return attached_?&vm_:nullptr;}
};
class NativeAdrenoGpuContext final {
    NativeAdrenoMemory tables_,buffers_[GpuBufferCount];
    uint64_t handles_[GpuBufferCount]{};
    NativeAdrenoSmmuContext smmu_;
    NativeAdrenoVm vm_;
    /* The alias checks' sort scratch: one entry per range of the smaller compared side, at most half the aperture's leaves. */
    uint32_t alias_scratch_[NativeAdrenoSmmuContext::MaxLeaves/2];
    bool ready_=false,used_=false;
public:
    enum Step : uint32_t {Allocate=1,Prepare,Program,Attach,VmAttach,Map,Ready};
    enum : size_t {TablePages=1027};
    NativeAdrenoGpuContext()=default;
    NativeAdrenoGpuContext(const NativeAdrenoGpuContext &)=delete;
    NativeAdrenoGpuContext &operator=(const NativeAdrenoGpuContext &)=delete;
    /* The report is the GMU context's: route counts cover both streams. */
    IOReturn bringUp(const ArmSmmu4KIo *io,unsigned bank,unsigned asid,const unsigned slots[2],
        const uint16_t sids[2],void (*progress)(uint32_t step),NativeAdrenoGmuContextReport *report);
    bool ready() const {return ready_;}
    const NativeAdrenoMemory &buffer(unsigned index) const {return buffers_[index];}
    NativeAdrenoVm *vm() {return ready_?&vm_:nullptr;}
    /* From now on the bank's TTBR0 may hold a process table: the kernel table's updates stop comparing the bank's TTBR0 with it. */
    IOReturn enableSwitching();
    /* The CPU's switch: TTBR0 (root | ASID << 48) into the bank. Every GPU master quiesced (the submit object waited for its jobs). */
    IOReturn writeTtbr0(uint64_t ttbr0);
    /* A new process address space (ASIDs 16..65535: the kernel's is the bring-up input); null when memory or ASIDs ran out. */
    IOReturn createVm(NativeAdrenoProcessVm **out);
    /* Only with every BO unmapped, the GPU idle and the table not installed (NativeAdrenoSubmit::forgetVm): drops its TLB entries, frees
     * the table and the ASID. A failure keeps the object (and its memory) for good. */
    IOReturn destroyVm(NativeAdrenoProcessVm *);
    uint32_t liveVms() const {return live_vms_;}
    /* The bank's register image against the kernel table's context (TTBR0 excepted): the kit status, and out[] = SCTLR, TCR, TCR2,
     * MAIR0, TTBR1 lo, TTBR1 hi, CBAR, FSR as read. */
    IOReturn observeContext(uint32_t *kit_status,uint32_t out[8]) const;
    /* Writes a latched fault status back (FSR is write-one-to-clear); *far gets FAR and FSYNR0 first (lo, hi, fsynr0). */
    IOReturn clearFault(uint32_t fsr,uint32_t far[3]);
    /* The bank's TTBR0 as the SMMU holds it now. */
    IOReturn readTtbr0(uint64_t *out) const;
    /* A one-dword indirect buffer (CP_NOP) in the scratch page of p's address space, for the CP table-switch check. */
    IOReturn probeBuffer(NativeAdrenoProcessVm *p,uint64_t *handle,uint64_t *iova,size_t *bytes);
private:
    /* Page tables of the process address spaces: blocks of TablePages pages cut from one cacheable contiguous allocation made at bring-up
     * (sheng: before the BO pool fragments the VM). The kit reads descriptors on every map and unmap; from the write-combined BO pool each
     * read went to DRAM uncached (int-153: 2.6x slower table updates). A VM beyond the blocks takes its tables from the BO pool. */
    static constexpr size_t TableBlocks=16;
    IOBufferMemoryDescriptor *table_blocks_=nullptr;
    unsigned char *table_virt_=nullptr;
    uint64_t table_physical_=0;
    uint32_t table_used_=0;
    uint64_t asids_[1024]{};   /* bit n: ASID n in use */
    uint32_t live_vms_=0;
    unsigned kernel_asid_=0;
};
extern "C" IOReturn NativeAdrenoGpuContextBringup(volatile uint8_t *smmu,size_t bytes,unsigned bank,unsigned asid,
    unsigned slot0,unsigned sid0,unsigned slot1,unsigned sid1,void (*progress)(uint32_t),
    NativeAdrenoGmuContextReport *report);
/* sqe, ring, memptrs, pwrup, scratch: physical base, size and GPU address, only after Ready. */
extern "C" IOReturn NativeAdrenoGpuContextBuffers(NativeAdrenoGpuBuffer out[5]);
/* The live VM of the ready GPU context (null before Ready), for the user-client core. */
extern "C" NativeAdrenoVm *NativeAdrenoGpuContextVm();
/* The board's context, for the published service's per-process address spaces (null before bring-up). */
extern "C" NativeAdrenoGpuContext *NativeAdrenoGpuContextObject();
#endif
