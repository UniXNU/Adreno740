/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_GMU_CONTEXT_H
#define NATIVE_ADRENO_GMU_CONTEXT_H
#include "NativeAdrenoGmuMemory.h"
#include "NativeAdrenoSmmuContext.h"

/* The GMU's own SMMU context, in the order a6xx_gmu_memory_probe and the arm-smmu
 * attach build it: allocate the payloads at their fixed GMU addresses, prepare a
 * native4K lower-region (TTBR0) table pool for the whole 0..2 GiB GMU space,
 * program one context bank, route the GMU stream to it, attach the VM and map the
 * payloads. Bank, ASID, SMR slot and stream ID are caller inputs. Every resource
 * is retained on every outcome; there is no teardown. Caller keeps CX powered and
 * the SMMU clocks running, and keeps the GMU halted (nothing is booted here). */
struct NativeAdrenoGmuContextReport {
    uint32_t step;                  /* last step started, NativeAdrenoGmuContext::Step */
    IOReturn status;                /* of that step */
    uint32_t program_phase,program_writes,program_readback_valid,program_enabled;
    uint32_t route_phase,route_writes,routes_verified;
    uint32_t id1,id2;               /* id2 is read before any allocation */
    uint32_t output_bits;           /* from ID2.OAS, capped at 40 */
    uint32_t program_status,route_status;   /* ArmSmmu4KStatus of program/attach */
    uint32_t sctlr,cbar,smr,s2cr;   /* register readback after the last step reached */
    uint64_t ttbr0;
    uint32_t spans;
    /* The program step's last context readback and which fields differ from the encoded image: bit 0 CBAR,
     * 1 CBA2R, 2 TCR, 3 TCR2, 4 MAIR0, 5 MAIR1, 6 TTBR0, 7 TTBR1, 8 SCTLR (SCTLR is 0 before enable). */
    ArmSmmu4KContextImage image;
    uint32_t mismatch;
};
class NativeAdrenoGmuContext final {
    NativeAdrenoGmuAllocator gmu_;
    NativeAdrenoSmmuContext smmu_;
    NativeAdrenoVm vm_;
    bool ready_=false,used_=false;
public:
    /* Allocate also reads ID2 first and refuses an output size the kit cannot encode. */
    enum Step : uint32_t {Allocate=1,Prepare,Program,Attach,VmAttach,Map,Ready};
    NativeAdrenoGmuContext()=default;
    NativeAdrenoGmuContext(const NativeAdrenoGmuContext &)=delete;
    NativeAdrenoGmuContext &operator=(const NativeAdrenoGmuContext &)=delete;
    /* One attempt per object. progress (may be null) is called before each step. */
    IOReturn bringUp(const ArmSmmu4KIo *io,unsigned bank,unsigned asid,unsigned slot,
        uint16_t sid,void (*progress)(uint32_t step),NativeAdrenoGmuContextReport *report);
    bool ready() const {return ready_;}
    NativeAdrenoGmuAllocator &allocator() {return gmu_;}
    NativeAdrenoVm &vm() {return vm_;}
};
/* Board entry: binds the kit's direct MMIO adapter to an owned live mapping of the
 * whole SMMU window and runs bringUp on a context that lives forever. */
extern "C" IOReturn NativeAdrenoGmuContextBringup(volatile uint8_t *smmu,size_t bytes,
    unsigned bank,unsigned asid,unsigned slot,unsigned sid,void (*progress)(uint32_t),
    NativeAdrenoGmuContextReport *report);
/* The payloads of the context the board entry built, in ShengGmuConfig order (icache, dummy, debug,
 * log, hfi): physical base, size and GMU address of each physically contiguous object. Only after
 * that context reached Ready. The GMU maps them Normal non-cacheable (MAIR index 0), as Linux maps
 * the GMU buffers without IOMMU_CACHE; the caller gives the CPU a matching non-cacheable view. */
struct NativeAdrenoGmuPayload {uint64_t physical,bytes,iova;};
extern "C" IOReturn NativeAdrenoGmuContextPayloads(NativeAdrenoGmuPayload out[5]);
#endif
