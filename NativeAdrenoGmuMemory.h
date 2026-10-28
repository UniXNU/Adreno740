/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_GMU_MEMORY_H
#define NATIVE_ADRENO_GMU_MEMORY_H
#include "NativeAdrenoMemory.h"
#include "NativeAdrenoVm.h"

#include "NativeAdrenoSmmuMemory.h"
/* Backing objects for the GMU cold start, laid out at the fixed IOVAs of the
 * GMU's own address space (0..2 GiB, the lower/TTBR0 region of its context
 * bank, as a6xx_gmu_memory_probe creates it): instruction cache payload, dummy,
 * debug and log buffers, HFI queues, plus the native4K table pages that will
 * translate them (taken on demand, no IOVA). Sizes are the ones the qualified GMU startup uses. All
 * objects are physically contiguous and page aligned; the allocator only
 * assigns addresses and never maps them or touches the SMMU. Any failure
 * releases what it allocated and leaves no layout. */
enum : uint64_t {
    GmuIcacheIova=0x4000, GmuUncachedBase=0x60000000, GmuDebugIova=0x60400000,
    GmuApertureBytes=0x80000000
};
struct NativeAdrenoGmuSizes {
    uint64_t icache,dummy,debug,log,hfi;
};
/* Exact sizes of the qualified startup. */
NativeAdrenoGmuSizes NativeAdrenoGmuDefaultSizes();
struct NativeAdrenoGmuMemoryLayout {
    uint64_t icache_iova,dummy_iova,debug_iova,log_iova,hfi_iova;
    uint64_t end_iova;            /* first address after the whole reservation */
};
class NativeAdrenoGmuAllocator final {
    NativeAdrenoMemory icache_,dummy_,debug_,log_,hfi_;
    NativeAdrenoTablePages tables_;
    NativeAdrenoGmuMemoryLayout layout_{};
    uint64_t handles_[5]{};
    bool ready_=false;
    IOReturn releaseAll();
public:
    NativeAdrenoGmuAllocator()=default;
    ~NativeAdrenoGmuAllocator()=default;
    NativeAdrenoGmuAllocator(const NativeAdrenoGmuAllocator &)=delete;
    NativeAdrenoGmuAllocator &operator=(const NativeAdrenoGmuAllocator &)=delete;
    /* Every size must be a nonzero 4KiB multiple and fit its fixed range. Every page lies below
     * 2^address_bits: the SMMU output size (ID2.OAS) the context will use. */
    IOReturn allocate(const NativeAdrenoGmuSizes &sizes,unsigned address_bits=40);
    /* Map every payload into the VM's lower region at its assigned IOVA with
     * read/write access (icache also privileged and executable). The VM must already be attached with a lower arena
     * covering the reservation. Handles are retained for unmapFrom. */
    IOReturn mapInto(NativeAdrenoVm *vm);
    /* Reverse order unmap; refuses while a mapping has outstanding jobs. */
    IOReturn unmapFrom(NativeAdrenoVm *vm);
    /* Only when every object has been released; on any refusal nothing is
     * destroyed and the caller keeps the quarantined objects alive. */
    IOReturn release();
    bool ready() const {return ready_;}
    const NativeAdrenoGmuMemoryLayout &layout() const {return layout_;}
    NativeAdrenoMemory &icache() {return icache_;}
    NativeAdrenoMemory &dummy() {return dummy_;}
    NativeAdrenoMemory &debug() {return debug_;}
    NativeAdrenoMemory &log() {return log_;}
    NativeAdrenoMemory &hfi() {return hfi_;}
    NativeAdrenoTablePages &tables() {return tables_;}
    const NativeAdrenoMemory &icache() const {return icache_;}
    const NativeAdrenoMemory &dummy() const {return dummy_;}
    const NativeAdrenoMemory &debug() const {return debug_;}
    const NativeAdrenoMemory &log() const {return log_;}
    const NativeAdrenoMemory &hfi() const {return hfi_;}
    const NativeAdrenoTablePages &tables() const {return tables_;}
};
#endif
