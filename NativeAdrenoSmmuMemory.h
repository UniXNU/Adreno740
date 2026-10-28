/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_SMMU_MEMORY_H
#define NATIVE_ADRENO_SMMU_MEMORY_H
#include "NativeAdrenoMemory.h"
#include "ArmSmmu4KLive.h"
/* The page-table pages of one GPU address space, taken from the system one 4 KiB cacheable page at a time as the kit's walk first needs
 * them (ArmSmmu4KTablePages): Linux io-pgtable-arm allocates its tables so (__arm_lpae_alloc_pages), and Apple's pmap its page-table
 * pages. No pool is reserved and no table of the aperture exists before a mapping needs it. Pages stay until release(), which frees
 * them all; a quarantined set keeps them for good (a walk may still reach them). Serialize every call under the driver gate. */
class NativeAdrenoTablePages final {
    struct Page {uint64_t physical;volatile uint64_t *mapping;IOBufferMemoryDescriptor *descriptor;};
    Page *pages_=nullptr;   /* sorted by physical address */
    size_t count_=0,capacity_=0;
    uint64_t limit_=0;      /* every page lies below 1 << output_bits */
    bool quarantined_=false;
    ArmSmmu4KTablePages provider_{};
    size_t lowerBound(uint64_t physical) const;
    static int allocatePage(void *,uint64_t *,volatile uint64_t **);
    static volatile uint64_t *findPage(void *,uint64_t);
    static int overlapsPages(void *,uint64_t,uint64_t);
public:
    NativeAdrenoTablePages()=default;
    ~NativeAdrenoTablePages()=default;
    NativeAdrenoTablePages(const NativeAdrenoTablePages &)=delete;
    NativeAdrenoTablePages &operator=(const NativeAdrenoTablePages &)=delete;
    /* Pages from now on lie below 2^output_bits (the context's output size). Only while empty. */
    IOReturn begin(unsigned output_bits);
    const ArmSmmu4KTablePages *provider() const {return limit_?&provider_:nullptr;}
    bool contains(const volatile void *mapping) const;
    bool overlaps(uint64_t physical,uint64_t bytes) const;
    size_t pageCount() const {return count_;}
    uint64_t pagePhysical(size_t i) const {return i<count_?pages_[i].physical:0;}
    const volatile uint64_t *pageMapping(size_t i) const {return i<count_?pages_[i].mapping:nullptr;}
    void quarantine() {quarantined_=true;}
    bool quarantined() const {return quarantined_;}
    /* Frees every page, only once no walk can reach them (the ASID invalidated, the table not installed). Refused when quarantined. */
    IOReturn release();
};
/* Prepare an INACTIVE empty native4K arena whose tables come from `pages`: takes its root page, binds the arena, audits it and
 * publishes the root. Caller retains pages/arena/state/visibility under an exclusive resource gate; state starts zeroed. This does not
 * program a context, route streams, allocate MMIO/IRQs or establish power/security. On failure no context has been exposed; the
 * caller keeps the pages and the poisoned state and handles cleanup. No automatic replay. */
IOReturn NativeAdrenoPrepareSmmuMemory(NativeAdrenoTablePages *,unsigned input_bits,
    unsigned output_bits,uint64_t iova,uint64_t bytes,unsigned upper_region,
    size_t leaf_budget,ArmSmmu4KLiveArena *,ArmSmmu4KLiveState *,ArmSmmu4KLiveVisibility *);
/* Bind only the native PoC publisher to the arena's table pages. No caller-supplied success callback or page-table copy. */
IOReturn NativeAdrenoBindSmmuVisibility(NativeAdrenoTablePages *,ArmSmmu4KLiveVisibility *);
/* The stage-1 input size of a context: ID2.UBS as arm_smmu_device_cfg_probe reads it (arm_smmu_id_size_to_bits, 48 for any larger
 * code), the 4 KiB granule's 48-bit maximum (io-pgtable-arm arm_lpae_restrict_pgsizes). 0 for an unreadable ID2. */
unsigned NativeAdrenoSmmuInputBits(uint32_t id2);
#endif
