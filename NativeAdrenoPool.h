/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_POOL_H
#define NATIVE_ADRENO_POOL_H
#include <IOKit/IOTypes.h>
#include <stddef.h>
#include <stdint.h>
/* System BOs from DRAM the kernel does not manage. A board whose kernel runs in a window of its DRAM registers the free runs outside
 * that window here; a write-combined system BO then takes its pages from them instead of wiring the window's memory. XNU has no
 * vm_page for such a page and phystokv panics on it, so nothing but this allocator, the SMMU tables and non-cacheable CPU mappings
 * ever names one:
 *  - the allocator's own state (a bitmap a range) lives in kernel memory;
 *  - each range has one write-combined kernel mapping, made at registration, which is the only way the kernel writes the pages;
 *  - every page of a grant is zero-filled through that mapping before the grant leaves NativeAdrenoPoolAllocate, whatever held the
 *    page before (boot firmware, another client), so a client never reads what it did not write. Freed pages keep their old bytes
 *    until then; no mapping but the range's own reaches a free page;
 *  - pages a client could map (a grant that had a client view) get their CPU cache lines dropped before that zero fill: a client can
 *    ask IOKit for a cacheable view, and a line it left dirty would otherwise be written over the next owner's data later;
 *  - a grant is reference counted: the BO holds one reference and every client view descriptor one, so its end comes only when no
 *    IOKit mapping of its pages is left;
 *  - a grant that had a client view is then retired: its pages never return to the pool. A client can copy its view's mapping in
 *    the VM (mach_vm_remap, a memory entry); the copy holds the VM object of the view's device pager, not the view, and the pmap
 *    keeps no list of the mappings of a page outside the kernel's DRAM, so the driver can neither see nor revoke it. `gpu-pool=2`
 *    gives such pages back instead (after the cache pass above), for a system whose GPU clients are trusted not to do that.
 * Lock order: the allocator's gate is a leaf. It is taken under the driver's lock, under IOKit's memory lock (a view descriptor's
 * free) or under no lock; nothing is called with it held. Registration has its own lock, taken before the gate, so the kernel
 * mapping of a new range is made without the gate. Zero fill and cache maintenance run with no lock at all. */
/* mapping: the run in its range's kernel mapping. viewed: a client could have mapped a page of it since its last zero fill. */
struct NativeAdrenoPoolRun {uint64_t physical,bytes;void *mapping;uint64_t viewed;};
struct NativeAdrenoPoolGrant {
    uint32_t references,count,capacity;
    uint8_t viewed,retired;
    size_t bytes;
    NativeAdrenoPoolRun runs[1];   /* capacity of them, count used, in BO offset order */
};
/* The board's registration: [physical, physical+bytes), whole pages, entirely outside the kernel's DRAM [gPhysBase, gPhysBase+gPhysSize)
 * and disjoint from every range registered before. Callable from any thread once IOKit runs (it allocates and maps), before or after
 * the driver starts; a range added later serves the allocations after it. Ranges are never removed.
 * kIOReturnBadArgument: not whole pages or wraps. kIOReturnNotPermitted: touches the kernel's DRAM (or the kernel's DRAM is not known
 * yet). kIOReturnBusy: overlaps a registered range. kIOReturnNoMemory: no memory for the bitmaps or no kernel mapping. */
extern "C" IOReturn native_adreno_pool_add(uint64_t physical,uint64_t bytes);
/* For the board log.
 * native_adreno_pool_state: [0] ranges, [1] KiB in them, [2] KiB free, [3] the lowest free KiB seen, [4] grants alive (BOs, and views
 * that outlive their BO), [5] their KiB, [6] allocations served, [7] grants freed back, [8] grants retired, [9] their KiB (lost to the
 * pool for good: grants a client had a view of, unless gpu-pool=2), [10] the gpu-pool boot argument (1 on, 2 on and viewed pages
 * recycled, 0 off, ~0 not read yet), [11] registrations refused.
 * native_adreno_pool_fallbacks: system BO allocations that went to the descriptor path, by reason: [0] no range (none registered, or
 * gpu-pool=0), [1] larger than the usable ranges hold in all, [2] not enough free pages, [3] fragmented (the free pages would take more
 * runs than a BO may have), [4] limit (no range lies below the client's physical address limit), [5] no kernel memory for the grant,
 * [6] kind (a BO that is not write-combined needs cacheable pages), [7] the zero fill or the cache pass before it failed.
 * native_adreno_pool_zero: [0] mach ticks zero-filling, [1] KiB zero-filled, [2] mach ticks dropping cache lines, [3] KiB of that. */
extern "C" volatile uint32_t native_adreno_pool_state[12],native_adreno_pool_fallbacks[8];
extern "C" volatile uint64_t native_adreno_pool_zero[4];
enum NativeAdrenoPoolFallback : unsigned {NativeAdrenoPoolNoRange,NativeAdrenoPoolTooLarge,NativeAdrenoPoolExhausted,
    NativeAdrenoPoolFragmented,NativeAdrenoPoolLimit,NativeAdrenoPoolNoMetadata,NativeAdrenoPoolKind,NativeAdrenoPoolScrubFailed};
/* A BO of `pages` pages has at most this many runs: sixteen up to 1 MiB, one a 64 KiB above. Past that the pool's free memory is
 * confetti and the descriptor path's single pages map as well. */
static inline size_t NativeAdrenoPoolRunLimit(size_t pages) {return pages<16?pages:pages/16>16?pages/16:16;}
/* Whether a system BO may come from the pool at all: a range is registered and the `gpu-pool` boot argument is not 0 (read once). */
bool NativeAdrenoPoolUsable();
/* bytes of zero-filled pages below `limit` (the client's physical address limit, exclusive) as one reference to a grant; the fewest
 * runs the free memory allows (one that fits best, else the largest first). Not for a caller that holds the driver's lock when it can
 * help it: the zero fill runs here. Any failure means the descriptor path and is counted in native_adreno_pool_fallbacks:
 * kIOReturnUnsupported when the pool is not for this BO (no range, gpu-pool=0, not write-combined), kIOReturnNoResources when it
 * cannot serve it now, kIOReturnNoMemory without kernel memory for the grant, kIOReturnIOError when the pages could not be made
 * clean (they go back to the pool as they were). */
IOReturn NativeAdrenoPoolAllocate(size_t bytes,uint64_t limit,bool write_combined,NativeAdrenoPoolGrant **out);
void NativeAdrenoPoolRetain(NativeAdrenoPoolGrant *);
/* Drops one reference; the last one gives the pages back to their ranges (neighbouring free pages are one run again), unless the
 * grant is retired: by NativeAdrenoPoolRetire, or because a client had a view of it and gpu-pool is not 2. */
void NativeAdrenoPoolRelease(NativeAdrenoPoolGrant *);
/* A client view of the grant's pages exists from now on: the grant is retired at its end, or with gpu-pool=2 its pages get their
 * cache lines dropped before their next zero fill. */
void NativeAdrenoPoolNoteView(NativeAdrenoPoolGrant *);
/* The grant's pages never go back to the pool: something the driver cannot revoke may still map them. */
void NativeAdrenoPoolRetire(NativeAdrenoPoolGrant *);
/* Whether any page of [physical, physical+bytes) lies in a registered range. */
bool NativeAdrenoPoolContains(uint64_t physical,uint64_t bytes);
/* The kernel services under the allocator (NativeAdrenoCpu.cpp; host doubles in tests/HostKernelHooks.cpp).
 * NativeAdrenoPoolMapRange: one write-combined kernel mapping of a whole range, kept for good; *owner takes the object that holds it.
 * NativeAdrenoPoolScrub: zero-fills [mapping, mapping+bytes) through that mapping and waits for the stores to reach DRAM; with
 * drop_lines it first drops every CPU cache line over the pages, by the mapping's addresses (data cache maintenance by address acts
 * on the lines of the physical page whatever the attributes of the mapping used). A failure leaves the pages unfit to hand out. */
void *NativeAdrenoPoolMapRange(uint64_t physical,size_t bytes,void **owner);
IOReturn NativeAdrenoPoolScrub(void *mapping,size_t bytes,bool drop_lines);
#endif
