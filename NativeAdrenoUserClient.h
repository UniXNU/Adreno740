/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_USERCLIENT_H
#define NATIVE_ADRENO_USERCLIENT_H
#include "NativeAdrenoABI.h"
#include "NativeAdrenoSubmit.h"
#include "NativeAdrenoTimer.h"
#include "NativeAdrenoStatus.h"
#include "NativeAdrenoHandleIndex.h"

/* Kernel-side body of the Adreno user client, deliberately free of IOKit so it
 * can be exercised on a host. The future IOService/IOUserClient wrapper only
 * copies one fixed ABI record in and out and calls these methods; it must not
 * add policy. Handles are this object's own identities, never VM, IOVA or
 * pointer values. A mapping is retirable only through freeBo, which unmaps
 * first and then releases the backing object; both steps must succeed.
 * Objects, queues and jobs are limited by memory and ring space only (msm's are: a fixed 4096-entry table crashed backboardd, i567). */
class NativeAdrenoUserClient final {
    struct Queue {uint32_t id=0;bool active=false;};
    struct Entry {
        NativeAdrenoMemory memory;
        uint64_t handle=0;
        uint64_t mapped=0;
        uint64_t iova=0;
        bool used=false;
        bool mapped_state=false;
        bool unreleasable=false;
        bool imported=false;
        uint32_t metadata_bytes=0;
        uint32_t token=0;
        uint32_t mapping_leases=0;
        uint32_t index=0;   /* its place in the chunks: token - 1 */
        char metadata[NativeAdrenoAbiMetadataBytes]{};
    };
    /* Entries in chunks of EntryChunk, allocated as the client's objects grow (a whole table per client was about 4 MiB, and every process
     * may now open the GPU); a chunk never moves, so the VM's pointers into an entry's memory stay valid, and only the directory of chunks
     * grows. All-zero is the free state. */
    enum : size_t {EntryChunk=64};
    Entry **chunks_=nullptr;
    size_t chunk_slots_=0;   /* directory size */
    size_t entry_count_=0;   /* entries in the allocated chunks */
    Entry &at(size_t index) const {return chunks_[index/EntryChunk][index%EntryChunk];}
    bool grow();
    void releaseEntries();
    uint32_t stats_slot_=4;   /* the board log's client row (4: the shared overflow row) */
    uint64_t owned_bytes_=0,owned_peak_=0;   /* its own BOs' bytes now and at most (imports not counted), for the board log */
    NativeAdrenoHandleIndex index_;   /* live handle -> its entries_ index */
    size_t free_hint_=0,live_=0;   /* no free entry below free_hint_; live_ entries are used */
    /* Queues as msm's submitqueues: as many as the client creates (queue 0 from open), grown on demand. */
    Queue *queues_=nullptr;
    uint32_t queue_capacity_=0;
    bool growQueues();
    /* A BO is limited by the client's GPU address window and memory only (Metal's maxBufferLength is the working set, not a constant). */
    uint64_t maxBoBytes() const {return window_bytes_?window_bytes_:UINT64_C(1)<<input_bits_;}
    NativeAdrenoVm *vm_=nullptr;
    NativeAdrenoSubmit *submit_=nullptr;
    NativeAdrenoRing *ring_=nullptr;
    const NativeAdrenoTimer *timer_=nullptr;
    const NativeAdrenoStatus *status_=nullptr;
    const NativeAdrenoAbiDeviceInfo *info_=nullptr;
    uint32_t input_bits_=0,output_bits_=0;
    /* Scope on the shared ring and GPU space: this client's queues are the submit object's queues [queue_base_, +2^32), and with a
     * window (window_bytes_ nonzero) every mapping must lie inside [window_start_, +window_bytes_), which GetDeviceInfo reports as the VA. */
    uint64_t queue_base_=0;
    uint32_t context_=0;   /* this client's GPU context for NativeAdrenoRing::useContext */
    /* A client owns a queue range once setScope() gave it one or it has submitted into the default one (a single client that never scopes owns
     * range 0). Before either, queue_base_ is only the default 0, which a scoped client may own, so a check of "this client's own range" would
     * test someone else's jobs. */
    bool scoped_=false,used_=false;
    NativeAdrenoSubmitBuffer submit_buffers_[NativeAdrenoAbiMaxBuffers]{};   /* submit()'s list: too large for the kernel stack */
    NativeAdrenoAbiSubmit submit_request_{};   /* the submit request as copied in, for the same reason */
    uint64_t window_start_=0,window_bytes_=0,window_peak_=0;
    uint64_t next_handle_=1;
    bool open_=false;
    Entry *find(uint64_t);
    Entry *free();
    void clear(Entry &);
    void publish(Entry &);
public:
    NativeAdrenoUserClient()=default;
    ~NativeAdrenoUserClient()=default;
    NativeAdrenoUserClient(const NativeAdrenoUserClient &)=delete;
    NativeAdrenoUserClient &operator=(const NativeAdrenoUserClient &)=delete;
    /* Mounts one already-qualified context. input/output bits describe the
     * active native4K arena and are reported, never guessed. */
    IOReturn open(uint32_t abi_version,NativeAdrenoVm *,NativeAdrenoSubmit *,
        NativeAdrenoRing *,uint32_t input_bits,uint32_t output_bits);
    /* One client of several on the same VM, submit object and ring: its queue range (queue_base a multiple of 2^32, NativeAdrenoQueueRange) and
     * GPU address window (4 KiB aligned, inside the input range; bytes 0 = unrestricted). Only while open with no live object or job. */
    IOReturn setScope(uint64_t queue_base,uint64_t window_start,uint64_t window_bytes);
    /* The board log's per-client row this client counts in (0..3; 4 the shared row). */
    void setStatsSlot(uint32_t slot) {stats_slot_=slot<4?slot:4;}
    /* Highest end address (exclusive) any mapping of this client has reached, 0 before the first. */
    uint64_t windowPeak() const {return window_peak_;}
    /* Whether any job is unobserved in the range this client owns (never, while it owns none). */
    bool ownRangeOutstanding() const;
    /* Refuses while any object is live or any of this client's jobs is unobserved. */
    IOReturn close();
    /* Connection teardown: waits for every outstanding job, then unmaps and frees every object, closes every queue and
     * closes the client. The first failure stops it and leaves what remains live; nothing is forced. */
    IOReturn releaseAll(const NativeAdrenoRingClock *,uint32_t timeout_us);
    /* Backing object of a token returned by cpuMap while its lease is held (clientMemoryForType); null otherwise. */
    NativeAdrenoMemory *memoryForToken(uint32_t token);
    /* CPU mappings of the BO of this token that BoCpuMap handed out and BoCpuUnmap has not given back; 0 for no such BO. */
    uint32_t mappingLeases(uint32_t token) const;
    IOReturn query(NativeAdrenoAbiParams *);
    IOReturn allocBo(const NativeAdrenoAbiBoAlloc *,NativeAdrenoAbiBoAllocReply *);
    IOReturn allocBoInner(const NativeAdrenoAbiBoAlloc *,NativeAdrenoAbiBoAllocReply *);
    IOReturn mapBo(const NativeAdrenoAbiBoMap *,NativeAdrenoAbiBoMapReply *);
    /* BoImport: pins the caller's pages. Only the IOKit wrapper can call it, with the calling task; dispatch() refuses the method
     * because it has no task (the kernel mailbox path cannot import). */
    IOReturn importBo(const NativeAdrenoAbiBoImport *,void *task,NativeAdrenoAbiBoAllocReply *);
    /* Unmaps if still mapped, then releases the backing object. A failure at
     * either step keeps the entry live and unmapped where possible. */
    IOReturn freeBo(const NativeAdrenoAbiBoFree *);
    IOReturn submit(const NativeAdrenoAbiSubmit *,NativeAdrenoAbiSubmitReply *,
        const NativeAdrenoRingClock *);
    IOReturn cpuMap(const NativeAdrenoAbiBoCpuMap *,NativeAdrenoAbiBoCpuMapReply *);
    IOReturn cpuUnmap(const NativeAdrenoAbiBoCpuUnmap *);
    IOReturn createQueue(NativeAdrenoAbiQueueReply *);
    IOReturn closeQueue(const NativeAdrenoAbiQueue *);
    IOReturn setMetadata(const NativeAdrenoAbiMetadata *);
    IOReturn getMetadata(const NativeAdrenoAbiMetadata *,NativeAdrenoAbiMetadata *);
    IOReturn waitFence(const NativeAdrenoAbiWaitFence *,const NativeAdrenoRingClock *);
    /* One external-method entry: exact record sizes are required, unknown
     * methods are unsupported and no output byte is modified on failure. The
     * IOKit wrapper should only copy these buffers and call this function. */
    IOReturn dispatch(uint32_t method,const void *input,size_t input_size,
        void *output,size_t output_size,const NativeAdrenoRingClock *);
    static size_t inputSize(uint32_t method);
    static size_t outputSize(uint32_t method);
    size_t liveObjects() const;
    /* Optional GPU timestamp source; absent means the method stays unsupported. */
    IOReturn attachTimer(const NativeAdrenoTimer *);
    IOReturn timestamp(NativeAdrenoAbiTimestamp *);
    /* Optional counters; absent means the status method stays unsupported. */
    IOReturn attachStatus(const NativeAdrenoStatus *);
    IOReturn status(NativeAdrenoAbiStatus *);
    /* Optional constant device facts; absent means the method stays unsupported. */
    IOReturn attachDeviceInfo(const NativeAdrenoAbiDeviceInfo *);
    IOReturn deviceInfo(NativeAdrenoAbiDeviceInfo *);
    bool mapped(uint64_t handle);
    /* Backing object of a live handle, for the IOKit wrapper's clientMemoryForType and for kernel self-tests;
     * null for an unknown or unreleasable handle. The client keeps ownership. */
    NativeAdrenoMemory *memoryOf(uint64_t handle);
};
#endif
