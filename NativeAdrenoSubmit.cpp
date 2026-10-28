/* SPDX-License-Identifier: MIT */
#include "NativeAdrenoSubmit.h"
#include "AdrenoPM4.h"
#include <IOKit/IOLib.h>

/* CP_INDIRECT_BUFFER is opcode 0x3f with a 3-word payload: 64-bit GPU address
 * and IB_SIZE in dwords (bits 0..19), exactly as the pinned adreno_pm4.xml
 * domain states and Linux msm_gem_submit.c converts the userspace byte size. */
enum : uint32_t {SubmitIndirectBuffer=0x3f,SubmitIbWords=3,SubmitMaxJobs=8,
    SubmitMaxBuffers=(uint32_t)NativeAdrenoSubmitMaxBuffers,SubmitMaxIbBytes=0xfffffu*4u,
    SubmitAuditBytes=256u*1024u};   /* IB bytes the write-combine audit compares per submission */
extern "C" {volatile uint32_t native_adreno_coherence[8];}
/* IBs per submission: [0] submissions, [1] IBs, [2] the most in one, [3] submissions with more than 16 (one frame was several before). */
extern "C" {volatile uint32_t native_adreno_submit_ibs[4];}
/* The submission (1-based count) at which the write-combine audit found a dirty IB line or failed; 0 = never. */
extern "C" {volatile uint32_t native_adreno_wc_distrusted;}
/* The write-combine audit before each submission while trusted: [0] submissions audited, [1] IB lines compared, [2] dirty lines,
 * [3] stale clean lines, [4] the last audit failure (any failure also ends the trust), [5] pages found back at the default attribute
 * and write-combined again. */
extern "C" {volatile uint32_t native_adreno_wc_audit[6];}
/* Fence waits: [0] calls, [1] timeouts, [2] the fence of the last timed-out wait, [3] the fence value the GPU had written then, [4] the newest fence handed out,
 * [5] the newest completed, [6] waits for a fence no job holds (observeFence path), [7] the last fence refused as never submitted. */
extern "C" {volatile uint32_t native_adreno_fence_wait[8];}
/* The CP's state once a fence stopped advancing (NativeAdrenoRing::stallState): [0] the fence, [1] the timed-out waits of it so far, then
 * the ring's words. Taken at the 10th and again at the 1000th timed-out wait of the fence that is the next to complete. native_adreno_stall_mem
 * holds, from the same moment, the SMMU context's FSR, FAR (lo, hi) and FSYNR0, then for the BR IB1, the BV IB1 and the BV IB2 17 words: the
 * dword index in the IB where the 16 words that follow start (the IB1s are found from the ring packet before each read pointer, so the index
 * counts from the IB's start; for the BV IB2 only the fetch address is known and the index is 0xffff0000 + words before it), then the words.
 * The last 4: the map handle that was next when the stalled job was submitted, then the handles of the mappings under the BR IB1, the BV IB1
 * and the BV IB2 (0: unmapped). A handle at or above the first was mapped after the submission: its memory was freed and reused under the GPU. */
extern "C" {volatile uint32_t native_adreno_stall[2+NativeAdrenoRingStallWords];volatile uint32_t native_adreno_stall_mem[4+3*17+4];}
/* 16 words of the IB at start, from 12 dwords before the dword the thread has reached (size minus what remains). */
static void peekIb(const NativeAdrenoVm *vm,uint64_t start,uint32_t size,uint32_t remaining,volatile uint32_t *slot)
{
    uint32_t words[16]={};
    const uint32_t reached=remaining<=size?size-remaining:0,first=reached>12?reached-12:0;
    if(vm->peekWordsUncached(start+4ull*first,words,16))for(unsigned i=0;i<16;++i)words[i]=0xdeadbeef;
    slot[0]=first;
    for(unsigned i=0;i<16;++i)slot[1+i]=words[i];
}
/* 16 words around a fetch address whose IB start is unknown: from 12 words before it, or nearer when that crosses into no buffer. */
static void peekFetch(const NativeAdrenoVm *vm,uint64_t fetch,volatile uint32_t *slot)
{
    uint32_t words[16]={},before=12;
    while(before && vm->peekWordsUncached(fetch-4*before,words,16))before-=4;
    if(!before && vm->peekWordsUncached(fetch,words,16))for(unsigned i=0;i<16;++i)words[i]=0xdeadbeef;
    slot[0]=0xffff0000u+before;
    for(unsigned i=0;i<16;++i)slot[1+i]=words[i];
}
static uint32_t stall_fence,stall_timeouts,stall_stage;   /* stage: 0 none yet, 1 write pointer given again, 2 translations dropped, 3 bus drained */
static uint64_t stall_began_us,stall_last_us;   /* the fence's first and latest timed-out wait */
/* How long fences stayed pending from their first timed-out wait to their last: [i] under 1 << i ms for i < 7, [7] 64 ms or more. */
extern "C" {volatile uint32_t native_adreno_fence_pending[8];}
/* What the stall handler tried: [0] attempts, [1] the fence of the last attempt, [2] attempts after which that fence completed, [3] the
 * timed-out waits that fence had seen when it did. */
/* [4] bus drains, [5] client halt acknowledge, [6] arbiter halt acknowledge, [7] the fence of the last drain, [8] drains after which that fence completed */
extern "C" {volatile uint32_t native_adreno_unstick[15];}
/* The IBs of the last submissions: [slot] fence, iova low/high, bytes, then the first 28 dwords of the first IB (read as the kernel maps it). */
extern "C" {volatile uint32_t native_adreno_ib_log[4][4+28];volatile uint32_t native_adreno_ib_log_count;}
extern "C" volatile uint32_t native_adreno_table_switches[8];
IOReturn NativeAdrenoSubmit::poison()
{
    poisoned_=true;return kIOReturnIOError;
}
NativeAdrenoSubmit::Job *NativeAdrenoSubmit::find(uint32_t fence)
{
    if(fence)for(auto &job:jobs_)if(job.active && job.fence==fence)return &job;
    return nullptr;
}
NativeAdrenoSubmit::Job *NativeAdrenoSubmit::free()
{
    for(auto &job:jobs_)if(!job.active)return &job;
    return nullptr;
}
IOReturn NativeAdrenoSubmit::attach(NativeAdrenoVm *vm,NativeAdrenoRing *ring)
{
    if(attached_ || poisoned_ || vm_ || ring_)return kIOReturnBusy;
    if(!vm || !ring || vm->poisoned() || ring->poisoned())return kIOReturnBadArgument;
    vm_=vm;ring_=ring;attached_=true;stats_={};table_vm_=vm;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::setTableSwitching(unsigned mode,IOReturn (*write)(void *,uint64_t),void *cookie)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(mode>2 || (mode && !write) || outstanding())return kIOReturnBadArgument;
    table_mode_=mode;write_table_=write;table_cookie_=cookie;table_vm_=vm_;
    if(mode==2)ring_->tableInstalled(vm_->ttbr0());
    return kIOReturnSuccess;
}
/* Makes vm the table the GPU runs for the next submission. Mode 1: once every earlier job has finished, the CPU writes TTBR0 (the bank
 * caches translations per ASID, so the switch needs no TLB operation). Mode 2: the ring's next prologue carries the switch, which the CP
 * runs after every earlier submission. */
IOReturn NativeAdrenoSubmit::useVm(NativeAdrenoVm *vm,const NativeAdrenoRingClock *clock)
{
    if(!table_mode_ || vm==table_vm_)return kIOReturnSuccess;
    if(table_mode_==2) {ring_->useTable(vm->ttbr0());table_vm_=vm;return kIOReturnSuccess;}
    if(!clock)return kIOReturnBadArgument;
    IOReturn status=waitAll(clock,2000000);
    if(status)return status;
    status=write_table_(table_cookie_,vm->ttbr0());
    if(status)return poison();
    native_adreno_table_switches[1]=native_adreno_table_switches[1]+1;
    table_vm_=vm;return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::forgetVm(NativeAdrenoVm *vm,const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!vm || vm==vm_)return kIOReturnBadArgument;
    IOReturn status=waitAll(clock,timeout_us);
    if(status)return status;
    if(table_vm_!=vm && !(table_mode_==2 && ring_->table()==vm->ttbr0()))return kIOReturnSuccess;
    /* The GPU is idle and every queued switch has run: the CPU puts the default table back. */
    status=write_table_?write_table_(table_cookie_,vm_->ttbr0()):kIOReturnNotReady;
    if(status)return poison();
    native_adreno_table_switches[1]=native_adreno_table_switches[1]+1;
    if(table_mode_==2)ring_->tableInstalled(vm_->ttbr0());
    table_vm_=vm_;return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::release(Job &job)
{
    IOReturn status=kIOReturnSuccess;
    NativeAdrenoVm *vm=job.vm?job.vm:vm_;
    for(size_t i=0;i<job.count;++i) {
        IOReturn retired=vm->retireJobAfterCompletion(job.handles[i]);
        if(retired)status=retired;else ++stats_.references_retired;
    }
    if(status)return poison();
    if(vm->acquireImported())return poison();
    job={};return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::detach()
{
    if(poisoned_ || outstanding())return kIOReturnBusy;
    if(!attached_)return kIOReturnNotReady;
    attached_=false;vm_=nullptr;ring_=nullptr;return kIOReturnSuccess;
}
bool NativeAdrenoSubmit::queueOutstanding(uint32_t queue) const
{
    return rangeOutstanding(queue,1);
}
bool NativeAdrenoSubmit::rangeOutstanding(uint32_t first,uint32_t count) const
{
    for(const auto &job:jobs_)if(job.active && job.queue-first<count)return true;
    return false;
}
size_t NativeAdrenoSubmit::outstanding() const
{
    size_t count=0;for(const auto &job:jobs_)if(job.active)++count;return count;
}
IOReturn NativeAdrenoSubmit::submit(const NativeAdrenoSubmitBuffer *buffers,size_t count,
    uint32_t queue,uint32_t *fence)
{
    return submit(nullptr,buffers,count,queue,fence,nullptr);
}
IOReturn NativeAdrenoSubmit::submit(NativeAdrenoVm *vm,const NativeAdrenoSubmitBuffer *buffers,size_t count,
    uint32_t queue,uint32_t *fence,const NativeAdrenoRingClock *clock)
{
    if(!fence)return kIOReturnBadArgument;*fence=0;
    if(!vm)vm=vm_;
    NativeAdrenoVm *const vm_=vm;   /* this submission's VM: every vm_ below is the job's own */
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!buffers || !count || count>SubmitMaxBuffers)return kIOReturnBadArgument;
    native_adreno_submit_ibs[0]=native_adreno_submit_ibs[0]+1;native_adreno_submit_ibs[1]=native_adreno_submit_ibs[1]+(uint32_t)count;
    if(count>native_adreno_submit_ibs[2])native_adreno_submit_ibs[2]=(uint32_t)count;
    if(count>16)native_adreno_submit_ibs[3]=native_adreno_submit_ibs[3]+1;
    /* Every buffer is validated against the live VM before any reference. */
    uint64_t *handles=scratch_handles_;size_t handle_count=0;
    uint32_t *words=scratch_words_;size_t word_count=0;
    static_assert(sizeof(scratch_words_)/sizeof(scratch_words_[0])>=SubmitMaxBuffers*(1+SubmitIbWords),"ring words");
    for(size_t i=0;i<count;++i) {
        const NativeAdrenoSubmitBuffer &buffer=buffers[i];
        if(!buffer.handle || !buffer.bytes || (buffer.bytes&3) || buffer.bytes>SubmitMaxIbBytes)
            return kIOReturnBadArgument;
        uint64_t iova=0;size_t bytes=0;unsigned permissions=0;
        IOReturn status=vm_->describe(buffer.handle,&iova,&bytes,&permissions);
        if(status)return status;
        if(!(permissions&ArmSmmu4KRead) || buffer.iova<iova ||
            buffer.iova-iova>bytes || buffer.bytes>bytes-(buffer.iova-iova))return kIOReturnBadArgument;
        uint32_t header=0;
        if(AdrenoPM4Type7(SubmitIndirectBuffer,SubmitIbWords,&header)!=AdrenoPM4Success)
            return poison();
        words[word_count++]=header;
        words[word_count++]=(uint32_t)buffer.iova;
        words[word_count++]=(uint32_t)(buffer.iova>>32);
        words[word_count++]=(uint32_t)(buffer.bytes/4);
        bool known=false;
        for(size_t j=0;j<handle_count;++j)if(handles[j]==buffer.handle) {known=true;break;}
        if(!known)handles[handle_count++]=buffer.handle;
    }
    IOReturn swept=retireCompleted();
    if(swept)return swept;
    Job *job=free();
    if(!job)return kIOReturnBusy;
    for(size_t i=0;i<handle_count;++i) {
        IOReturn status=vm_->holdJob(handles[i]);
        if(status) {
            while(i) {vm_->retireJobAfterCompletion(handles[--i]);++stats_.references_retired;}
            return status;
        }
        ++stats_.references_held;
    }
    uint32_t value=0;
    /* int-070: the first IB word as the kernel's cacheable alias and DRAM see it before any maintenance, then DRAM after the CPU
     * caches over every mapped BO are cleaned and invalidated. The board shows the last submission's record on a stall. */
    uint32_t cached=~0u,before=~0u,ignored=0,after=~0u;
    /* Write-combined stores this CPU made before the call (the client's command words) complete before anything below. */
    __asm__ volatile("dsb sy":::"memory");
    /* int-098/099: with write-combined user BOs and no maintenance the CP stopped at the end of the first IB, as in int-065..069 when an
     * IB sat in a CPU cache. While write-combined BOs are trusted, every line of this submission's IBs is compared as a CPU cache and
     * DRAM hold it (bounded). A dirty line means a client view still caches its BOs: maintain every BO from now on, this submission
     * included (the audit already wrote the IB lines back; the rest of the BOs may hold more). */
    if(vm_->writeCombinedTrusted()) {
        /* int-100: the dirty IB page was back at XNU's default (cacheable) attribute. Write-combine every such page again first; the
         * pmap writes its lines back on the way, so the audit below should then find none. */
        uint32_t reset=0;
        IOReturn audited=vm_->reassertWriteCombined(&reset);
        native_adreno_wc_audit[5]=native_adreno_wc_audit[5]+reset;
        uint32_t lines=0,dirty=0,stale=0;size_t budget=SubmitAuditBytes;
        /* The re-assertion above is the correction and runs every time; the audit only detects. It found no dirty line in int-141's
         * 133798 nor in i566's 2765132 (1358 audits of every eighth submission, about 2 ms each under the publish lock, so every eighth
         * frame of every client waited), so only the first 16 submissions are compared now. */
        ++audit_tick_;
        const bool audit_now=audit_tick_<=16;
        if(!audit_now)budget=0;
        for(size_t i=0;i<count && budget && !audited;++i) {   /* a failed re-assertion skips the audit and ends the trust */
            const size_t bytes=buffers[i].bytes<budget?(size_t)buffers[i].bytes:budget;
            audited=vm_->auditRange(buffers[i].iova,bytes,&lines,&dirty,&stale);budget-=bytes;
        }
        if(audit_now && audited!=kIOReturnUnsupported) {   /* no board hook (host): nothing to compare */
            native_adreno_wc_audit[0]=native_adreno_wc_audit[0]+1;
            native_adreno_wc_audit[1]=native_adreno_wc_audit[1]+lines;
            native_adreno_wc_audit[2]=native_adreno_wc_audit[2]+dirty;
            native_adreno_wc_audit[3]=native_adreno_wc_audit[3]+stale;
            if(audited)native_adreno_wc_audit[4]=audited;
        }
        if(dirty || (audited && audited!=kIOReturnUnsupported)) {
            vm_->distrustWriteCombined();
            native_adreno_wc_distrusted=native_adreno_coherence[0]+1;
        }
    }
    /* The coherence sample reads the first IB's word through a fresh uncached mapping before and after the publication: every 64th
     * submission only (two such mappings per submission were CPU time on every frame). */
    static uint32_t coherence_turn;
    const bool sample=!(coherence_turn++&63);
    if(sample)(void)vm_->sampleWord(buffers[0].iova,&cached,&before);
    IOReturn status=vm_->publishMapped();
    if(!status && sample) {
        (void)vm_->sampleWord(buffers[0].iova,&ignored,&after);
        native_adreno_coherence[0]=native_adreno_coherence[0]+1;
        native_adreno_coherence[1]=cached;native_adreno_coherence[2]=before;native_adreno_coherence[3]=after;
        if(cached!=before || before!=after) {
            if(!native_adreno_coherence[4]) {
                native_adreno_coherence[5]=cached;native_adreno_coherence[6]=before;native_adreno_coherence[7]=after;
            }
            native_adreno_coherence[4]=native_adreno_coherence[4]+1;
        }
    }
    /* No TLB invalidation before a submission: a map fills only invalid leaves (no TLB holds an invalid descriptor), an unmap drops
     * its own pages before it returns, and the tables above the leaves never change. The TLBIALL here emptied the GPU's TLB on every
     * submission (10944 in i568), so each frame began with page walks. A stalled fence still invalidates (below). */
    if(status) {
        for(size_t i=0;i<handle_count;++i) {vm_->retireJobAfterCompletion(handles[i]);++stats_.references_retired;}
        return status;
    }
    {
        const uint32_t slot=native_adreno_ib_log_count&3;uint32_t head[28];
        (void)vm_->peekWords(buffers[0].iova,head,28);
        native_adreno_ib_log[slot][0]=ring_->lastSubmitted()+1;native_adreno_ib_log[slot][1]=(uint32_t)buffers[0].iova;
        native_adreno_ib_log[slot][2]=(uint32_t)(buffers[0].iova>>32);native_adreno_ib_log[slot][3]=(uint32_t)buffers[0].bytes;
        for(unsigned i=0;i<28;++i)native_adreno_ib_log[slot][4+i]=head[i];
        native_adreno_ib_log_count=native_adreno_ib_log_count+1;
    }
    status=useVm(vm,clock);
    if(status) {
        for(size_t i=0;i<handle_count;++i) {vm_->retireJobAfterCompletion(handles[i]);++stats_.references_retired;}
        return status;
    }
    status=ring_->submitFramed(words,word_count,&value);
    if(status) {
        /* A poisoned ring may already have consumed the WPTR write, so the
         * referenced buffers cannot be retired here. */
        if(ring_->poisoned()) {
            *job=Job{};++stats_.poisoned_jobs;
            for(size_t i=0;i<handle_count;++i)job->handles[i]=handles[i];
            job->count=handle_count;job->queue=queue;job->active=true;job->vm_next=vm_->nextHandle();job->vm=vm;
            return poison();
        }
        while(handle_count) {vm_->retireJobAfterCompletion(handles[--handle_count]);++stats_.references_retired;}
        return status;
    }
    for(size_t i=0;i<handle_count;++i)job->handles[i]=handles[i];
    job->count=handle_count;job->fence=value;job->queue=queue;job->active=true;job->vm_next=vm_->nextHandle();job->vm=vm;
    ++stats_.submissions;
    if(outstanding()>stats_.peak_jobs)stats_.peak_jobs=outstanding();
    *fence=value;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::observeFence(uint32_t fence,const NativeAdrenoRingClock *clock,
    uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    if(!fence)return kIOReturnBadArgument;
    const NativeAdrenoRingFence result=ring_->observeFence(fence,clock,timeout_us,nullptr);
    if(result==NativeAdrenoRingFencePoisoned)return poison();
    if(result==NativeAdrenoRingFenceArgument)return kIOReturnBadArgument;
    if(result==NativeAdrenoRingFenceTimeout)return kIOReturnTimeout;
    return kIOReturnSuccess;
}
/* A stage runs once per stalled fence: when the stage before it has run and the fence has been pending long enough. */
static bool stage_due(uint32_t stage,uint32_t before,uint64_t pending,uint64_t after_us)
{
    return stage==before && pending>=after_us;
}
/* The CP snapshot is taken at 50 ms and at 1 s of one stall. */
static uint32_t stall_snapshots;
static bool snapshot_due(uint32_t *taken,uint64_t pending)
{
    const uint32_t want=pending>=1000000?2:pending>=50000?1:0;
    if(want<=*taken)return false;
    *taken=want;return true;
}
void NativeAdrenoSubmit::noteStall(uint32_t observed,const NativeAdrenoRingClock *clock)
{
    const uint32_t next=observed+1;   /* the fence the CP is working on */
    const uint64_t now=clock && clock->now_us?clock->now_us(clock->cookie):0;
    if(next!=stall_fence) {
        if(stall_timeouts && now) {
            const uint64_t ms=(stall_last_us-stall_began_us)/1000;
            uint32_t bucket=0;while(bucket<7 && ms>=(1ull<<bucket))++bucket;
            native_adreno_fence_pending[bucket]=native_adreno_fence_pending[bucket]+1;
        }
        stall_began_us=now;stall_snapshots=0;
        /* The CP moved on: what the stall that just ended took to clear. [9] first-stage recoveries, [10] second, [11] third, [12] timeouts of the longest. */
        if(stall_stage && stall_timeouts) {
            native_adreno_unstick[8+stall_stage]=native_adreno_unstick[8+stall_stage]+1;
            if(stall_timeouts>native_adreno_unstick[12])native_adreno_unstick[12]=stall_timeouts;
        }
        stall_fence=next;stall_timeouts=0;stall_stage=0;
    }
    stall_last_us=now;
    const uint64_t pending=now-stall_began_us;
    /* A fence that stops while the CP sits idle with work in the ring (int-317 hung with RBBM idle, RPTR short of WPTR, both threads at the same
     * cache clean) is nearly always a write pointer the CP never took: giving it again after a few waits ends it in a couple of milliseconds
     * (i343: 206 of 213 stalls ended at the second stage, 60 to 85 ms each, most of it the 100 waits before the write). After 10 waits now:
     * i469 stalled 143 times in about 200 s, each a visible hitch. The same value written again changes nothing when the CP is busy. */
    /* After 10 waits the translations are dropped and the write pointer is given again. i494/i495 measured what ends these: a job lasts about
     * 5 ms against 1 ms waits, so most "stalls" are ordinary jobs still running; the write pointer is never dropped (fenced writes saw no
     * drop) and invalidating at map time changed nothing. i553: the handler ran on 6712 of 6911 jobs. It now waits for 50 ms without
     * progress, measured in time (a timed-out wait is one poll, 100 us or less), and so does the snapshot below. */
    if(now && stage_due(stall_stage,0,pending,50000)) {
        native_adreno_unstick[0]=native_adreno_unstick[0]+1;native_adreno_unstick[1]=stall_fence;stall_stage=1;
        (void)vm_->invalidateTlb();(void)ring_->kick();
    }
    /* Still stopped well after that: the way a6xx drains the bus before a reset, which lets a write that was never acknowledged end. */
    if(now && stage_due(stall_stage,1,pending,200000)) {
        uint32_t ack[2]={};
        native_adreno_unstick[4]=native_adreno_unstick[4]+1;native_adreno_unstick[7]=stall_fence;stall_stage=3;
        (void)ring_->drainBus(ack);
        native_adreno_unstick[5]=ack[0];native_adreno_unstick[6]=ack[1];
    }
    ++stall_timeouts;
    if(!now || !snapshot_due(&stall_snapshots,pending))return;
    uint32_t words[NativeAdrenoRingStallWords]={},fault[4]={};
    if(ring_->stallState(words))return;
    (void)vm_->faultState(fault);
    for(uint32_t i=0;i<4;++i)native_adreno_stall_mem[i]=fault[i];
    uint64_t ib=0;uint32_t size=0;
    if(!ring_->ibBefore(words[8],&ib,&size))peekIb(vm_,ib,size,words[12],&native_adreno_stall_mem[4]);
    else peekFetch(vm_,(uint64_t)words[11]<<32|words[10],&native_adreno_stall_mem[4]);
    if(!ring_->ibBefore(words[16],&ib,&size))peekIb(vm_,ib,size,words[18],&native_adreno_stall_mem[4+17]);
    else peekFetch(vm_,(uint64_t)words[11]<<32|words[17],&native_adreno_stall_mem[4+17]);   /* BV IB1 base: the low word only */
    peekFetch(vm_,(uint64_t)words[21]<<32|words[20],&native_adreno_stall_mem[4+34]);
    {
        const Job *job=find(next);
        uint64_t brIb=(uint64_t)words[11]<<32|words[10],bvIb=(uint64_t)words[11]<<32|words[17];
        if(!ring_->ibBefore(words[8],&ib,&size))brIb=ib;
        if(!ring_->ibBefore(words[16],&ib,&size))bvIb=ib;
        native_adreno_stall_mem[55]=job?(uint32_t)job->vm_next:0;
        native_adreno_stall_mem[56]=(uint32_t)(table_vm_?table_vm_:vm_)->handleAt(brIb);
        native_adreno_stall_mem[57]=(uint32_t)(table_vm_?table_vm_:vm_)->handleAt(bvIb);
        native_adreno_stall_mem[58]=(uint32_t)(table_vm_?table_vm_:vm_)->handleAt((uint64_t)words[21]<<32|words[20]);
    }
    for(uint32_t i=0;i<NativeAdrenoRingStallWords;++i)native_adreno_stall[2+i]=words[i];
    native_adreno_stall[1]=stall_timeouts;
    native_adreno_stall[0]=next;
}
IOReturn NativeAdrenoSubmit::wait(uint32_t fence,const NativeAdrenoRingClock *clock,
    uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    Job *job=find(fence);
    uint32_t observed=0;
    native_adreno_fence_wait[0]=native_adreno_fence_wait[0]+1;native_adreno_unstick[13]=timeout_us;
    if(!job) {
        native_adreno_fence_wait[6]=native_adreno_fence_wait[6]+1;
        const NativeAdrenoRingFence passed=ring_->observeFence(fence,clock,timeout_us,&observed);
        if(passed==NativeAdrenoRingFencePoisoned)return poison();
        if(passed==NativeAdrenoRingFenceArgument) {native_adreno_fence_wait[7]=fence;return kIOReturnBadArgument;}
        if(passed==NativeAdrenoRingFenceTimeout) {
            ++stats_.timeouts;
            native_adreno_fence_wait[1]=native_adreno_fence_wait[1]+1;native_adreno_fence_wait[2]=fence;native_adreno_fence_wait[3]=observed;
            native_adreno_fence_wait[4]=ring_->lastSubmitted();native_adreno_fence_wait[5]=ring_->completedFence();
            noteStall(observed,clock);
            return kIOReturnTimeout;
        }
        return kIOReturnSuccess;
    }
    const NativeAdrenoRingFence result=ring_->wait(fence,clock,timeout_us,&observed);
    if(result==NativeAdrenoRingFencePoisoned)return poison();
    if(result==NativeAdrenoRingFenceArgument) {native_adreno_fence_wait[7]=fence;return kIOReturnBadArgument;}
    if(result==NativeAdrenoRingFenceTimeout) {
        ++stats_.timeouts;
        native_adreno_fence_wait[1]=native_adreno_fence_wait[1]+1;native_adreno_fence_wait[2]=fence;native_adreno_fence_wait[3]=observed;
        native_adreno_fence_wait[4]=ring_->lastSubmitted();native_adreno_fence_wait[5]=ring_->completedFence();
        noteStall(observed,clock);
        return kIOReturnTimeout;
    }
    IOReturn status=release(*job);
    if(status)return status;
    ++stats_.completed;
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::waitAll(const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    while(outstanding()) {
        Job *oldest=nullptr;
        for(auto &job:jobs_)if(job.active && (!oldest || job.fence-oldest->fence>0x80000000u))oldest=&job;
        const IOReturn status=wait(oldest->fence,clock,timeout_us);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::waitRange(uint32_t first,uint32_t count,const NativeAdrenoRingClock *clock,uint32_t timeout_us)
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    while(rangeOutstanding(first,count)) {
        Job *oldest=nullptr;
        for(auto &job:jobs_)if(job.active && job.queue-first<count && (!oldest || job.fence-oldest->fence>0x80000000u))oldest=&job;
        const IOReturn status=wait(oldest->fence,clock,timeout_us);
        if(status)return status;
    }
    return kIOReturnSuccess;
}
IOReturn NativeAdrenoSubmit::retireCompleted()
{
    if(!attached_ || poisoned_)return kIOReturnNotReady;
    for(auto &job:jobs_) {
        if(!job.active)continue;
        const NativeAdrenoRingFence state=ring_->observeFence(job.fence,nullptr,0,nullptr);
        if(state==NativeAdrenoRingFencePoisoned)return poison();
        if(state!=NativeAdrenoRingFenceComplete)continue;
        const IOReturn status=release(job);
        if(status)return status;
        ++stats_.completed;
    }
    return kIOReturnSuccess;
}
