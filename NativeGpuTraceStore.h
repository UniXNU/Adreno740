/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_GPU_TRACE_STORE_H
#define NATIVE_GPU_TRACE_STORE_H
#include "NativeGpuTrace.h"

/* Nonblocking writers and readers. A busy slot is dropped, never partly read.
 * Storage is allocated once by the diagnostic owner, outside submission locks. */
struct NativeGpuTraceSlot { uint32_t busy; NativeGpuTraceRecord record; };
struct NativeGpuTraceStore {
    NativeGpuTraceSlot *slots=nullptr;
    uint32_t capacity=0;
    uint64_t issued=0,dropped=0;
    uint64_t push(NativeGpuTraceRecord record) {
        if(!slots || !capacity)return false;
        const uint64_t seq=__atomic_add_fetch(&issued,1,__ATOMIC_RELAXED);
        auto &slot=slots[(seq-1)%capacity];uint32_t empty=0;
        if(!__atomic_compare_exchange_n(&slot.busy,&empty,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED)) {
            __atomic_add_fetch(&dropped,1,__ATOMIC_RELAXED);return false;
        }
        // A delayed writer must not overwrite a newer lap of this slot.
        if(slot.record.sequence<seq){record.sequence=seq;slot.record=record;}
        else {__atomic_add_fetch(&dropped,1,__ATOMIC_RELAXED);__atomic_store_n(&slot.busy,0,__ATOMIC_RELEASE);return 0;}
        __atomic_store_n(&slot.busy,0,__ATOMIC_RELEASE);return seq;
    }
    bool read(uint64_t seq,NativeGpuTraceRecord *out) {
        if(!slots || !capacity || !seq || !out)return false;
        auto &slot=slots[(seq-1)%capacity];uint32_t empty=0;
        if(!__atomic_compare_exchange_n(&slot.busy,&empty,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED))return false;
        const bool found=slot.record.sequence==seq;if(found)*out=slot.record;
        __atomic_store_n(&slot.busy,0,__ATOMIC_RELEASE);return found;
    }
};
#endif
