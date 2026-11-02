/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_GPU_THREAD_TARGETS_H
#define NATIVE_GPU_THREAD_TARGETS_H
#include <stdint.h>
/* Diagnostic thread identities. Writers publish the caller's own PID only. */
struct NativeGpuThreadTargets {
    struct Slot {uint32_t pid;uint64_t tid;};
    Slot slots[32]{};
    void record(uint32_t pid,uint64_t tid) {
        if(!pid || !tid)return;
        for(auto &slot:slots) {
            uint32_t found=__atomic_load_n(&slot.pid,__ATOMIC_ACQUIRE);
            if(!found && !__atomic_compare_exchange_n(&slot.pid,&found,pid,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))continue;
            if(!found || found==pid){__atomic_store_n(&slot.tid,tid,__ATOMIC_RELEASE);return;}
        }
    }
    uint64_t find(uint32_t pid) const {
        for(const auto &slot:slots)if(__atomic_load_n(&slot.pid,__ATOMIC_ACQUIRE)==pid)
            return __atomic_load_n(&slot.tid,__ATOMIC_ACQUIRE);
        return 0;
    }
};
#endif
