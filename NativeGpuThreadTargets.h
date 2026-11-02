/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_GPU_THREAD_TARGETS_H
#define NATIVE_GPU_THREAD_TARGETS_H
#include <stdint.h>
/* Diagnostic thread identities by process and role (1 render, 2 encode,
 * 3 completion). Writers publish the caller's own PID only. */
struct NativeGpuThreadTargets {
    struct Slot {uint64_t key;uint64_t tid;};
    Slot slots[32]{};
    static uint64_t keyOf(uint32_t pid,uint32_t role) {return (uint64_t)pid<<8|role;}
    void record(uint32_t pid,uint32_t role,uint64_t tid) {
        if(!pid || !role || role>255 || !tid)return;
        const uint64_t key=keyOf(pid,role);
        for(auto &slot:slots) {
            uint64_t found=__atomic_load_n(&slot.key,__ATOMIC_ACQUIRE);
            if(!found && !__atomic_compare_exchange_n(&slot.key,&found,key,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))continue;
            if(!found || found==key){__atomic_store_n(&slot.tid,tid,__ATOMIC_RELEASE);return;}
        }
    }
    uint64_t find(uint32_t pid,uint32_t role) const {
        const uint64_t key=keyOf(pid,role);
        for(const auto &slot:slots)if(__atomic_load_n(&slot.key,__ATOMIC_ACQUIRE)==key)
            return __atomic_load_n(&slot.tid,__ATOMIC_ACQUIRE);
        return 0;
    }
};
#endif
