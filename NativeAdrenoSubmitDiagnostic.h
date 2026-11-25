/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_SUBMIT_DIAGNOSTIC_H
#define NATIVE_ADRENO_SUBMIT_DIAGNOSTIC_H
#include <stdint.h>
#include <IOKit/IOLib.h>

enum NativeAdrenoSubmitStage : uint32_t {
    NativeAdrenoSubmitRequest=1, NativeAdrenoSubmitQueue,
    NativeAdrenoSubmitDependency, NativeAdrenoSubmitWireSize,
    NativeAdrenoSubmitHandle, NativeAdrenoSubmitBackend,
    NativeAdrenoSubmitIbSize, NativeAdrenoSubmitMapping,
    NativeAdrenoSubmitRange, NativeAdrenoSubmitRetirement,
    NativeAdrenoSubmitReference, NativeAdrenoSubmitPublish,
    NativeAdrenoSubmitTable, NativeAdrenoSubmitRing, NativeAdrenoSubmitTransport
};
/* Called under the submission lock. Diagnostic data never changes the returned
 * status or authorizes a rejected command buffer to run. */
#ifdef __APPLE__
#define NATIVE_ADRENO_REJECT_OPTIONAL __attribute__((weak_import))
#else
#define NATIVE_ADRENO_REJECT_OPTIONAL __attribute__((weak))
#endif
extern "C" void native_adreno_submit_rejected(uint32_t stage,uint32_t index,
    uint32_t status,uint64_t handle,uint64_t iova,uint64_t bytes,
    uint64_t mapped_iova,uint64_t mapped_bytes,uint64_t detail) NATIVE_ADRENO_REJECT_OPTIONAL;
#undef NATIVE_ADRENO_REJECT_OPTIONAL
static inline IOReturn nativeAdrenoSubmitRejected(IOReturn status,uint32_t stage,
    uint32_t index=UINT32_MAX,uint64_t handle=0,uint64_t iova=0,uint64_t bytes=0,
    uint64_t mapped_iova=0,uint64_t mapped_bytes=0,uint64_t detail=0)
{
    if(&native_adreno_submit_rejected && status!=kIOReturnBusy)
        native_adreno_submit_rejected(stage,index,uint32_t(status),handle,iova,
            bytes,mapped_iova,mapped_bytes,detail);
    return status;
}
#endif
