// SPDX-License-Identifier: MIT
// Kernel services absent from the host model. Tests can replace these weak defaults.
#include "XnuMemoryFixture.h"
#include "NativeAdrenoMemory.h"
extern "C" __attribute__((weak)) boolean_t PE_parse_boot_argn(const char *,void *,int) {return 0;}
__attribute__((weak)) void NativeAdrenoNotePhase(unsigned,size_t) {}
__attribute__((weak)) bool NativeAdrenoPagesUncached(const NativeAdrenoPhysicalSpan *,size_t) {return false;}
extern "C" __attribute__((weak)) unsigned native_pmap_user_mappings(ppnum_t,void **,uint64_t *,char *) {return 0;}
extern "C" {__attribute__((weak)) volatile uint32_t native_gpu_dcvs[15]{};}
