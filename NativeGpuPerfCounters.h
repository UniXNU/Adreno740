/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_GPU_PERF_COUNTERS_H
#define NATIVE_GPU_PERF_COUNTERS_H
#include <stdint.h>
/* A7XX BR counters from Mesa a6xx.xml and a7xx_perfcntrs.xml. Slots are reserved
 * only with sheng-gpu-perf=1; selector readback detects another counter owner. */
struct NativeGpuPerfCounter {uint32_t select,value,low;};
static constexpr NativeGpuPerfCounter NativeGpuPerfPlan[]={
 {0x08dd,0,0x031a}, /* CP_ALWAYS_COUNT, slot 13 */
 {0x0e22,1,0x0382},{0x0e23,2,0x0384},{0x0e24,3,0x0386},
 {0x0e25,4,0x0388},{0x0e26,8,0x038a},{0x0e27,9,0x038c},
 {0xb619,0,0x03a0},{0xb61a,6,0x03a2},{0xb61b,7,0x03a4},
 {0xae91,0,0x03c8},{0xae92,1,0x03ca},{0xae93,4,0x03cc},
 {0xae94,40,0x03ce},{0xae95,51,0x03d0},{0xae96,52,0x03d2},{0xae97,101,0x03d4},
 {0x8e2e,7,0x03f6},{0x8e2f,8,0x03f8}
};
enum {NativeGpuPerfCount=sizeof(NativeGpuPerfPlan)/sizeof(NativeGpuPerfPlan[0])};
struct NativeGpuPerfSample {
 uint64_t values[NativeGpuPerfCount]{};uint32_t mismatch=0,unstable=0,reset=0;bool first=false;
};
struct NativeGpuPerfCounters {
 uint64_t previous[NativeGpuPerfCount]{};bool sampled=false;
 template<class Write> void configure(Write write) const {
  for(const auto &c:NativeGpuPerfPlan)write(c.select,c.value);
 }
 template<class Read> NativeGpuPerfSample capture(Read read) {
  NativeGpuPerfSample out{};out.first=!sampled;
  for(unsigned i=0;i<NativeGpuPerfCount;++i) {
   const auto &c=NativeGpuPerfPlan[i];if(read(c.select)!=c.value)out.mismatch|=1u<<i;
   bool stable=false;uint32_t lo=0,hi=0;
   for(unsigned retry=0;retry<3;++retry) {
    hi=read(c.low+1);lo=read(c.low);if(hi==read(c.low+1)){stable=true;break;}
   }
   if(!stable)out.unstable|=1u<<i;
   out.values[i]=(uint64_t)hi<<32|lo;
   if(sampled && out.values[i]<previous[i])out.reset|=1u<<i;
   previous[i]=out.values[i];
  }
  sampled=true;return out;
 }
};
#endif
