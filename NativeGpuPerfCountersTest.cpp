// SPDX-License-Identifier: MIT
#include "NativeGpuPerfCounters.h"
#include <cassert>
#include <map>
int main() {
 std::map<uint32_t,uint32_t> registers;NativeGpuPerfCounters counters;
 counters.configure([&](uint32_t r,uint32_t v){registers[r]=v;});
 for(const auto &c:NativeGpuPerfPlan)registers[c.low]=100;
 auto read=[&](uint32_t r){return registers[r];};
 auto first=counters.capture(read);assert(first.first && !first.mismatch && !first.reset);
 registers[NativeGpuPerfPlan[1].select]=999;registers[NativeGpuPerfPlan[2].low]=1;
 auto next=counters.capture(read);assert(!next.first && next.mismatch==2 && next.reset==4);
 auto bad=counters.capture([&](uint32_t r){for(const auto &c:NativeGpuPerfPlan)if(r==c.low+1)return ++registers[r];return registers[r];});
 assert(bad.unstable==((1u<<NativeGpuPerfCount)-1));
}
