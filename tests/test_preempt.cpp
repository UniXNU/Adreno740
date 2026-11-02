// SPDX-License-Identifier: MIT
#include "NativeAdrenoPreempt.h"
#include <cassert>
#include <cstdio>
#include <vector>
struct Peer {uint32_t cntl=0;uint64_t now=0;bool fail=false;std::vector<uint32_t> order;};
static int write(void *v,uint32_t r,uint32_t n) {auto &p=*(Peer*)v;p.order.push_back(r);if(p.fail)return 1;if(r==0x08a0)p.cntl=n;return 0;}
static int read(void *v,uint32_t r,uint32_t *n) {*n=r==0x08a0?((Peer*)v)->cntl:0;return 0;}
static int barrier(void *) {return 0;}
static int gmu(void *v,uint32_t,uint32_t n) {((Peer*)v)->order.push_back(n?0xaabb:0xccdd);return 0;}
static uint64_t now(void *v) {return ((Peer*)v)->now;}
int main() {
 Peer p;NativeAdrenoPreempt a;NativeAdrenoPreemptIo io{&p,write,read,barrier,&p,gmu};
 uint8_t r0[64]{},r1[64]{},s0[32]{},s1[32]{};
 NativeAdrenoPreemptRings rings{{0x10000,0x20000},{0x30000,0x40000},{r0,r1},{s0,s1}};
 assert(!a.attach(&io,rings,0,now,&p));p.order.clear();
 assert(a.trigger(1,true) && p.order.front()==0xaabb && p.cntl==0x341);
 assert(!a.complete() && a.stats().faulted==0);p.now=40000;assert(!a.timedOut(1000000));
 p.cntl=0x340;assert(a.complete() && a.loaded()==1 && p.order.back()!=0xccdd);
 assert(a.releaseKeepalive() && p.order.back()==0xccdd);
 assert(a.trigger(0,true));p.now+=1000001;assert(a.timedOut(1000000));a.noteTimeout(nullptr);assert(a.stats().faulted==1);
 a.restart(1);assert(p.order.back()==0xccdd && !a.inFlight());
 p.fail=true;assert(!a.trigger(0,true) && p.order.back()==0xccdd && a.stats().faulted==2);
 assert(!a.detach());std::puts("PASS preemption GMEM, keepalive ordering, pending poll, timeout, restart and refused trigger");
}
