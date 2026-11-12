# Adreno740

A740 kernel GPU driver.

The implementation covers A740 on SM8550. GPU commands and scanout copy have been verified on sheng. Platform inputs provide firmware, power policy, and memory resources.

Two priority rings preserve queue FIFO and per-ring page tables. Context switching uses the executed-table snapshot, GMEM save/restore and a preemption postamble. The host scheduling check covers packet sequencing, priority changes and fence-history rollover; board policy selects the mode.

Build and interface details: [documentation](https://unixnu.com/drivers/adreno740). Component sources and dependencies: [catalog](https://unixnu.com/reference/components).

## Fence waits

A waiter sleeps until the completion pass finds its fence written on the ring that issued it; its only timeout is the caller's deadline. The board's GPU interrupt handler asks for the pass through `native_adreno_gpu_interrupt(status)`; the pass retires finished jobs, looks at the switch between the two rings again and wakes the waiters. The hang check runs one pass per period. A wait that pauses under the driver's lock drives the ring switch itself. `gpu-fence-wake=0` keeps the board's timed pause for the waiters. Counters: `native_adreno_fence_wake`.

## CPU cache maintenance

The GPU maps its own memory non-cacheable, so it sees what DRAM holds. User BOs are write-combined for the CPU. The GPU context's eleven kernel buffers (SQE, rings, memptrs, preemption records, 8548 KiB) are declared synchronized (`NativeAdrenoMemory::declareSynchronized`): their users write them through a non-cacheable view or clean what they stored, so a map, an unmap and a submission leave them alone. The first 8 submissions and every 1024th compare the head of each buffer as a CPU cache and DRAM hold it; a dirty line ends that trust and the buffers are cleaned at every submission again. A submission, a completion and the write-combine re-assertion visit only the mappings they have work for. `gpu-maint-all=1` visits every mapping and maintains the synchronized buffers like any other memory, for a comparison. Counters: `native_adreno_maint_kind` (maintenance at submissions by kind of mapping), `native_adreno_maint_walk` (mappings visited), `native_adreno_sync_audit`, `native_adreno_sync_skips`, `native_adreno_submit_us` (a submission's time by phase). The submission log reads the first IB's words only with `diag=1`.

## Memory accounting

`native_adreno_bo_owners` and `native_adreno_bo_sizes` count the system BOs by allocating process and by size. `native_adreno_import_memory` and `native_adreno_import_owners` count the imported memory the driver keeps wired, in all and by importing process. `native_adreno_view_owners` counts the BO views each process maps; the VM bills a view to the mapping task by its size.

## Pool of DRAM outside the kernel's

A board whose kernel manages only a window of its DRAM registers the free runs outside that window with `native_adreno_pool_add(physical, bytes)`: whole pages, entirely outside `[gPhysBase, gPhysBase+gPhysSize)`, from any thread once IOKit runs, before or after the driver starts. A write-combined system BO then takes its pages from the pool instead of wiring the window's memory: one run that fits best, else the longest runs first, at most sixteen runs up to 1 MiB and one a 64 KiB above. Whatever the pool does not serve takes the descriptor path: no range, a BO that is not write-combined, a request above the client's physical address limit, not enough free pages, or free pages too scattered.

The kernel has no `vm_page` and no cacheable mapping for these pages. Each range has one write-combined kernel mapping; every page is zero-filled through it before a BO gets it. A pool BO has no descriptor, is never cleaned or audited, and its client view maps its physical runs write-combined. The BO and each client view hold the pages; an import of a range that maps pool pages is refused. Pages a client had a view of do not return to the pool, because a copy of that mapping made in the VM cannot be seen or revoked; `gpu-pool=2` returns them after dropping their cache lines. `gpu-pool=0` turns the pool off.

Counters: `native_adreno_pool_state` (ranges, KiB total, free and lowest free, grants alive and their KiB, served, freed, retired and their KiB, the boot argument, refused registrations), `native_adreno_pool_fallbacks` (by reason), `native_adreno_pool_zero` (zero-fill and cache-pass time and KiB).

## Frame diagnostics

`gpu-trace=1` enables the kernel GPU ring: submission, retirement, display arrival, display fence and latch, build identity, and the render server's render, encode and completion thread identities, 65536 records. Repeated pending display queries are coalesced. See [GPU frame diagnostics](https://unixnu.com/porting/gpu-diagnostics) for fields and decoding.

`native_adreno_stall_ring` holds, with each stall snapshot, the ring of the waited fence, the ring the CP runs, a switch in flight and the unfinished jobs of each ring.

The hang check takes a CP snapshot before recovery and retains it until the next hang. `native_adreno_stall_hang` identifies that hang. For a fault with a zero upper address word, `native_adreno_stall_fault` searches the job's IB1s and up to 512 referenced IB2s for the low word and a nearby base followed by zero. Matches are diagnostic candidates, not decoded address operands. Each new snapshot clears the previous search.

`NativeGpuPerfCounters.h` defines the opt-in 19-counter A7XX BR plan used by sheng diagnostics. Capture records selector readback, unstable 64-bit reads, reset masks and preemption epochs. The host check verifies the register plan against pinned Mesa XML.
