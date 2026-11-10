# Adreno740

A740 kernel GPU driver.

The implementation covers A740 on SM8550. GPU commands and scanout copy have been verified on sheng. Platform inputs provide firmware, power policy, and memory resources.

Two priority rings preserve queue FIFO and per-ring page tables. Context switching uses the executed-table snapshot, GMEM save/restore and a preemption postamble. The host scheduling check covers packet sequencing, priority changes and fence-history rollover; board policy selects the mode.

Build and interface details: [documentation](https://unixnu.com/drivers/adreno740). Component sources and dependencies: [catalog](https://unixnu.com/reference/components).

## Fence waits

A waiter sleeps until the completion pass finds its fence written on the ring that issued it; its only timeout is the caller's deadline. The board's GPU interrupt handler asks for the pass through `native_adreno_gpu_interrupt(status)`; the pass retires finished jobs, looks at the switch between the two rings again and wakes the waiters. The hang check runs one pass per period. A wait that pauses under the driver's lock drives the ring switch itself. `gpu-fence-wake=0` keeps the board's timed pause for the waiters. Counters: `native_adreno_fence_wake`.

## Memory accounting

`native_adreno_bo_owners` and `native_adreno_bo_sizes` count the system BOs by allocating process and by size. `native_adreno_import_memory` and `native_adreno_import_owners` count the imported memory the driver keeps wired, in all and by importing process. `native_adreno_view_owners` counts the BO views each process maps; the VM bills a view to the mapping task by its size.

## Frame diagnostics

`gpu-trace=1` enables the kernel GPU ring: submission, retirement, display arrival, display fence and latch, build identity, and the render server's render, encode and completion thread identities, 65536 records. Repeated pending display queries are coalesced. See [GPU frame diagnostics](https://unixnu.com/porting/gpu-diagnostics) for fields and decoding.

`native_adreno_stall_ring` holds, with each stall snapshot, the ring of the waited fence, the ring the CP runs, a switch in flight and the unfinished jobs of each ring.

`NativeGpuPerfCounters.h` defines the opt-in 19-counter A7XX BR plan used by sheng diagnostics. Capture records selector readback, unstable 64-bit reads, reset masks and preemption epochs. The host check verifies the register plan against pinned Mesa XML.
