# Adreno740

A740 kernel GPU driver.

The implementation covers A740 on SM8550. GPU commands and scanout copy have been verified on sheng. Platform inputs provide firmware, power policy, and memory resources.

Two priority rings preserve queue FIFO and per-ring page tables. Context switching uses the executed-table snapshot, GMEM save/restore and a preemption postamble. The host scheduling check covers packet sequencing, priority changes and fence-history rollover; board policy selects the mode.

Build and interface details: [documentation](https://unixnu.com/drivers/adreno740). Component sources and dependencies: [catalog](https://unixnu.com/reference/components).

## Frame diagnostics

`sheng-gpu-trace=1` records allocation, command, completion, CPU/GPU and display latency evidence. Pixel capture is disabled. Repeated pending queries are coalesced and idle scheduler stages are excluded. The sheng exporter prioritizes the complete frontend timeline. See [GPU frame diagnostics](https://unixnu.com/porting/gpu-diagnostics) for fields and decoding.

`NativeGpuPerfCounters.h` defines the opt-in 19-counter A7XX BR plan used by sheng diagnostics. Capture records selector readback, unstable 64-bit reads, reset masks and preemption epochs. The host check verifies the register plan against pinned Mesa XML. Dispatch diagnostic events retain task and queue identities through the board latency hook.
