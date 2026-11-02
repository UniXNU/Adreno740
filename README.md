# Adreno740

A740 kernel GPU driver.

The implementation covers A740 on SM8550. GPU commands and scanout copy have been verified on sheng. Platform inputs provide firmware, power policy, and memory resources.

Two priority rings preserve queue FIFO and per-ring page tables. Context switching uses the executed-table snapshot, GMEM save/restore and a preemption postamble. The host scheduling check covers packet sequencing, priority changes and fence-history rollover; board policy selects the mode.

Build and interface details: [documentation](https://unixnu.com/drivers/adreno740). Component sources and dependencies: [catalog](https://unixnu.com/reference/components).
