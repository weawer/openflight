# IWR6843 2 ms IQ16 shadow-selector diagnostic

- Binary: `l3_dump_2ms_iq16_shadow_selector_20260926.bin`
- SHA-256: `1776630373b3da41a336fa02c945449a53ccc9bdeff4409a06ef3016d974c6ab`
- Config: `config/iwr6843_l3dump_compact_16f2ms_32bin_iq16.cfg`

This phase-3 diagnostic retains the same fixed 32-bin IQ16 reference window as
the validated compaction image. It also scans the complete staged 128-bin frame,
removes stationary power using a one-frame temporal baseline, finds at most two
candidates, associates outbound motion, and proposes a 12-bin window. Proposed
windows do not affect retained data in this image.

The shadow scan samples four of the twelve loops, all three TX channels, and two
of the four RX channels. The fixed reference capture still retains every loop,
TX, and RX. This bounds selector work so the high-priority rearm task cannot
starve the firmware CLI.

Firmware statistics expose candidate acceptance, ambiguity, misses, errors,
the latest proposed window, and maximum selector execution time. The hardware
test fails if selection plus compaction reaches the 2 ms frame period.
