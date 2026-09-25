# IWR self-trigger candidate — 2026-09-25

Built from the working-tree trigger fixes based on `3989ac97f547aa0d31fb1365245781df64f5a8f0`.
This image has **not been flashed or validated on hardware** in this change.

- Image: `l3_dump_self_trigger_20260925.bin` (358340 bytes)
- SHA-256: `59c87e7760467cdb294c90c2f1035fe2fdaf53b68ea8076d609a2096fa8426ef`
- `l3_dump.c` SHA-256: `6bff1901a102921f35d6cb1b40a40937c23247e5414d0d98c773b903d3e7ea32`
- SDK: TI mmWave 3.6.2 LTS; ARM compiler: 20.2.7 LTS
- SYS/BIOS: 6.73.1.1; XDCtools: 3.61.0.16
- Build: `make -C firmware build-native RELEASE_NAME=l3_dump_self_trigger_20260925.bin` with local toolchain paths.
- Packaging: TI image/CRC tools; TI out2rprc executable ran through Wine.

Uses the existing configurable IQ16/IQ8 build flags and memory layout. The
self-trigger waits for full pre-trigger history and requires outward progression
after approach motion. It resets stale motion and the latch on sensor startup.
Both full and selected-sample dumps can consume the same self-triggered freeze.

Use the matching application changes. Selected-sample transfer stays the default;
`--iwr6843-full-capture --debug` saves full diagnostic captures. Neither mode
extends the default 72 ms recording window.

Validation: TI compilation, linking and image/CRC generation succeeded; the native
C detector tests exercise 2/3/4 ms frame periods. The relevant Python/C test run
passed 869 tests; six tests requiring local session data were skipped. Actual
trigger timing, false-trigger rate, camera/OPS synchronization and selected-sample
transfer latency still need measurement on the rig.
