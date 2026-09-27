# Adaptive 2 ms IQ16 + self-trigger: stop RF after a self-triggered freeze

- Binary: `l3_dump_2ms_iq16_adaptive_selftrigger_20260927.bin`
- SHA-256: `2635d640588323b17394a9f1559eb912cfca8eea7c2f6a368313e40753616e62`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, native build.
- Supersedes `l3_dump_2ms_iq16_adaptive_coherent2_20260926.bin`. Selector,
  coherent gate, capture layout and host protocol are unchanged.
- Hardware status: not flashed or tested by the agent.

## Problem

First kiosk run with `--iwr6843-self-trigger` on the adaptive profile
(2026-09-27):

```
RuntimeError: IWR6843 dump completed but firmware restart failed:
Error: RF restart failed
```

followed by a forced server exit and, on the next launch,
`no IWR6843 CLI found` until the board was reset.

## Root cause

A host-requested `l3dump` freezes the HWA ring and then stops RF
(`l3_stopCaptureAtBoundary` -> `l3_finishCaptureStop` -> `MMWave_stop`)
before streaming. A firmware self-trigger only freezes the HWA ring (it
stops re-arming frames and clears `gCaptureActive`) -- RF keeps running.

1. `l3_freezeCapture`'s self-triggered branch waited for that freeze and
   cleared the latch, but never stopped RF. Every rearm after a
   self-triggered freeze (`l3dump`, `l3sparse`, `l3track`) then called
   `MMWave_start` on a running sensor: "RF restart failed". This affects
   every self-triggered capture, not only the startup release.
2. `sensorStop` stopped RF only when `gCaptureActive` was set. After an
   unconsumed self-trigger it is 0, so `sensorStop` closed mmWave on a live
   sensor, leaving the board unresponsive until reset.

The self-trigger image (`l3_dump_self_trigger_20260925`) was never
hardware-validated, so neither path had run on hardware before.

## Fix

- `l3_freezeCapture`: after the self-triggered freeze completes, call
  `l3_finishCaptureStop()` -- the same stop a host-requested dump uses. The
  latch is cleared only after RF stops, so if the stop fails, a retry stops
  RF again instead of taking the unlatched path (which returns early once
  `gCaptureActive` is 0).
- `sensorStop`: if a self-triggered freeze is still latched, stop RF before
  `MMWave_close`.

Host-side, commit `eb8e895` already releases unwanted adaptive16 freezes
with `l3dump` instead of the unsupported `l3sparse`; that is still required.

## Verification

- Native C harness (`tests/test_iwr6843_trigger_firmware.py`) compiles the
  real `l3_freezeCapture`: self-triggered freezes now stop RF exactly once,
  a freeze timeout leaves RF and the latch untouched, host-requested dumps
  still stop through the boundary path, and a failed RF stop keeps the latch
  for a retry.
- Source regression tests pin both RF stops.
- Full suite: 1967 passed, 8 skipped. TI build and packaging: passed,
  warnings-as-errors. DATA_RAM unchanged (98,393 of 196,608 bytes).

## Operator test

Flash, set functional mode, **press RESET** (the board may still be wedged
from the failed run), then start kiosk mode as before. Expected on startup:
"Releasing an unaccepted self-trigger capture" (a normal early notice) with
no "RF restart failed" error, and the server finishing startup.
