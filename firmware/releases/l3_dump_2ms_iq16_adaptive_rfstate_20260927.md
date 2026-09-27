# Adaptive 2 ms IQ16 + self-trigger: idempotent RF stop

- Binary: `l3_dump_2ms_iq16_adaptive_rfstate_20260927.bin`
- SHA-256: `bf2cc66c7493ac7a6a38c7a135ccc9451c396aa34dc031551d236f656dd75103`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Supersedes `l3_dump_2ms_iq16_adaptive_l3release_20260927.bin`. It includes
  `l3release` and everything in that image.
- Hardware status: not flashed or tested by the agent.

## Problem

On restart after a failed session:

```
config rejected: 'sensorStop': Error: MMWave_stop failed (-203227134)
```

`-203227134` is `0xF3E3/0002`: mmWave error -3101 (`MMWAVE_EINVAL`) with
subsystem code 2, meaning `MMWave_stop` was called on a front end that was
already stopped.

## Root cause

A self-trigger can latch on a frame just before, or while, `sensorStop` shuts
the capture down. The shutdown stops RF and closes mmWave, but the latch
survives. The next session's `sensorStop` saw the latch and stopped RF again.

The latched branches (`l3_stopFrozenRing` and `sensorStop`) used the latch to
decide that RF was still running, and a latch that survives a shutdown breaks
that assumption. `feat/iwr-calcs` has the same flaw in `l3_awaitFrozenRing`.

## Fix

- `gFrontEndRunning` is set when `MMWave_start` succeeds, and cleared by a
  successful `MMWave_stop` or by `MMWave_close`.
- `l3_finishCaptureStop` calls `MMWave_stop` only while RF is running, so every
  stop path is safe to repeat.
- `sensorStop` disables the trigger before shutting down, so no new latch can
  land, and clears any latch once the session is closed.

## Verification

- A native C harness compiles the real `l3_finishCaptureStop`. It checks that:
  - a running front end is stopped;
  - a failed stop keeps RF counted as running;
  - stopping an already-stopped front end is a no-op that succeeds.
- Source tests cover where the flag is set and cleared, and the `sensorStop`
  ordering.
- Full suite: 1991 passed, 8 skipped. TI build passes with warnings as errors.
  DATA_RAM is unchanged.

## Operator test

The board may be wedged from the failed run: flash this image, set
functional mode, and press RESET.
