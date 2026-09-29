# l3_dump_hann_dumpfix_detectprio_balltuned_3ms_20260929.bin

SHA-256: `a96ecdf5c17064cbdc2c8539a7ca214011240ec03251e49a0d2e204362dd90f5`

Supersedes `l3_dump_hann_dumpfix_detectprio_3ms_20260929.bin`. Same source
plus new ball tracker defaults:

| Setting | Was | Now |
|---|---|---|
| `useHypotheses` (`l3_ball_track.c`) | 0 | 1 |
| `hyps.fastBallMps` (`l3_ball_hyp.c`) | 0 (off) | 26.5 m/s |
| `minDepartureMps` (both) | 10 m/s | 20 m/s |

## Why

Replayed (host-compiled firmware) on the August TrackMan-matched captures,
using each shot's true ball origin (median bin 39, not the configured tee
at 34):

| Variant | 3 ms: follows ball | 3 ms: within 5 mph | Drivers | 2 ms: follows ball | 2 ms: within 5 mph |
|---|---|---|---|---|---|
| old defaults | 68/124 | 42 | 2/18 | 31/36 | 8 |
| hypotheses alone | 115 | 87 | 15/18 | 26 (worse) | 9 |
| hyp + fast 26.5 | 120 | 90 | 16/18 | 32 | 13 |
| **hyp + fast 26.5 + minDep 20** | **121** | **92** | 16/18 | **34** | 13 |

The 2 ms set was held out; hypotheses alone regress there, so the three
settings go together. Median |error| on ball-following 3 ms shots: 3.2 mph.

## Reporting change

With the search on, `ball_points` (and so `ball_origin_bin` in the replay
manifest) start where the winning line was committed, typically 3-4 frames
past the tee. The launch still fits that line's earlier points. The five
manifest `ball_origin_bin` ranges moved accordingly; every speed range
still holds. The one unexpected-entry recording that used to report a
659 m/s launch now reports none.

## What was run

- TI toolchain build. DATA_RAM 23,814 B free (unchanged).
- Full host suite: 3250 passed, 30 skipped.

## Status: not yet run on the rig

The tracker already overran the 3 ms frame (`balltrack` mean 4174 us on
the detectprio image). The hypothesis search adds work per post frame, so
expect `scratch_stale` to get worse, not better. The point of this image is to
measure that cost:

1. `swing_trigger.py` with a few real swings. Each health failure saves
   `triggerLog perf`; compare the `balltrack` stage mean/max with 4174/4234 us.
2. Check that the readback is a full-size `.l3dump` and the shot reports a
   ball speed.

Revert to the detectprio image if it is unusable. Nothing else changed.
