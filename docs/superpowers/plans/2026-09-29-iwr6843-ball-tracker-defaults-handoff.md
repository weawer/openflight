# Handoff: IWR6843 ball tracker defaults and 3 ms frame overrun (2026-09-29)

Branch `test_build` (Cormac's `feat/iwr-calcs` at `bb06293e`, plus fixes).
It supersedes `feat/iwr-2ms-pipeline` and `feat/iwr-2ms-trigger-merge`.
2 ms is parked: get 3 ms working on the rig first.

## State

| Commit | What |
|---|---|
| `f16ae54d` | `l3_cli_dump` waits for the frozen ring (`l3_awaitFrozenRing`). Cormac's branch doesn't need this because his host reads captures with `l3track`/`l3sparse`. We compiled those out to save DATA_RAM, so we read captures with `l3dump`, which is what exposed the bug. |
| `506bab66` | Task priorities: CLI 3 < detect 4 < HWA rearm 5 < control 6 |
| `8d216dd0`, `e9bdb9a7` | `swing_trigger.py` saves the triggered capture and `triggerLog perf` before it fails a health check |
| `d0ad53fc` | Ball tracker defaults: hypotheses on, `fastBallMps` 26.5, `minDepartureMps` 20 (in both `l3_ball_track.c` and `l3_ball_hyp.c`) |

The latest image is `firmware/releases/l3_dump_hann_dumpfix_detectprio_balltuned_3ms_20260929.bin`
(SHA-256 `a96ecdf5…90f5`, 23,814 B DATA_RAM free). It builds and the host
tests pass. **It has not been run on the rig.**

## The open problem: the detect task overruns the 3 ms frame

- On the detectprio image, the rig fails its health check with
  `scratch_stale=15` while armed.
- `triggerLog perf` shows:
  - `residual` about 1237 µs per frame (16 trigger bins);
  - `balltrack` about 4174 µs mean, 4234 µs max (53 post-window bins).
- Raising detect priority did not fix it (stale went from 3 to 15). The cause
  is too much work per frame, not starvation.
- The new defaults add the hypothesis search, so expect the overrun to grow.
  The balltuned image exists to measure that.

**Next rig step:**

1. Flash the balltuned image.
2. Run `swing_trigger.py --tee-m 1.845 --config config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg --capture-dir …`
   and take a few real swings.
3. Record the `balltrack` mean/max from the saved `triggerLog perf`, and
   compare it with 4174/4234 µs.
4. Check that each readback is a full-size `.l3dump` with a ball speed.

## Why these ball settings (TrackMan evidence)

- **Data:** John Pacino's August sessions. They are untracked, at
  `openflight_sessions/openflight_trackman_sessions/`, with aligned CSVs in
  `sessions/<date>/comparisons/`.
- **Method:** replay through `firmware_replay` with the true ball origin
  (`dest_bin`, median bin 39). The configured tee (bin 34) was wrong on
  these sessions, so using it gives a −76 mph median error.

| Variant | 3 ms: follows ball (of 124) | 3 ms: within 5 mph | Drivers (of 18) | 2 ms held-out: follows ball (of 36) |
|---|---|---|---|---|
| old defaults | 68 | 42 | 2 | 31 |
| hypotheses only | 115 | 87 | 15 | 26 (regressed) |
| **all three settings** | **121** | **92** | 16 | **34** |

- Median |error| on ball-following shots is 3.2 mph.
- `minDepartureMps` alone does nothing. Hypotheses without the 26.5 m/s
  fast-ball floor regress at 2 ms. So the three settings go together.

**Reporting change:** with the search on, `ball_points` start once the
winning line is committed, 3–4 frames past the tee. The launch still fits
the earlier points. The manifest `ball_origin_bin` ranges moved to match,
and the speed ranges were unchanged.

## Ball reach (context for any search window)

| Range | 2.0 m | 2.5 m | 3.0 m | 3.5 m | 4.0 m | 4.5 m |
|---|---|---|---|---|---|---|
| Median SNR on the true ball line | 20 dB | 10 dB | 7 dB | 6 dB | 4.6 dB | 2.5 dB |
| Frames above the 4.8 dB cut | 100% | 92% | 86% | 69% | 41% | 2% |

Tracks end where the SNR falls below the tracker's 4.8 dB cut. Only 4 of
124 shots reported a vertical launch angle.

## The bin search window is not ready

Scoring only a window around the expected ball position cuts the scored
bins per frame from 53 to about 16. That is the obvious fix for the overrun,
but it loses accuracy:

| | Full window | Search window |
|---|---|---|
| Follows ball (of 124) | 121 | 100 |
| Within 5 mph | 92 | 58 |

The cause is unknown. Suspects:

- the seeking cone;
- the interplay between the club claim and the hypotheses.

**Trap:** the noise floor must come from the whole window, or from every 4th
bin. A median over the narrow window is dominated by the impact echo, and
tracking collapses to 1 of 124 shots.

**Next offline step:** replay full and windowed side by side and find the
first frame where the tracks diverge. The prototype is
`tm_ball_window.py`, a scratch script that is not in the repo.

## For Cormac (he asked to be told before we change his tracker's per-frame cost)

- The new defaults above, and the TrackMan table.
- The overrun: about 4.2 ms of `balltrack` in a 3 ms frame.
- The window-floor trap.
- The `l3dump` freeze bug, which is latent on his branch.
