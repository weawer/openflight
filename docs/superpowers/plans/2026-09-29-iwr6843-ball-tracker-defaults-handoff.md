# Handoff: IWR6843 ball tracker tuning on Cormac's pipeline (updated 2026-10-01)

## Branch

`test_build_v2` is Cormac's `feat/iwr-calcs` at `ca8c7512`, plus one
commit: the TrackMan data and this handoff.

It replaces `test_build`, which stays as the record of the earlier work.
Most of `test_build` is now covered on his branch:

| Our `test_build` work | Status on his branch |
|---|---|
| `l3dump` freeze fix | He has `l3_awaitFrozenRing` |
| Detect task above the CLI | He has the same ordering: CLI 3 < detect 4 < HWA rearm 5 < control 6 |
| `swing_trigger` health diagnostics | Superseded. His acceptance run (`scripts/hardware-test/iwr6843_dsp_probe.py`) prints the detect timing, budget, `over_budget` and margins per phase |
| DATA_RAM trim, Hann window | Superseded by his rewrite |
| TrackMan ball defaults (`d0ad53fc`) | **Not ported.** See below |

He has also moved detect bin scoring to the DSS, with an L2 gather
(`1237bb39`, `8ce808b3`). And he made the detect frame cheaper (`6024a48b`):
- the ball detector runs every 8th frame;
- the club angle moved to a queued task, off the decision path.

## Ball tracker defaults: not ported, needs Cormac

The TrackMan replay still favours the hypothesis search on his tracker.
These are the 3 ms wide captures, run with the true ball origin
(`dest_bin`):

| Variant | Follows ball (of 124) | Within 5 mph | Drivers (of 18) |
|---|---|---|---|
| his defaults | 68 | 42 | 2 |
| hypotheses only | 95 | 65 | 14 |
| hyp + fastBall 26.5 | 119 | 84 | 17 |
| hyp + fastBall 26.5 + minDep 20 | 119 | 84 | 17 |

But his labelled kiosk replay regresses with the search on:

- **Test:** `test_iwr6843_labelled_replay.py::test_the_labelled_swings_report_their_launch_at_the_kiosk_settings`.
- **Result:** at least 25 good launches are required. With the search on,
  14 of 34 are good and 20 report no launch at all. Hypotheses alone give
  15 of 34, so the search is the cause. `minDepartureMps` 20 alone passes.
- **Likely cause:** the hypothesis path bypasses his newer single-track
  logic:
  - the ball-leave fallback, which seeds the tracker from the fallback's
    two points after a late fire;
  - confident-departure displacement;
  - the late-launch-angle window.

  Those swings are self-triggered and fire late, unlike the sound-triggered
  TrackMan captures.

**Question for Cormac:** should the hypothesis search take the leave
fallback's seed and his displacement rules? Or should the TrackMan wins
come into the single-track path some other way (e.g. a fast-ball floor)?

Enabling the search also changes reporting. `ball_points` start where the
winning line is committed, 3–4 frames past the tee, and that lowers label
coverage even when the launch is right.

## Before turning the search on, its tests fail on his tip

49 IWR tests already fail on `ca8c7512`, before any of our changes:

| Failing on his tip | Count |
|---|---|
| `test_iwr6843_labelled_replay.py::test_firmware_tracks_match_the_labels` | 33 |
| `test_iwr6843_firmware_angle_table.py::test_the_scan_answers_as_it_did_before_the_table` | 11 |
| `test_iwr6843_firmware_replay.py::test_with_the_band_on_every_recording_has_the_club_after_impact` | 2 |
| `test_iwr6843_labelled_replay.py`: two kiosk self-trigger tests | 2 |
| `test_iwr6843_memory_layout.py::test_tracked_reference_map_agrees_with_the_live_map` | 1 |

The memory-layout failure may only be a stale local build map. Confirm
these with Cormac before reading anything into new failures.

## Next rig step

1. Build his tip unchanged.
2. Run his acceptance run (`iwr6843_dsp_probe.py`) on the 3 ms profile.
3. Read `over_budget` and the margins with detect on the MSS and on the DSS.

That gives the first real detect budget on his pipeline. It also settles
the 2 ms question. Cormac says detect would need to run in under 0.3 ms,
which is the ~0.38 ms idle gap of the 2 ms profile. The double-buffered
scratch suggests the budget is closer to one frame. The measured margin
shows which is right.

## Evidence kept from the earlier work

- **Ball signal strength on the true ball line, 3 ms TrackMan captures:**

  | Range | 2.0 m | 2.5 m | 3.0 m | 3.5 m | 4.0 m | 4.5 m |
  |---|---|---|---|---|---|---|
  | Median SNR | 20 dB | 10 dB | 7 dB | 6 dB | 4.6 dB | 2.5 dB |

- **The configured tee was wrong on the August sessions.** It was set at bin
  34, but the true ball origin is a median of bin 39. Replays need
  `dest_bin`.
- **A bin search window was not ready.**
  - It scores ~16 bins per frame instead of 53.
  - But it loses accuracy: 121 → 100 shots follow the ball, and 92 → 58 are
    within 5 mph.
  - Trap: take the noise floor from the full window or from every 4th bin,
    never from the narrow window. The impact echo dominates a narrow-window
    median.
- The comparison scripts live outside the repo (`tm_ball_variants*.py`,
  `tm_ball_window.py`, `tm_ball_line*.py`). On his branch, `tee_global_bin`
  no longer takes `range_bias_m`.
