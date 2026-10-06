# OpenFlight × TrackMan aligned set: 2026-10-04, session 20261004_095102

This is a TrackMan-aligned **input** file for calibrating IWR6843 launch angles. It is in the format `scripts/analysis/evaluate_iwr_horizontal_models.py` reads, with extra columns for VLA work.

It also carries a **provisional** calibration file, `iwr6843_calibration_robinante_provisional_20261004.json`, with an effective tilt fitted to this set's 7 ball-track shots. That is a starting point to test, not a validated calibration. Only 2 rows are usable by the stock evaluator, and 7 are usable with the workaround below. The CSV is meant to be pooled with bigger datasets.

Contributed by Doug (Robinante). Built from his session archive and TrackMan export. The original data was not modified, and every copied file matches the archive's SHA256SUMS.

## Contents
| File | What |
|---|---|
| `trackman_openflight_aligned.csv` | 36 rows, one per TrackMan shot hit while OpenFlight was running (TM #28–#63) |
| `session_20261004_095102_trackman.jsonl` | OpenFlight session log (session_start config, triggers, shots, captures, OPS I/Q) |
| `iwr6843/*.l3dump` | The 26 IWR captures the CSV points at: 11 on-time and 15 early-trigger |
| `trackman_swingsync-2026-10-04.csv` | Raw TrackMan export (all 63 shots) |
| `iwr6843_calibration_robinante_provisional_20261004.json` | Provisional calibration: the stock reference with `tilt_deg` 10 → 4.75 (see below) |
| `SHA256SUMS` | Checksums for everything above |

## Run it
Extract this folder at the **OpenFlight repo root**. The CSV paths are relative (`trackman_20261004_095102/...`), and the evaluator loads `config/iwr6843_calibration_reference.json` relative to the working directory.

```bash
uv run python scripts/analysis/evaluate_iwr_horizontal_models.py trackman_20261004_095102/trackman_openflight_aligned.csv
```

To pool with other sessions, concatenate the rows into one CSV.

The stock evaluator needs at least two wide-IQ16 sessions and one dense-IQ8 session. It trains on the session whose `of_session_id` sorts first, holds out the other wide sessions, and also scores a dense-IQ8 cohort. With any of those cohorts empty it stops with `ZeroDivisionError` in `_metrics`. That is a limit of the script, not of this data; a one-session run fails that way. Extraction of this file's matched rows was verified with the unmodified script.

## match_status
| Value | Rows | Meaning |
|---|---|---|
| `matched` | 2 (TM 38, 47) | On-time trigger, ball track in the dump, impact inside the capture window. The stock evaluator uses these. |
| `matched_impact_before_window` | 5 (TM 30, 45, 46, 52, 54) | On-time trigger with a good ball track, but the back-extrapolated impact falls 2–18 ms before dump frame 0 (`impact_vs_dump_frame0_ms`). The stock `impact_time_s()` returns None, so `_extract` would raise "no ball range evidence"; the stock script skips these rows. They are usable if impact time is clamped or ignored for the horizontal features, which only need the ball track. |
| `matched_club_track` | 3 (TM 51, 56, 57) | Woods. The host tracker followed the club follow-through (~45–48 mph) instead of the ball, and the runtime rejected it (`rejected_track_speed`). Don't use for angles. |
| `matched_no_ball` | 1 (TM 62) | Driver. The trigger fired ~20–30 ms before impact; no ball in the dump or the OPS buffer. |
| `early_trigger` | 15 | The IWR self-trigger fired 0.5–0.8 s before impact (takeaway). No ball. Included for trigger work only. |
| `missed_busy` | 9 | No capture. The system was busy (~7.8 s readback plus radar restart) from a false trigger, named in `blocking_capture`. That dump is not included; it is in the original archive. |
| `missed_no_trigger` | 1 | Nothing fired. |

## Setup (as logged; not all measured)
- Board: IWR6843LEVM in Doug's OpenFlight unit. Firmware 1.0.2, hybrid-cadence (git 1d23e2fdfab1).
- Code: Cormac's `feat/iwr-calcs` merge 4c86e6f, branch `bench/trackman-2026-10-04`.
- Cfg: `iwr6843_l3dump_wide_24f2ms_53bin_iq16_window_hann.cfg` (wide IQ16, Hann window, 2 ms frames, 24 frames).
- Calibration: **stock `config/iwr6843_calibration_reference.json`**. It was made for a different board.
- **Tilt 10° and tee 1.575 m are configured defaults, not measurements.** No inclinometer. `of_effective_tilt_deg` = 10.0.
- Other geometry: net 4.6 m, radar height 0.1524 m, ball height 0.04 m, `tx_order` normal, TDM sign positive. `horizontal_phase_reference_rad` was **unset**.
- Trigger: `triggerCfg 36 1.0 1` (self-trigger, no sound detector). OPS243 at 30 ksps, S#24 (102 ms before, 34.5 ms after the trigger).
- Clock: the Pi clock was 59 min 27.55 s slow (no NTP at the range). Rows were aligned with TM_utc = Pi_local(EDT) + 3567.55 s. The ten shots with a usable OPS ball speed then land within ±20 ms of TrackMan's shot times.
- An earlier session that day (20261004_094014) was a living-room bench run and is excluded.

## Column notes
- Evaluator columns: `tm_sequence`, `of_session_id`, `of_shot_number`, `match_status`, `session_log_path`, `iwr_dump_path`, `iwr_config`, `of_club`, `tm_club`, `tm_ball_speed_mph`, `tm_launch_direction_deg` (TrackMan HLA, negative = left), `of_effective_tilt_deg`.
- Truth: the `tm_*` columns come straight from the TrackMan export.
- `iwr_trigger_minus_tm_s`: IWR trigger time (dump filename) minus TrackMan shot time.
- `of_*`: what OpenFlight logged. **Every displayed VLA was the club-default estimate** (`of_vla_displayed_source` = estimated), because the firmware onboard result was missing on 15 of 16 shots.
- `iwr_host_*`: host LCMF-v1 output (logged in debug mode, never published). It reproduces exactly on offline replay.
- `impact_vs_dump_frame0_ms`: ball-track impact back-extrapolated to the tee (evaluator geometry). Negative means before the capture window.
- `ops_railed_fraction`: share of OPS I/Q samples at the ADC rails. Junk ~25 mph "shots" sit at 11–20%; real balls at ≤1.3%.

## Provisional calibration (in this bundle)
`iwr6843_calibration_robinante_provisional_20261004.json` is `config/iwr6843_calibration_reference.json` with **only `tilt_deg` changed, 10 → 4.75**. Use it with `--iwr6843-cal <file>`, or keep the stock file and pass `--iwr6843-tilt-deg 4.75`. Both also set the board's pitch.

The fit: offline LCMF-v1 replay of the 7 ball-track dumps, with tilt swept 0–12° in 0.25° steps, choosing the tilt with the lowest mean |IWR VLA − TrackMan VLA|.

| | MAE | Bias |
|---|---|---|
| Tilt 10° (as run) | 4.71° | +4.61° |
| Tilt 4.75° | 1.48° | +0.99° |
| Leave-one-out | 1.73° | (picked tilt stayed 4.25–4.75°) |

Per shot at 4.75° (IWR − TrackMan): TM30 +1.5, TM38 +0.7, TM45 −1.7, TM46 +1.9, TM47 +2.4, TM52 0.0, TM54 +2.1.

Treat 4.75° as an **effective** tilt. The real tilt was never measured, so it may be standing in for per-board element-phase error, because the element calibration is the reference board's. If a measured tilt comes out near 10°, the per-board corner-reflector solve is the real fix. The error vs tilt curve is jumpy (estimator mode switching); 4.25–5.0° is its flattest region.

It does not fix the woods (TM 51, 56, 57): the host tracker still follows the club (~45–48 mph) and the runtime still rejects them.

**Horizontal is not calibrated.** No reference under the repo convention fits (leave-one-out MAE 14.9°). A sign-flipped model fits better (4.9°), but in these 7 shots TrackMan HLA and VLA correlate at 0.92: the two biggest pulls are also the two low launches. The TX2 phase could be tracking launch height as much as direction. Separating the two needs high pulls and pushes plus low straight shots; that is the main thing a bigger set can answer.

## Other findings from this set (7 ball-track shots; hypotheses to test at scale)
1. **Logged HLA disagrees with TrackMan:** +3.9° / +4.1° vs −7.4° / −12.2° (reference unset). See the confound above before reading this as a sign problem.
2. **Vertical bias at tilt 10°:** irons read +4.1 to +6.7° high. The low launches read TM54 2.1° vs 2.4° and TM52 6.2° vs 3.2°. Tee distance moves irons only ~1.6° per 10 cm, so it isn't the cause.
3. **Impact timing.** On most on-time captures the ball track crosses the tee before frame 0, while the club line in the pre frames reaches the tee around frame 6. The firmware writes planned frame deltas, not measured ones. This blocks impact-time-based outputs (club path and AoA were all `rejected_no_impact_time`).
4. Ball speed (9 good contacts): displayed +1.2 mph vs TrackMan (sd 0.5); raw −1.8 mph (sd 0.7).

Full write-up: `claude/trackman_20261004_session_review.md` in the OpenFlight project.
