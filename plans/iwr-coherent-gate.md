# Plan: coherent gate for the adaptive IQ16 selector

> Date: 2026-09-26. Branch `feat/iwr-offloading`, after commit `6be231a`.
> Status: scoped and evaluated offline; not implemented. Replaces "option 2"
> (`confirmFrames = 4`) as the fix for static-scene false confirmations.

## Problem

The 100-cycle static-scene soak (`openflight_sessions/iwr-adaptive-confirm-soak-no-fail/`)
kept flight frames in 7 of 100 captures with nothing moving. The selector input is
the frame-to-frame rise in summed `|I|+|Q|`. That rectified amplitude difference
flickers on static clutter (the ~88 dB reflector at bins 38–40 sits inside the
impact window), and the flicker occasionally forms a 3-frame outward chain that
the confirmation rule accepts. No amplitude threshold separates it: noise peaks
reach 10× the in-window mean, higher than the real movers we recorded.

## Evidence (offline replay, 2026-09-26)

Replay of the firmware selector input from retained IQ16 matched the firmware's
logged candidates on 645/645 impact frames (15–19) across 129 static captures.
Only frames 15–19 are exactly reproducible offline (pre-trigger dumps keep bins
20–51, the selector sees 32–84); they are the frames that decide flight retention.

| Detector | Would keep a flight frame (129 static captures) | Injected ball 3.2 bins/frame | 1.5 bins/frame | On-ball of accepted |
|---|---:|---:|---:|---|
| Magnitude rise, confirm 3 (current) | 20 | 25/40 | 21/40 | 43/44, 28/33 |
| Magnitude rise, confirm 4 (option 2) | — (not re-run with the corrected metric below) | 15/40 | 8/40 | all |
| Plain coherent `|x_t − x_{t−1}|` | — | trails the ghost — rejected | | |
| **Hybrid: rise gated by coherent ≥ 2× mean** | **0** | **28/40** | **30/40** | 53/56, 46/57 |

The "would keep a flight frame" column replays the selector continuously from
frame 1 with a state reset at the first impact frame (matching the firmware's
`memset(&gShadowState, ...)` at `slot == preFrames`), then runs the full
`retention_window()` check — margins and ambiguity included — on the last
impact frame's result, exactly as `l3_dump.c` does for the first flight frame.
An `accepted` flag appearing anywhere *inside* the pre-trigger interval is not
by itself a problem (the 2026-09-26 soak logs show this happening in most
cycles that still kept zero flight frames); the reproducible tool at
`scripts/analysis/iwr_selector_replay.py` implements this distinction.
This offline rate (20/129, ≈15%) is the same order of magnitude as the
hardware soak's 7/100 (≈7%) and the same direction; exact match is not
expected from an offline fixed-point approximation.

- Coherent differencing cancels static clutter almost perfectly on this hardware:
  the static in-window coherent peak never exceeded **1.51×** its mean.
- Plain coherent differencing is unusable on its own: a target that changes bin
  appears at both its new and its old bin with equal energy, and the tracker
  locks onto the old one. Gating by magnitude drop does not fix it because the
  ball crosses clutter bins stronger than itself (ghost magnitude ratio 0.62–1.0).
- The hybrid keeps the magnitude rise for *where* (a departure is a fall, so no
  ghost) and uses coherent SNR for *whether*. Results are identical with the
  firmware's 128-bin noise estimate, so the selector needs no change.
- Both recorded indoor motion captures show a persistent coherent mover at bin 13
  (≈0.6 m) at 8–17× mean; the hybrid follows it.

Injected-ball limits: a single-bin target with random per-chirp phase added to
real static IQ, frames 15–19 only, fresh tracker state at frame 15. Absolute
detection rates are pessimistic; the comparison between detectors is fair. There
is no real golf-ball data yet.

## Design

Per completed frame, in the same loop that already sums `|I|+|Q|` over the
sampled rows (every 3rd loop, 3 TX, RX 0/2 → 24 rows × 128 bins):

1. `coherent[bin] = Σ_rows |ΔI| + |ΔQ|` against the same rows of the previous
   frame. Store the current rows as the new previous frame.
2. Existing magnitude rise, unchanged.
3. **Gate:** `rise[bin] = 0` unless `coherent[bin] × 256 ≥ mean(coherent over the
   active window) × kQ8`, with `kQ8 = 512` (2×). The active window is the impact
   window during pre+impact frames and all 128 bins in flight frames, matching the
   existing masking.
4. Selector, confirmation, coasting and retention unchanged.

The gate is a pure function `l3_coherent_gate(rise, coherent, start, count, kQ8)`
in `live_selector.c`, mirrored in `live_selector.py`, so it is unit-testable and
parity-tested exactly like the selector.

Resources:

- Previous-frame rows: 24 × 128 × 2 × int16 = **12,288 bytes**, DATA_RAM
  (78,865 of 196,608 bytes used before this change). Coherent array: 512 bytes.
- CPU: the L3 sample reads are shared with the magnitude pass; the additions are
  one TCM read, one TCM write and a subtract/abs per sample. Current worst case is
  778 µs selector + 256 µs compaction = 1,034 µs of 2,000 µs. **Must be measured**;
  the hardware check already fails if selector + compaction reaches the frame period.
- Reset: the previous-frame rows are invalid whenever `gShadowHavePrevious` is 0
  (sensor start, rearm), so the gate outputs zero on the first frame, as now.

## Steps (tests first)

1. **Replay tool into the repo.** Move the offline A/B harness from the scratchpad
   into `scripts/analysis/iwr_selector_replay.py` (reuse `_shadow_powers` from the
   selector tests; one implementation). It reproduces firmware candidates and
   reports false confirms and injected-ball detection per detector.
2. **Failing tests.** In `tests/test_iwr6843_live_selector.py`:
   - recorded static captures (a representative subset of the 7 soak false
     positives plus the earlier smoke captures) → no confirmed track through the gate;
   - injected ball at 3.2 and 1.5 bins/frame into a recorded static capture →
     confirmed, and every accepted frame within 1 bin of truth (never the ghost);
   - Phase 3 motion captures → still tracked;
   - C/Python parity for `l3_coherent_gate`, including all-zero input, a single
     bin, window edges, and saturated IQ16 differences (±65,535 per component).
3. **Python reference** `coherent_gate()` + coherent power helper.
4. **Firmware**: `l3_coherent_gate` in `live_selector.c`; previous-row buffer and
   coherent accumulation in `l3_storeCompletedScratchFrame`; gate applied before
   `l3_live_select`. Add `coherent_max_us` or fold into `shadow_max_us`; add a
   `g=` gated-bin count to the `SHD` report (optional field, older firmware parses
   as absent).
5. **Build** a new release name (`..._adaptive_coherent_20260926.bin`) with notes.
6. **Hardware**: 1,000-frame smoke, then the 100-cycle static soak **with**
   `--expect-no-flight-frames`, then a motion test.

## Acceptance

- [ ] Offline: 0 false confirms on all 129 recorded static captures; injected-ball
  detection ≥ current detector at 3.2 and 1.5 bins/frame; no ghost-trailing.
- [ ] C and Python agree on every gate test vector.
- [ ] Hardware: selector + compaction worst case < 1,600 µs (≥ 400 µs margin).
- [ ] Hardware: 100 static cycles with `--expect-no-flight-frames` pass
  (current firmware: 7/100 fail).
- [ ] Hardware: a swung object is confirmed and keeps flight frames.

## Risks and open questions

1. **Gate factor.** `k = 2` sits 1.3× above the largest static coherent peak seen
   (1.51×). `k = 3` gives the same results except at the weakest injected balls.
   Recommend 2, with the soak as the check.
2. **Blind speed.** Coherent differencing between frames exactly 2 ms apart cancels
   a target that stays in one bin with radial speed a multiple of λ/2 / 2 ms ≈
   1.25 m/s. A ball leaves the bin within a frame or two; slow in-bin movers can
   drop out for a frame, which coasting already tolerates.
3. **Real ball SNR is unknown.** All ball evidence here is synthetic. The motion
   test and Phase 5 range data decide it.
4. **Timing.** If the extra pass exceeds budget, compute coherent only over the
   active window (53 bins during impact) before considering anything else.
5. **Frame 14 boundary** is handled correctly on the device (it computes all 128
   bins) even though offline replay cannot reproduce it.

## Fallback

If timing or hardware results fail and cannot be fixed within the active-window
optimisation, fall back to option 2 (`confirmFrames = 4`): 1/129 static false
confirms offline, at the cost of roughly half the injected-ball detections.
