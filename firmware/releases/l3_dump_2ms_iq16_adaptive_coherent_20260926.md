# Adaptive 2 ms IQ16 retention: coherent gate against static clutter

- Binary: `l3_dump_2ms_iq16_adaptive_coherent_20260926.bin`
- SHA-256: `eafd9c1352c6d04465d02fd05f095148a9485e331c04518120756f0c6910efb8`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, native build.
- Supersedes `l3_dump_2ms_iq16_adaptive_confirm_20260926.bin`. Capture layout,
  format version and host wire protocol are unchanged.
- Hardware status: not flashed or tested by the agent. The operator runs
  hardware tests, starting with the first operator test below.

## What changed and why

The 100-cycle static-scene soak on the previous (confirm) release
(`openflight_sessions/iwr-adaptive-confirm-soak-no-fail/`) kept flight frames
in 7 of 100 cycles with nothing moving. The selector's input — a rise in
summed rectified `|I|+|Q|` — flickers on static clutter (a ~88 dB reflector
sits inside the fixed impact window), and the flicker occasionally forms a
3-frame chain the confirmation rule accepts. No amplitude threshold separates
it: the false chains reached higher peak/noise ratios than the real motion
recorded in the Phase 3 reference captures.

Full scoping, offline evidence and rejected alternatives are in
`plans/iwr-coherent-gate.md`. Summary: static and quasi-static returns cancel
almost exactly under coherent (complex, not rectified) frame-to-frame
differencing — every recorded static capture stayed under 1.51x its in-window
mean coherent difference. Plain coherent differencing alone is unusable
(a target that changes bin appears at both its old and new bin with equal
energy, and the tracker follows the ghost at the old bin). The fix keeps the
existing magnitude-rise detector to locate a candidate (a departing target is
a fall in rectified amplitude, so it has no ghost) and gates it: a bin is kept
only if its coherent difference is at least 2x the in-window mean.

Offline replay of the recorded evidence, reproducing the firmware's exact
masking/reset behavior:

| Detector | Static captures that would keep a flight frame (of 129) |
|---|---:|
| Magnitude rise (previous release) | 20 |
| **Coherent-gated magnitude rise (this release)** | **0** |

Against a synthetic point target with random per-chirp phase injected into
real recorded static IQ (no real ball data exists yet — this bounds relative,
not absolute, detection), the gated detector confirmed at least as many
injected targets as the ungated one at both tested speeds.

## Implementation

- `l3_coherent_gate()` (`firmware/iwr6843/live_selector.c` /
  `live_selector.h`): keeps `rise[bin]` only if `coherent[bin]` is at least
  `L3_COHERENT_GATE_Q8/256` (2x) times the mean of `coherent` over the same
  bins. Mirrored exactly in `src/openflight/iwr6843/live_selector.py`'s
  `coherent_gate()` and parity-tested against the C implementation on
  synthetic vectors and on real recorded static/motion captures.
- `l3_storeCompletedScratchFrame()` (`l3_dump.c`) now also retains the raw
  sampled rows (every 3rd loop, 3 TX, RX 0/2 — the existing shadow sampling)
  of the previous frame, computes the coherent difference against them,
  applies the gate, and feeds the gated array to `l3_live_select` instead of
  the raw rise. The gate is skipped (all-zero) on the first usable frame,
  same as before any previous-frame data exists.
- No change to the selector, confirmation, coasting or retention logic from
  the previous release, and no change to the capture format, memory
  partitioning for retained samples, or host wire protocol.
- New offline analysis tool, `scripts/analysis/iwr_selector_replay.py`,
  reproduces the firmware's selector input (including the pre-trigger/impact
  window masking and the state reset at the first impact frame) from any
  retained IQ16 dump. Used to produce the table above and to reproduce the
  known false confirmations as a regression test.

## Memory and timing

- DATA_RAM: 98,393 of 196,608 bytes used (previous release: 78,865). The
  increase is the previous-frame row buffer needed for coherent differencing:
  `ceil(16/3) loops x 3 TX x ceil(4/2) RX x 128 bins x 2 (I/Q) x int16 =
  18,432 bytes`, sized for the compile-time maximum loop count rather than the
  12-loop profile actually used, plus two 128-bin `uint32` scratch arrays.
  98,215 bytes remain free.
- CPU: the coherent accumulation reuses the existing per-sample read inside
  the shadow sampling loop (one extra TCM read/write and a subtract/abs per
  sample); the gate itself is one pass over 128 bins. **Not yet measured on
  hardware.** The previous release's worst case was 778 us selector +
  256 us compaction = 1,034 us of 2,000 us; this change must be re-measured
  before trusting that margin. If it does not fit, the first fallback is to
  compute the coherent difference only over the active analysis window
  (53 bins during pre-trigger/impact) rather than all 128 bins.

## Local verification

- `uv run pytest tests/test_iwr6843* -q`: 535 passed, 5 skipped (missing
  local recorded-capture fixtures, unchanged from prior releases). 13 new
  tests cover the gate's C/Python parity (including on real recorded
  captures), removal of all 129 recorded static false confirmations, retained
  tracking of both recorded motion captures, and synthetic-ball detection
  parity against the ungated detector.
- `live_selector.c` compiles standalone with `cc -std=c99 -Wall -Wextra
  -Werror` (the same flags the test fixture uses).
- Ruff check and format: passed on changed Python files.
- Pylint on `src/openflight/iwr6843/live_selector.py`: 9.67/10 (pre-existing
  missing-docstring style findings only).
- TI native build and meta-image packaging: passed; image is 363,652 bytes
  with the TI `MSTR` signature (320 bytes larger than the previous release).
- No hardware validation of this image was performed by the agent.

## First operator test

Same procedure as prior releases, with the guard flag added after the
previous release's soak proved it catches this exact failure mode:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-adaptive-coherent-smoke \
  --output openflight_sessions/iwr-adaptive-coherent-smoke/run.jsonl
```

Watch the `rearm max`/`compact max`/`selector max` lines closely on this run —
timing was not measured on hardware before release and is the most likely way
this build could regress. If selector-plus-compaction approaches the 2,000 us
frame period, stop and report before continuing.

Once timing looks healthy, run the 100-cycle static soak **with**
`--expect-no-flight-frames`:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 100000 --poll-s 2 --cycles 100 \
  --capture-dir openflight_sessions/iwr-adaptive-coherent-soak \
  --output openflight_sessions/iwr-adaptive-coherent-soak/run.jsonl
```

The previous release failed this at 7/100 cycles. This release is expected to
pass all 100; if it does not, that is a real regression, not an expected
residual, and should be investigated before a motion test. After it passes,
move to a motion test: swing something through the capture range during the
trigger and confirm flight frames are kept and follow the target. Real-shot
window coverage, club/ball association and measurement accuracy remain
unqualified, as in every prior adaptive release.

## Known remaining issues (not addressed in this build)

- No hardware timing measurement of the added coherent computation (see above).
- No real golf-ball evidence; all ball-detection numbers in this release and
  its plan are from a synthetic point target injected into real static IQ.
- No clipped-sample count reported (carried over from the previous release).
- No explicit net-distance stop; RF acquisition still runs the full
  post-trigger interval even after retention ends.
- A slow in-bin target can cancel under coherent differencing if its radial
  speed is close to a multiple of ~1.25 m/s (see `plans/iwr-coherent-gate.md`);
  a ball is expected to cross bins within a frame or two, and one dropped
  frame is already tolerated by coasting, but this is unverified on real shots.
