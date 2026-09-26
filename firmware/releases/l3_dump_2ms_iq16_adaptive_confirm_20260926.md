# Adaptive 2 ms IQ16 retention: confirmed tracks, coasting through misses

- Binary: `l3_dump_2ms_iq16_adaptive_confirm_20260926.bin`
- SHA-256: `b23a49b10f00bdc15496689b8bebd09b3f16fbb6dea842e7531bf28d0f92f455`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, native build.
- Built with the same `firmware/Makefile build-native` recipe as the prior
  adaptive release. Supersedes `l3_dump_2ms_iq16_adaptive_20260926.bin` for
  selector behavior; the capture layout, format version and host wire
  protocol are unchanged except for one additive shadow-report field.
- Hardware status: not flashed or tested by the agent. The operator runs
  hardware tests, starting with the first operator test below.

## What changed and why

The 2026-09-26 hardware smoke test
(`openflight_sessions/iwr-adaptive-smoke/`) surfaced two problems in the
bounded live selector, found by replaying the retained IQ16 samples through
the same power calculation the firmware uses:

1. **Static-scene noise was accepted as a moving target.** Across the three
   recorded empty-room captures, 34% of shadow frames were `accepted`, with
   the selected bin jumping around (`81 -> 73 -> 84 -> 79 -> ...`) with no
   real motion present. A track could start and be trusted on a single
   frame's association.
2. **One missed frame ended flight retention.** The Phase 3 motion reference
   showed the tracker tolerates `maxMisses = 2`, but
   `l3_retention_window` stopped keeping flight frames on the very first
   frame that was not accepted, well before the tracker itself gave up.

### Fix 1: confirm before accepting

A new track is now tentative until it has been associated across
`confirmFrames` (3) consecutive frames. Two recorded empty-room captures
contained two-frame noise chains, so two frames was not enough; three was
sufficient to reject every recorded noise chain in the smoke and Phase 3
reference data while still confirming the real moving target on its third
frame. Raising the SNR threshold was evaluated and rejected: the real
target's measured confidence (7-12x noise) was consistently *lower* than the
noise peaks that were wrongly accepted (12-22x noise), so no threshold could
separate them on a single frame. Consistency across frames can.

### Fix 2: coast through bounded misses

A confirmed track now coasts on its last measured velocity through up to
`maxMisses` missed frames, instead of stopping retention on the first miss.
The retained window for a coasting frame covers both the last measured bin
and the predicted bin, each with the same 2-bin margin required for a normal
accepted frame. If the spread is too wide for the 12-bin window, or misses
exceed the bound, retention still ends explicitly (`track_lost`).

### Host and diagnostic changes

- `l3_live_select` / `l3_retention_window` (`firmware/iwr6843/live_selector.c`)
  implement confirmation and coasting; `L3LiveSelectorParams` gained
  `confirmFrames` and `L3LiveSelectorState` gained `hits`.
  `L3LiveSelectorResult` gained `heldBin` and `coasting`.
- The per-frame shadow report gained an additive `co=<0|1>` field
  (`SHD f=... ok=... a=... co=...`). Firmware without this field is parsed
  as `coasting=0` by `read_shadow_dump` in `src/openflight/iwr6843/driver.py`.
- `scripts/hardware-test/check_iwr_2ms_iq16.py` treats a coasting flight
  frame the same as an accepted one when checking that the stored window
  matches the selector's decision, since a coasting frame is not `accepted`
  by design.
- A Python reference in `src/openflight/iwr6843/live_selector.py` implements
  the same confirmation/coasting/retention logic and is checked against the
  C implementation, including the retention decision, on every test case
  (`tests/test_iwr6843_live_selector.py`).
- No change to `dump_format.h`, capture geometry, memory budget, or the
  36-frame layout from the prior adaptive release.

## Local verification

- `uv run pytest tests/test_iwr6843* -q`: 522 passed, 5 skipped (missing
  local recorded-capture fixtures, same as the prior release).
- Two of the new tests replay the recorded 2026-09-26 smoke captures
  (`openflight_sessions/iwr-adaptive-smoke/shadow-reference-00{1,2,3}.l3dump`)
  and the Phase 3 reference capture
  (`openflight_sessions/iwr-shadow-association/shadow-reference-001.l3dump`)
  through the rebuilt selector input, and require the confirmed selector to
  reject the recorded noise while confirming the recorded motion.
- Ruff check and format: passed on changed files.
- Pylint on `src/openflight/iwr6843/live_selector.py`: 9.65/10 (pre-existing
  missing-docstring style findings only).
- TI native build and meta-image packaging: passed; image is 363,332 bytes
  with the TI `MSTR` signature (256 bytes larger than the prior adaptive
  image, from the added selector state and shadow-report field).
- No hardware validation of this image was performed by the agent.

## First operator test

Same procedure as the prior adaptive release, run against this binary:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-adaptive-confirm-smoke \
  --output openflight_sessions/iwr-adaptive-confirm-smoke/run.jsonl
```

This can run at home, including with no movement. With the confirmation fix,
an empty-room run is now expected to report `track_lost` at frame 14 (the
first impact frame, before any track has had 3 frames to confirm), rather
than sometimes fabricating a confirmed track from noise as the prior image
could. If a static-scene run reports `complete` or keeps any flight frames,
that is a regression from this fix and should be investigated before
further testing.

After reviewing the smoke files, increase to 100,000 frames and 100 cycles,
then move to a motion test: swing something through the capture range during
the trigger and confirm at least one capture keeps flight frames. Real-shot
window coverage, club/ball association and measurement accuracy remain
unqualified, as in the prior release. Adjust the fixed pre/impact window
origins to the measured tee geometry before range tests.

## Known remaining issues (not addressed in this build)

- No clipped-sample count is reported. Offline analysis of the smoke dumps
  found roughly 0.1% of I/Q samples at full scale in a strong stationary
  reflector; the plan requires clipping status to be preserved.
- No explicit net-distance stop; RF acquisition still runs the full
  post-trigger interval even after retention ends.
- Runtime stack peaks and real-shot flight-window coverage remain
  unmeasured, as noted in the prior adaptive release.
