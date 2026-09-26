# Adaptive 2 ms IQ16 retention: fix the coherent gate's diluted mean

- Binary: `l3_dump_2ms_iq16_adaptive_coherent2_20260926.bin`
- SHA-256: `fd2c3ca769038ac1035e829656646e3b27f13be19d5cbd417d1e9158443db0ed`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, native build.
- Supersedes `l3_dump_2ms_iq16_adaptive_coherent_20260926.bin`, which the
  operator's own hardware soak caught as insufficiently fixed (see below).
  Capture layout, format version and host wire protocol are unchanged.
- Hardware status: not flashed or tested by the agent. The operator runs
  hardware tests, starting with the first operator test below.

## What was wrong with the previous (coherent) release

The operator ran the previous release's own recommended 100-cycle static soak
with `--expect-no-flight-frames`. It still failed, at cycle 18, keeping 2
flight frames on a static reflector drifting 42 -> 42 -> 43 with confidence
up to 3435/256 (~13.4x noise) — comparable to real motion, not suppressed as
the offline evidence predicted.

Root cause, found by reproducing the exact frame from the operator's own
capture: `l3_dump.c` computed the coherent gate's threshold by calling
`l3_coherent_gate()` with the *full* 128-bin array, after that array had
already been masked to zero everywhere outside the ~53-bin active analysis
window (the existing pre-trigger/impact masking). `l3_coherent_gate()`
averages over whatever it is given, so its "mean" was diluted to roughly
53/128 (~41%) of the true in-window mean by all the zeroed bins. The intended
2x-mean gate was therefore an effective ~0.83x-mean gate — weak enough for
ordinary static clutter, whose coherent ratio in the recorded evidence
reached up to 1.51x the true in-window mean, to still pass.

This was missed in the previous release because the offline evaluation in
`plans/iwr-coherent-gate.md` computed each detector's mean over just the
sliced active-window array (32, 53 or 128 bins as appropriate), never over a
zero-padded 128-bin array — an accurate model of the *intended* design, not
of what `l3_dump.c` actually called. No test exercised the firmware's call
site with the masking applied first; the C/Python parity tests exercised
`l3_coherent_gate()` itself correctly, on whatever array they passed it.

## The fix

`l3_dump.c` now calls `l3_coherent_gate()` on the active-window slice only
(`&gShadowPower[activeStart]`, `&gShadowCoherent[activeStart]`, `activeBins`),
matching the offline evidence exactly, and only masks the full 128-bin arrays
to zero *after* the gate has run over the correct slice. `activeStart`/
`activeBins` are the impact window during pre-trigger/impact frames and the
full 128 bins during flight frames, same as before. No other logic changed.

Re-verified on the operator's own cycle-18 capture (frames 15-19, the only
frames a retained dump can exactly reproduce): with the corrected mean, the
gate's ratio at the falsely-confirmed bin (42) was 0.92-1.02x the true
in-window mean on every one of those frames — nowhere near the 2x threshold
that should have rejected it, versus the diluted mean that let it through.

The rest of the plan's offline evidence is unaffected, since it already
modeled the corrected behavior: 0/129 recorded static false confirmations
(now confirmed to be a real, matching model of what the firmware does), and
the same synthetic injected-ball detection numbers.

## Memory and timing

- DATA_RAM: 98,393 of 196,608 bytes, unchanged from the previous release —
  this fix only changes which slice is passed to an existing function, not
  any new storage.
- CPU: unchanged from the previous release's *measured* hardware result
  (worst case 1,551 us selector + 151 us compaction = 1,702 us of 2,000 us,
  ~300 us margin) — the gate call now runs over fewer bins during pre-trigger
  frames (53 or fewer instead of always 128), so this build should be no
  slower; not yet re-measured on hardware.

## Local verification

- `uv run pytest tests/test_iwr6843* -q`: 535 passed, 5 skipped (unchanged;
  the bug was in `l3_dump.c`'s integration, which these Python-level tests
  do not exercise — they test `l3_coherent_gate()` itself, which was always
  correct for whatever array it is given).
- TI native build and meta-image packaging: passed; image is 363,652 bytes
  (identical size to the previous release), TI `MSTR` signature present.
- No hardware validation of this image was performed by the agent.

## First operator test

Same procedure as the previous release. Given the previous release's own
smoke test looked clean and only the full 100-cycle soak caught this, run the
full soak again, not just the 3-cycle smoke, before trusting this build:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-adaptive-coherent2-smoke \
  --output openflight_sessions/iwr-adaptive-coherent2-smoke/run.jsonl

uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 100000 --poll-s 2 --cycles 100 \
  --capture-dir openflight_sessions/iwr-adaptive-coherent2-soak \
  --output openflight_sessions/iwr-adaptive-coherent2-soak/run.jsonl
```

If the 100-cycle soak still fails, please share the failing capture again —
that would mean either a second, different problem, or that the 2x gate
factor itself is not strong enough even when correctly computed, which would
call for a larger factor rather than another integration fix.

## Known remaining issues (carried over, unchanged)

Same as the previous release: no clipped-sample count, no explicit
net-distance stop, no real golf-ball evidence, and the slow-in-bin-target
blind-speed caveat from `plans/iwr-coherent-gate.md`.
