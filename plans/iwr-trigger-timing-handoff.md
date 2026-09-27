# IWR Trigger and Timing Handoff

Updated: 2026-09-27

## Goal

Offload IWR6843 acquisition and range selection while retaining IQ16 data at a
2 ms frame period. The current near-term goal is to establish that the IWR
self-trigger is quiet in a stationary scene, meets the frame deadline, and
then fires reliably on real golf swings without losing useful impact or ball
flight data.

## Current State

- Branch: `feat/iwr-2ms-trigger-merge`.
- HEAD: `445c0e8` (`reject one-bin jitter triggers`).
- The working tree has one uncommitted change in
  `firmware/iwr6843/l3_trigger.c`; it is indentation-only. Preserve it unless
  the user explicitly asks otherwise.
- The reported hardware image is `l3_dump_2ms_iq16_timingfix4_trigger_20260927.bin`,
  SHA-256 `ff7b1f3ef070279f070aedde0d3723c9859844c232cdab73948fbd73e651a363`.
  The latest test output does not itself identify the flashed binary hash;
  confirm the image if firmware identity matters.
- The matching checker includes a wait for the full configured pre-trigger
  history between capture cycles. Without this host-side update, back-to-back
  captures can begin before the prebuffer refills and be incorrectly rejected.
- Detector parameters observed in hardware logs: tee bin 41, SNR 6, track age
  2, approach 12 bins, gate 3 bins, minimum step 1 bin/frame, peak statistic;
  coherence and Doppler gates are disabled.

## Work Completed

1. Implemented the IWR club-track self-trigger with an adaptive noise floor,
   range continuity, one-frame miss tolerance, an impact gate, and a trigger
   flight recorder. The detector runs from the firmware rearm path.
2. Established 2 ms IQ16 acquisition and lossless sample compaction. A prior
   hardware validation reported 100,000 continuous frames and 100/100
   capture/rearm cycles with no missed frames. The adaptive profile retains 14
   pre-trigger frames, 6 impact frames, and up to 16 tracked flight frames.
3. Added same-frame stage timing attribution. `frame_work_max_us` is the
   combined work value; `workmax_*` reports the component costs from the frame
   that set it. Independent stage maxima must not be added together.
4. Reduced wasted work by skipping the full shadow selector during adaptive
   pre-trigger buffering and scanning only the configured impact bins during
   the impact phase. Full-range flight selection resumes after a baseline
   reset.
5. Made the raw trigger trace opt-in with `triggerLog trace on|off`; normal
   event records and counters remain available.
6. Fixed two stationary false-trigger patterns in the detector:
   - Equal-bin observations no longer reset the approach origin.
   - Multi-frame triggers require at least two bins of net approach, in
     addition to the configured rate check. Explicit `trackFrames=1` mode is
     unchanged.
7. Updated the hardware checker to save a frozen dump and trigger log on an
   unexpected latch, enforce the 2 ms hard deadline, and wait for a complete
   prebuffer between capture cycles. The 1,500 us figure was previously
   discussed as a provisional target; it is informational, not the current
   hardware pass/fail threshold.

## Hardware Evidence

The following results were supplied by the user from the Pi. They are
operator-reported hardware runs; raw timingfix4 artifacts are not currently
present in the local `openflight_sessions` directory.

- Earlier timingfix images exposed real false triggers: repeated bin 38 then
  bin 39, followed by a separate `39 -> missed -> 38 -> 39` sequence. Both
  sequences reproduced in native detector tests and motivated the current
  guards.
- The user reports two successful stationary runs with the timingfix4 test
  command. Each run completed a 1,000-frame soak and 3 capture cycles; all six
  captures ended with `track_lost`, retained zero flight frames, and no
  unexpected trigger was reported.
- First run maximums: rearm 72 us, compaction 314 us, selector 1,037 us,
  combined frame work 1,452 us.
- Second run maximums: rearm 66 us, compaction 320 us, selector 1,037 us,
  combined frame work 1,453 us.
- Both totals are below the 2,000 us frame period. They leave about 547 us of
  measured headroom. No HWA misses or rearm errors were reported.
- These static runs validate stationary false-trigger resistance, timing,
  prebuffer refill, and repeated capture/rearm. They do **not** validate
  successful triggering on a moving club, impact timing, ball-flight
  selection, or angle accuracy.

The release note
[`l3_dump_2ms_iq16_timingfix4_trigger_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix4_trigger_20260927.md)
predates the reported successful hardware runs; use this handoff for the
current status.

## Next Steps

1. **At the range, test real swings with current settings unchanged.** Use the
   normal session workflow so OPS, IWR, camera, and trigger evidence are
   retained. Do not use `scripts/iwr6843/watch_trigger.py` for acceptance; it
   discards captures.
2. **Collect a small representative batch first.** Include several ordinary
   shots and any relevant practice swings/waggles. Save the session JSONL,
   IWR dumps, shadow decisions, and camera evidence under a uniquely named
   directory and sync them into `openflight_sessions/` for analysis.
3. **Check behavior, not just trigger count.** For each shot determine whether
   the detector fired, when it fired relative to the observed swing/impact,
   whether the capture contains impact and ball flight, and whether the
   adaptive selector's stored windows contain the moving return.
4. **Check runtime health in the same run.** Require combined work below
   2,000 us, no missed HWA frames, no rearm errors, and successful repeated
   captures. Treat per-stage maxima as diagnostics unless they are the
   same-frame `workmax_*` tuple.
5. **Tune only from evidence.** If real swings fail or false-trigger, inspect
   the saved detector log and raw capture, add a native regression for the
   observed pattern, then adjust one qualification rule at a time. Do not
   change SNR, range gate, minimum approach, coherence, or Doppler thresholds
   based only on the stationary runs.
6. **After any firmware change**, run the native trigger tests and full IWR
   tests, build a distinctly named TI image, record its SHA-256, and ask the
   user to flash and run the corresponding hardware check. Do not claim
   hardware validation from simulation or host tests.

## Instructions for the Next LLM

- Read repository `AGENTS.md`, this handoff, and the latest release note before
  proposing or changing code.
- Start with `git status --short --branch` and inspect the current diff,
  especially `firmware/iwr6843/l3_trigger.c`. The existing indentation-only
  edit is user/formatter work; do not discard or rewrite it casually.
- Treat the user-reported timingfix4 static results above as hardware
  evidence, while noting that the raw timingfix4 artifacts have not been
  synced locally and the output does not prove the flashed binary checksum.
- The next validation gap is real-swing trigger and capture quality, not
  further static soak repetition unless new evidence indicates a problem.
- Do not lower/raise detector thresholds or change the two-bin guard without
  reviewing real-shot artifacts and adding a regression that reproduces the
  observed failure.
- Keep the frame period at 2 ms and IQ16. The hard timing criterion is
  combined `frame_work_max_us < 2000` with zero missed frames/errors; 1,500 us
  is informational only.
- For Python commands use `uv run`. Keep firmware binaries uniquely named,
  retain prior images, and report exact checksums and exactly which tests ran.
- Never claim the next swing test passed until the user supplies the results
  or the artifacts are available for inspection.