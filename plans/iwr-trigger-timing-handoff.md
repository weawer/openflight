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
- HEAD: `c8f9005` (`range session 3`); working tree was clean when this handoff
  was updated.
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

The following results were supplied by the user from the Pi. Static timingfix4
results are operator-reported. Range-session JSONL, raw OPS logs, and IWR dumps
are tracked under `openflight_sessions/`.

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

### Range Session Artifact Review

Session `openflight_sessions/session_20260927_130535_range.jsonl` and raw log
`openflight_sessions/radar_raw_20260927_130535.log` show two accepted OPS
captures. The first produced mode-based estimates of 25.0 mph ball and 18.2
mph club, but its IWR dump was 2,034 bytes short (439,314 received versus
441,348 expected). The second produced 53.2 mph ball and 43.1 mph club; a
78.9 mph outbound reading was treated as an outlier. About 29.83% of the OPS
I/Q samples were repaired for clipping, and its 7,910 RPM spin result had
evidence 0.72 and was labelled experimental/low confidence. Camera capture
was disabled in this session.

The second IWR dump is present at
[`openflight_sessions/iwr6843_20260927_131018_934_002.l3dump`](../openflight_sessions/iwr6843_20260927_131018_934_002.l3dump),
SHA-256 `6c660e5ffb10e943964613bbd27a71549124a5707ff6c28661139bb45b6abf9f`.
It parses as version 9, timed IQ16, 36 chirps/frame, 3 TX x 4 RX, 2 ms frame
period, and retention reason `ambiguous`: 14 pre-trigger frames, 6 impact
frames, and 2 tracked flight frames (22 frames total). The recorded IWR
temperatures were about 59-64 C.

Important timestamp correction: the session reports the IWR trigger at
`1790507418.934044` and the OPS `shot_timestamp` at `1790507419.396846`, a
delta of -462.802 ms. This does **not** establish that the IWR triggered 463
ms before physical impact. In
`src/openflight/rolling_buffer/monitor.py`, `shot.impact_timestamp` is assigned
the OPS capture trigger epoch. The OPS impact estimator for this capture fell
back to the capture trigger (`sound_trigger`, reason `speed_delta_below_threshold`),
so there is no independent contact-time estimate in this session. The observed
delta is between the IWR firmware-trigger notice and the OPS onboard ST event.
The server message `Self-trigger impact -> S!` consequently has a misleading
label in this OPS-ST/SM configuration: the value is relative to that OPS
trigger epoch, and ST/SM mode suppresses the actual IWR-to-OPS `S!` relay.

The dump ends after two compact flight frames. The selector decision that
caused the `ambiguous` stop is not stored in the dump, so the precise competing
bins cannot be recovered from this artifact. The offline selector replay can
inspect retained frames but cannot reconstruct the rejected next frame's full
128-bin candidate set. Do not infer an exact ambiguity cause or tune selector
thresholds from the retained frames alone.

The first capture's `capture_path` is null in the session event, so its short
binary was not saved under the configured IWR output directory. The second
capture dump was saved even though runtime validation rejected it for
`ambiguous` retention; session `capture_bytes: 0` means it was not accepted as
a valid measurement, not that the file is empty.

The release note
[`l3_dump_2ms_iq16_timingfix4_trigger_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix4_trigger_20260927.md)
predates the reported successful hardware runs; use this handoff for the
current status.

### Labeled 7-Iron Session

Session `openflight_sessions/session_20260927_145556_range.jsonl` and its raw
OPS log and ten IWR dumps were added in commit `c8f9005`. The operator labeled
the ten attempts in chronological dump order as `xxxx-x--/-` (`x` = hit,
`-` = miss, `/` = semi-hit). This alignment is provisional for dump 001 because
it has no matching `iwr6843_capture` or `shot_detected` entry; the remaining
nine dumps map to logged shots 1-9. Camera capture was disabled.

| Dump | Operator label | Session shot | Retention | Frames retained | Flight frames |
| --- | --- | ---: | --- | ---: | ---: |
| `145619_364_001` | hit | — | ambiguous | 27 | 7 |
| `145637_785_002` | hit | 1 | ambiguous | 29 | 9 |
| `145651_065_003` | hit | 2 | track_lost | 22 | 2 |
| `145705_474_004` | hit | 3 | track_lost | 28 | 8 |
| `145722_516_005` | miss | 4 | track_lost | 30 | 10 |
| `145740_398_006` | hit | 5 | track_lost | 21 | 1 |
| `145756_628_007` | miss | 6 | track_lost | 33 | 13 |
| `145811_717_008` | miss | 7 | track_lost | 27 | 7 |
| `145826_318_009` | semi-hit | 8 | track_lost | 27 | 7 |
| `145841_086_010` | miss | 9 | track_lost | 22 | 2 |

All ten captures stopped before the planned 36 frames: two ended `ambiguous`
and eight ended `track_lost`. The latest session configuration used
`triggerCfg 41 6.0 2 12 3 0.0 1.0 1 1.5`. For the nine captures associated
with an OPS shot, IWR and OPS event timestamps differed by -5.466 to +5.266 ms.
Thus timing alignment is good, but capture retention failed even on operator-
labeled hits; the hit/miss labels do not explain selector loss. Dump 001 has no
session shot record, so its label and triggering event cannot be independently
verified from the JSONL.

No IWR launch or club-path measurement was produced by these adaptive captures.
The published vertical launch values were marked `estimated`; they are not IWR
measurements. OPS spin confidence ranged from about 0.013 to 0.404, and several
estimates repeated at 10,986 RPM, so these results do not validate spin
measurement. Do not use the OPS speed, launch, carry, or spin values as truth
labels for the operator's hit/miss assessment.

The selector's last evaluated frame is still not included in the dump when
retention stops. Retained windows show the track immediately before the stop,
but cannot identify which candidate(s), powers, or selector state caused the
rejection. Do not tune selector thresholds from these dumps alone.

## Next Steps

0. **Selector-stop diagnostics: implemented, not yet built or flashed.**
   Firmware writes one `RST r= f= cc= c= p= s= h= ok= a= co= n= w=` line before
   an adaptive16 `l3dump` binary whenever the live selector ended retention
   (`gRetentionStopRecorded`; `short_history` set at dump time emits nothing).
   `f` is the dump-order index of the first unretained frame. The binary format
   is unchanged. Host: `IWR6843Radar.read_adaptive_dump()` →
   `parse_retention_stop()` → `IWR6843Capture.retention_stop` →
   `iwr6843_capture.retention_stop` in the session JSONL. Tests: native
   round-trip for `track_lost` (lost and coasting) and `ambiguous` plus a
   128-byte buffer bound in `test_iwr6843_live_selector.py`; parser, monitor,
   and session-log tests. Native tests were run on Windows with `ziglang cc`;
   `l3_dump.c` has **not** been compiled with the TI toolchain. Next: build a
   distinctly named image, record its SHA-256, flash, and repeat step 3.
1. **Add selector-stop diagnostics before tuning.** (See step 0.) At the exact frame where
  `l3_retention_window()` returns a nonzero reason, preserve the frame index,
  reason, candidate bins and powers, selected/held bins, accepted/coasting/
  ambiguous flags, noise, and proposed window. Emit this compact record in the
  CLI text prefix before the binary dump and parse it into the host capture
  event. Do not add the rejected frame's samples to the measured capture or
  change the dump binary format for this diagnostic. Add a native firmware
  regression for both `track_lost` and `ambiguous` stop records plus a host
  parser/session-log test. This is the next implementation task.
2. **Correct the timing log terminology.** The current `Self-trigger impact ->
  S!` log compares IWR notification time with `shot.impact_timestamp`, which
  is populated from the OPS trigger epoch in rolling-buffer mode. Rename or
  compute this against a genuine impact estimate; do not call the current
  delta impact-to-trigger latency. In ST/SM mode it also does not mean `S!`
  was sent.
3. **Repeat labeled real shots after diagnostics land.** Keep trigger and
  selector parameters unchanged. Use the normal session workflow and label
  each attempt in order, including misses and semi-hits. Do not use
  `scripts/iwr6843/watch_trigger.py` for acceptance; it discards captures.
  Enable camera capture only when validating the full camera/IWR path.
4. **Analyze per-shot evidence.** Separate OPS trigger time, IWR trigger
  notice time, inferred physical impact time, and notification/dump times.
  Inspect clipping, detector flight-recorder entries, selector candidate
  bins, retained frame windows, and whether impact/ball-flight data survived.
5. **Check runtime health in the same run.** Require combined work below
   2,000 us, no missed HWA frames, no rearm errors, and successful repeated
   captures. Treat per-stage maxima as diagnostics unless they are the
   same-frame `workmax_*` tuple.
6. **Tune only from evidence.** If real swings fail or false-trigger, inspect
   the saved detector log and raw capture, add a native regression for the
   observed pattern, then adjust one qualification rule at a time. Do not
   change SNR, range gate, minimum approach, coherence, or Doppler thresholds
   based only on the stationary runs.
7. **After any firmware change**, run the native trigger tests and full IWR
   tests, build a distinctly named TI image, record its SHA-256, and ask the
   user to flash and run the corresponding hardware check. Do not claim
   hardware validation from simulation or host tests.

## Instructions for the Next LLM

- Read repository `AGENTS.md`, this handoff, and the latest release note before
  proposing or changing code.
- Start with `git status --short --branch`; current handoff baseline is
  `c8f9005` (`range session 3`). The 14:55 session artifacts and operator
  labels are tracked.
- Treat the timingfix4 static results as user-reported hardware evidence; the
  logs do not prove the flashed binary checksum.
- The 14:55 range batch has ten chronological IWR dumps and ten operator
  labels, but only nine shot-event associations. Every dump stopped adaptive
  retention early. First add stop-frame diagnostics; then correlate clean hits
  and misses before changing selector policy.
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