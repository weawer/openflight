# IWR track separation, capture timing, and DSP decision

Status: proposed sequence; implementation has not started under this plan.
Prepared with Codex assistance from repository inspection, offline replay, and
the user's example plots. The user runs hardware tests and flashing.

## Recommendation and relationship to existing plans

Keep 2 ms frames and IQ16. First establish whether multiple candidate tracks
and better retention can preserve the departing ball. Do not make beamforming
or DSP migration prerequisites for fixing candidate selection.

This document sets the next investigation and implementation gates alongside
the [current pipeline handoff](2026-09-28-iwr-2ms-pipeline-handoff.md).
Reuse applicable work from the
[joint club/ball tracking plan](2026-09-28-iwr-joint-club-ball-tracking.md),
after checking what is actually present on this branch. Its older dataset
counts, thresholds, and branch prerequisites are not this plan's acceptance
criteria. Do not copy another tracker where an existing module can be extended.

The [DSP solve plan](2026-09-25-iwr6843-onchip-solve.md) concerns processing a
frozen capture. Live DSP tracking additionally needs bounded frame handoff,
ownership, cache handling, and deadlines. It is a conditional later step here.

## Evidence and limits

- The September 27 review covered 119 dumps in `openflight_sessions/iwr6843`,
  all with nominal 2 ms spacing and IQ16 range snapshots. Retention headers
  reported 94 `track_lost`, 14 `ambiguous`, and 11 `complete` outcomes. These
  are not detection-accuracy statistics. Separate smoke/soak recordings were
  inventoried but were not counted as shots.
- Session records were matched by basename for 92 captures. The current
  viewer replays different firmware from the original capturing branch and
  only sees retained bins; it cannot reconstruct discarded observations.
- In `iwr6843_20260927_190132_470_006.l3dump`,
  `iwr6843_20260927_190148_765_007.l3dump`, and
  `iwr6843_20260927_190202_788_008.l3dump`, six manually selected outgoing
  peaks at 28–38 ms gave radial fits of 43.5, 50.2, and 52.2 m/s. Corresponding
  OPS speeds were 46.3, 51.4, and 51.6 m/s. At 40 ms the saved windows narrowed
  around closer returns and excluded the candidate continuations. All three
  captures ended with `track_lost`. These examples motivate an experiment;
  they do not validate an automatic classifier.
- User-supplied August plots show outgoing fits around 44–46 m/s and earlier
  club candidates around 31–33 m/s across 2 ms IQ8 and 3 ms IQ16 captures.
  Their captions explicitly state that identities and inferred crossings need
  external validation. An extrapolated range crossing is not proof of impact.
- The recent armed-soak failure involved stale scratch data during diagnostic
  output. The scheduling fix still needs hardware validation. That failure
  alone does not demonstrate insufficient arithmetic capacity on the R4F.

## Shared requirements

- Keep provisional detection, confirmed ball detection, estimated impact,
  end of recording, and serial dump timestamps distinct. Define the existing
  firmware's meaning of “freeze” before changing its timing.
- Physical limits use elapsed time and calibrated range. Normalize legacy
  bins-per-frame and frame-count rules before comparing 2 ms and 3 ms behavior.
- Treat unsaved bins as unknown, never as zero return or a missed detection.
- Use OPS as offline corroboration, not an online dependency when IWR must
  trigger OPS. Radial range speed and total ball speed are not identical.
- Use fixed-size candidate storage and deterministic assignment. Reject or
  report uncertainty rather than inventing a ball track or impact timestamp.
- Make small, separately reviewable changes. Bug fixes start with reproducing
  tests. Preserve raw dumps and the baseline configuration for comparison.

## Step 1 — Establish the baseline and evaluation dataset

- [ ] Have the user run the scheduling-fixed image through short and long
  armed soaks, including diagnostic output, then actual triggered captures.
  Require zero scratch reuse errors, queue drops, and capture overruns.
- [ ] Create a reproducible capture manifest: file hash, session association,
  firmware/config where known, cadence, calibration, retained windows, stop
  reason, and OPS measurements. Deduplicate copies before counting outcomes.
- [ ] Label real strikes, practice swings, stationary controls, and unknown
  cases from available evidence. Folder names alone are not ground truth.
- [ ] Keep the three selected examples as development cases; reserve separate
  sessions as held-out evaluation. Include weaker and failed captures.
- [ ] Reproduce baseline heat maps and candidate/retention reports using the
  viewer data layer. Save a script and machine-readable results, not only plots.

Exit: trustworthy timing baseline and a versioned evaluation manifest.
Missing impact references and unavailable range regions are explicitly marked.

## Step 2 — Test causal multiple-candidate tracking offline

- [ ] Inspect `l3_club_track`, `l3_ball_track`, observation extraction, and
  `firmware_replay` before deciding which existing interfaces to extend.
- [ ] Normalize cadence-dependent motion gates and test equivalent physical
  trajectories sampled at 2 ms, 3 ms, and uneven intervals.
- [ ] Compare current selection against a bounded set of candidates ranked by
  range progression, continuity, fitted radial speed, residual, and plausible
  origin. Strength is supporting evidence, not the sole identity criterion.
- [ ] Allow a ball candidate when the club track is missing. Do not reject a
  true candidate solely because its aliased Doppler disagrees in sign with
  range-derived motion; apply the configured wrapping model where justified.
- [ ] Reveal frames sequentially. Each decision at frame N may use only frames
  up to N; do not use final OPS speed, later frames, or whole-capture fits.
  Fit baselines from past data too. Test that changing future frames cannot
  change an earlier decision.
- [ ] Record first candidate time, first stable confirmation, identity changes,
  false confirmations, missed visible candidates, and speed-fit uncertainty.
  Separately report no evidence versus evidence discarded by retention.
- [ ] Test clutter stronger than the ball, crossing/merged peaks, missing club,
  slow shots, candidate overflow, dropped frames, and stationary/practice cases.

Exit: held-out results show a useful improvement over baseline with every new
false confirmation explained. Choose and document numerical operating gates
from development cases before scoring held-out sessions. Do not tune on them.

## Step 3 — Design retention and trigger timing from the measured tracks

- [ ] Replay proposed retained windows and identify which visible candidate
  points each policy would preserve. Report requests outside original coverage
  as untestable; replay cannot prove what those bins would contain.
- [ ] Preserve a broad tee/early-flight region until identity is sufficiently
  stable, then retain a predicted outgoing corridor with uncertainty margins.
  Evaluate keeping both candidates briefly when ambiguous. Calculate the
  exact IQ16 byte budget, metadata, and processing cost before selecting widths.
- [ ] Avoid terminating solely because the strongest/selected closer return
  was lost while another physically plausible outgoing candidate exists.
  Bound waiting and capture length; do not let ambiguity grow memory use.
- [ ] Back-estimate impact from confirmed tracks constrained by calibrated tee
  geometry. Report uncertainty and no estimate when evidence is insufficient;
  do not equate a fitted range crossing with physical collision.
- [ ] Measure the earliest reliable causal confirmation and its distribution.
  Three observations at 2 ms spacing span 4 ms, but this excludes time to first
  visibility, processing, transport, and trigger output.
- [ ] Budget IWR prehistory and posthistory against that delay. For OPS verify
  that confirmation plus output/transport delay fits its configured prehistory
  while preserving required post-impact data. If it does not, use a provisional
  near-impact event for OPS and confirm/classify later. Camera history and
  capture completion must be checked against the same timeline.

Exit: documented arm/event/confirmation/recording-stop/dump timeline and a
memory-feasible retention policy. No universal club-speed-based delay is assumed.

## Step 4 — Validate firmware in shadow mode, then enable changes separately

- [ ] Port the proven candidate calculation without simultaneously changing
  capture policy. Run in shadow mode only if measured memory and timing allow;
  otherwise compare separate builds against the same recorded inputs.
- [ ] Compare host and board decisions using identical frame IDs, timestamps,
  calibration, and candidate evidence. Log bounded summaries without blocking
  capture tasks. Preserve numerical differences for review.
- [ ] Measure worst-case pre-trigger and post-trigger work, queue depth,
  end-to-end result latency, and linker/stack headroom at 500 frames/s. Capture
  and processing overlap; do not treat nominal inter-chirp idle time as the
  entire CPU budget or average processing time as sufficient proof.
- [ ] User tests: static armed soak, arm-motion/rearm tests, then real strikes
  with representative clubs/speeds. Home motion tests validate mechanics and
  failure handling, not ball identity, launch accuracy, or impact timing.
- [ ] Enable retention changes first, then trigger-timing changes in a separate
  build once the retained evidence supports them. Verify repeated captures,
  OPS/camera timing, and recovery after lost tracks or failed dumps.

Exit: no dropped/corrupted frames, preserved outgoing evidence, and acceptable
held-out/hardware detection behavior. Publish measured limitations and keep a
known-good image for rollback. The user performs flashing and hardware runs.

## Step 5 — Decide on DSP and beamforming from measured limitations

DSP is worthwhile if useful processing exceeds the R4F timing budget, required
working state compromises its memory margin, or demonstrated beamforming gains
cannot fit alongside capture. Do not migrate solely because a target is obscured
or an association rule selected the wrong return.

- [ ] If simpler tracking works with adequate margins, ship it first and defer
  DSP migration. If directional clutter remains the issue, compare a small set
  of receive beams offline using per-antenna IQ and motion compensation before
  considering heavier methods. Simultaneous transmit operation is not required.
- [ ] If DSP migration is justified, budget actual L2 code, cache, stacks,
  queues, and scratch. Move computation and its working state together;
  shared L3 remains the capture arena, not additional memory created by the DSP.
- [ ] Start with a measured expensive stage. Verify frozen-frame parity, then
  live shadow processing. Use bounded frame descriptors, ownership/acknowledgment,
  translated addresses, cache maintenance, sequence/configuration IDs, and
  explicit handling of late results, restart, and buffer reuse.
- [ ] Initially leave HWA/EDMA capture-buffer placement unchanged. Any later
  relocation requires a separate memory-access and capture-timing validation.
- [ ] Let DSP results control tracking only after sustained 500-frame/s service
  and end-to-end deadlines pass. Keep R4F capture control and trigger output.
  Add algorithm improvements separately from moving an existing calculation.

Exit: a measured go/no-go decision. DSP implementation is conditional, not a
required deliverable to claim the tracking experiment successful.

## First deliverable

Build the reproducible offline manifest and sequential replay comparison from
steps 1–2 while the user validates the scheduling fix. Produce per-capture
decisions, retention overlays, confirmation delays, and failure examples. Do
not change production triggers, flash firmware, or expand the DSP implementation
as part of that first change.
