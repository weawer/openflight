# IWR6843 split trigger — plan

Base: `feat/iwr-calcs` at `bb06293e` (Cormac's branch, now `feat/iwr-2ms-trigger-merge`).

## Goal

Freeze the L3 ring when the radar has **seen the ball leave**, not when the club reaches the tee:

1. Rolling buffer runs continuously (exists).
2. **Arm** when the club track is coming down toward the ball.
3. Look for a **separate outbound track** leaving the ball's origin.
4. **Confirm the split**: club and ball are distinct tracks and the ball keeps going out.
5. **Freeze**, keeping pre- and post-impact frames. How many of each is sized from the measured
   distribution of split-confirmation latency (median and tail), not guessed.

Why: today the freeze fires at the gate (`l3_trig_update`) or the geometric detector
(`l3_impact_update`), and the ball is looked for afterwards in fixed post windows. On the 93
evaluated captures only 23 hold a ball-like chain in the saved data at all
(`docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md`). A freeze that waits for the
ball can, by construction, only produce captures that contain it.

## What already exists (reused, not rewritten)

| Need | Existing code |
|---|---|
| Rolling ring, freeze, rearm | `l3_dump.c` (`gHwaFreezeRequested`, `l3_awaitFrozenRing`) |
| Club coming down | `l3_club_track.c` (live every pre frame), `l3_trigger.c` approach rules, `l3_impact.c` closest-approach time |
| Per-frame targets | `l3_observation.c` `l3_obs_extract` |
| Separate outbound track, joint with the club's claim | `l3_ball_hyp.c` (`l3_ball_hyps_update` / `_classify`: origin crossing, outward rate, straight fit, Doppler, weaker-than-club) |
| Shot sequence | `l3_shot.c` |
| State-aware retention windows | `l3_retain.c` (`l3_retain_window`), `adaptive16` |
| Same code on board and replay | `firmware_host.py` ctypes mirrors, `firmware_replay.py` |
| Scoring against OPS | `scripts/analysis/evaluate_iwr_tracking.py` |

The gap: `l3_ball_hyps` only runs **after** the freeze, on post-epoch frames
(`l3_detectTask` → `l3_considerBallTrack`). The split trigger needs it on **live pre frames**, and
needs its verdict to decide the freeze.

## Hard constraints found in the code

1. **Split detection is not yet reliable.** With hypotheses on: ball within 15 % of OPS on 10–11/93,
   no launch on 47–48/93. Split cannot be the *only* freeze condition without losing shots; it
   needs a fallback (Phase 2).
2. **DATA_RAM is full.** The hypotheses add 1432 B and are compiled out of the board image
   (`L3_BALL_HYPOTHESES=0`, commit `25398c2d`); the spec records 0x2b bytes free before that trim.
   Putting them back on the board is a blocker to settle first (Phase 0b).
3. **Detect-task deadline.** The detect task must finish inside one frame (`scratch_stale` stays 0).
   Running hypotheses plus a processing region that now extends beyond the tee adds per-frame cost
   at 2–3 ms cadence.
4. **CLI table is at the SDK limit.** New config rides an existing command as a sub-mode.
5. **Ring coverage.** Freezing later spends ring history: every frame of confirmation latency is a
   frame of club approach lost off the old end. Size on the tail (P90), not the median.

The host needs little: it waits for `Triggered` and reads the dump, whose per-frame timestamps carry
the timing (`driver.py` `TRIGGER_NOTICE`). OPS correlation is unaffected.

## Phases

### Phase 0 — Measure before building (offline, no firmware change)

**0a. Split-latency analysis.** New `scripts/analysis/split_latency.py` (+ `tests/test_split_latency.py`).
Replay each capture through the compiled modules (`firmware_replay.py`), running `l3_ball_hyps`
armed at *club-arrival* (club track within N bins of the ball, or `l3_impact`'s predicted contact
time) instead of at the gate. Per shot report: gate time, geometric impact time, first frame a
hypothesis classifies as the ball, latency (confirm − impact) in ms and frames, and whether the
ball was ever confirmed. Corpus: median, P90, P95, confirm rate, by club and profile.

Data: `tests/radar/recordings/` (7 captures, local) and the 93-capture `iwr-test-sessions/`
(**not on this machine** — needs to come from Cormac).

Decision gate: the confirm rate says whether split can be a primary trigger or only a better
freeze time with a fallback; P90 latency sizes Phase 3.

**0b. DATA_RAM feasibility.** Build the board image with `L3_BALL_HYPOTHESES=1`
(`make clean && make bin L3_FEATURE_DEFS=...` in the Docker toolchain) and read the map's
`DATA_RAM` unused bytes against the 16,384 B floor in `docs/development/firmware.md`. If it does
not fit, options in order: shrink `L3_BALL_HYP_MAX`/`L3_BALL_HYP_POINTS` for the board, move
hypothesis state into `.dataScratch` or L3, trim `L3_TRIG_LOG_DEPTH`/`TRACE_DEPTH` further.

### Phase 1 — Live split detector (pure C, host-tested first)

New `firmware/iwr6843/l3_split.c/.h`, a thin layer over `l3_ball_hyps` (no copied association
logic):

- `l3_split_arm(...)`: arm on club arrival; origin = locked ball bin, else the tee.
- `l3_split_update(...)`: feeds the frame's targets and the club's claimed index to
  `l3_ball_hyps_update`, then `l3_ball_hyps_classify`.
- Confirmation rule: a hypothesis classified as the ball **and** the club track still distinct
  (club claim ≠ ball target on the confirming frame) **and** at least `confirmPoints` ball points
  outward of the origin.
- States: `IDLE → ARMED → CONFIRMED | TIMED_OUT | ABORTED` (club track dropped before arrival).
- Timeout in µs from arm, by timestamp (matches R5 of the joint-tracking spec).

Tests first, `tests/test_iwr6843_firmware_split.py`, via `firmware_host.py`:
clean split; merged club/ball returns for 1–2 frames; slow chip (10–15 m/s); fast driver
(70+ m/s); practice swing (no ball) → timeout; hand at tee (the `5875e272` case); stationary
clutter holding hypothesis slots (the `20260927_144220` failure); club track inactive after impact
(the `20260927_183542` failure); frame gaps and uneven timestamps; rearm mid-shot.

ctypes mirror and struct-size check added to `firmware_host.py`.

### Phase 2 — Trigger integration in `l3_dump.c`

In `l3_considerSelfTrigger`:

- New trigger mode: `gate` (today), `geometry` (today, `gImpactArmed`), `split`.
- In `split` mode the gate/geometry fire **arms** the split detector instead of freezing.
- Freeze on `CONFIRMED`. On `TIMED_OUT`, behaviour is a config choice (open question 1).
- Widen the per-frame processing region after arm from the trigger region
  (`l3_trig_region`) to origin + `splitBeyondBins`, only while armed, to bound CPU cost.
- `gTrigFireSource` and `l3_shot` gain `L3_SHOT_IMPACT_SPLIT` (4) and `L3_SHOT_IMPACT_TIMEOUT` (8).
- `l3_shot`: in split mode IMPACT is recorded at the arm/geometric time while the ball is tracked
  live; BALL_TRACK can begin before the freeze. The confirmed hypothesis seeds `l3_ball_track`
  so post frames continue the same track rather than restarting the search.
- `triggerLog` gains a `split` sub-mode printing arm time, hypotheses, verdict and latency.
- `triggerLog perf` gets a `split` stage in `l3_profile.c`.

Config as a sub-mode (CLI limit): `captureCfg split <enable> <confirmPoints> <timeoutMs>
<beyondBins> <onTimeout>`.

### Phase 3 — Capture plan sized from Phase 0

In split mode impact and early flight land in the **pre** ring, and post becomes a short tail.

- Pre windows must reach from club approach to origin + ball-confirmation distance; drive them with
  `l3_retain_window` from live tracker state (it already moves from tee → club → ball by shot state;
  in split mode the state changes before the freeze).
- Post frames: enough to extend the ball track for the launch fit (`launchPoints`), not the whole
  flight search.
- Pre/post counts: history needed = club approach duration + P90 confirmation latency + margin.
  `l3_retain_budget` already cuts oldest club history first and never impact frames.
- New profile `.cfg` under `config/`; the default profile is unchanged.

### Phase 4 — Host and replay plumbing

- `firmware_replay.py`: same calls, same order as the board (R7).
- `runtime.py` / server: `--iwr6843-trigger-mode {gate,geometry,split}`, emits `captureCfg split`.
- `firmware_checks.py` + `scripts/hardware-test/test_iwr_firmware.py`: CLI suite entry for the new
  sub-mode, and a swing check that reports split verdict and latency.
- `evaluate_iwr_tracking.py`: `--trigger-mode split` so the 93 captures are scored both ways.
- Docs: `docs/development/firmware.md` (Onboard Self-Trigger section), operator profile table.

### Phase 5 — Rig validation (the long pole)

A/B in the same session, alternating modes, with OPS:

| Metric | Why |
|---|---|
| Missed shots (no freeze on a real shot) | the cost split must not add |
| Ball present in saved data | the benefit; today 23/93 |
| Ball within 15 % of OPS / no launch | the joint-tracking spec's R8 numbers |
| Timeout-fallback rate | how often split is actually deciding |
| False freezes (practice swings, walk-ups, hand at tee) | split should lower these |
| `scratch_stale`, `hwa_missed`, `triggerLog perf` split stage | the CPU budget held |
| Confirm latency on the rig vs Phase 0 | re-size Phase 3 if the rig differs |

Split becomes the default only when it improves "ball present" and does not raise missed shots,
the same judged-on-data rule `useHypotheses` follows.

## Effort

| Phase | Estimate | Gated on |
|---|---|---|
| 0a latency analysis | 1–2 days | the 93-capture sessions from Cormac |
| 0b DATA_RAM check | 0.5 day (+1–2 if it doesn't fit) | Docker toolchain built |
| 1 `l3_split` + tests | 3–4 days | — |
| 2 trigger integration | 2–3 days | 0b |
| 3 capture plan | 1–2 days | 0a numbers |
| 4 host/replay/docs | 1–2 days | — |
| 5 rig validation | open-ended | range time |

About 2 weeks of code, then rig time.

## Open questions

1. **On timeout** (armed, no split confirmed): freeze anyway (never lose a shot, practice swings
   make captures) or disarm (rejects practice swings, loses shots the detector misses)?
   Recommendation: freeze anyway, tagged `source=timeout`, until the rig numbers show the confirm
   rate is high.
2. **Arm point**: gate fire, geometric predicted contact, or club within N bins of the ball?
   Recommendation: whichever Phase 0a shows gives the earliest arm without arming on hands at the
   tee; default to the gate since it already carries the hand-at-tee fixes.
3. **Hypothesis reliability**: fix the two traced failures (stationary clutter holding slots; club
   track inactive after impact) inside this work, or treat them as prerequisites?
   Recommendation: inside Phase 1, as tests first; split depends on them directly.
4. **Coordination with Cormac**: this touches the same files as his active branch
   (`l3_dump.c`, `l3_ball_track.c`, `l3_shot.c`). Agree ownership before Phase 2.
