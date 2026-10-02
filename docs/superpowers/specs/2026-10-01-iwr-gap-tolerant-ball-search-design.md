# IWR6843 gap-tolerant, kinematics-driven ball search — spec

Base: `feat/iwr-calcs` at `62ecedb4`. Direction: a proposal the user pasted on 2026-10-01
("make the tracker gap-tolerant and kinematics-driven"); design agreed in conversation the same
day. Builds on `2026-09-28-iwr-joint-club-ball-tracking.md` (the hypothesis search, R1–R8).

## Problem

At a 3 ms frame period a 116 mph (51.9 m/s) ball moves 15.6 cm, ~3.3 bins (4.7 cm), per frame.
Around impact the club, the golfer and the tee band produce strong returns and the ball is often
missing for several frames. The hypothesis search (`l3_ball_hyp.c`) exists but is off
(`useHypotheses = 0`): on 93 captures it cut "wrong" from 70 to 34 but raised "no launch" from 8 to
47. The two traced failures are what this spec targets:

- `20260927_144220_262_005`: two near-stationary returns hold two of four slots for eight frames
  and evict the departing ball. Cause: the spawn band
  `[origin − 1 bin, origin + 10 bins + v_max·Δt]` (`l3_ball_hyp.c:284`) has no lower edge, and a
  hypothesis whose rate fell to ~0 is never dropped.
- `20260927_183542_142_009`: the winning hypothesis mixes two objects (19.9 m/s; OPS 36.5 m/s).
  Nothing rejects a track whose speed changes by more than drag allows.

Three further gaps against the proposal:

- Coasting is `maxMisses = 2` frames everywhere; the ball's impact-region gap is longer.
- With a valid tee band the tracker is armed at the band's **far edge** (`l3_ballArmBin`,
  `l3_dump.c:3516`): one number serves as both where the ball was at impact and where points may
  start. Back-projection needs the former.
- No backward pass: points the forward search missed are never recovered once the ball is known.

The evaluator's own label shares the forward assumption: `ball_present`
(`scripts/analysis/evaluate_iwr_tracking.py:104`) needs targets in **consecutive frames**, so a
ball with an impact-region gap is labelled absent.

## Evidence the design must respect

From the 2026-09-28 spec, still binding:

- Per-point Doppler-continuity and range-rate/Doppler gates made the ball result worse at every
  setting tried. **Doppler is supporting evidence, never a per-point gate.**
- Strength as a hard filter made it worse. Strength is supporting evidence only.
- Frame spacing differs by profile (2, 3, 4, 6 ms); the dump format allows uneven spacing.

New in this spec:

- A drag-only ball loses ≤ ~1 m/s over 50 ms and departs from a straight line by ~2–3 cm (under
  half a bin), so a **linear** range-vs-time fit is kept; a free acceleration term would mostly
  give a two-object mix room to fit.
- Only 23/93 captures hold a ball-like chain under the strict label; weights are not tuned on so
  few.

## Requirements

### Forward search (`l3_ball_hyp.c`)

- **G1 Corridor gate (switchable).** With `corridorGate` on, a target may start or extend a
  hypothesis only if some impact time within the anchor's tolerance and some speed in
  `[minDepartureMps, maxSpeedMps]` explain it:
  `r − r_anchor ∈ [minDepartureMps·(Δt − tol) − anchorRangeTolM, maxSpeedMps·(Δt + tol) + anchorRangeTolM]`,
  `Δt = t − anchorUs`, `tol = anchorTolUs`. The lower edge is negative (no constraint) until
  `Δt > tol`, then rises with time, so a return that stays put is refused once enough time has
  passed. Off: today's spawn band, far edge `spawnBeyondM` included, so the gate's effect is
  measured alone (`spawnBeyondM` is used only with the gate off).
- **G2 Stalled hypotheses drop.** A hypothesis with ≥ 3 points whose fitted rate is under
  `minDepartureMps` is dropped and its slot freed (counted in `dropped`).
- **G3 Impact-region coast.** A hypothesis whose newest point is short of
  `anchor + impactRegionM` (default 0.5 m) may coast `impactCoastUs` (default 18 000 µs); beyond it, `coastUs`
  (default 6 000 µs, today's two frames at 3 ms). Both replace the frame-counted `maxMisses`. The
  association gate keeps widening by `gateMps · Δt`.
- **G4 Deceleration reject.** At classification (≥ `classifyPoints` points), fit the older and
  newer halves (≥ 2 points each; the odd point goes to the newer half); if the rate drops by more
  than `maxDecelMps2 · Δt_mid + 2·σ_Δ` (default `maxDecelMps2` 200 m/s²) the hypothesis does not
  qualify. `Δt_mid` is the time between the halves' mean times; `σ_Δ` combines the halves' slope
  uncertainties `rangeNoiseM / sqrt(Σ(t − t̄)²)` with a fixed `rangeNoiseM` (default 0.012 m,
  about a quarter bin). With 4 points over 6 ms the test cannot fire (σ_Δ ≈ 12 m/s); with 6
  points at 3 ms a 35 → 20 m/s mix is rejected. Its reach therefore depends on `classifyPoints`,
  which the ablation varies. 0 disables.
- **G5 Metric configuration.** `gateBins → gateM`, `spawnBehindBins → spawnBehindM`,
  `spawnBeyondBins → spawnBeyondM`, `farWindowBins → farWindowM`, `maxMisses → coastUs`; converted to bins once in
  `l3_ball_hyps_init` from `binWidthM`. Points stay in bins internally (targets arrive in bins).
  No setting assumes a frame period.

### Anchor and score

- **A1 Anchor apart from acceptance.** A new `l3_ball_anchor_t` (`l3_ball_anchor.c`) carries
  `anchorBin` (the tee bin), `acceptFromBin` (the band's far edge when valid, else the tee bin:
  today's arm bin), `gateUs` (today's arm time), `anchorUs`, `anchorTolUs`, `anchorSigmaUs` and
  `source`. `l3_ball_hyps_arm` and `l3_ball_track_arm` take it. The hypotheses back-project to
  `anchorBin` at `anchorUs`; their spawn band and far window are measured from `acceptFromBin`
  as they were from the arm bin. The **legacy acquisition is unchanged**: it keeps
  `acceptFromBin` as its origin and `gateUs` as its impact time, so its baseline holds.
- **A2 Club-predicted impact time.** At arm, `l3_impact_fit_track(L3_FIT_CLUB_IN)` over the club
  track's newest points against the tee range. `why == OK` and `sigmaUs ≤ anchorMaxSigmaUs`
  (default 3 000): `anchorUs` is its time, `anchorTolUs = max(3σ, 2 000)`. Otherwise the gate /
  range-crossing time as today with `anchorTolUs = impactToleranceUs` (15 000). The verdict
  records `anchorSource` (club fit / gate) and `anchorSigmaUs`.
- **A3 Track score.** Hard gates first (speed bounds, origin within `anchorTolUs`, residual,
  G4). Then
  `S = wBack·S_back + wVel·S_vel + wResid·S_resid + wDoppler·S_doppler + wCoherence·S_coherence + wWeaker·S_weaker`,
  defaults 3, 2, 1, 1, 0.5, 0.5, each term in 0..1:
  `S_back = 1 − |originOffsetUs| / anchorTolUs`;
  `S_vel = 1 − min(1, std(v_i) / v_fit)` over the points' implied velocities;
  `S_resid = 1 − residual / maxResidualBins`;
  `S_doppler`, `S_weaker` as today; `S_coherence` = mean point `coherence` (new point field).
  Raw power is not a term. Weights are configuration, not tuned on the captures.
- **A4 Fastest credible** (`fastBallMps`) unchanged and off.

### Backward recovery (`l3_ball_recover.c`, new)

- **B1 History.** `l3_ball_history_t`: `L3_BALL_HISTORY_FRAMES = 24` frames × up to
  `L3_BALL_HISTORY_TARGETS = 6` targets (`rangeBin`, `dopplerAliasMps`, `stat`, `coherence`, a
  club-claimed flag), plus each frame's number and timestamp; a ring, oldest overwritten. Filled
  every post-impact frame from arming with the same targets the hypotheses see (after the band and
  clutter filters). `historySnr` selects the extraction threshold; default equal to the ball
  track's `snr` (no extra extraction). Build switch `L3_BALL_RECOVER` (host 1, board 0).
- **B2 One pass at adoption.** When `l3_ball_track_adopt` takes the winning hypothesis: fit its
  line; for every history frame from arming to its newest point without a hypothesis point, take
  the candidates within `recoverGateM` (default 0.6 bin, ≈ 2.8 cm) of the line, beyond
  `acceptFromBin`, not club-claimed. Several: the nearest; equal distance (within 0.1 bin): the one
  whose Doppler agrees with the rate (**tie-break only**, per the evidence above). Merge in time
  order, refit; keep the merge only if the residual stays within `maxResidualBins`, otherwise
  adopt the hypothesis points alone. Seed the core in time order as today. Recovered points carry
  no angles (`anglesValid = 0`).
- **B3 Visibility.** The verdict gains `recovered` and `recoveredMask` (bit k: frame
  `recoveredFirstFrame + k`); `l3_ball_track_format_status` prints the count; `ReplayResult`
  gains `recovered_frames`. Drawing them in the viewer is a follow-up.
- **B4 Lives in the ball track.** The history is a member of `l3_ball_track_t` and is fed by
  `l3_ball_track_update_joint` itself, so the board and the replay call nothing new (R7 holds by
  construction). With `historySnr < snr` the caller extracts at the lower threshold
  (`l3_ball_track_extract_snr`) and the track passes only targets with `snr >= snr` to the
  searches. With recovery and the hypotheses on and `historySnr < snr`, the lowered extraction
  list is not the history's alone: it also reaches the club follow and the replay's recorded
  frame targets (and so the evaluator's `ball_present` label); only the ball track itself
  filters to `snr`. `--history-snr` runs are diagnostic only and are not judged by E2.

### Evaluation (`scripts/analysis/evaluate_iwr_tracking.py`)

- **E1 Label first.** `ball_present` becomes gap-tolerant (frames up to `impactCoastUs` apart
  join a chain) and its chain must back-project to the tee within the anchor tolerance. The old
  definition stays as `ball_present_strict`. Hand labels (`2026-09-29-track-labels-design.md`)
  override the heuristic where a capture has them. Measured and committed, with a re-run legacy
  baseline, **before** any tracker change.
- **E2 Split acceptance (replaces R8 for `useHypotheses`).** On the same captures as the legacy
  baseline (the evaluation corpus is `OF Sessions`, 125 captures; see Results) and the repo
  recordings, against legacy re-run under E1:
  ball-present `ok` (within 15 % of OPS) higher than legacy's;
  ball-absent `none` higher and `wrong` lower than legacy's;
  club at impact not below legacy's on the same captures;
  every repo recording's manifest expectation holds.
  `summarize` / `--compare` report the split.
- **E3 Net diagnostic.** Optional `--net-range-m`: for each `ok` capture, whether the confirmed
  line reaches that range within ±2 frame periods of the time its speed predicts. A report column
  only; no firmware code knows the net.
- **E4 Ablation.** `BallTuning` and the CLI expose `--corridor-gate on|off`,
  `--impact-coast-ms`, `--max-decel`, `--classify-points`, `--recover on|off`, `--recover-gate-m`,
  `--history-snr`, and `--far-window-m` (replacing `--far-window-bins`);
  the Results section reports each change's effect alone and together.

### Parity and rollout

- **P1 Same code on board and replay (R7).** New config fields and structs get ctypes mirrors in
  `firmware_host.py` with size tests; `firmware_replay.py` makes the history and recovery calls
  in the same order as `l3_dump.c`.
- **P2 Defaults.** `useHypotheses` stays 0 until E2 passes. The board keeps
  `L3_BALL_HYPOTHESES = 0` and `L3_BALL_RECOVER = 0`; the DATA_RAM added by each is measured and
  recorded here. Fitting them on the board is a separate decision.

## Tests (written first, through `firmware_host`, in the existing ball-track test style)

Forward: a stationary pair cannot start a hypothesis once Δt ≥ one frame (144220 regression); a
hypothesis decelerating to 0 drops at 3 points; a ball absent 5 frames inside the far window
survives, the same gap beyond it drops; the same metric scene gives the same verdict at 2 ms and
3 ms periods; a 35 → 20 m/s two-object mix fails G4; implied velocities just inside and just
outside each corridor edge; the timestamp wrap across 2³² µs; `corridorGate` off reproduces
today's spawn behaviour.

Anchor and score: a clean club approach anchors from the club fit; a noisy or two-point club
falls back to the gate; with a band the anchor is the tee and points short of the far edge are
refused; an origin crossing 12 ms off loses to one within 1 ms despite a better residual; each
score term at its 0 and 1 edges.

Recovery: the ring wraps; club-flagged targets are never recovered; frames 2, 3 and 5 missing
from a synthetic ball but present in the history are recovered in order; a nearer decoy wins over
a farther agreeing one, an equal-distance decoy loses on Doppler; a merge that breaks the residual
is undone whole; nothing short of `acceptFromBin` is recovered; ctypes size checks.

Evaluation: gap-tolerant `ball_present` on synthetic frames with a 4-frame gap (true) and a
stationary pair (false); the split summary and comparison.

## Out of scope

Lower-threshold re-detection from the L3 IQ16; the batch anchored line search at RESULT (the
alternative to forward association, worth adding on the same history if this falls short);
fitting the board's DATA_RAM; angles for recovered points; any club-tracker change.

## Results

### Legacy baseline under the gap-tolerant label (2026-10-01)

Corpus: `OF Sessions` (125 captures). The 2026-09-28 93-capture set was unavailable, so the
numbers are not comparable with that baseline, and the E2 club gate is relative to legacy on this
corpus. Run: `--ball-hypotheses off`; JSON in `2026-10-01-ball-search-baseline-legacy.json`.

| | Legacy |
|---|---|
| captures | 125 |
| club at impact / stuck / few | 72 / 12 / 41 |
| ball ok / wrong / none | 16 / 60 / 49 |
| `ball_present` strict | 76 |
| `ball_present` gap-tolerant | 85 |
| ball present: ok / wrong / none | 16 / 59 / 10 |
| ball absent: ok / wrong / none | 0 / 1 / 39 |

Nine captures change from absent (strict) to present (gap-tolerant); verdicts are unchanged:

- `iwr6843_20260919_190247_674_019` (none)
- `iwr6843_20260919_190419_290_021` (none)
- `iwr6843_20260919_190523_340_023` (none)
- `iwr6843_20260919_190702_657_027` (none)
- `iwr6843_20260919_190834_768_029` (none)
- `iwr6843_20260919_191005_759_032` (none)
- `iwr6843_20260919_191119_600_034` (none)
- `iwr6843_20260923_183847_564_010` (none)
- `iwr6843_20260923_184805_180_019` (wrong)

### Hypothesis search and ablation (2026-10-01)

Corpus: `OF Sessions` (125 captures), as for the legacy baseline. Legacy re-run on this branch's
HEAD (`--ball-hypotheses off`) reproduces the committed baseline capture for capture (club, ball
verdict and launch speed), so it is the comparison as committed. Every hypothesis run is
`--ball-hypotheses on --accept 2026-10-01-ball-search-baseline-legacy.json`; JSON in
`2026-10-01-ball-search-hypotheses-<variant>.json`. The radar-to-net distance is unknown, so no
run uses `--net-range-m` and E3 has no column. The repo recordings' manifest check was not run
because E2 failed on the corpus.

`--accept` splits the run's verdicts by the **legacy baseline's** per-capture `ball_present`, so
both sides are judged on the same populations; a capture whose run label differs is printed as a
`NOTE` and is not a failure (no all-on or recover-off capture's label differs).

The all-on and `--recover off` rows were re-run after recovery's club exclusion was wired (the
club follow's claimed target now reaches the history, `l3_ball_track_note_club`); before it, the
history's club flags were always clear. The other ablation rows predate that fix and are compared
with the pre-fix all-on (32 / 29 / 24); with recovery on in each of them, their numbers may move
when re-run.

| Run | present ok / wrong / none | absent ok / wrong / none | club | `--accept` |
|---|---|---|---|---|
| legacy | 16 / 59 / 10 | 0 / 1 / 39 | 72 | — |
| all-on (defaults) | 33 / 28 / 24 | 0 / 1 / 39 | 72 | absent none 39 not above 39; absent wrong 1 not below 1 |
| `--corridor-gate off` | 31 / 30 / 24 | 0 / 1 / 39 | 72 | same two |
| `--impact-coast-ms 6` | 32 / 28 / 25 | 0 / 1 / 39 | 72 | same two |
| `--max-decel 0` | 29 / 35 / 21 | 0 / 1 / 39 | 72 | same two |
| `--recover off` | 33 / 28 / 24 | 0 / 1 / 39 | 72 | same two |
| `--classify-points 6` | 37 / 11 / 37 | 0 / 1 / 39 | 72 | same two |
| `--history-snr 0.7` | 32 / 30 / 24 | 0 / 0 / 39 | 72 | diagnostic only (B4); not judged by E2 |

Club at impact is 72 in every run, with no capture's club verdict changed.

`20260927_144220_262_005` and `20260927_183542_142_009` are not in this corpus, so they have no
capture-by-capture entry. Instead, every capture whose ball verdict changed between legacy and
all-on (40; all are `ball_present` under both runs):

| Capture (`iwr6843_` omitted) | OPS m/s | legacy launch | all-on launch | verdict |
|---|---|---|---|---|
| 20260916_184748_410_001 | 35.9 | 52.2 | 35.2 | wrong → ok |
| 20260916_184825_011_002 | 40.0 | – | 41.6 | none → ok |
| 20260916_184937_318_004 | 38.7 | 9.2 | 38.7 | wrong → ok |
| 20260916_185013_572_005 | 39.7 | 8.6 | 41.5 | wrong → ok |
| 20260916_190937_131_002 | 32.7 | 46.2 | – | wrong → none |
| 20260919_185429_574_003 | 25.3 | 14.9 | – | wrong → none |
| 20260919_185532_646_005 | 30.5 | 10.6 | 34.3 | wrong → ok |
| 20260919_185657_244_008 | 28.7 | 20.0 | 31.5 | wrong → ok |
| 20260919_190333_892_020 | 48.3 | 1.9 | – | wrong → none |
| 20260919_190633_265_026 | 42.6 | 23.5 | 41.8 | wrong → ok |
| 20260919_190749_481_028 | 49.8 | 21.3 | 47.0 | wrong → ok |
| 20260919_190902_385_030 | 45.5 | 55.3 | 43.9 | wrong → ok |
| 20260919_191037_862_033 | 48.3 | 28.9 | 48.4 | wrong → ok |
| 20260919_191229_528_036 | 34.3 | 16.3 | – | wrong → none |
| 20260923_130505_440_008 | 31.5 | 40.0 | 27.6 | wrong → ok |
| 20260923_130517_126_009 | 41.2 | 48.6 | – | wrong → none |
| 20260923_130532_189_010 | 43.3 | 2.7 | 44.2 | wrong → ok |
| 20260923_130603_754_011 | 40.0 | 16.7 | – | wrong → none |
| 20260923_130725_534_013 | 29.9 | 8.4 | 25.6 | wrong → ok |
| 20260923_130736_303_014 | 33.5 | 63.1 | 33.0 | wrong → ok |
| 20260923_130804_075_015 | 37.4 | 50.5 | 33.2 | wrong → ok |
| 20260923_130813_241_016 | 36.4 | 35.0 | 18.4 | ok → wrong |
| 20260923_130832_920_017 | 35.6 | 14.8 | 34.7 | wrong → ok |
| 20260923_130936_910_020 | 36.1 | 47.6 | 35.8 | wrong → ok |
| 20260923_131012_999_022 | 48.1 | 17.9 | 52.4 | wrong → ok |
| 20260923_131031_161_023 | 39.7 | 16.8 | 41.1 | wrong → ok |
| 20260923_131232_075_029 | 40.2 | 15.1 | – | wrong → none |
| 20260923_131458_319_037 | 50.6 | 20.7 | 46.2 | wrong → ok |
| 20260923_183514_618_004 | 46.6 | 40.3 | 65.0 | ok → wrong |
| 20260923_184026_592_001 | 70.4 | 60.2 | – | ok → none |
| 20260923_184122_450_003 | 70.4 | 63.1 | 46.3 | ok → wrong |
| 20260923_184155_650_004 | 70.0 | 46.0 | – | wrong → none |
| 20260923_184223_357_005 | 65.5 | 15.7 | – | wrong → none |
| 20260923_184247_221_006 | 70.5 | 54.6 | – | wrong → none |
| 20260923_184351_044_009 | 64.0 | 52.5 | – | wrong → none |
| 20260923_184447_518_011 | 68.6 | 69.2 | – | ok → none |
| 20260923_184523_117_013 | 66.0 | 6.8 | – | wrong → none |
| 20260923_184547_322_014 | 68.0 | 14.4 | 64.0 | wrong → ok |
| 20260923_184734_911_017 | 69.8 | 19.4 | – | wrong → none |
| 20260923_184831_766_021 | 63.1 | 4.9 | 63.7 | wrong → ok |

Totals: 21 wrong → ok, 1 none → ok, 13 wrong → none, 3 ok → wrong, 2 ok → none.

**Board bytes** (host `ctypes` sizes, the arithmetic of
`tests/test_iwr6843_firmware_board_image.py`; not a TI link map — a board build with both switches
on was not run): `L3_BALL_HYPOTHESES` adds 1796 B (`BallHyps` 1628 + `BallHypVerdict` 60 +
`BallHypsCfg` 108); `L3_BALL_RECOVER` adds 2632 B in the track (`BallHistory` 2600 + `BallRecoverCfg` 24 + 8)
and 2016 B of function-static `.bss`: `l3_ball_track_update_joint`'s `usable[L3_OBS_MAX_TARGETS]`
(8 × `sizeof(l3_target_obs_t)` 72 = 576 B) and `original[L3_OBS_MAX_TARGETS]` (8 × 4 = 32 B), and
`l3_ball_track_adopt`'s `merged[L3_TRACK_POINTS]` (32 × `sizeof(l3_ball_hyp_point_t)` 44 =
1408 B); 4648 B static. `l3_ball_recover` also puts `found[L3_BALL_HISTORY_FRAMES]` on the stack
(24 × 44 = 1056 B), which the detect task's stack must hold. Together: 6444 B static plus 1056 B of
stack. (`sizeof` from `fw.TargetObs` and `fw.BallHypPoint`; the board's layout of these
all-4-byte-field structs is expected to match, but a TI link map is still needed for the board
figure.)

**What moved what.** The search doubles ball-present `ok` (16 → 33) and halves `wrong` (59 → 28),
at the cost of more `none` (10 → 24); club is untouched. E2 fails on the absent side only, and the
absent side is one capture: `20260923_184647_713_016` (OPS 69.6 m/s, no ball chain) is `wrong` under
legacy (17.2 m/s) and under every hypothesis run (23.7 m/s), and with 40 absent captures `none`
can only rise above 39 if that capture goes to `none`. Recovery, with the club's claim now
excluded, changes no verdict: on and off give 33 / 28 / 24 capture for capture, and recovery moves
13 captures' launch speeds (by 0.1 to 5.3 m/s), none across the 15 % line (before the club
exclusion was wired, recovery cost 1 `ok`: on `20260919_185532_646_005` it moved the launch from
33.4 to 38.4 m/s against OPS 30.5; it now gives 34.3). Per switch against the pre-fix all-on
(32 / 29 / 24; net counts, the number of captures whose verdict changes in brackets; these runs
were not repeated after the fix): the corridor gate is worth 1 `ok` (off: 31 / 30 / 24 [5]); a 6 ms
impact coast turns 1 `wrong` into `none` net (32 / 28 / 25 [5]); the deceleration ceiling is worth
3 `ok` and 6 `wrong` (off: 29 / 35 / 21 [7]); 6 classify points is the largest lever
(37 / 11 / 37 [25]: +5 `ok`, −18 `wrong`, +13 `none`). `--history-snr 0.7` is diagnostic only
(B4) and not judged by E2: it leaves every tracker verdict as the pre-fix all-on [0] but flips
`20260923_184647_713_016`'s label from absent to present, because the lowered extraction list also
reaches the replay's frame targets the label is computed from — its absent `wrong` 0 is a label
artifact, not a tracker gain. Under the new `--accept` that capture would be judged by the
baseline's label (absent) and its label change reported as a `NOTE`.

**Decision.** `--accept` (baseline-labelled split) printed two problems for the all-defaults run
after the club-exclusion fix (`ball-absent none 39 not above 39`; `ball-absent wrong 1 not below
1`), so `useHypotheses` stays 0 and
the board keeps both switches off.
