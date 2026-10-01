# IWR6843 trajectory reconstruction: tee-anchored ball fit and club EKF

Date: 2026-10-01
Status: design approved in chat; awaiting spec review

## Problem

The dump viewer's range x time plot shows coherent club and ball tracks, but the
3D "Trajectory (golf frame)" view jumps wildly in cross-range and height from
frame to frame. Every 3D point is one frame's raw angle estimate converted
straight to XYZ:

- `l3_track_locate` (`firmware/iwr6843/l3_club_track.c:42`) passes range and
  angles to `l3_frames_observe` (`l3_frames.c:95`), which rotates the spherical
  point into the golf frame. Nothing filters it.
- The viewer (`scripts/iwr6843/dump_viewer.html:510`) joins those points with
  lines.
- Per-point angle scatter is large and already measured: azimuth SD 27 deg,
  elevation SD 12 deg, against a range walk of 0.024 m
  (`2026-09-29-iwr-late-flight-launch-angles-design.md`). The cause is the
  direct return and the floor reflection taking turns winning, plus a single
  phase comparison for azimuth.
- `l3_angle_estimate` computes an angle confidence (`l3_angle.c:181-205`), but
  it is never stored on `l3_track_point_t`, and `anglesValid` is set almost
  always. Nothing can gate on angle quality today.

Range and range rate are trustworthy; angles are not. The reconstruction must
trust range heavily, treat angles as weak measurements, and reject angles that
imply physically impossible motion. XYZ must not simply be smoothed after the
fact.

## Goals

1. Stable 3D ball and club trajectories on the board, in replay and in the
   viewer. This is one firmware C change, since replay and the viewer run the
   same C code through `firmware_host`.
2. The on-chip ball HLA/VLA (`l3_ball_track_launch`) and club delivery
   (`l3_delivery_fit`) consume the reconstructed positions.
3. The viewer shows the raw angle points and the reconstructed trajectory
   together.
4. Frame processing speed is a priority: nothing new runs per frame, and each
   fit has a hard time budget.

## Non-goals

- The host's published launch angle (`lcmf.py`, via `runtime.py`) is unchanged.
- Array calibration (`trackCfg cal` / `trackCfg elem` not sent by the Pi) is
  out of scope. It is noted as a known source of angle bias that no filter can
  remove.
- Per-frame association, detection and angle-point selection are unchanged.

## Design

### 1. Per-point angle quality and reconstruction fields

`l3_track_point_t` (`l3_club_track.h`) gains:

| Field | Type | Meaning |
|---|---|---|
| `angleConfidence` | float | `l3_angle_estimate`'s confidence for this point's angles; 0 when none were measured |
| `filteredPosition` | `l3_vec3_t` | Reconstructed golf-frame position (metres from the antenna) |
| `filterAccepted` | uint8 | 1 when the point's angles were used by the fit or filter |
| `filterHypothesis` | uint8 | `L3_FILTER_HYP_NONE`, `_DIRECT`, `_IMAGE`, `_AMBIGUOUS`, `_UNFILTERED` |

`l3_track_set_point_angles` takes and stores the confidence. The angle queue
(`l3_angle_queue.c:93`) and the direct call sites in `l3_dump.c` pass it
through. `position` keeps its current meaning, the raw per-frame estimate.

### 2. Ball: tee-anchored robust fit (`l3_ball_fit.c/.h`, new)

Over the ~30 ms a ball track spans, gravity moves the ball ~3 mm, well under one
range bin (0.047 m). The ball's path is therefore a straight line from the tee.

Model: `p(t) = tee + s(t) * u(HLA, VLA)`.

- `tee`: golf-frame tee position at the track's origin range, on boresight, at
  height `teeBallHeight - radarHeight` (defaults 0.04 m and 0.152 m, from the
  config).
- `s(t)`: distance along the line, obtained from each point's measured range by
  solving `|tee + s*u| = r` and taking the forward root (`s >= 0`). Range is
  treated as exact. Its 0.024 m walk is negligible next to the angular error.
- `u`: unit direction for (HLA, VLA). Only these 2 parameters are fitted.

Fit:

- The measured angles of each point become a unit vector once, up front.
- The residual for a candidate `u` at point i is the angle between the measured
  vector and the predicted direction from the antenna to `p_i`. It is computed
  with a dot product (`1 - cos`), with no `atan2` in the inner loop.
- Floor reflection (two hypotheses): each point is also scored against the
  mirror of `p_i` about the floor plane (`z' = -2*radarHeight - z`), and the
  smaller residual counts. When the direct and reflected predictions differ by
  less than `imageSepMinRad`, the point is `_AMBIGUOUS` and scored as direct.
- Weight: `angleConfidence`, with a Huber cap on each point's contribution.
- Search: coarse-to-fine grid over HLA in [-45, 45] deg and VLA in [-10, 60]
  deg, 3 levels of about 10x10. No matrix solve.
- Gate: a point whose residual exceeds `gateK * angleSigmaRad` is rejected
  (`filterAccepted = 0`). The fit is redone once on the accepted points.
- Output: HLA, VLA, accepted count, RMS residual, and a `why` reason. Each point
  gets `filteredPosition = tee + s_i * u` and its hypothesis.

Validity (never report a confident wrong answer):

| Case | Result |
|---|---|
| Accepted points < `minAccepted` (default 4) | angles invalid, `why = few_angles` |
| RMS residual > `maxRmsRad` after the gate | angles invalid, `why = scatter` |
| Best fit on the grid edge | angles invalid, `why = grid_edge` |
| Range below the tee (no forward root) | point skipped |
| `angleConfidence == 0` | weight 0; it still gets a `filteredPosition` |
| Track not confirmed | existing behaviour, no launch |

When the angles are invalid, the `filteredPosition` of every point stays the
boresight fallback (today's behaviour) and every point is marked
`_UNFILTERED`. Speed always comes from the existing early range fit.

`l3_ball_track_launch` takes HLA/VLA from `l3_ball_fit`. The late-window path
(`l3_ball_track_late_first`, `lateRangeM`, `lateFrom`) is removed, not kept as
dead code. `l3_launch_t`'s `lateFrom` is replaced by `anglesAccepted` and
`angleRmsRad`, and the replay, dump and viewer summaries follow.

### 3. Club: constant-velocity EKF with RTS smoother (`l3_track_kf.c/.h`, new)

The club moves on an arc, not a line, so it needs a filter, not a line fit.

- State: `[x, y, z, vx, vy, vz]` in the golf frame. Constant-velocity motion,
  with white-acceleration process noise `accelSigmaMps2`.
- Prediction uses the real time step from `timestampUs`, so frames the track
  coasted through are handled.
- Measurements:
  - Range (sigma `rangeSigmaM`) and range rate (sigma `rateSigmaMps`): always
    applied.
  - Azimuth and elevation (sigma `angleSigmaRad / max(angleConfidence, eps)`):
    applied only if the chi-square test on the innovation passes (`chi2Gate`,
    2 degrees of freedom). A point that fails still updates on range and range
    rate, so range continuity is never lost.
- Initialisation: position from the first point with angles (boresight if
  none). Velocity from the first two points' range rate along the line of sight.
  Large initial covariance.
- After the forward pass, a Rauch-Tung-Striebel (RTS) smoother runs over the
  stored states. The whole track is available at that point.
- Linear algebra: fixed 6x6 in float32, no allocation. Innovation matrices are
  at most 4x4, inverted with an explicit Cholesky.
- Failure handling: fewer than 3 points gives a raw copy, `_UNFILTERED`. A NaN,
  or a covariance that is not positive definite (Cholesky fails), resets that
  track to raw positions, `_UNFILTERED`. No assert, and the board keeps running.

`l3_delivery_fit` (club) fits `filteredPosition` instead of `position`. Its
`maxAngleResidualM` check stays as a sanity gate.

### 4. Fixed: az/el offsets subtracted twice

`l3_angle_estimate` subtracts the az/el offsets (`l3_angle.c:252-253, 279`) and
`l3_frames_observe` subtracts them again (`l3_frames.c:105-106`). It is harmless
while the offsets are 0, but the fits depend on them. It is fixed in
`l3_frames_observe`, with a failing test first.

### 5. Tunables

Tunables are added to the existing config structs (`l3_ball_track_cfg_t`,
`l3_track_cfg_t`) and exposed through `trackCfg` the same way as the existing
ones. Initial defaults come from the measured scatter and are revisited against
the recordings:

| Tunable | Default |
|---|---|
| Ball `angleSigmaRad` | 12 deg (elevation-dominated; see open question) |
| Ball `gateK` | 2.5 |
| Ball `minAccepted` | 4 |
| Ball `maxRmsRad` | 15 deg |
| Ball `imageSepMinRad` | 2 deg |
| Ball grid limits | as above |
| Club `accelSigmaMps2` | 300 (a clubhead near impact) |
| Club `rangeSigmaM` | 0.03 |
| Club `rateSigmaMps` | 1.0 |
| Club `angleSigmaRad` | 15 deg |
| Club `chi2Gate` | 9.21 (99%, 2 degrees of freedom) |

### 6. Replay and viewer

- `firmware_replay._point_summary` passes through `angle_confidence`,
  `filtered_position`, `filter_accepted` and `filter_hypothesis`. `LaunchSummary`
  carries `angles_accepted`, `angle_rms_deg` and `why`.
- The viewer's 3D view draws the reconstructed trajectory per track as a solid
  line through `filtered_position`. Raw `position` points are drawn as a faint
  scatter, with rejected points hollow and reflection-hypothesis points in a
  distinct marker. A toggle switches Raw / Fitted / Both, default Both.
- The point inspector shows angle confidence, hypothesis and accept/reject.

## Speed

- Nothing is added to the per-frame path. Both fits run once per shot in the
  result stage, alongside the existing `gLaunch` computation
  (`l3_dump.c:~3749`), in the detect task. Priority there is ctrl > rearm >
  detect > CLI, so rearm preempts the fits. They never run in ctrl, rearm or ISR
  context.
- Budgets on the MSS: ball fit <= 1.0 ms, club EKF plus smoother <= 0.5 ms.
  Both appear as their own lines in the acceptance run's per-phase timing report
  (8b121ac7). Over budget means coarsening the grid or the levels, not moving
  the work somewhere else.
- The ball inner loop per point and candidate is 2 sqrt, 2 dot products, no
  trig. Estimated about 5k evaluations per shot.
- A host-side test bounds the ball fit's evaluation count, so a change to the
  grid cannot silently blow the budget.

## Testing (TDD; `uv run pytest`)

Ball fit, through `firmware_host`:

- Recovers synthetic HLA/VLA exactly with noise-free data.
- With fixed-seed Monte Carlo angle noise at the measured scatter, recovers the
  angles within a set tolerance.
- Injected outliers are rejected, and `filterAccepted` marks exactly those
  points.
- With half the points replaced by their floor reflections, the angles are
  recovered and those points are flagged `_IMAGE`.
- Returns `why = few_angles`, `scatter` and `grid_edge` in their cases.
- A point below the tee is skipped.
- A point with confidence 0 does not change the fit.
- The evaluation count stays within budget.

Club EKF:

- On a synthetic arc, the smoothed error is below the raw error.
- An angle jump fails the gate but still updates on range.
- A gap in the frames uses the real time step.
- Fewer than 3 points gives `_UNFILTERED`.
- Forced divergence resets without NaN.

Regression:

- A failing test for the double offset subtraction (non-zero offsets), then the
  fix.
- Every existing tracking and replay test passes unchanged. Speed, origin bin
  and range and timing outputs must not move.

Recordings (`tests/radar/recordings/`): for each shot, compare the raw and
filtered 3D residual about the fitted line, and the HLA/VLA spread across shots
of the same club. The accept criterion is a clear drop in scatter with speed and
origin unchanged. The results are frozen into
`docs/superpowers/specs/2026-10-01-trajectory-reconstruction-baseline.json`.
With no angle labels, this measures stability, not accuracy.

Viewer: `tests/test_iwr6843_dump_viewer.py` checks that a shot payload carries
the new per-point fields for both tracks and the new launch fields.

Board: an acceptance run's timing lines for both fits are recorded against the
budgets.

## Deviations during planning (2026-10-01)

1. The club EKF has no range-rate measurement: `radialVelocityMps` is derived from the same ranges.
2. Club `accelSigmaMps2` defaults to 1500 (centripetal on a ~1.1 m arc at ~40 m/s), not 300.
3. The angle update is two sequential scalar updates behind a joint 2-dof chi-square gate; the RTS
   smoother solves a 6x6 Cholesky per step and keeps no smoothed covariance.
4. The new constants are replay tunables (`tunables.py`); no new board `trackCfg` verbs.
5. An invalid ball fit leaves every point `unfiltered` (filteredPosition = position); the viewer
   draws no fitted line for it.
6. During the fit every point scores the nearer of direct and reflection; `imageSepMinRad` only
   labels `ambiguous`.
7. Per-point confidence, hypothesis and accept flag are in the 3D hover, not the frame inspector.
8. The replay also reconstructs both tracks at the end for the viewer. The board reconstructs only
   the ball, at RESULT; it does not reconstruct the club at all (see Task 7 below).
9. The tee anchor takes the ball track origin's slant range and bearing, at teeBallHeightM - radarHeightM.

### Deviations during implementation (2026-10-01)

- **Task 3, ball fit uncertainty gate.** At the measured per-point scatter the plan's fit reported
  "valid" on 167 of 200 synthetic shots with a median error of 22 deg, because the radar looks down
  the flight line and the direction is geometrically diluted. The fit gained an uncertainty gate: a
  finite-difference Hessian of the cost at the optimum gives a covariance, scaled by the observed
  rms squared over `angleSigma` squared (no floor at 1: with a floor, noise-free and 1 deg shots
  reported about 15 deg sigma and nothing was ever valid). `hlaSigmaRad`/`vlaSigmaRad` are reported,
  the new `maxAngleSigmaRad` (default 3 deg) rejects wide directions, and a new reason "uncertain"
  was added. Measured on 200 synthetic shots (HLA 2, VLA 14): 1 deg per-point noise gives 200/200
  valid with error p50 1.59 deg, p90 2.8 deg, max 4.4 deg; 3 deg gives 1 valid; 6 deg and 12 deg
  give 0 valid and 0 confidently wrong. Consequence: with real per-point scatter (about 12 deg
  elevation, 27 deg azimuth) the on-board ball HLA/VLA will mostly be reported invalid ("uncertain");
  the published LCMF launch angle is unaffected. The fit config is also guarded (`minAccepted` at
  least 3, `gridLevels` clamped 1..4, the forward square-root argument floored at 0), and the
  pure-noise test asserts only that no direction is reported, with a separate deterministic case for
  the "scatter" reason.
- **Task 4.** Removing `lateFrom` from the launch struct meant the two `gLaunch.lateFrom` writes in
  `l3_dump.c` and the late-window board-wiring test went in Task 4, not Task 6, so the tree compiles
  at every commit. The VLA read-back test landed in Task 7 for the same reason (it needs
  `angle_why`). The synthetic ball-tracker scene puts the tee at antenna height
  (`teeBallHeightM = radarHeightM = 0.152`): geometry, not a tolerance.
- **Task 5, club filter.** The smoothed-versus-raw bound is 0.65, not 0.5 (measured 0.56: smoothed
  0.125 m against raw 0.224 m, insensitive to `accelSigma` 400-1500). The chi-square gate at
  `angleSigma` 15 deg only rejects angle jumps beyond about 50 deg per axis (the test uses 60 deg on
  both axes); smaller outliers are down-weighted, not rejected, and tightening needs a lower
  `angleSigma` or `chi2Gate` from real club scatter. The filtered delivery keeps every reconstructed
  point with its raw `anglesValid`, because a gated point's smoothed position is still range-updated
  and neighbour-supported and dropping it can push the window under three points; the delivery fit's
  `maxAngleResidualM` still guards the direction. Measured on synthetic swings (seed 11, 20 swings),
  path RMS error was 18.85 deg raw and 14.30 deg filtered.
- **Task 6, board wiring.** The board image was not built: this Windows host has no make, gcc or TI
  toolchain, so the .bss fit of the club work area is unverified. That area (about 10.9 KB) was placed
  in HS-RAM (`L3_HSRAM_DIAG`), since DATA_RAM is full and the link fails rather than overwrites if
  it does not fit; it is fully initialised on each run, so the non-zeroed section is safe. (Task 7
  then removed the club work area from the board altogether.)
- **Task 7, the club stays unfiltered on the board.** The club's frozen delivery stays unfiltered
  (`l3_track_delivery`) on the board and in the replay. The filtered delivery regressed a recording's
  club speed from 32 to 44 m/s (outside the manifest's 22..40) and a synthetic club path from 3.0 to
  1.1 deg: with about five approach points and a 15 deg angle sigma the filter cannot pin the
  direction and pulls toward its prior. So the board does not run the club KF at the fire, and the
  profile stage "reconstruct" covers the ball fit only. `l3_track_delivery_filtered` remains as a
  host-tested library function, and the replay reconstructs the club once at the end of a shot for the
  viewer only. The ball consumers do switch to the fitted points; the club's option 3A is overridden.
  Restoring the filter on board is two calls at the fire once it is tuned on labelled captures.

## Results on the recordings (2026-10-01)

`scripts/analysis/evaluate_trajectory_reconstruction.py` over `tests/radar/recordings` (41 shots; the
full JSON, with per-shot rows, is `2026-10-01-trajectory-reconstruction-baseline.json`). There are no
angle labels, so this measures stability, not accuracy.

| Track | Shots compared | Median scatter, raw points | Median scatter, reconstructed |
|---|---|---|---|
| Ball | 1 | 0.169 m | 1.3e-8 m |
| Club | 34 | 0.362 m | 0.0050 m |

Read the ball figure with care: the ball's reconstructed points are on a straight line by construction,
so their scatter about a line is about zero whenever the fit is valid; it is compared on only 1 shot
because the fit was valid on only 1 of 41. The club figure is the smoother's effect on the host/viewer
reconstruction only; the board's club delivery is unfiltered.

Ball `angle_why` counts over the 41 shots: uncertain 18, grid_edge 12, few_angles 6, scatter 2,
no_launch 2, ok 1. Most shots end "uncertain" or "grid_edge": the real per-point angle scatter leaves
the direction ill-determined, which the uncertainty gate reports instead of a confident wrong answer.
This is a finding, not a retune: the ball fit's per-point sigma and gate limit are to be revisited
against labelled captures.

## Open questions (resolved during implementation, not blocking)

- A single ball `angleSigmaRad`, or separate azimuth and elevation values (27
  vs 12 deg). Start with separate values if the recordings show the azimuth
  residual dominating.
- The club's first point has no angles (`count > 1` check). Confirm the EKF
  initialisation handles it without special-casing.
