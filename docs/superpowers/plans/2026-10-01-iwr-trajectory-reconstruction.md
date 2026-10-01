# IWR6843 Trajectory Reconstruction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the raw per-frame XYZ points in the firmware's 3D ball and club
tracks with reconstructed trajectories: a tee-anchored robust direction fit for
the ball and a constant-velocity EKF with an RTS smoother for the club. Both
trust range heavily and angles weakly. The on-chip launch and delivery read the
reconstructions, and the dump viewer draws raw and fitted side by side.

**Architecture:** Two new pure-C firmware modules, `l3_ball_fit.c` and
`l3_track_kf.c`, write `filteredPosition`, `filterAccepted` and
`filterHypothesis` back onto `l3_track_point_t`, next to the raw `position`.
Neither runs per frame:

- The club EKF runs once at the fire, before the frozen delivery.
- The ball fit runs once when the shot reaches RESULT.
- Replay and the viewer run the same C through `firmware_host` (ctypes), so one
  change covers the board, replay and the viewer.

**Tech Stack:** C99 in float32 (TI R4F MSS; host gcc/clang for tests), Python
3.12 ctypes, pytest, Plotly.js in `scripts/iwr6843/dump_viewer.html`.

**Spec:** `docs/superpowers/specs/2026-10-01-iwr-trajectory-reconstruction-design.md`.
Read it first. Task 9 records the deviations listed below back into it.

## Global Constraints

- Every Python command goes through `uv`: `uv run pytest ...`,
  `uv run pylint src/openflight/ --fail-under=9`,
  `uv run ruff check src/openflight/`, `uv run ruff format --check src/openflight/`.
- Firmware C: float32 only, no `malloc`, no libm beyond `sqrtf`, `sinf`,
  `cosf`, `atan2f` (already linked). No `isfinite`; use the explicit
  `l3_kf_finite` helper.
- Nothing new on the per-frame path. `l3_ball_track_launch`, which runs every
  post frame, gets cheaper (the late-window fit goes). `l3_track_delivery`,
  every pre frame, is unchanged.
- Budgets on the MSS: ball fit ≤ 1.0 ms, club EKF plus smoother ≤ 0.5 ms. Both
  report under the profile stage `reconstruct`.
- Every ctypes mirror in `src/openflight/iwr6843/firmware_host.py` changes in
  the same commit as its C header. `test_iwr6843_firmware_ball_track.py`'s
  `sizeof(fw.BallTrack) == l3_ball_track_struct_bytes()` guards the layout.
- Units: metres, seconds, radians in C; degrees only in Python summaries. Golf
  frame: x downrange, y right, z up, origin at the antenna (`l3_frames.h`).
- Recordings: `fr.recording_configs()` over `tests/radar/recordings/`. There
  are no angle labels, so the evaluation measures stability, not accuracy.

**Deviations from the approved spec** (each one is recorded in the spec in
Task 9):

1. The club EKF drops the range-rate measurement. `radialVelocityMps` is
   derived from the same ranges, so using it would count them twice.
2. The club default `accelSigmaMps2` is 1500, not 300. A clubhead on its arc,
   about 40 m/s at about 1.1 m radius, pulls about 1400 m/s² of centripetal
   acceleration.
3. The EKF angle update is two sequential scalar updates, behind a joint 2-dof
   chi-square gate. That is equivalent for a diagonal R, with no 4x4 Cholesky.
   The RTS smoother solves one 6x6 Cholesky per step and computes no smoothed
   covariance, since only the positions are needed.
4. New tunables go in the replay registry (`tunables.py`). No new board
   `trackCfg` verbs.
5. On an invalid ball fit, `filteredPosition` is a copy of `position` and the
   point is flagged `unfiltered`, so the viewer draws no fitted line. The spec
   said "boresight fallback".
6. During the fit, every point scores the smaller of the direct and reflected
   residuals. `imageSepMinRad` only labels a point `ambiguous`.
7. The per-point confidence, hypothesis and accept flag appear in the 3D hover
   text, not the frame inspector.
8. The replay also reconstructs both tracks once at the end, for the viewer.
   The board reconstructs the club only at the fire.

## Review Focus

These are the five inputs or conditions most likely to bite that no feature
test would otherwise exercise. Each has a pinning test in the task named.

1. A ball adopted from a hypothesis (`useHypotheses`): its seeded points carry
   angles set before adoption. They must keep their angle confidence, not reset
   to 0, which would silently drop them from the fit (Task 2).
2. Club timestamps wrapping past 2³² mid-track: the EKF's Δt must come from an
   `int32` difference, as the rest of the firmware does, and not become a
   71-minute step (Task 5).
3. A ball track armed with a zero origin (no destination measured): the fit
   must report `no_tee` and leave every point unfiltered, never divide by zero
   (Task 3).
4. Two club points with the same timestamp (a re-emerged or tentative point):
   Δt 0 must neither crash nor produce NaN (Task 5).
5. The viewer opening a payload with no `filtered_position` (an older replay, or
   a failed fit): the 3D view must still draw the raw track and no fitted line
   (Task 8).

---

## File Structure

| File | Responsibility |
|---|---|
| `firmware/iwr6843/l3_frames.c` | Stops subtracting the az/el offsets that `l3_angle_estimate` already removed (Task 1). |
| `firmware/iwr6843/l3_club_track.h/.c` | Point gains `angleConfidence`, `filteredPosition`, `filterAccepted`, `filterHypothesis`. New `L3_FILTER_HYP_*`, `l3_track_kf_cfg_t` in `l3_track_cfg_t`, `l3_track_point_mut`, `l3_track_point_unfilter`, `l3_track_unfilter_all`, `l3_track_newest_first`. The set-angles functions take a confidence. |
| `firmware/iwr6843/l3_ball_hyp.h/.c` | Hypothesis point keeps `angleConfidence`; `l3_ball_hyps_set_angles` takes it. |
| `firmware/iwr6843/l3_angle_queue.c` | Passes `obs->confidence` on. |
| `firmware/iwr6843/l3_ball_fit.h/.c` **(new)** | Tee-anchored two-parameter ball direction fit: grid search, Huber, gate, floor image. |
| `firmware/iwr6843/l3_track_kf.h/.c` **(new)** | Club EKF, RTS smoother, `l3_track_delivery_filtered`. |
| `firmware/iwr6843/l3_ball_track.h/.c` | `fit` cfg replaces `lateRangeM`; `l3_ball_track_launch` is speed-only; new `l3_ball_track_reconstruct`. |
| `firmware/iwr6843/l3_launch.h/.c` | `lateFrom` becomes `anglesAccepted`, `angleWhy`, `angleRmsRad`; the format follows. |
| `firmware/iwr6843/l3_joint_search.c` | Drops its `lateFrom` line. |
| `firmware/iwr6843/l3_profile.h/.c` | New stage `reconstruct`, left out of the per-frame total. |
| `firmware/iwr6843/l3_dump.c` | Board wiring: confidences, reconstruct at the fire and at RESULT, profile stage. |
| `firmware/iwr6843/makefile` | New sources. |
| `src/openflight/iwr6843/firmware_host.py` | ctypes mirrors, constants, signatures, host sources. |
| `src/openflight/iwr6843/firmware_replay.py` | Confidences, fire-time club reconstruct, end-of-replay reconstruct, summaries; `late_range_m` removed. |
| `src/openflight/iwr6843/tunables.py` | New tunables. |
| `scripts/iwr6843/dump_viewer.html` | Raw/fitted/both toggle, and the new hover. |
| `scripts/analysis/evaluate_trajectory_reconstruction.py` **(new)** | Scatter before and after over the recordings, into a baseline JSON. |
| Tests | `tests/test_iwr6843_firmware_ball_fit.py` (new), `tests/test_iwr6843_firmware_track_kf.py` (new), plus updates to existing firmware, replay, viewer, profile, board-wiring and tunables tests. |

---

### Task 1: The az/el offsets are subtracted once

`l3_angle_estimate` already subtracts `elevationOffsetRad` (`l3_angle.c:252`)
and applies `azimuthOffsetRad` as a phase offset (`l3_angle.c:279`).
`l3_frames_observe` subtracts both again, as angles (`l3_frames.c:105-106`). The
existing test pins the wrong behaviour, so it is rewritten first.

**Files:**
- Modify: `firmware/iwr6843/l3_frames.c:95-109`, `firmware/iwr6843/l3_frames.h:89` (comment)
- Test: `tests/test_iwr6843_firmware_frames.py:147-153`

**Interfaces:**
- Consumes: nothing new.
- Produces: `l3_frames_observe(cal, rangeM, azimuthRad, elevationRad, golf)` takes angles already corrected by `l3_angle_estimate`. It removes only `rangeBiasM`.

- [ ] **Step 1: Rewrite the test to state the correct contract**

Replace `test_observe_removes_the_range_bias_and_baseline_offsets_before_rotating`
in `tests/test_iwr6843_firmware_frames.py` with:

```python
def test_observe_removes_the_range_bias_but_not_the_angle_offsets(lib):
    """l3_angle_estimate already removed the offsets (the azimuth one as a
    phase); observing them again would count them twice."""
    c = cal(lib, rangeBiasM=0.066, azimuthOffsetRad=2.0 * DEG, elevationOffsetRad=-1.0 * DEG)
    golf = fw.Vec3()
    lib.l3_frames_observe(ctypes.byref(c), 2.066, 0.0, 0.0, ctypes.byref(golf))
    assert vec(golf) == pytest.approx((2.0, 0.0, 0.0), abs=1e-5)
    lib.l3_frames_observe(ctypes.byref(c), 2.066, 3.0 * DEG, -2.0 * DEG, ctypes.byref(golf))
    expected = (
        2.0 * math.cos(-2.0 * DEG) * math.cos(3.0 * DEG),
        2.0 * math.cos(-2.0 * DEG) * math.sin(3.0 * DEG),
        2.0 * math.sin(-2.0 * DEG),
    )
    assert vec(golf) == pytest.approx(expected, abs=1e-5)
    lib.l3_frames_observe(ctypes.byref(c), 0.01, 0.0, 0.0, ctypes.byref(golf))
    assert vec(golf) == pytest.approx((0.0, 0.0, 0.0)), "a range under the bias clamps at zero"
```

- [ ] **Step 2: Run it to verify it fails**

Run: `uv run pytest tests/test_iwr6843_firmware_frames.py -k observe -v`
Expected: FAIL. The first assert reads y ≈ -0.0698 (the offset applied again).

- [ ] **Step 3: Fix `l3_frames_observe`**

In `firmware/iwr6843/l3_frames.c`, replace the two offset lines:

```c
    spherical.azimuthRad = azimuthRad;
    spherical.elevationRad = elevationRad;
```

In `l3_frames.h`, above the `l3_frames_observe` declaration (line 89), replace
the comment with:

```c
/* A measured point in the golf frame: the range less rangeBiasM, then the
 * attitude rotation. The angles are l3_angle_estimate's, which has already
 * removed the az/el offsets (the azimuth one as a phase), so they are not
 * applied here again. */
```

- [ ] **Step 4: Run the frames, angle and ball-track suites**

Run: `uv run pytest tests/test_iwr6843_firmware_frames.py tests/test_iwr6843_firmware_angle.py tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_club_track.py -v`
Expected: all PASS. Every other test uses zero offsets.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_frames.c firmware/iwr6843/l3_frames.h tests/test_iwr6843_firmware_frames.py
git commit -m "fix(iwr6843): observe no longer re-applies the angle offsets l3_angle_estimate removed"
```

---

### Task 2: Per-point angle confidence and reconstruction fields

**Files:**
- Modify: `firmware/iwr6843/l3_club_track.h`, `firmware/iwr6843/l3_club_track.c`
- Modify: `firmware/iwr6843/l3_ball_hyp.h:35-45,127`, `firmware/iwr6843/l3_ball_hyp.c:316-335`
- Modify: `firmware/iwr6843/l3_ball_track.h` (`l3_ball_track_set_angles`), `firmware/iwr6843/l3_ball_track.c:386-390` and adopt (`p->anglesValid` block)
- Modify: `firmware/iwr6843/l3_angle_queue.c:93`
- Modify: `firmware/iwr6843/l3_dump.c:3702-3703,3742-3743`
- Modify: `src/openflight/iwr6843/firmware_host.py` (TrackCfg, TrackPoint, BallHypPoint, signatures, constants)
- Modify: `src/openflight/iwr6843/firmware_replay.py:2007-2012,2168-2170`
- Test: `tests/test_iwr6843_firmware_club_track.py`, `tests/test_iwr6843_firmware_ball_track.py`, `tests/test_iwr6843_firmware_ball_hyp.py`, `tests/test_iwr6843_firmware_angle_queue.py`, `tests/test_iwr6843_firmware_sparse.py`

**Interfaces:**
- Produces (C):
  - `l3_track_point_t` gains `float angleConfidence; l3_vec3_t filteredPosition; uint8_t filterAccepted; uint8_t filterHypothesis;`, in that order, after `position`.
  - `enum { L3_FILTER_HYP_NONE, L3_FILTER_HYP_DIRECT, L3_FILTER_HYP_IMAGE, L3_FILTER_HYP_AMBIGUOUS, L3_FILTER_HYP_UNFILTERED, L3_FILTER_HYP_COUNT }`.
  - `l3_track_kf_cfg_t` (7 floats; see Step 3), a new last field `l3_track_kf_cfg_t kf;` of `l3_track_cfg_t`.
  - `l3_track_point_t *l3_track_point_mut(l3_club_track_t *track, uint32_t index)` (NULL when out of range).
  - `void l3_track_point_unfilter(l3_track_point_t *point)`.
  - `void l3_track_unfilter_all(l3_club_track_t *track)`.
  - `uint32_t l3_track_newest_first(const l3_club_track_t *track, uint32_t maxPoints)`.
  - `l3_track_set_angles(track, az, el, anglesValid, float angleConfidence)`.
  - `l3_track_set_point_angles(track, index, az, el, anglesValid, float angleConfidence)`.
  - `l3_ball_track_set_angles(track, az, el, anglesValid, float angleConfidence)`.
  - `l3_ball_hyps_set_angles(hyps, index, az, el, anglesValid, float angleConfidence)`.
  - `l3_ball_hyp_point_t` gains `float angleConfidence;` after `anglesValid`.
  - `l3_track_kf_cfg_defaults` is declared in `l3_track_kf.h` (Task 5). Until then, `l3_track_cfg_defaults` fills `kf` inline through the static `l3_track_kf_defaults_inline` below, which Task 5 replaces with the call.
- Produces (Python): `fw.FILTER_HYP_NAMES = ("none", "direct", "image", "ambiguous", "unfiltered")`, `fw.FILTER_HYP_UNFILTERED = 4`, `fw.TrackKfCfg`, and new fields on `fw.TrackPoint`, `fw.BallHypPoint` and `fw.TrackCfg`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_firmware_club_track.py`. Reuse that file's `lib`
fixture and its target and track helpers; read the top of the file for their
names. Where the snippet below says `new_track(lib)` / `target(frame, bin)`,
use the file's existing equivalents.

```python
BOTH_ANGLES = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION


def _held(lib, track, index):
    point = fw.TrackPoint()
    assert lib.l3_track_point(ctypes.byref(track), index, ctypes.byref(point)) == 1
    return point


def test_set_point_angles_stores_the_angle_confidence_and_unfilters(lib):
    track = new_track(lib)
    for frame, rng in ((1, 30.0), (2, 31.0), (3, 32.0)):
        assert lib.l3_track_update(
            ctypes.byref(track), (fw.TargetObs * 1)(target(frame, rng)), 1, frame, frame * 3000
        )
    assert lib.l3_track_set_point_angles(ctypes.byref(track), 1, 0.1, 0.2, BOTH_ANGLES, 0.37) == 1
    point = _held(lib, track, 1)
    assert point.angleConfidence == pytest.approx(0.37)
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED and point.filterAccepted == 0
    assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
        point.position.x,
        point.position.y,
        point.position.z,
    )


def test_an_appended_point_starts_unfiltered_with_no_angle_confidence(lib):
    track = new_track(lib)
    assert lib.l3_track_update(ctypes.byref(track), (fw.TargetObs * 1)(target(1, 30.0)), 1, 1, 3000)
    point = _held(lib, track, 0)
    assert point.angleConfidence == 0.0
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED


def test_track_cfg_defaults_fill_the_reconstruction_constants(lib):
    cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.kf.accelSigmaMps2 == pytest.approx(1500.0)
    assert cfg.kf.rangeSigmaM == pytest.approx(0.03)
    assert cfg.kf.angleSigmaRad == pytest.approx(math.radians(15.0))
    assert cfg.kf.minAngleConfidence == pytest.approx(0.05)
    assert cfg.kf.chi2Gate == pytest.approx(9.21)
    assert cfg.kf.initPositionSigmaM == pytest.approx(0.5)
    assert cfg.kf.initVelocitySigmaMps == pytest.approx(50.0)
```

Append to `tests/test_iwr6843_firmware_ball_track.py` (Review Focus 1):

```python
def test_an_adopted_hypothesis_keeps_its_points_angle_confidence(lib):
    """Adoption re-seeds the core from the hypothesis's points; each keeps the
    confidence its angles were measured with, or the direction fit would drop them."""
    ball = Ball(lib, useHypotheses=1)
    ball.arm()
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
    rng = ORIGIN_BIN + 2.0
    for frame in range(8, 16):
        rng += 3.0
        arr = (fw.TargetObs * 1)(target(frame, rng, doppler=12.0))
        lib.l3_ball_track_update_joint(ctypes.byref(ball.track), arr, 1, frame, frame * FRAME_US, 0xFFFFFFFF)
        for i in range(fw.BALL_HYP_MAX):
            lib.l3_ball_hyps_set_angles(ctypes.byref(ball.track.hyps), i, 0.02, 0.2, both, 0.61)
        if ball.track.confirmed:
            break
    assert ball.track.confirmed, "the hypothesis search never adopted the ball"
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(ball.track.core), 0, ctypes.byref(point))
    assert point.anglesValid == both
    assert point.angleConfidence == pytest.approx(0.61)
```

Append to `tests/test_iwr6843_firmware_angle_queue.py`:

```python
def test_a_drained_angle_carries_its_confidence_onto_the_point(lib, cal):
    """l3_angle_queue_apply passes obs->confidence to the point."""
    q = queue(lib)
    tr = track_of(lib, 4)
    newest = tr.points()[-1]
    job = fw.AngleJob(timestampUs=newest.timestampUs, snapshot=snapshot(lib, 2))
    obs = fw.AngleObs()
    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q), ctypes.byref(cal), ctypes.byref(job), ctypes.byref(tr.track), ctypes.byref(obs)
        )
        == 1
    )
    assert obs.confidence > 0.0
    assert tr.points()[-1].angleConfidence == pytest.approx(obs.confidence)
```

Also, in that file's `test_the_queue_path_gives_exactly_the_immediate_estimate`,
the immediate call passes the estimate's own confidence, so the byte-equality
there pins the same thing:
`lib.l3_track_set_angles(ctypes.byref(immediate.track), obs.azimuthRad, obs.elevationRad, flags, obs.confidence)`.

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_club_track.py tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_angle_queue.py -k "confidence or reconstruction_constants or unfiltered" -v`
Expected: FAIL. `TrackPoint` has no `angleConfidence`; `set_point_angles` takes 5 arguments.

- [ ] **Step 3: C: point fields, cfg, helpers**

In `firmware/iwr6843/l3_club_track.h`:

1. Add, after the `L3_TRACK_TENTATIVE_ADVANCE_BINS` define:

```c
/* What reconstruction (l3_ball_fit.h, l3_track_kf.h) made of a point. */
enum {
    L3_FILTER_HYP_NONE = 0,      /* reconstructed, but its angles were not used
                                  * (none measured, no weight, or gated out) */
    L3_FILTER_HYP_DIRECT,        /* its angles were used as the direct return */
    L3_FILTER_HYP_IMAGE,         /* ... as the floor reflection */
    L3_FILTER_HYP_AMBIGUOUS,     /* ... direct and reflection too close to tell */
    L3_FILTER_HYP_UNFILTERED,    /* not reconstructed: filteredPosition is position */
    L3_FILTER_HYP_COUNT
};

/* The club reconstruction's constants (l3_track_kf.h). */
typedef struct {
    float accelSigmaMps2;        /* white-acceleration process noise */
    float rangeSigmaM;           /* one range measurement */
    float angleSigmaRad;         /* one angle at full angle confidence */
    float minAngleConfidence;    /* floor on the confidence dividing angleSigmaRad */
    float chi2Gate;              /* 2-dof gate on the angle pair's innovation */
    float initPositionSigmaM;    /* first point's position uncertainty */
    float initVelocitySigmaMps;  /* first point's velocity uncertainty (starts at 0) */
} l3_track_kf_cfg_t;
```

2. In `l3_track_point_t`, after `position`:

```c
    float     angleConfidence;    /* l3_angle_estimate's confidence for these angles, 0 none */
    l3_vec3_t filteredPosition;   /* reconstructed GOLF-frame position; position when
                                   * filterHypothesis is L3_FILTER_HYP_UNFILTERED */
    uint8_t   filterAccepted;     /* 1 when the reconstruction used this point's angles */
    uint8_t   filterHypothesis;   /* L3_FILTER_HYP_* */
```

3. As the last field of `l3_track_cfg_t` (after `standingFrames`):

```c
    /* The club reconstruction (l3_track_kf.h). The ball's core carries it too,
     * unused: the ball has its own fit (l3_ball_fit.h). */
    l3_track_kf_cfg_t kf;
```

4. Change the declarations and add the helpers, next to
   `l3_track_set_point_angles`:

```c
int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid, float angleConfidence);
/* The same for any stored point (index 0 is the oldest held): sets its angles
 * and their confidence, recomputes its golf-frame position and marks it
 * unfiltered (a new angle voids an earlier reconstruction). Returns 0 when
 * index is not stored. */
int32_t l3_track_set_point_angles(l3_club_track_t *track, uint32_t index, float azimuthRad,
                                  float elevationRad, uint8_t anglesValid, float angleConfidence);
/* A stored point for reconstruction to write into (index 0 oldest); NULL when
 * not stored. */
l3_track_point_t *l3_track_point_mut(l3_club_track_t *track, uint32_t index);
/* Mark one point, or every held point, not reconstructed. */
void l3_track_point_unfilter(l3_track_point_t *point);
void l3_track_unfilter_all(l3_club_track_t *track);
/* The index of the first of the newest maxPoints held points. */
uint32_t l3_track_newest_first(const l3_club_track_t *track, uint32_t maxPoints);
```

In `firmware/iwr6843/l3_club_track.c`:

```c
/* The reconstruction's defaults; Task 5 moves them to l3_track_kf_cfg_defaults. */
static void l3_track_kf_defaults_inline(l3_track_kf_cfg_t *kf)
{
    kf->accelSigmaMps2 = 1500.0F;    /* a clubhead on its arc: ~40 m/s at ~1.1 m radius */
    kf->rangeSigmaM = 0.03F;
    kf->angleSigmaRad = 15.0F * (3.14159265F / 180.0F);
    kf->minAngleConfidence = 0.05F;
    kf->chi2Gate = 9.21F;            /* 99 % for 2 degrees of freedom */
    kf->initPositionSigmaM = 0.5F;
    kf->initVelocitySigmaMps = 50.0F;
}
```

- Call `l3_track_kf_defaults_inline(&cfg->kf);` at the end of
  `l3_track_cfg_defaults`.
- Add the helpers:

```c
void l3_track_point_unfilter(l3_track_point_t *point)
{
    point->filteredPosition = point->position;
    point->filterAccepted = 0U;
    point->filterHypothesis = L3_FILTER_HYP_UNFILTERED;
}

l3_track_point_t *l3_track_point_mut(l3_club_track_t *track, uint32_t index)
{
    uint32_t oldest;

    if (index >= track->count) {
        return NULL;
    }
    oldest = (track->next + L3_TRACK_POINTS - track->count) % L3_TRACK_POINTS;
    return &track->points[(oldest + index) % L3_TRACK_POINTS];
}

void l3_track_unfilter_all(l3_club_track_t *track)
{
    uint32_t i;

    for (i = 0U; i < track->count; i++) {
        l3_track_point_unfilter(l3_track_point_mut(track, i));
    }
}

uint32_t l3_track_newest_first(const l3_club_track_t *track, uint32_t maxPoints)
{
    uint32_t used;

    if (maxPoints > L3_TRACK_POINTS) {
        maxPoints = L3_TRACK_POINTS;
    }
    used = (track->count < maxPoints) ? track->count : maxPoints;
    return track->count - used;
}
```

- In `l3_track_append`, after `l3_track_locate(track, point);`:

```c
    /* A target's own angles (a seeded or synthetic point) carry no estimate
     * of their quality: full weight when present, none when absent. */
    point->angleConfidence = target->anglesValid ? 1.0F : 0.0F;
    l3_track_point_unfilter(point);
```

- In `l3_track_append_point`, after `l3_track_locate(track, slot);` add
  `l3_track_point_unfilter(slot);`.
- Replace `l3_track_set_point_angles` and `l3_track_set_angles`:

```c
int32_t l3_track_set_point_angles(l3_club_track_t *track, uint32_t index, float azimuthRad,
                                  float elevationRad, uint8_t anglesValid, float angleConfidence)
{
    l3_track_point_t *point = l3_track_point_mut(track, index);

    if (point == NULL) {
        return 0;
    }
    point->azimuthRad = azimuthRad;
    point->elevationRad = elevationRad;
    point->anglesValid = anglesValid;
    point->angleConfidence = angleConfidence;
    l3_track_locate(track, point);
    l3_track_point_unfilter(point);
    return 1;
}

int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid, float angleConfidence)
{
    if (track->lastTargetIndex == L3_TRACK_NO_TARGET || track->count == 0U) {
        return 0;
    }
    return l3_track_set_point_angles(track, track->count - 1U, azimuthRad, elevationRad,
                                     anglesValid, angleConfidence);
}
```

- Make `l3_track_delivery` use the helper:

```c
uint32_t l3_track_delivery(const l3_club_track_t *track, uint32_t maxPoints, l3_delivery_t *out)
{
    uint32_t first = l3_track_newest_first(track, maxPoints);

    return l3_track_delivery_range(track, first, track->count - first, L3_TRACK_FULL_POINTS, out);
}
```

- [ ] **Step 4: C: hypotheses, ball track, angle queue, board**

1. `l3_ball_hyp.h`: add `float angleConfidence;` after `anglesValid` in
   `l3_ball_hyp_point_t`, and add `float angleConfidence` as the last parameter
   of `l3_ball_hyps_set_angles`.
2. `l3_ball_hyp.c`: in `l3_ball_hyps_set_angles`, store
   `point->angleConfidence = angleConfidence;` beside `point->anglesValid`.
   Where a hypothesis point is appended (`l3_ball_hyps_append`), set
   `angleConfidence = 0.0F` next to wherever `anglesValid` is zeroed.
3. `l3_ball_track.h/.c`: `l3_ball_track_set_angles(track, az, el, anglesValid,
   float angleConfidence)` forwards to `l3_track_set_angles(..., angleConfidence)`.
   In `l3_ball_track_adopt`:

```c
        if (p->anglesValid) {
            (void)l3_track_set_angles(&track->core, p->azimuthRad, p->elevationRad,
                                      p->anglesValid, p->angleConfidence);
        }
```

4. `l3_angle_queue.c:93`:

```c
    (void)l3_track_set_point_angles(track, index, obs->azimuthRad, obs->elevationRad, flags,
                                    obs->confidence);
```

5. `l3_dump.c:3702`: `(void)l3_ball_track_set_angles(&gBallTrack, angle.azimuthRad,
   angle.elevationRad, flags, angle.confidence);`.
   `l3_dump.c:3742`: `(void)l3_ball_hyps_set_angles(&gBallTrack.hyps, index,
   angle.azimuthRad, angle.elevationRad, flags, angle.confidence);`.

- [ ] **Step 5: Python mirrors and call sites**

In `src/openflight/iwr6843/firmware_host.py`:

```python
# l3_club_track.h L3_FILTER_HYP_*
FILTER_HYP_NAMES = ("none", "direct", "image", "ambiguous", "unfiltered")
FILTER_HYP_UNFILTERED = FILTER_HYP_NAMES.index("unfiltered")


class TrackKfCfg(ctypes.Structure):
    """``l3_track_kf_cfg_t``."""

    _fields_ = [
        ("accelSigmaMps2", ctypes.c_float),
        ("rangeSigmaM", ctypes.c_float),
        ("angleSigmaRad", ctypes.c_float),
        ("minAngleConfidence", ctypes.c_float),
        ("chi2Gate", ctypes.c_float),
        ("initPositionSigmaM", ctypes.c_float),
        ("initVelocitySigmaMps", ctypes.c_float),
    ]
```

Define `TrackKfCfg` above `TrackCfg`, and add `("kf", TrackKfCfg)` as
`TrackCfg`'s last field. Add to `TrackPoint` after `("position", Vec3)`:

```python
        ("angleConfidence", ctypes.c_float),
        ("filteredPosition", Vec3),
        ("filterAccepted", ctypes.c_uint8),
        ("filterHypothesis", ctypes.c_uint8),
```

Add `("angleConfidence", ctypes.c_float)` after `anglesValid` in `BallHypPoint`.

In `_SIGNATURES`:

```python
    "l3_track_set_angles": ([_P(ClubTrack), _F32, _F32, ctypes.c_uint8, _F32], ctypes.c_int32),
    "l3_track_set_point_angles": (
        [_P(ClubTrack), _U32, _F32, _F32, ctypes.c_uint8, _F32],
        ctypes.c_int32,
    ),
    "l3_track_append_point": ([_P(ClubTrack), _P(TrackPoint)], None),
    "l3_track_find_point": ([_P(ClubTrack), _U32, _P(_U32)], ctypes.c_int32),
    "l3_track_unfilter_all": ([_P(ClubTrack)], None),
    "l3_ball_track_set_angles": (
        [_P(BallTrack), _F32, _F32, ctypes.c_uint8, _F32],
        ctypes.c_int32,
    ),
    "l3_ball_hyps_set_angles": (
        [_P(BallHyps), _U32, ctypes.c_float, ctypes.c_float, ctypes.c_uint8, ctypes.c_float],
        ctypes.c_int32,
    ),
```

(`l3_track_append_point` and `l3_track_find_point` may already be listed;
don't duplicate an entry.) Export `FILTER_HYP_NAMES`, `FILTER_HYP_UNFILTERED`
and `TrackKfCfg` in the module's `__all__` if it has one (it lists
`PROFILE_STAGE_NAMES` at ~2191).

In `src/openflight/iwr6843/firmware_replay.py`:
- In the `l3_ball_track_set_angles` call (~2007), add
  `float(obs_angle.confidence),` after `flags,`.
- In the `l3_ball_hyps_set_angles` call (~2168), add
  `float(obs_angle.confidence)` after `flags`.

`_estimate_angles` returns a `fw.AngleObs` with `.confidence`, so both sites
already have it.

- [ ] **Step 6: Update the existing call sites in tests**

Every call to `l3_track_set_point_angles`, `l3_track_set_angles`,
`l3_ball_track_set_angles` and `l3_ball_hyps_set_angles` in
`tests/test_iwr6843_firmware_ball_track.py` (8),
`tests/test_iwr6843_firmware_club_track.py` (9),
`tests/test_iwr6843_firmware_ball_hyp.py` (3) and
`tests/test_iwr6843_firmware_angle_queue.py` (1, which takes `obs.confidence`;
see Step 1) gains `1.0` as its new last argument. Change `Ball.set_angles` in `test_iwr6843_firmware_ball_track.py` to:

```python
    def set_angles(self, az, el, flags=fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION, confidence=1.0) -> int:
        return self.lib.l3_ball_track_set_angles(ctypes.byref(self.track), az, el, flags, confidence)
```

In `tests/test_iwr6843_firmware_sparse.py:642`, the source-text assertion
becomes:

```python
    assert "l3_ball_track_set_angles(&gBallTrack, angle.azimuthRad," in consider
    assert "angle.elevationRad, flags, angle.confidence);" in " ".join(consider.split())
```

Find the sites with:
`uv run python -c "import subprocess;print(subprocess.run(['git','grep','-n','set_point_angles\\|set_angles(','tests/'],capture_output=True,text=True).stdout)"`

- [ ] **Step 7: Run the firmware suites**

Run: `uv run pytest tests/ -k "firmware or replay or dump_viewer" -q`
Expected: all PASS, including the new tests and the `sizeof(fw.BallTrack)`
layout check.

- [ ] **Step 8: Commit**

```bash
git add firmware/iwr6843 src/openflight/iwr6843/firmware_host.py src/openflight/iwr6843/firmware_replay.py tests/
git commit -m "feat(iwr6843): track points keep their angle confidence and a reconstruction slot"
```

---

### Task 3: The tee-anchored ball direction fit (`l3_ball_fit`)

**Files:**
- Create: `firmware/iwr6843/l3_ball_fit.h`, `firmware/iwr6843/l3_ball_fit.c`
- Modify: `firmware/iwr6843/makefile:85` (SOURCES), `src/openflight/iwr6843/firmware_host.py` (HOST_SOURCES, structs, signatures, names)
- Test: `tests/test_iwr6843_firmware_ball_fit.py` (new)

**Interfaces:**
- Consumes: `l3_track_point_mut`, `l3_track_unfilter_all`, `L3_FILTER_HYP_*` (Task 2).
- Produces:
  - `l3_ball_fit_cfg_t`, `l3_ball_fit_t`, `L3_BALL_FIT_WHY_{NONE,OK,FEW_ANGLES,SCATTER,GRID_EDGE,NO_TEE,COUNT}`.
  - `void l3_ball_fit_cfg_defaults(l3_ball_fit_cfg_t *cfg)`.
  - `uint32_t l3_ball_fit_max_evaluations(const l3_ball_fit_cfg_t *cfg)`.
  - `void l3_ball_fit_direction(float hlaRad, float vlaRad, l3_vec3_t *u)`.
  - `uint32_t l3_ball_fit_run(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin, l3_club_track_t *core, l3_ball_fit_t *out)`, which returns the accepted count, 0 when invalid.
  - `const char *l3_ball_fit_why_name(uint8_t why)`.
  - Python: `fw.BallFitCfg`, `fw.BallFit`, `fw.BALL_FIT_WHY_NAMES = ("none", "ok", "few_angles", "scatter", "grid_edge", "no_tee")`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_iwr6843_firmware_ball_fit.py`:

```python
"""Tests for the tee-anchored ball direction fit, firmware/iwr6843/l3_ball_fit.c.

Over the ~30 ms a ball track spans, gravity moves it ~3 mm, so its path is a
straight line from the tee. The fit takes each point's range as exact, the
tee as the anchor, and finds only the direction (HLA, VLA) that best explains
the points' measured angles, scoring each against both the direct return and
its floor reflection. Angles further than gateK sigma from the fit are
rejected. Identity calibration throughout: the radar frame is the golf frame.
"""

from __future__ import annotations

import ctypes
import math
import random

import pytest

from openflight.iwr6843 import firmware_host as fw

DEG = math.pi / 180.0
BIN_M = 6.0 / 128
FRAME_US = 3000
IMPACT_US = 21_700
ORIGIN_BIN = 38.0
BOTH = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
HYP = {name: index for index, name in enumerate(fw.FILTER_HYP_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def defaults(lib, **overrides) -> fw.BallFitCfg:
    cfg = fw.BallFitCfg()
    lib.l3_ball_fit_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    return cfg


def direction(hla_deg, vla_deg):
    h, v = hla_deg * DEG, vla_deg * DEG
    return (math.cos(v) * math.cos(h), math.cos(v) * math.sin(h), math.sin(v))


class Shot:
    """A ball leaving the fit's tee anchor along (hla, vla): one point per frame
    on a track core, each with its true direction as the measured angles."""

    def __init__(self, lib, *, hla_deg, vla_deg, speed=50.0, frames=8, first_frame=8, cfg=None):
        self.lib = lib
        self.cfg = cfg or defaults(lib)
        tcfg = fw.TrackCfg()
        lib.l3_track_cfg_defaults(ctypes.byref(tcfg))
        tcfg.maxAngleResidualM = 0.0  # the raw line fit, for comparisons, never gives up
        self.core = fw.ClubTrack()
        lib.l3_track_init(ctypes.byref(self.core), ctypes.byref(tcfg))
        self.origin = fw.Vec3(ORIGIN_BIN * BIN_M, 0.0, 0.0)
        height = self.cfg.teeBallHeightM - self.cfg.radarHeightM
        rng = ORIGIN_BIN * BIN_M
        self.tee = (math.sqrt(rng * rng - height * height), 0.0, height)
        u = direction(hla_deg, vla_deg)
        self.truth = []
        for index, frame in enumerate(range(first_frame, first_frame + frames)):
            s = speed * (frame * FRAME_US - IMPACT_US) * 1e-6
            p = tuple(self.tee[i] + s * u[i] for i in range(3))
            self.truth.append(p)
            point = fw.TrackPoint()
            point.frame = frame
            point.timestampUs = frame * FRAME_US
            point.rangeM = math.sqrt(sum(c * c for c in p))
            point.rangeBin = point.rangeM / BIN_M
            lib.l3_track_append_point(ctypes.byref(self.core), ctypes.byref(point))
            self.measure(index, p)

    def angles_of(self, golf):
        cal = self.core.cfg.cal
        radar = fw.Vec3()
        self.lib.l3_frames_golf_to_radar(
            ctypes.byref(cal), ctypes.byref(fw.Vec3(*golf)), ctypes.byref(radar)
        )
        sph = fw.Spherical()
        self.lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        return sph.azimuthRad, sph.elevationRad

    def measure(self, index, golf, *, az_err=0.0, el_err=0.0, confidence=1.0, flags=BOTH):
        az, el = self.angles_of(golf)
        assert (
            self.lib.l3_track_set_point_angles(
                ctypes.byref(self.core), index, az + az_err, el + el_err, flags, confidence
            )
            == 1
        )

    def reflect(self, index):
        x, y, z = self.truth[index]
        self.measure(index, (x, y, -2.0 * self.cfg.radarHeightM - z))

    def run(self, origin=None):
        fit = fw.BallFit()
        accepted = self.lib.l3_ball_fit_run(
            ctypes.byref(self.cfg),
            ctypes.byref(origin or self.origin),
            ctypes.byref(self.core),
            ctypes.byref(fit),
        )
        return accepted, fit

    def point(self, index) -> fw.TrackPoint:
        point = fw.TrackPoint()
        assert self.lib.l3_track_point(ctypes.byref(self.core), index, ctypes.byref(point)) == 1
        return point

    def raw_line_direction(self):
        """The unconstrained 3D line fit over the raw points (l3_delivery_fit)."""
        out = fw.Delivery()
        n = self.core.count
        self.lib.l3_track_delivery_range(ctypes.byref(self.core), 0, n, n, ctypes.byref(out))
        v = out.velocity
        return math.degrees(math.atan2(v.y, v.x)), math.degrees(math.atan2(v.z, math.hypot(v.x, v.y)))


def why(fit) -> str:
    return fw.BALL_FIT_WHY_NAMES[fit.why]


def test_defaults_are_the_measured_scatter_and_a_three_level_grid(lib):
    cfg = defaults(lib)
    assert cfg.angleSigmaRad == pytest.approx(12.0 * DEG)
    assert cfg.gateK == pytest.approx(2.5) and cfg.huberK == pytest.approx(1.5)
    assert cfg.minAccepted == 4 and cfg.maxRmsRad == pytest.approx(15.0 * DEG)
    assert cfg.imageSepMinRad == pytest.approx(2.0 * DEG)
    assert cfg.radarHeightM == pytest.approx(0.152) and cfg.teeBallHeightM == pytest.approx(0.04)
    assert (cfg.hlaMinRad, cfg.hlaMaxRad) == pytest.approx((-45.0 * DEG, 45.0 * DEG))
    assert (cfg.vlaMinRad, cfg.vlaMaxRad) == pytest.approx((-10.0 * DEG, 60.0 * DEG))
    assert cfg.gridSteps == 10 and cfg.gridLevels == 3


def test_why_names_match_the_firmware(lib):
    for index, name in enumerate(fw.BALL_FIT_WHY_NAMES):
        assert lib.l3_ball_fit_why_name(index).decode() == name


@pytest.mark.parametrize(
    "hla_deg,vla_deg", [(0.0, 12.0), (3.0, 10.0), (-4.5, 15.0), (1.2, 25.0), (8.0, 35.0)]
)
def test_noise_free_points_give_back_the_direction_and_the_line(lib, hla_deg, vla_deg):
    shot = Shot(lib, hla_deg=hla_deg, vla_deg=vla_deg)
    accepted, fit = shot.run()
    assert why(fit) == "ok" and fit.valid == 1 and accepted == 8 == fit.accepted
    assert fit.hlaRad / DEG == pytest.approx(hla_deg, abs=0.3)
    assert fit.vlaRad / DEG == pytest.approx(vla_deg, abs=0.3)
    assert (fit.tee.x, fit.tee.y, fit.tee.z) == pytest.approx(shot.tee, abs=1e-4)
    for index, truth in enumerate(shot.truth):
        point = shot.point(index)
        assert point.filterAccepted == 1 and point.filterHypothesis == HYP["direct"]
        filtered = (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z)
        assert filtered == pytest.approx(truth, abs=0.01)


def test_noisy_angles_fit_better_than_the_unconstrained_line(lib):
    """Fixed-seed Monte Carlo: 1 deg angle noise per point. The tee anchor and
    the exact ranges leave two unknowns, so the fit beats the free 3D line.
    If the 3 deg bound fails with a correct implementation, report the RMS
    achieved; do not loosen the bound silently."""
    rng = random.Random(20261001)
    fit_err, line_err = [], []
    for _ in range(40):
        shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
        for index, truth in enumerate(shot.truth):
            shot.measure(index, truth, az_err=rng.gauss(0, 1.0) * DEG, el_err=rng.gauss(0, 1.0) * DEG)
        _, fit = shot.run()
        assert fit.valid
        fit_err.append(math.hypot(fit.hlaRad / DEG - 2.0, fit.vlaRad / DEG - 14.0))
        hla, vla = shot.raw_line_direction()
        line_err.append(math.hypot(hla - 2.0, vla - 14.0))
    fit_rms = math.sqrt(sum(e * e for e in fit_err) / len(fit_err))
    line_rms = math.sqrt(sum(e * e for e in line_err) / len(line_err))
    assert fit_rms <= 3.0
    assert fit_rms < 0.5 * line_rms, (fit_rms, line_rms)


def test_wild_angles_are_gated_out_and_marked(lib):
    shot = Shot(lib, hla_deg=1.0, vla_deg=15.0)
    for index in (2, 5):
        shot.measure(index, shot.truth[index], az_err=40.0 * DEG, el_err=-40.0 * DEG)
    accepted, fit = shot.run()
    assert fit.valid and accepted == 6
    assert fit.hlaRad / DEG == pytest.approx(1.0, abs=0.5)
    assert fit.vlaRad / DEG == pytest.approx(15.0, abs=0.5)
    for index in range(8):
        point = shot.point(index)
        if index in (2, 5):
            assert point.filterAccepted == 0 and point.filterHypothesis == HYP["none"]
        else:
            assert point.filterAccepted == 1
    wild = shot.point(2)
    assert (wild.filteredPosition.x, wild.filteredPosition.y, wild.filteredPosition.z) == pytest.approx(
        shot.truth[2], abs=0.01
    ), "a rejected point still lies on the fitted line at its range"


def test_floor_reflections_count_for_the_same_direction(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=20.0)
    for index in (0, 2, 4, 6):
        shot.reflect(index)
    accepted, fit = shot.run()
    assert fit.valid and accepted == 8
    assert fit.vlaRad / DEG == pytest.approx(20.0, abs=0.5)
    labels = [shot.point(i).filterHypothesis for i in range(8)]
    assert all(labels[i] in (HYP["image"], HYP["ambiguous"]) for i in (0, 2, 4, 6))
    assert sum(labels[i] == HYP["image"] for i in (0, 2, 4, 6)) >= 2
    assert all(labels[i] == HYP["direct"] for i in (1, 3, 5, 7))


def test_too_few_angles_report_no_direction_and_leave_points_unfiltered(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    for index in range(3, 8):
        shot.measure(index, shot.truth[index], flags=0, confidence=0.0)
    accepted, fit = shot.run()
    assert accepted == 0 and fit.valid == 0 and why(fit) == "few_angles"
    for index in range(8):
        point = shot.point(index)
        assert point.filterHypothesis == HYP["unfiltered"]
        assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
            point.position.x,
            point.position.y,
            point.position.z,
        )


def test_scattered_angles_report_scatter(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    for index, truth in enumerate(shot.truth):
        sign = 1.0 if index % 2 else -1.0
        shot.measure(index, truth, az_err=sign * 25.0 * DEG, el_err=-sign * 25.0 * DEG)
    _, fit = shot.run()
    assert fit.valid == 0 and why(fit) == "scatter"
    assert all(shot.point(i).filterHypothesis == HYP["unfiltered"] for i in range(8))


def test_a_best_fit_on_the_search_limit_is_not_reported(lib):
    shot = Shot(lib, hla_deg=60.0, vla_deg=12.0)
    _, fit = shot.run()
    assert fit.valid == 0 and why(fit) == "grid_edge"


def test_points_short_of_the_tee_are_skipped(lib):
    """Frames before impact put the line behind the tee: no forward root."""
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0, first_frame=6, frames=10)
    accepted, fit = shot.run()
    assert fit.valid
    behind = [i for i, p in enumerate(shot.truth) if math.dist(p, (0, 0, 0)) <= math.dist(shot.tee, (0, 0, 0))]
    assert behind, "the setup must put a point short of the tee"
    for index in behind:
        assert shot.point(index).filterHypothesis == HYP["unfiltered"]
    assert accepted == 10 - len(behind)


def test_a_zero_confidence_angle_has_no_say(lib):
    clean = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    _, reference = clean.run()
    shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    shot.measure(3, shot.truth[3], az_err=30.0 * DEG, el_err=30.0 * DEG, confidence=0.0)
    _, fit = shot.run()
    assert (fit.hlaRad, fit.vlaRad) == (reference.hlaRad, reference.vlaRad)
    point = shot.point(3)
    assert point.filterAccepted == 0 and point.filterHypothesis == HYP["none"]


def test_a_point_with_one_angle_is_not_weighted(lib):
    shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    shot.measure(4, shot.truth[4], el_err=30.0 * DEG, flags=fw.ANGLE_ELEVATION)
    _, fit = shot.run()
    assert fit.valid and fit.used == 7
    assert shot.point(4).filterHypothesis == HYP["none"]


def test_a_zero_origin_has_no_tee(lib):
    """Review Focus 3: armed with no destination measured."""
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    accepted, fit = shot.run(origin=fw.Vec3(0.0, 0.0, 0.0))
    assert accepted == 0 and why(fit) == "no_tee"
    assert all(shot.point(i).filterHypothesis == HYP["unfiltered"] for i in range(8))


def test_the_search_stays_inside_its_evaluation_budget(lib):
    cfg = defaults(lib)
    assert lib.l3_ball_fit_max_evaluations(ctypes.byref(cfg)) == 2 * 3 * 11 * 11
    clean = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    _, fit = clean.run()
    assert fit.evaluations == 3 * 11 * 11, "nothing gated: the refit is skipped"
    wild = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    wild.measure(2, wild.truth[2], az_err=40.0 * DEG, el_err=40.0 * DEG)
    _, fit = wild.run()
    assert fit.evaluations == 2 * 3 * 11 * 11
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_fit.py -v`
Expected: FAIL/ERROR with `AttributeError: module ... has no attribute 'BallFitCfg'`.

- [ ] **Step 3: Write `l3_ball_fit.h`**

```c
/* IWR6843 ball direction fit: the ball's path from the tee, from its points'
 * ranges (trusted) and angles (not).
 *
 * Over the ~30 ms a ball track spans, gravity moves the ball ~3 mm, well under
 * a range bin, so the path is a straight line from the tee:
 * p(t) = tee + s(t) u(HLA, VLA). Each point's measured range fixes s (the
 * forward root of |tee + s u| = r), which leaves the direction's two angles
 * as the only unknowns. They are found by a coarse-to-fine grid search that
 * scores each point's measured direction against the direct return at p and
 * against its floor reflection (mirrored about the floor at radarHeightM),
 * the nearer counting, weighted by the point's angle confidence under a Huber
 * cap. A point further than gateK sigma from that fit is rejected and the
 * fit is redone without it. Too few kept points, too large a residual or a
 * best direction on the search limit gives no angles rather than a confident
 * wrong one.
 *
 * The tee: the ball track's origin gives the tee's slant range and bearing;
 * its height is teeBallHeightM above the floor, the antenna radarHeightM.
 *
 * Writes filteredPosition, filterAccepted and filterHypothesis onto every held
 * point: on the line when the fit is valid, unfiltered when not. Pure C, no
 * hardware, no allocation.
 */
#ifndef L3_BALL_FIT_H
#define L3_BALL_FIT_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_frames.h"

typedef struct {
    float    angleSigmaRad;   /* one point's angle scatter */
    float    gateK;           /* rejected beyond gateK sigma of the fit */
    float    huberK;          /* quadratic within huberK sigma, linear beyond */
    uint32_t minAccepted;     /* fewer kept angles: no direction */
    float    maxRmsRad;       /* kept points' RMS beyond this: scatter, no direction */
    float    imageSepMinRad;  /* direct and reflection closer than this: ambiguous */
    float    radarHeightM;    /* antenna centre above the floor */
    float    teeBallHeightM;  /* ball centre above the floor at rest */
    float    hlaMinRad;       /* search limits */
    float    hlaMaxRad;
    float    vlaMinRad;
    float    vlaMaxRad;
    uint32_t gridSteps;       /* intervals per axis per level (at most 16) */
    uint32_t gridLevels;      /* each level spans one step either side of the last best */
} l3_ball_fit_cfg_t;

enum {
    L3_BALL_FIT_WHY_NONE = 0,     /* not run */
    L3_BALL_FIT_WHY_OK,
    L3_BALL_FIT_WHY_FEW_ANGLES,   /* under minAccepted angles, before or after the gate */
    L3_BALL_FIT_WHY_SCATTER,      /* the kept angles' RMS exceeds maxRmsRad */
    L3_BALL_FIT_WHY_GRID_EDGE,    /* the best direction is on a search limit */
    L3_BALL_FIT_WHY_NO_TEE,       /* the origin gives no tee (range under the tee height) */
    L3_BALL_FIT_WHY_COUNT
};

typedef struct {
    float     hlaRad;         /* horizontal launch, positive right */
    float     vlaRad;         /* vertical launch, positive up */
    float     rmsRad;         /* kept points' weighted RMS angle residual */
    l3_vec3_t tee;            /* the anchor, golf frame */
    uint32_t  used;           /* points whose angles were weighed */
    uint32_t  accepted;       /* of those, kept by the gate */
    uint32_t  evaluations;    /* candidate directions scored */
    uint8_t   valid;
    uint8_t   why;            /* L3_BALL_FIT_WHY_* */
} l3_ball_fit_t;

void l3_ball_fit_cfg_defaults(l3_ball_fit_cfg_t *cfg);
/* The most candidate directions one run can score: two passes of every level. */
uint32_t l3_ball_fit_max_evaluations(const l3_ball_fit_cfg_t *cfg);
/* Unit vector for (HLA, VLA) in the golf frame. */
void l3_ball_fit_direction(float hlaRad, float vlaRad, l3_vec3_t *u);
/* Fit the core's held points from origin (the ball track's golf-frame origin)
 * and write each point's reconstruction. Returns the accepted count, 0 when
 * the fit is not valid (out->why says why). */
uint32_t l3_ball_fit_run(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                         l3_club_track_t *core, l3_ball_fit_t *out);
const char *l3_ball_fit_why_name(uint8_t why);

#endif /* L3_BALL_FIT_H */
```

- [ ] **Step 4: Write `l3_ball_fit.c`**

```c
/* IWR6843 ball direction fit. See l3_ball_fit.h. */
#include <math.h>
#include <string.h>

#include "l3_ball_fit.h"

#define L3_BALL_FIT_DEG (3.14159265F / 180.0F)
#define L3_BALL_FIT_MAX_STEPS 16U
#define L3_BALL_FIT_EDGE_RAD 1.0e-4F
#define L3_BALL_FIT_ANGLES (L3_OBS_ANGLE_AZIMUTH | L3_OBS_ANGLE_ELEVATION)

static const char *const kWhyNames[L3_BALL_FIT_WHY_COUNT] = {
    "none", "ok", "few_angles", "scatter", "grid_edge", "no_tee"
};

/* One held point, as the fit reads it. */
typedef struct {
    l3_vec3_t m;      /* measured direction (unit), golf frame */
    float     r;      /* measured range, bias removed */
    float     w;      /* the angle confidence; 0 leaves the point out */
    uint8_t   ahead;  /* beyond the tee: the line reaches its range going forward */
    uint8_t   use;    /* weighed in the current pass */
} l3_ball_fit_obs_t;

/* The line at one point's range, and the measured direction's squared angle
 * (small-angle, 2 (1 - cos)) from the direct return and from the reflection. */
typedef struct {
    l3_vec3_t p;
    l3_vec3_t q;        /* p mirrored about the floor */
    float     qNorm;
    float     directSq;
    float     imageSq;
} l3_ball_fit_pred_t;

static float l3_ball_fit_dot(const l3_vec3_t *a, const l3_vec3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

static uint32_t l3_ball_fit_steps(const l3_ball_fit_cfg_t *cfg)
{
    if (cfg->gridSteps < 1U) {
        return 1U;
    }
    return (cfg->gridSteps > L3_BALL_FIT_MAX_STEPS) ? L3_BALL_FIT_MAX_STEPS : cfg->gridSteps;
}

void l3_ball_fit_cfg_defaults(l3_ball_fit_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->angleSigmaRad = 12.0F * L3_BALL_FIT_DEG;  /* elevation SD, 2026-09-29 */
    cfg->gateK = 2.5F;
    cfg->huberK = 1.5F;
    cfg->minAccepted = 4U;
    cfg->maxRmsRad = 15.0F * L3_BALL_FIT_DEG;
    cfg->imageSepMinRad = 2.0F * L3_BALL_FIT_DEG;
    cfg->radarHeightM = 0.152F;                    /* calibration.py default */
    cfg->teeBallHeightM = 0.04F;                   /* a ball on a mat */
    cfg->hlaMinRad = -45.0F * L3_BALL_FIT_DEG;
    cfg->hlaMaxRad = 45.0F * L3_BALL_FIT_DEG;
    cfg->vlaMinRad = -10.0F * L3_BALL_FIT_DEG;
    cfg->vlaMaxRad = 60.0F * L3_BALL_FIT_DEG;
    cfg->gridSteps = 10U;
    cfg->gridLevels = 3U;
}

uint32_t l3_ball_fit_max_evaluations(const l3_ball_fit_cfg_t *cfg)
{
    uint32_t side = l3_ball_fit_steps(cfg) + 1U;

    return 2U * cfg->gridLevels * side * side;
}

void l3_ball_fit_direction(float hlaRad, float vlaRad, l3_vec3_t *u)
{
    float cv = cosf(vlaRad);

    u->x = cv * cosf(hlaRad);
    u->y = cv * sinf(hlaRad);
    u->z = sinf(vlaRad);
}

const char *l3_ball_fit_why_name(uint8_t why)
{
    return (why < L3_BALL_FIT_WHY_COUNT) ? kWhyNames[why] : "?";
}

/* The tee: the origin's slant range and bearing, at the ball's height. */
static int32_t l3_ball_fit_tee(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                               l3_vec3_t *tee)
{
    float range = sqrtf(l3_ball_fit_dot(origin, origin));
    float height = cfg->teeBallHeightM - cfg->radarHeightM;
    float ground;
    float bearing;

    if (!(range > ((height < 0.0F) ? -height : height))) {
        return 0;
    }
    ground = sqrtf(range * range - height * height);
    bearing = atan2f(origin->y, origin->x);
    tee->x = ground * cosf(bearing);
    tee->y = ground * sinf(bearing);
    tee->z = height;
    return 1;
}

static void l3_ball_fit_predict(float mirrorZ, const l3_vec3_t *tee, const l3_vec3_t *u,
                                float teeU, float teeSq, const l3_ball_fit_obs_t *o,
                                l3_ball_fit_pred_t *out)
{
    /* Forward root of s^2 + 2 s (tee.u) + |tee|^2 - r^2 = 0; o->r > |tee|
     * (ahead), so the root is real and non-negative. */
    float s = -teeU + sqrtf(teeU * teeU + o->r * o->r - teeSq);
    float qSq;

    out->p.x = tee->x + s * u->x;
    out->p.y = tee->y + s * u->y;
    out->p.z = tee->z + s * u->z;
    out->directSq = 2.0F * (1.0F - l3_ball_fit_dot(&out->p, &o->m) / o->r);
    out->q.x = out->p.x;
    out->q.y = out->p.y;
    out->q.z = mirrorZ - out->p.z;
    qSq = l3_ball_fit_dot(&out->q, &out->q);
    out->qNorm = (qSq > 0.0F) ? sqrtf(qSq) : 0.0F;
    out->imageSq = (out->qNorm > 0.0F)
                       ? 2.0F * (1.0F - l3_ball_fit_dot(&out->q, &o->m) / out->qNorm)
                       : 4.0F;
}

static float l3_ball_fit_nearer(const l3_ball_fit_pred_t *pred)
{
    return (pred->directSq <= pred->imageSq) ? pred->directSq : pred->imageSq;
}

/* Sum over the points in use of w * huber(angle / sigma). */
static float l3_ball_fit_cost(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *tee,
                              const l3_vec3_t *u, const l3_ball_fit_obs_t *obs, uint32_t n)
{
    float teeU = l3_ball_fit_dot(tee, u);
    float teeSq = l3_ball_fit_dot(tee, tee);
    float mirrorZ = -2.0F * cfg->radarHeightM;
    float invSigmaSq = 1.0F / (cfg->angleSigmaRad * cfg->angleSigmaRad);
    float kSq = cfg->huberK * cfg->huberK;
    float cost = 0.0F;
    l3_ball_fit_pred_t pred;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        float zSq;

        if (!obs[i].use) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, tee, u, teeU, teeSq, &obs[i], &pred);
        zSq = l3_ball_fit_nearer(&pred) * invSigmaSq;
        cost += obs[i].w *
                ((zSq <= kSq) ? 0.5F * zSq : cfg->huberK * sqrtf(zSq) - 0.5F * kSq);
    }
    return cost;
}

/* Coarse to fine over (HLA, VLA): each level a (steps+1)^2 grid, the next
 * spanning one step either side of the best so far, inside the limits.
 * Returns the evaluations made. */
static uint32_t l3_ball_fit_search(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *tee,
                                   const l3_ball_fit_obs_t *obs, uint32_t n, float *hla,
                                   float *vla)
{
    uint32_t steps = l3_ball_fit_steps(cfg);
    float hLo = cfg->hlaMinRad;
    float hHi = cfg->hlaMaxRad;
    float vLo = cfg->vlaMinRad;
    float vHi = cfg->vlaMaxRad;
    float best = 1.0e30F;
    float bestH = 0.5F * (hLo + hHi);
    float bestV = 0.5F * (vLo + vHi);
    uint32_t evaluations = 0U;
    uint32_t level;

    for (level = 0U; level < cfg->gridLevels; level++) {
        float hStep = (hHi - hLo) / (float)steps;
        float vStep = (vHi - vLo) / (float)steps;
        float cosH[L3_BALL_FIT_MAX_STEPS + 1U];
        float sinH[L3_BALL_FIT_MAX_STEPS + 1U];
        float cosV[L3_BALL_FIT_MAX_STEPS + 1U];
        float sinV[L3_BALL_FIT_MAX_STEPS + 1U];
        uint32_t a;
        uint32_t b;

        for (a = 0U; a <= steps; a++) {
            cosH[a] = cosf(hLo + (float)a * hStep);
            sinH[a] = sinf(hLo + (float)a * hStep);
            cosV[a] = cosf(vLo + (float)a * vStep);
            sinV[a] = sinf(vLo + (float)a * vStep);
        }
        for (a = 0U; a <= steps; a++) {
            for (b = 0U; b <= steps; b++) {
                l3_vec3_t u;
                float cost;

                u.x = cosV[b] * cosH[a];
                u.y = cosV[b] * sinH[a];
                u.z = sinV[b];
                cost = l3_ball_fit_cost(cfg, tee, &u, obs, n);
                evaluations++;
                if (cost < best) {
                    best = cost;
                    bestH = hLo + (float)a * hStep;
                    bestV = vLo + (float)b * vStep;
                }
            }
        }
        hLo = (bestH - hStep > cfg->hlaMinRad) ? bestH - hStep : cfg->hlaMinRad;
        hHi = (bestH + hStep < cfg->hlaMaxRad) ? bestH + hStep : cfg->hlaMaxRad;
        vLo = (bestV - vStep > cfg->vlaMinRad) ? bestV - vStep : cfg->vlaMinRad;
        vHi = (bestV + vStep < cfg->vlaMaxRad) ? bestV + vStep : cfg->vlaMaxRad;
    }
    *hla = bestH;
    *vla = bestV;
    return evaluations;
}

static uint32_t l3_ball_fit_fail(l3_club_track_t *core, l3_ball_fit_t *out, uint8_t why)
{
    l3_track_unfilter_all(core);
    out->valid = 0U;
    out->why = why;
    return 0U;
}

uint32_t l3_ball_fit_run(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                         l3_club_track_t *core, l3_ball_fit_t *out)
{
    l3_ball_fit_obs_t obs[L3_TRACK_POINTS];
    l3_ball_fit_pred_t pred;
    uint32_t n = core->count;
    float gateSq = cfg->gateK * cfg->angleSigmaRad * cfg->gateK * cfg->angleSigmaRad;
    float mirrorZ = -2.0F * cfg->radarHeightM;
    float sepMinSq = cfg->imageSepMinRad * cfg->imageSepMinRad;
    float teeRange;
    float teeU;
    float teeSq;
    float sumW = 0.0F;
    float sumWSq = 0.0F;
    l3_vec3_t u;
    uint32_t i;

    memset(out, 0, sizeof(*out));
    memset(obs, 0, sizeof(obs));
    l3_track_unfilter_all(core);
    if (!l3_ball_fit_tee(cfg, origin, &out->tee)) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_NO_TEE);
    }
    teeRange = sqrtf(l3_ball_fit_dot(&out->tee, &out->tee));
    for (i = 0U; i < n; i++) {
        const l3_track_point_t *point = l3_track_point_mut(core, i);
        float r = sqrtf(l3_ball_fit_dot(&point->position, &point->position));
        uint8_t both = (uint8_t)((point->anglesValid & L3_BALL_FIT_ANGLES) == L3_BALL_FIT_ANGLES);

        obs[i].r = r;
        obs[i].ahead = (uint8_t)(r > teeRange);
        if (!obs[i].ahead) {
            continue;  /* short of the tee: no forward root, stays unfiltered */
        }
        obs[i].m.x = point->position.x / r;
        obs[i].m.y = point->position.y / r;
        obs[i].m.z = point->position.z / r;
        obs[i].w = (both && point->angleConfidence > 0.0F) ? point->angleConfidence : 0.0F;
        obs[i].use = (uint8_t)(obs[i].w > 0.0F);
        out->used += obs[i].use;
    }
    if (out->used < cfg->minAccepted) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_FEW_ANGLES);
    }
    out->evaluations = l3_ball_fit_search(cfg, &out->tee, obs, n, &out->hlaRad, &out->vlaRad);
    /* The gate: an angle further than gateK sigma from the fit is not the
     * ball's direction, whatever made it. */
    l3_ball_fit_direction(out->hlaRad, out->vlaRad, &u);
    teeU = l3_ball_fit_dot(&out->tee, &u);
    teeSq = l3_ball_fit_dot(&out->tee, &out->tee);
    for (i = 0U; i < n; i++) {
        if (!obs[i].use) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, &out->tee, &u, teeU, teeSq, &obs[i], &pred);
        if (l3_ball_fit_nearer(&pred) > gateSq) {
            obs[i].use = 0U;
        } else {
            out->accepted++;
        }
    }
    if (out->accepted < cfg->minAccepted) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_FEW_ANGLES);
    }
    if (out->accepted < out->used) {
        /* Something was gated out: fit again without it. */
        out->evaluations += l3_ball_fit_search(cfg, &out->tee, obs, n, &out->hlaRad,
                                               &out->vlaRad);
        l3_ball_fit_direction(out->hlaRad, out->vlaRad, &u);
        teeU = l3_ball_fit_dot(&out->tee, &u);
    }
    for (i = 0U; i < n; i++) {
        l3_track_point_t *point = l3_track_point_mut(core, i);

        if (!obs[i].ahead) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, &out->tee, &u, teeU, teeSq, &obs[i], &pred);
        point->filteredPosition = pred.p;
        point->filterAccepted = obs[i].use;
        if (!obs[i].use) {
            point->filterHypothesis = L3_FILTER_HYP_NONE;
            continue;
        }
        sumW += obs[i].w;
        sumWSq += obs[i].w * l3_ball_fit_nearer(&pred);
        {
            float sepSq = (pred.qNorm > 0.0F)
                              ? 2.0F * (1.0F - l3_ball_fit_dot(&pred.p, &pred.q) /
                                                   (obs[i].r * pred.qNorm))
                              : 4.0F;

            if (sepSq < sepMinSq) {
                point->filterHypothesis = L3_FILTER_HYP_AMBIGUOUS;
            } else if (pred.directSq <= pred.imageSq) {
                point->filterHypothesis = L3_FILTER_HYP_DIRECT;
            } else {
                point->filterHypothesis = L3_FILTER_HYP_IMAGE;
            }
        }
    }
    out->rmsRad = sqrtf(sumWSq / sumW);
    if (out->hlaRad <= cfg->hlaMinRad + L3_BALL_FIT_EDGE_RAD ||
        out->hlaRad >= cfg->hlaMaxRad - L3_BALL_FIT_EDGE_RAD ||
        out->vlaRad <= cfg->vlaMinRad + L3_BALL_FIT_EDGE_RAD ||
        out->vlaRad >= cfg->vlaMaxRad - L3_BALL_FIT_EDGE_RAD) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_GRID_EDGE);
    }
    if (out->rmsRad > cfg->maxRmsRad) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_SCATTER);
    }
    out->valid = 1U;
    out->why = L3_BALL_FIT_WHY_OK;
    return out->accepted;
}
```

`l3_ball_fit_fail` keeps `hlaRad`, `vlaRad`, `rmsRad` and the counts for
diagnostics; `valid` is 0. The first loop holds `l3_track_point_mut`'s pointer
as `const` on purpose: that loop only reads.

- [ ] **Step 5: Register the module**

- `firmware/iwr6843/makefile:85`: add `l3_ball_fit.c` to `SOURCES` after
  `l3_ball_track.c`.
- `firmware_host.py` `HOST_SOURCES`: add `"l3_ball_fit.c",` after
  `"l3_ball_track.c",`.
- Add the mirrors and names:

```python
# l3_ball_fit.h
BALL_FIT_WHY_NAMES = ("none", "ok", "few_angles", "scatter", "grid_edge", "no_tee")


class BallFitCfg(ctypes.Structure):
    """``l3_ball_fit_cfg_t``."""

    _fields_ = [
        ("angleSigmaRad", ctypes.c_float),
        ("gateK", ctypes.c_float),
        ("huberK", ctypes.c_float),
        ("minAccepted", ctypes.c_uint32),
        ("maxRmsRad", ctypes.c_float),
        ("imageSepMinRad", ctypes.c_float),
        ("radarHeightM", ctypes.c_float),
        ("teeBallHeightM", ctypes.c_float),
        ("hlaMinRad", ctypes.c_float),
        ("hlaMaxRad", ctypes.c_float),
        ("vlaMinRad", ctypes.c_float),
        ("vlaMaxRad", ctypes.c_float),
        ("gridSteps", ctypes.c_uint32),
        ("gridLevels", ctypes.c_uint32),
    ]


class BallFit(ctypes.Structure):
    """``l3_ball_fit_t``."""

    _fields_ = [
        ("hlaRad", ctypes.c_float),
        ("vlaRad", ctypes.c_float),
        ("rmsRad", ctypes.c_float),
        ("tee", Vec3),
        ("used", ctypes.c_uint32),
        ("accepted", ctypes.c_uint32),
        ("evaluations", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
    ]
```

Signatures:

```python
    # l3_ball_fit.h
    "l3_ball_fit_cfg_defaults": ([_P(BallFitCfg)], None),
    "l3_ball_fit_max_evaluations": ([_P(BallFitCfg)], _U32),
    "l3_ball_fit_direction": ([_F32, _F32, _P(Vec3)], None),
    "l3_ball_fit_run": ([_P(BallFitCfg), _P(Vec3), _P(ClubTrack), _P(BallFit)], _U32),
    "l3_ball_fit_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
```

- [ ] **Step 6: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_fit.py -v`
Expected: all PASS. If `test_noisy_angles_fit_better_than_the_unconstrained_line`
misses its 3° bound, stop and report the measured `fit_rms`/`line_rms`. The
bound is an estimate; the relative claim is the requirement.

- [ ] **Step 7: Commit**

```bash
git add firmware/iwr6843/l3_ball_fit.h firmware/iwr6843/l3_ball_fit.c firmware/iwr6843/makefile src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_fit.py
git commit -m "feat(iwr6843): tee-anchored ball direction fit with floor-image and gating"
```

---

### Task 4: The ball track reads its direction from the fit

**Files:**
- Modify: `firmware/iwr6843/l3_ball_track.h`, `firmware/iwr6843/l3_ball_track.c` (cfg, defaults, `l3_ball_track_late_first` removed, `l3_ball_track_launch`, new `l3_ball_track_reconstruct`)
- Modify: `firmware/iwr6843/l3_launch.h`, `firmware/iwr6843/l3_launch.c`, `firmware/iwr6843/l3_joint_search.c:880`
- Modify: `src/openflight/iwr6843/firmware_host.py` (BallTrackCfg, Launch, signatures, remove `LAUNCH_NO_LATE`)
- Modify: `src/openflight/iwr6843/firmware_replay.py:345-346,996-997` (remove `late_range_m`), `src/openflight/iwr6843/tunables.py`
- Test: `tests/test_iwr6843_firmware_ball_track.py`, `tests/test_iwr6843_firmware_joint.py:355-371`, `tests/test_iwr6843_firmware_replay.py:1205-1216`, `tests/test_iwr6843_tunables.py`

**Interfaces:**
- Consumes: `l3_ball_fit_run`, `l3_ball_fit_direction`, `l3_ball_fit_cfg_t` (Task 3).
- Produces:
  - `l3_ball_track_cfg_t.fit` (`l3_ball_fit_cfg_t`) in place of `lateRangeM`.
  - `uint32_t l3_ball_track_reconstruct(l3_ball_track_t *track, l3_launch_t *launch)`, which returns the accepted count.
  - `l3_launch_t` loses `lateFrom` and gains `float angleRmsRad; uint8_t anglesAccepted; uint8_t angleWhy;`.
  - The launch format gains `angles=<n> rms=<deg> why=<name>` and loses `late=`.

- [ ] **Step 1: Write the failing tests**

In `tests/test_iwr6843_firmware_ball_track.py`:

1. In `Ball.__init__`, before `lib.l3_ball_track_init(...)`, add:

```python
        # These tests' ORIGIN sits at antenna height (z = 0): anchor the tee there.
        cfg.fit.teeBallHeightM = cfg.fit.radarHeightM
```

2. Replace `Ball.launch` with:

```python
    def launch(self):
        """The per-frame launch, then the once-per-shot direction fit, as the board does."""
        out = fw.Launch()
        used = self.lib.l3_ball_track_launch(ctypes.byref(self.track), ctypes.byref(out))
        self.lib.l3_ball_track_reconstruct(ctypes.byref(self.track), ctypes.byref(out))
        return used, out
```

3. Delete the late-window tests:
   `test_defaults_put_the_late_window_0p6_m_past_the_ball`,
   `test_launch_angles_come_from_the_late_points_when_the_early_ones_flip`,
   `test_the_speed_is_still_the_early_fit`,
   `test_a_short_flight_reports_speed_and_no_angles`,
   `test_the_late_window_is_measured_from_the_origin`. Also delete their helper
   `_set_point_angles` if nothing else uses it.
   `test_late_points_without_angles_give_no_angles` and
   `test_scattered_late_angles_are_still_rejected` stay, renamed to
   `test_points_without_angles_give_no_angles` and
   `test_scattered_angles_are_still_rejected`.
4. Add:

```python
def test_the_per_frame_launch_is_speed_only(lib):
    """l3_ball_track_launch runs every post frame: it must not fit a direction."""
    ball, _ = fly(lib, speed=60.0, hla_deg=3.0, vla_deg=12.0)
    out = fw.Launch()
    assert lib.l3_ball_track_launch(ctypes.byref(ball.track), ctypes.byref(out)) == 6
    assert out.speedValid and not out.hlaValid and not out.vlaValid
    assert out.angleWhy == 0, "none: the direction fit has not run"


def test_reconstruct_sets_the_direction_the_velocity_and_the_launch_position(lib):
    ball, (vx, vy, vz) = fly(lib, speed=60.0, hla_deg=3.0, vla_deg=12.0, frames=8)
    _, launch = ball.launch()
    assert launch.hlaValid and launch.vlaValid
    assert fw.BALL_FIT_WHY_NAMES[launch.angleWhy] == "ok"
    assert launch.anglesAccepted == 8, "fly() sets every point's angles"
    assert launch.angleRmsRad == pytest.approx(0.0, abs=0.01)
    assert launch.hlaRad / DEG == pytest.approx(3.0, abs=0.3)
    assert launch.vlaRad / DEG == pytest.approx(12.0, abs=0.3)
    speed = launch.speedMps
    assert (launch.velocity.x, launch.velocity.y, launch.velocity.z) == pytest.approx(
        (vx / 60.0 * speed, vy / 60.0 * speed, vz / 60.0 * speed), abs=0.3
    )
    assert (launch.launchPosition.x, launch.launchPosition.y, launch.launchPosition.z) == pytest.approx(
        ORIGIN, abs=1e-3
    )


def test_reconstruct_on_an_unconfirmed_track_leaves_everything_unfiltered(lib):
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, ORIGIN_BIN + 3.0)])
    out = fw.Launch()
    assert lib.l3_ball_track_reconstruct(ctypes.byref(ball.track), ctypes.byref(out)) == 0
    assert out.angleWhy == 0 and not out.hlaValid
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(ball.track.core), 0, ctypes.byref(point))
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED


def test_the_launch_line_reports_the_direction_fit(lib):
    ball, _ = fly(lib, speed=60.0, hla_deg=0.0, vla_deg=12.0, frames=8)
    _, launch = ball.launch()
    text = fw.c_text(lib.l3_launch_format, ctypes.byref(launch))
    assert " angles=8 rms=" in text and " why=ok valid=shv" in text
    assert "late=" not in text


def test_defaults_carry_the_direction_fit_constants(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    reference = fw.BallFitCfg()
    lib.l3_ball_fit_cfg_defaults(ctypes.byref(reference))
    assert bytes(cfg.fit) == bytes(reference)
```

`fly()` creates 6 frames by default; the `launch()` helper now fits over every
held point, so `test_launch_recovers_speed_hla_and_vla_with_the_documented_signs`
keeps working unchanged. Leave it.

In `tests/test_iwr6843_firmware_joint.py:355-371`, replace
`assert launch.lateFrom == fw.LAUNCH_NO_LATE` with
`assert launch.angleWhy == 0`, and change the docstring to "the joint launch
carries no direction fit (angleWhy none)".

In `tests/test_iwr6843_firmware_replay.py`:
- Delete `test_replay_late_range_reaches_the_ball_track`.
- Replace `test_synthetic_shot_late_flight_vla_matches_its_launch` with:

```python
def test_synthetic_shot_vla_is_read_back_by_the_direction_fit(lib):
    """End to end: the synthesized 12 deg launch is read back by the tee-anchored
    fit. The synthetic scene has no floor and its tee is at antenna height, so
    the anchor is put there too."""
    raw = synth_shot_dump(ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=TEE_RANGE_M, n_frames=24)
    config = ReplayConfig(
        tee_bin=TEE_BIN,
        dest_bin=TEE_BIN,
        overrides={"ball.fit.teeBallHeightM": 0.152, "ball.fit.radarHeightM": 0.152},
    )
    result = replay_dump(raw, config, lib=lib)
    assert result.launch is not None and result.launch.vla_deg is not None
    assert result.launch.vla_deg == pytest.approx(12.0, abs=2.0)
    assert result.launch.angle_why == "ok"
```

(`angle_why` arrives in Task 7. Until then this test fails on that one
attribute; it is listed in Task 7's run too.)

In `tests/test_iwr6843_tunables.py`, nothing to write. The parametrized
`test_every_tunable_is_a_field_of_its_struct` and the defaults-in-bounds test
cover the new registry entries (Step 5).

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py -v`
Expected: FAIL. `BallTrackCfg` has no `fit`; `l3_ball_track_reconstruct` is
undefined.

- [ ] **Step 3: C changes**

`l3_launch.h`: replace the `lateFrom` field, the `L3_LAUNCH_NO_LATE` define and
their comments with:

```c
    float     angleRmsRad;        /* the direction fit's RMS angle residual */
    uint8_t   speedValid;
    uint8_t   hlaValid;
    uint8_t   vlaValid;
    uint8_t   anglesAccepted;     /* ball points whose angles the direction fit kept */
    uint8_t   angleWhy;           /* L3_BALL_FIT_WHY_*: none until l3_ball_track_reconstruct */
} l3_launch_t;
```

The field order is `... residualM, confidence, angleRmsRad, speedValid,
hlaValid, vlaValid, anglesAccepted, angleWhy`. Update the format comment to
`"launch points=5 speed=61.20 radial=58.90 hla=1.20 vla=12.40 residualmm=... conf=... angles=6 rms=3.10 why=ok valid=shv"`.

`l3_launch.c`:
- Add `#include "l3_ball_fit.h"`.
- In `l3_launch_from_delivery`, delete `out->lateFrom = L3_LAUNCH_NO_LATE;`.
- In `l3_launch_format`, delete `lateText` and its block, add
  `char rmsText[16]; l3_text_degrees2(launch->angleRmsRad, rmsText, sizeof(rmsText));`,
  and make the format:

```c
    return snprintf(out, cap,
                    "launch points=%u speed=%s radial=%s hla=%s vla=%s residualmm=%s conf=%s "
                    "angles=%u rms=%s why=%s valid=%s",
                    (unsigned)launch->points, speedText, radialText, hlaText, vlaText,
                    residualText, confidenceText, (unsigned)launch->anglesAccepted, rmsText,
                    l3_ball_fit_why_name(launch->angleWhy), (v > 0U) ? valid : "none");
```

`l3_joint_search.c:880`: delete `out->lateFrom = L3_LAUNCH_NO_LATE;`. The
`memset` leaves `angleWhy` at `L3_BALL_FIT_WHY_NONE`.

`l3_ball_track.h`:
- Include `"l3_ball_fit.h"`.
- Replace the `lateRangeM` field and its comment with:

```c
    /* The ball's direction: the tee-anchored fit over every held point
     * (l3_ball_fit.h), once per shot by l3_ball_track_reconstruct. */
    l3_ball_fit_cfg_t fit;
```

- Replace the `l3_ball_track_launch` comment and add the new declaration:

```c
/* The launch SPEED from the earliest cfg.launchPoints confirmed points (at
 * least 3); cheap enough for every post frame. The direction is not fitted
 * here (hlaValid, vlaValid 0): l3_ball_track_reconstruct does that once.
 * Returns the points used, 0 when too few. */
uint32_t l3_ball_track_launch(const l3_ball_track_t *track, l3_launch_t *out);
/* Once per shot: fit the ball's direction from the tee (l3_ball_fit.h), write
 * every held point's reconstruction, and set launch's HLA/VLA (valid only when
 * the fit is), angle fields, launch position (the tee) and, with a valid speed,
 * its velocity along the fitted direction. An unconfirmed track leaves every
 * point unfiltered and the angles invalid. Returns the accepted angles, 0 when
 * the direction is not valid. */
uint32_t l3_ball_track_reconstruct(l3_ball_track_t *track, l3_launch_t *launch);
```

`l3_ball_track.c`:
- In the defaults, replace `cfg->lateRangeM = 0.6F; ...` with
  `l3_ball_fit_cfg_defaults(&cfg->fit);`.
- Delete `l3_ball_track_late_first`.
- Replace `l3_ball_track_launch`:

```c
uint32_t l3_ball_track_launch(const l3_ball_track_t *track, l3_launch_t *out)
{
    l3_delivery_t fit;
    uint32_t used;

    memset(out, 0, sizeof(*out));
    if (!track->confirmed) {
        return 0U;
    }
    used = l3_track_delivery_range(&track->core, 0U, track->cfg.launchPoints,
                                   track->cfg.launchPoints, &fit);
    if (used == 0U) {
        return 0U;
    }
    l3_launch_from_delivery(&fit, track->impactTimestampUs, out);
    /* The direction is l3_ball_track_reconstruct's, once per shot. */
    out->hlaValid = 0U;
    out->vlaValid = 0U;
    out->hlaRad = 0.0F;
    out->vlaRad = 0.0F;
    return used;
}

uint32_t l3_ball_track_reconstruct(l3_ball_track_t *track, l3_launch_t *launch)
{
    l3_ball_fit_t fit;
    l3_vec3_t u;

    launch->hlaValid = 0U;
    launch->vlaValid = 0U;
    launch->hlaRad = 0.0F;
    launch->vlaRad = 0.0F;
    launch->anglesAccepted = 0U;
    launch->angleRmsRad = 0.0F;
    launch->angleWhy = L3_BALL_FIT_WHY_NONE;
    if (!track->confirmed) {
        l3_track_unfilter_all(&track->core);
        return 0U;
    }
    (void)l3_ball_fit_run(&track->cfg.fit, &track->origin, &track->core, &fit);
    launch->anglesAccepted = (uint8_t)((fit.accepted > 0xFFU) ? 0xFFU : fit.accepted);
    launch->angleRmsRad = fit.rmsRad;
    launch->angleWhy = fit.why;
    if (!fit.valid) {
        return 0U;
    }
    launch->hlaRad = fit.hlaRad;
    launch->vlaRad = fit.vlaRad;
    launch->hlaValid = 1U;
    launch->vlaValid = 1U;
    launch->launchPosition = fit.tee;
    if (launch->speedValid) {
        l3_ball_fit_direction(fit.hlaRad, fit.vlaRad, &u);
        launch->velocity.x = launch->speedMps * u.x;
        launch->velocity.y = launch->speedMps * u.y;
        launch->velocity.z = launch->speedMps * u.z;
    }
    return fit.accepted;
}
```

- [ ] **Step 4: Python mirrors**

In `firmware_host.py`:
- `BallTrackCfg`: replace `("lateRangeM", ctypes.c_float)` with
  `("fit", BallFitCfg)`. Define `BallFitCfg` above `BallTrackCfg`, moving it if
  Task 3 put it later.
- `Launch`: after `("confidence", ctypes.c_float)`:

```python
        ("angleRmsRad", ctypes.c_float),
        ("speedValid", ctypes.c_uint8),
        ("hlaValid", ctypes.c_uint8),
        ("vlaValid", ctypes.c_uint8),
        ("anglesAccepted", ctypes.c_uint8),
        ("angleWhy", ctypes.c_uint8),
```

  (replacing the old four uint8 fields).
- Delete `LAUNCH_NO_LATE` and its comment.
- Signature:
  `"l3_ball_track_reconstruct": ([_P(BallTrack), _P(Launch)], _U32),`.

In `firmware_replay.py`: delete the `late_range_m` field (345-346) and the
two lines applying it (996-997).

In `tunables.py` `TUNABLES`, after the `ball` entries:

```python
    _t("ball", "fit.angleSigmaRad", "float", 0.02, 0.6, 0.02),
    _t("ball", "fit.gateK", "float", 1.0, 5.0, 0.25),
    _t("ball", "fit.minAccepted", "int", 2, 8, 1),
    _t("ball", "fit.maxRmsRad", "float", 0.05, 0.6, 0.05),
    _t("ball", "fit.radarHeightM", "float", 0.05, 0.6, 0.01),
    _t("ball", "fit.teeBallHeightM", "float", 0.0, 0.6, 0.01),
```

- [ ] **Step 5: Run the suites**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_joint.py tests/test_iwr6843_tunables.py tests/test_iwr6843_firmware_launch.py -v`
Expected: all PASS. If `test_iwr6843_firmware_launch.py` asserts the old format
string, update its expected line to the new format (`angles=0 rms=0.00 why=none`
for a launch the fit never touched).

Run: `uv run pytest tests/ -k "firmware or replay or dump_viewer" -q`
Expected: all PASS except `test_synthetic_shot_vla_is_read_back_by_the_direction_fit`
(needs `angle_why`, Task 7) and the board-wiring `lateFrom` test (Task 6).

- [ ] **Step 6: Commit**

```bash
git add firmware/iwr6843 src/openflight/iwr6843 tests/
git commit -m "feat(iwr6843): ball launch angles come from the tee-anchored fit, not the late window"
```

---

### Task 5: The club EKF, RTS smoother and filtered delivery (`l3_track_kf`)

**Files:**
- Create: `firmware/iwr6843/l3_track_kf.h`, `firmware/iwr6843/l3_track_kf.c`
- Modify: `firmware/iwr6843/l3_club_track.c` (the defaults call `l3_track_kf_cfg_defaults`; delete `l3_track_kf_defaults_inline`), `firmware/iwr6843/makefile`, `src/openflight/iwr6843/firmware_host.py`, `src/openflight/iwr6843/tunables.py`
- Test: `tests/test_iwr6843_firmware_track_kf.py` (new)

**Interfaces:**
- Consumes: `l3_track_kf_cfg_t`, `l3_track_point_mut`, `l3_track_unfilter_all`, `l3_track_newest_first`, `l3_delivery_fit`, `l3_track_point` (Task 2 and existing).
- Produces:
  - `void l3_track_kf_cfg_defaults(l3_track_kf_cfg_t *cfg)`.
  - `l3_track_kf_work_t` (caller-owned scratch, about 11 KB) and `uint32_t l3_track_kf_work_bytes(void)`.
  - `l3_track_kf_result_t { uint32_t points; uint32_t accepted; uint8_t why; }`, `L3_TRACK_KF_WHY_{NONE,OK,FEW_POINTS,DIVERGED,COUNT}`.
  - `uint32_t l3_track_kf_run(const l3_track_kf_cfg_t *cfg, l3_club_track_t *track, l3_track_kf_work_t *work, l3_track_kf_result_t *out)`.
  - `uint32_t l3_track_delivery_filtered(const l3_club_track_t *track, uint32_t maxPoints, l3_delivery_t *out)`.
  - `const char *l3_track_kf_why_name(uint8_t why)`.
  - Python: `fw.TrackKfResult`, `fw.TRACK_KF_WHY_NAMES = ("none", "ok", "few_points", "diverged")`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_iwr6843_firmware_track_kf.py`:

```python
"""Tests for the club's trajectory reconstruction, firmware/iwr6843/l3_track_kf.c.

A constant-velocity EKF over the club track's held points, then an RTS
smoother: range is trusted (small sigma), azimuth and elevation are weak
(large sigma, scaled by 1 / angle confidence) and pass a 2-dof chi-square
gate on their innovation; a point that fails still updates on its range. The
clubhead here swings on a 1.1 m arc at ~24 m/s, identity calibration.
"""

from __future__ import annotations

import ctypes
import math
import random

import pytest

from openflight.iwr6843 import firmware_host as fw

DEG = math.pi / 180.0
BIN_M = 6.0 / 128
FRAME_US = 3000
BOTH = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
HYP = {name: index for index, name in enumerate(fw.FILTER_HYP_NAMES)}
WHY = {name: index for index, name in enumerate(fw.TRACK_KF_WHY_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def arc(t_s: float) -> tuple[float, float, float]:
    """A clubhead on a 1.1 m arc, sweeping 0.6 rad in 27 ms toward the ball."""
    theta = -0.6 + 22.0 * t_s
    return (1.2 + 1.1 * math.sin(theta), 0.1 * math.sin(theta), 1.0 - 1.1 * math.cos(theta))


class Club:
    def __init__(self, lib, *, frames=10, start_us=FRAME_US, skip=(), step_us=FRAME_US, **kf):
        self.lib = lib
        cfg = fw.TrackCfg()
        lib.l3_track_cfg_defaults(ctypes.byref(cfg))
        cfg.maxAngleResidualM = 0.0
        for name, value in kf.items():
            setattr(cfg.kf, name, value)
        self.track = fw.ClubTrack()
        lib.l3_track_init(ctypes.byref(self.track), ctypes.byref(cfg))
        self.truth = []
        self.times = []
        for k in range(frames):
            if k in skip:
                continue
            stamp = (start_us + k * step_us) & 0xFFFFFFFF
            p = arc(k * step_us * 1e-6)
            point = fw.TrackPoint()
            point.frame = k
            point.timestampUs = stamp
            point.rangeM = math.sqrt(sum(c * c for c in p))
            point.rangeBin = point.rangeM / BIN_M
            lib.l3_track_append_point(ctypes.byref(self.track), ctypes.byref(point))
            self.truth.append(p)
            self.times.append(stamp)
        for index, p in enumerate(self.truth):
            self.measure(index, p)

    def measure(self, index, golf, *, az_err=0.0, el_err=0.0, range_err=0.0, confidence=1.0):
        radar = fw.Vec3()
        self.lib.l3_frames_golf_to_radar(
            ctypes.byref(self.track.cfg.cal), ctypes.byref(fw.Vec3(*golf)), ctypes.byref(radar)
        )
        sph = fw.Spherical()
        self.lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        if range_err:
            # Range noise: nudge the held ring slot's rangeM through the ctypes
            # mirror; set_point_angles below relocates the point from it.
            slot = (self.track.next + fw.TRACK_POINTS - self.track.count + index) % fw.TRACK_POINTS
            self.track.points[slot].rangeM = sph.rangeM + range_err
        assert (
            self.lib.l3_track_set_point_angles(
                ctypes.byref(self.track), index, sph.azimuthRad + az_err, sph.elevationRad + el_err, BOTH, confidence
            )
            == 1
        )

    def run(self, cfg=None):
        work = ctypes.create_string_buffer(self.lib.l3_track_kf_work_bytes())
        out = fw.TrackKfResult()
        accepted = self.lib.l3_track_kf_run(
            ctypes.byref(cfg or self.track.cfg.kf), ctypes.byref(self.track), work, ctypes.byref(out)
        )
        return accepted, out

    def point(self, index) -> fw.TrackPoint:
        point = fw.TrackPoint()
        assert self.lib.l3_track_point(ctypes.byref(self.track), index, ctypes.byref(point)) == 1
        return point

    def errors(self, attr):
        out = []
        for index, truth in enumerate(self.truth):
            v = getattr(self.point(index), attr)
            out.append(math.dist((v.x, v.y, v.z), truth))
        return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def test_defaults_come_from_one_place(lib):
    cfg = fw.TrackKfCfg()
    lib.l3_track_kf_cfg_defaults(ctypes.byref(cfg))
    track_cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
    assert bytes(track_cfg.kf) == bytes(cfg)
    assert cfg.accelSigmaMps2 == pytest.approx(1500.0)


def test_why_names_match_the_firmware(lib):
    for index, name in enumerate(fw.TRACK_KF_WHY_NAMES):
        assert lib.l3_track_kf_why_name(index).decode() == name


def test_the_work_area_fits_the_mss(lib):
    assert 0 < lib.l3_track_kf_work_bytes() <= 12 * 1024


def test_noise_free_points_are_reproduced(lib):
    club = Club(lib)
    accepted, out = club.run()
    assert out.why == WHY["ok"] and out.points == 10 and accepted == 9 == out.accepted
    assert max(club.errors("filteredPosition")) < 0.05
    assert club.point(0).filterHypothesis == HYP["none"], "the first point seeds the state"
    assert all(club.point(i).filterHypothesis == HYP["direct"] for i in range(1, 10))


def test_the_smoothed_track_beats_the_raw_angles(lib):
    """Fixed-seed: 10 deg angle noise, 1 cm range noise, 20 swings."""
    rng = random.Random(7)
    raw, smoothed = [], []
    for _ in range(20):
        club = Club(lib)
        for index, p in enumerate(club.truth):
            club.measure(
                index, p, az_err=rng.gauss(0, 10.0) * DEG, el_err=rng.gauss(0, 10.0) * DEG,
                range_err=rng.gauss(0, 0.01),
            )
        club.run()
        raw += club.errors("position")
        smoothed += club.errors("filteredPosition")
    assert rms(smoothed) < 0.5 * rms(raw), (rms(smoothed), rms(raw))


def test_an_angle_jump_is_gated_but_its_range_still_counts(lib):
    club = Club(lib)
    club.measure(5, club.truth[5], az_err=40.0 * DEG)
    club.run()
    jumped = club.point(5)
    assert jumped.filterAccepted == 0 and jumped.filterHypothesis == HYP["none"]
    f = jumped.filteredPosition
    assert math.dist((f.x, f.y, f.z), club.truth[5]) < 0.1
    assert math.hypot(f.x, f.y, f.z) == pytest.approx(jumped.rangeM, abs=0.02)
    assert all(club.point(i).filterAccepted == 1 for i in range(1, 10) if i != 5)


def test_a_gap_uses_the_real_time_step(lib):
    club = Club(lib, frames=12, skip=(5, 6))
    club.run()
    assert max(club.errors("filteredPosition")) < 0.08


def test_timestamps_wrapping_past_2_to_the_32_filter_the_same(lib):
    """Review Focus 2."""
    plain = Club(lib)
    plain.run()
    wrapped = Club(lib, start_us=2**32 - 4 * FRAME_US)
    assert wrapped.times[4] < wrapped.times[3], "the setup must wrap mid-track"
    wrapped.run()
    for index in range(10):
        a, b = plain.point(index).filteredPosition, wrapped.point(index).filteredPosition
        assert (a.x, a.y, a.z) == pytest.approx((b.x, b.y, b.z), abs=1e-4)


def test_a_repeated_timestamp_neither_crashes_nor_goes_nan(lib):
    """Review Focus 4."""
    club = Club(lib, frames=6)
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(club.track), 5, ctypes.byref(point))
    lib.l3_track_append_point(ctypes.byref(club.track), ctypes.byref(point))
    club.truth.append(club.truth[5])
    club.measure(6, club.truth[6])
    _, out = club.run()
    assert out.why == WHY["ok"]
    for index in range(7):
        f = club.point(index).filteredPosition
        assert all(math.isfinite(c) for c in (f.x, f.y, f.z))


def test_fewer_than_three_points_are_left_unfiltered(lib):
    club = Club(lib, frames=2)
    accepted, out = club.run()
    assert accepted == 0 and out.why == WHY["few_points"]
    assert all(club.point(i).filterHypothesis == HYP["unfiltered"] for i in range(2))


def test_a_degenerate_filter_resets_to_raw_without_nans(lib):
    """No process noise, no initial uncertainty and zero measurement variance:
    the first range update has S = 0, which is divergence, not a crash."""
    club = Club(lib, accelSigmaMps2=0.0, initPositionSigmaM=0.0, initVelocitySigmaMps=0.0, rangeSigmaM=0.0)
    accepted, out = club.run()
    assert accepted == 0 and out.why == WHY["diverged"]
    for index in range(10):
        point = club.point(index)
        assert point.filterHypothesis == HYP["unfiltered"]
        assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
            point.position.x,
            point.position.y,
            point.position.z,
        )


def test_the_filtered_delivery_reads_the_reconstruction(lib):
    rng = random.Random(11)
    raw_err, filtered_err = [], []
    for _ in range(20):
        club = Club(lib)
        for index, p in enumerate(club.truth):
            club.measure(index, p, az_err=rng.gauss(0, 10.0) * DEG, el_err=rng.gauss(0, 10.0) * DEG)
        club.run()
        # The delivery is a line over the newest 8 points (indices 2..9): their chord.
        start, end = arc(2 * FRAME_US * 1e-6), arc(9 * FRAME_US * 1e-6)
        truth_path = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        raw, filtered = fw.Delivery(), fw.Delivery()
        lib.l3_track_delivery(ctypes.byref(club.track), 8, ctypes.byref(raw))
        lib.l3_track_delivery_filtered(ctypes.byref(club.track), 8, ctypes.byref(filtered))
        if raw.pathValid:
            raw_err.append(math.degrees(raw.pathRad) - truth_path)
        assert filtered.pathValid
        filtered_err.append(math.degrees(filtered.pathRad) - truth_path)
    assert rms(filtered_err) < rms(raw_err)


def test_an_unfiltered_track_delivers_exactly_as_before(lib):
    club = Club(lib)
    raw, filtered = fw.Delivery(), fw.Delivery()
    lib.l3_track_delivery(ctypes.byref(club.track), 8, ctypes.byref(raw))
    lib.l3_track_delivery_filtered(ctypes.byref(club.track), 8, ctypes.byref(filtered))
    assert bytes(raw) == bytes(filtered)
```

`Club.measure` mutates the ctypes ring slot directly for range noise.
`fw.TRACK_POINTS` is the mirror's ring depth (32), already defined in
`firmware_host.py:236`.

The 0.05 m / 0.08 m position tolerances below allow for the constant-velocity
model on a curved arc: the 27 ms sweep has about 5 cm of sagitta. If a correct
implementation misses them, report the measured errors rather than retuning.

- [ ] **Step 2: Run to verify it fails**

Run: `uv run pytest tests/test_iwr6843_firmware_track_kf.py -v`
Expected: FAIL with `AttributeError: ... TrackKfResult`.

- [ ] **Step 3: Write `l3_track_kf.h`**

```c
/* IWR6843 club trajectory reconstruction: a constant-velocity EKF over the
 * club track's held points and an RTS smoother after it.
 *
 * State [x y z vx vy vz], golf frame. Each point is measured as the range,
 * azimuth and elevation of its raw golf-frame position: range trusted
 * (rangeSigmaM), the angles weak (angleSigmaRad / max(angleConfidence,
 * minAngleConfidence)) and applied only when their innovation passes a 2-dof
 * chi-square gate (chi2Gate). A point failing it still updates on its range,
 * so the range track is never lost. The prediction uses each point's own time
 * step, so frames the track coasted through are handled. The whole track is
 * available when this runs (at the fire, once), so the RTS smoother runs
 * over it and each point's filteredPosition is the smoothed one.
 *
 * Range rate is not a measurement: radialVelocityMps is derived from the same
 * ranges, and using both would count them twice.
 *
 * On too few points or a numerical failure every point is left unfiltered
 * (filteredPosition = position): never a NaN. Pure C, no allocation: the
 * caller owns the work area (static on the board).
 */
#ifndef L3_TRACK_KF_H
#define L3_TRACK_KF_H

#include <stdint.h>

#include "l3_club_track.h"

#define L3_TRACK_KF_STATES 6U

typedef struct {
    float   x[L3_TRACK_POINTS][L3_TRACK_KF_STATES];   /* filtered, then smoothed */
    float   P[L3_TRACK_POINTS][L3_TRACK_KF_STATES][L3_TRACK_KF_STATES];
    float   xp[L3_TRACK_POINTS][L3_TRACK_KF_STATES];  /* predicted from the point before */
    float   Pp[L3_TRACK_POINTS][L3_TRACK_KF_STATES][L3_TRACK_KF_STATES];
    float   dt[L3_TRACK_POINTS];                      /* seconds since the point before */
    uint8_t accepted[L3_TRACK_POINTS];
} l3_track_kf_work_t;

enum {
    L3_TRACK_KF_WHY_NONE = 0,
    L3_TRACK_KF_WHY_OK,
    L3_TRACK_KF_WHY_FEW_POINTS,   /* under three held points */
    L3_TRACK_KF_WHY_DIVERGED,     /* a non-positive variance or a non-finite state */
    L3_TRACK_KF_WHY_COUNT
};

typedef struct {
    uint32_t points;
    uint32_t accepted;            /* points whose angles updated the state */
    uint8_t  why;
} l3_track_kf_result_t;

void l3_track_kf_cfg_defaults(l3_track_kf_cfg_t *cfg);
uint32_t l3_track_kf_work_bytes(void);
/* Filter and smooth the held points; write filteredPosition, filterAccepted
 * and filterHypothesis (DIRECT when its angles were used, NONE when not,
 * UNFILTERED on failure). Returns the accepted count, 0 on failure. */
uint32_t l3_track_kf_run(const l3_track_kf_cfg_t *cfg, l3_club_track_t *track,
                         l3_track_kf_work_t *work, l3_track_kf_result_t *out);
/* l3_track_delivery over the reconstruction: each point's filteredPosition,
 * with its angles counted only when the filter accepted them. A point left
 * unfiltered reads exactly as l3_track_delivery reads it. */
uint32_t l3_track_delivery_filtered(const l3_club_track_t *track, uint32_t maxPoints,
                                    l3_delivery_t *out);
const char *l3_track_kf_why_name(uint8_t why);

#endif /* L3_TRACK_KF_H */
```

- [ ] **Step 4: Write `l3_track_kf.c`**

```c
/* IWR6843 club trajectory reconstruction. See l3_track_kf.h. */
#include <math.h>
#include <string.h>

#include "l3_track_kf.h"

#define N L3_TRACK_KF_STATES
#define L3_KF_PI 3.14159265F
#define L3_KF_MIN_RANGE_M 1.0e-3F
#define L3_KF_ANGLES (L3_OBS_ANGLE_AZIMUTH | L3_OBS_ANGLE_ELEVATION)

static const char *const kWhyNames[L3_TRACK_KF_WHY_COUNT] = {
    "none", "ok", "few_points", "diverged"
};

void l3_track_kf_cfg_defaults(l3_track_kf_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->accelSigmaMps2 = 1500.0F;   /* a clubhead on its arc: ~40 m/s at ~1.1 m radius */
    cfg->rangeSigmaM = 0.03F;
    cfg->angleSigmaRad = 15.0F * (L3_KF_PI / 180.0F);
    cfg->minAngleConfidence = 0.05F;
    cfg->chi2Gate = 9.21F;           /* 99 % for 2 degrees of freedom */
    cfg->initPositionSigmaM = 0.5F;
    cfg->initVelocitySigmaMps = 50.0F;
}

uint32_t l3_track_kf_work_bytes(void)
{
    return (uint32_t)sizeof(l3_track_kf_work_t);
}

const char *l3_track_kf_why_name(uint8_t why)
{
    return (why < L3_TRACK_KF_WHY_COUNT) ? kWhyNames[why] : "?";
}

static int32_t l3_kf_finite(float v)
{
    return (v == v) && v < 1.0e30F && v > -1.0e30F;
}

static float l3_kf_wrap(float a)
{
    while (a > L3_KF_PI) {
        a -= 2.0F * L3_KF_PI;
    }
    while (a < -L3_KF_PI) {
        a += 2.0F * L3_KF_PI;
    }
    return a;
}

static void l3_kf_symmetrize(float P[N][N])
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        for (j = i + 1U; j < N; j++) {
            float mean = 0.5F * (P[i][j] + P[j][i]);

            P[i][j] = mean;
            P[j][i] = mean;
        }
    }
}

/* Range, azimuth and elevation of a state's position, with their rows of H
 * (position columns; velocity columns 0). 0 at the origin or on the z axis. */
static int32_t l3_kf_measure(const float x[N], float z[3], float H[3][N])
{
    float rhoSq = x[0] * x[0] + x[1] * x[1];
    float rSq = rhoSq + x[2] * x[2];
    float rho = sqrtf(rhoSq);
    float r = sqrtf(rSq);

    memset(H, 0, 3U * N * sizeof(float));
    if (r < L3_KF_MIN_RANGE_M || rho < L3_KF_MIN_RANGE_M) {
        return 0;
    }
    z[0] = r;
    z[1] = atan2f(x[1], x[0]);
    z[2] = atan2f(x[2], rho);
    H[0][0] = x[0] / r;
    H[0][1] = x[1] / r;
    H[0][2] = x[2] / r;
    H[1][0] = -x[1] / rhoSq;
    H[1][1] = x[0] / rhoSq;
    H[2][0] = -x[0] * x[2] / (rSq * rho);
    H[2][1] = -x[1] * x[2] / (rSq * rho);
    H[2][2] = rho / rSq;
    return 1;
}

/* x, P forward by dt: F = [I dt*I; 0 I], white-acceleration Q per axis. */
static void l3_kf_predict(const l3_track_kf_cfg_t *cfg, float dt, const float x[N],
                          const float P[N][N], float xo[N], float Po[N][N])
{
    float q = cfg->accelSigmaMps2 * cfg->accelSigmaMps2;
    float dt2 = dt * dt;
    float FP[N][N];
    uint32_t a;
    uint32_t i;

    for (a = 0U; a < 3U; a++) {
        xo[a] = x[a] + dt * x[a + 3U];
        xo[a + 3U] = x[a + 3U];
    }
    for (i = 0U; i < N; i++) {
        for (a = 0U; a < 3U; a++) {
            FP[a][i] = P[a][i] + dt * P[a + 3U][i];
            FP[a + 3U][i] = P[a + 3U][i];
        }
    }
    for (i = 0U; i < N; i++) {
        for (a = 0U; a < 3U; a++) {
            Po[i][a] = FP[i][a] + dt * FP[i][a + 3U];
            Po[i][a + 3U] = FP[i][a + 3U];
        }
    }
    for (a = 0U; a < 3U; a++) {
        Po[a][a] += 0.25F * dt2 * dt2 * q;
        Po[a][a + 3U] += 0.5F * dt2 * dt * q;
        Po[a + 3U][a] += 0.5F * dt2 * dt * q;
        Po[a + 3U][a + 3U] += dt2 * q;
    }
}

/* One scalar measurement: innovation innov, row H, variance R. 0 when the
 * innovation variance is not positive (divergence). */
static int32_t l3_kf_update1(float x[N], float P[N][N], const float H[N], float innov, float R)
{
    float PH[N];
    float S = R;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        PH[i] = 0.0F;
        for (j = 0U; j < N; j++) {
            PH[i] += P[i][j] * H[j];
        }
        S += H[i] * PH[i];
    }
    if (!(S > 0.0F)) {
        return 0;
    }
    for (i = 0U; i < N; i++) {
        x[i] += PH[i] / S * innov;
    }
    for (i = 0U; i < N; i++) {
        for (j = 0U; j < N; j++) {
            P[i][j] -= PH[i] * PH[j] / S;
        }
    }
    l3_kf_symmetrize(P);
    return 1;
}

/* The angle pair's chi-square nu' S^-1 nu, S = H P H' + var I (2x2). -1 when
 * S is singular. */
static float l3_kf_angle_chi2(const float P[N][N], float H[3][N], const float nu[2],
                              float var)
{
    float PH1[N];
    float PH2[N];
    float s11 = var;
    float s22 = var;
    float s12 = 0.0F;
    float det;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        PH1[i] = 0.0F;
        PH2[i] = 0.0F;
        for (j = 0U; j < N; j++) {
            PH1[i] += P[i][j] * H[1][j];
            PH2[i] += P[i][j] * H[2][j];
        }
    }
    for (i = 0U; i < N; i++) {
        s11 += H[1][i] * PH1[i];
        s22 += H[2][i] * PH2[i];
        s12 += H[1][i] * PH2[i];
    }
    det = s11 * s22 - s12 * s12;
    if (!(det > 0.0F)) {
        return -1.0F;
    }
    return (s22 * nu[0] * nu[0] - 2.0F * s12 * nu[0] * nu[1] + s11 * nu[1] * nu[1]) / det;
}

/* A y = b for symmetric positive-definite A by Cholesky; 0 when A is not. */
static int32_t l3_kf_solve(const float A[N][N], const float b[N], float y[N])
{
    float L[N][N];
    float t[N];
    int32_t i;
    int32_t j;
    int32_t k;

    memset(L, 0, sizeof(L));
    for (i = 0; i < (int32_t)N; i++) {
        for (j = 0; j <= i; j++) {
            float sum = A[i][j];

            for (k = 0; k < j; k++) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                if (!(sum > 0.0F)) {
                    return 0;
                }
                L[i][i] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }
    for (i = 0; i < (int32_t)N; i++) {
        float sum = b[i];

        for (k = 0; k < i; k++) {
            sum -= L[i][k] * t[k];
        }
        t[i] = sum / L[i][i];
    }
    for (i = (int32_t)N - 1; i >= 0; i--) {
        float sum = t[i];

        for (k = i + 1; k < (int32_t)N; k++) {
            sum -= L[k][i] * y[k];
        }
        y[i] = sum / L[i][i];
    }
    return 1;
}

static uint32_t l3_kf_fail(l3_club_track_t *track, l3_track_kf_result_t *out, uint8_t why)
{
    l3_track_unfilter_all(track);
    out->accepted = 0U;
    out->why = why;
    return 0U;
}

/* The point's range, then (if it has both angles and they pass the gate) its
 * azimuth and elevation, each a scalar update relinearised at the latest
 * state. Returns -1 on divergence, else whether the angles were used. */
static int32_t l3_kf_update(const l3_track_kf_cfg_t *cfg, const l3_track_point_t *point,
                            float x[N], float P[N][N])
{
    float measured[N] = {0};
    float zm[3];
    float z[3];
    float H[3][N];
    float Hm[3][N];
    float nu[2];
    float confidence;
    float var;
    float chi2;

    measured[0] = point->position.x;
    measured[1] = point->position.y;
    measured[2] = point->position.z;
    if (!l3_kf_measure(measured, zm, Hm) || !l3_kf_measure(x, z, H)) {
        return -1;
    }
    if (!l3_kf_update1(x, P, H[0], zm[0] - z[0], cfg->rangeSigmaM * cfg->rangeSigmaM)) {
        return -1;
    }
    if ((point->anglesValid & L3_KF_ANGLES) != L3_KF_ANGLES || !(point->angleConfidence > 0.0F)) {
        return 0;
    }
    if (!l3_kf_measure(x, z, H)) {
        return -1;
    }
    confidence = (point->angleConfidence > cfg->minAngleConfidence) ? point->angleConfidence
                                                                    : cfg->minAngleConfidence;
    var = (cfg->angleSigmaRad / confidence) * (cfg->angleSigmaRad / confidence);
    nu[0] = l3_kf_wrap(zm[1] - z[1]);
    nu[1] = zm[2] - z[2];
    chi2 = l3_kf_angle_chi2(P, H, nu, var);
    if (chi2 < 0.0F) {
        return -1;
    }
    if (chi2 > cfg->chi2Gate) {
        return 0;
    }
    if (!l3_kf_update1(x, P, H[1], nu[0], var) || !l3_kf_measure(x, z, H) ||
        !l3_kf_update1(x, P, H[2], zm[2] - z[2], var)) {
        return -1;
    }
    return 1;
}

uint32_t l3_track_kf_run(const l3_track_kf_cfg_t *cfg, l3_club_track_t *track,
                         l3_track_kf_work_t *work, l3_track_kf_result_t *out)
{
    uint32_t n = track->count;
    uint32_t k;
    uint32_t i;

    memset(out, 0, sizeof(*out));
    out->points = n;
    if (n < 3U) {
        return l3_kf_fail(track, out, L3_TRACK_KF_WHY_FEW_POINTS);
    }
    for (k = 0U; k < n; k++) {
        const l3_track_point_t *point = l3_track_point_mut(track, k);

        if (k == 0U) {
            /* Seed from the first point: its position (whatever angles it has)
             * with a wide uncertainty, at rest. */
            float sp = cfg->initPositionSigmaM * cfg->initPositionSigmaM;
            float sv = cfg->initVelocitySigmaMps * cfg->initVelocitySigmaMps;

            memset(work->x[0], 0, sizeof(work->x[0]));
            memset(work->P[0], 0, sizeof(work->P[0]));
            work->x[0][0] = point->position.x;
            work->x[0][1] = point->position.y;
            work->x[0][2] = point->position.z;
            for (i = 0U; i < 3U; i++) {
                work->P[0][i][i] = sp;
                work->P[0][i + 3U][i + 3U] = sv;
            }
            memcpy(work->xp[0], work->x[0], sizeof(work->x[0]));
            memcpy(work->Pp[0], work->P[0], sizeof(work->P[0]));
            work->dt[0] = 0.0F;
            work->accepted[0] = 0U;
            continue;
        } else {
            const l3_track_point_t *before = l3_track_point_mut(track, k - 1U);
            /* int32 difference: a wrap of the microsecond clock is one step. */
            float dt = (float)(int32_t)(point->timestampUs - before->timestampUs) * 1.0e-6F;
            int32_t used;

            work->dt[k] = (dt > 0.0F) ? dt : 0.0F;
            l3_kf_predict(cfg, work->dt[k], work->x[k - 1U], work->P[k - 1U], work->xp[k],
                          work->Pp[k]);
            memcpy(work->x[k], work->xp[k], sizeof(work->x[k]));
            memcpy(work->P[k], work->Pp[k], sizeof(work->P[k]));
            used = l3_kf_update(cfg, point, work->x[k], work->P[k]);
            if (used < 0) {
                return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
            }
            work->accepted[k] = (uint8_t)used;
            out->accepted += (uint32_t)used;
        }
    }
    /* RTS, positions only: x_k += P_k F' Pp_{k+1}^-1 (xs_{k+1} - xp_{k+1}). */
    for (k = n - 1U; k-- > 0U;) {
        float d[N];
        float y[N];
        float g[N];

        for (i = 0U; i < N; i++) {
            d[i] = work->x[k + 1U][i] - work->xp[k + 1U][i];
        }
        if (!l3_kf_solve(work->Pp[k + 1U], d, y)) {
            return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
        }
        for (i = 0U; i < 3U; i++) {
            g[i] = y[i];
            g[i + 3U] = work->dt[k + 1U] * y[i] + y[i + 3U];
        }
        for (i = 0U; i < N; i++) {
            uint32_t j;

            for (j = 0U; j < N; j++) {
                work->x[k][i] += work->P[k][i][j] * g[j];
            }
        }
    }
    for (k = 0U; k < n; k++) {
        if (!l3_kf_finite(work->x[k][0]) || !l3_kf_finite(work->x[k][1]) ||
            !l3_kf_finite(work->x[k][2])) {
            return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
        }
    }
    for (k = 0U; k < n; k++) {
        l3_track_point_t *point = l3_track_point_mut(track, k);

        point->filteredPosition.x = work->x[k][0];
        point->filteredPosition.y = work->x[k][1];
        point->filteredPosition.z = work->x[k][2];
        point->filterAccepted = work->accepted[k];
        point->filterHypothesis = work->accepted[k] ? L3_FILTER_HYP_DIRECT : L3_FILTER_HYP_NONE;
    }
    out->why = L3_TRACK_KF_WHY_OK;
    return out->accepted;
}

static int32_t l3_track_filtered_at(const void *ctx, uint32_t index, l3_track_point_t *out)
{
    if (!l3_track_point((const l3_club_track_t *)ctx, index, out)) {
        return 0;
    }
    if (out->filterHypothesis != L3_FILTER_HYP_UNFILTERED) {
        out->position = out->filteredPosition;
        if (!out->filterAccepted) {
            out->anglesValid = 0U;
        }
    }
    return 1;
}

uint32_t l3_track_delivery_filtered(const l3_club_track_t *track, uint32_t maxPoints,
                                    l3_delivery_t *out)
{
    uint32_t first = l3_track_newest_first(track, maxPoints);

    memset(out, 0, sizeof(*out));
    return l3_delivery_fit(l3_track_filtered_at, track, first, track->count, L3_TRACK_FULL_POINTS,
                           track->cfg.binWidthM, track->cfg.maxAngleResidualM, out);
}
```

Implementation notes to keep:

- `l3_kf_update` returns -1 when the raw position sits at the origin or on the
  z axis (`l3_kf_measure` 0). That is divergence by definition: no range can be
  measured there.
- The first point seeds the state and is never "accepted". The noise-free test
  expects `accepted == n - 1`.
- The finiteness check runs over the smoothed positions only. Every earlier
  non-finite value surfaces there, since `S > 0` comparisons are false for NaN.
- `test_an_unfiltered_track_delivers_exactly_as_before`: with every point
  unfiltered, `l3_track_filtered_at` returns the raw point, and
  `l3_track_delivery` calls the same `l3_delivery_fit` over the same range, so
  the bytes match.

In `l3_club_track.c`: include `"l3_track_kf.h"`, replace the
`l3_track_kf_defaults_inline(&cfg->kf);` call with
`l3_track_kf_cfg_defaults(&cfg->kf);`, and delete `l3_track_kf_defaults_inline`.

- [ ] **Step 5: Register the module**

- `makefile:85` `SOURCES`: add `l3_track_kf.c` after `l3_club_track.c`.
- `HOST_SOURCES`: add `"l3_track_kf.c",` after `"l3_club_track.c",`.
- `firmware_host.py`:

```python
# l3_track_kf.h
TRACK_KF_WHY_NAMES = ("none", "ok", "few_points", "diverged")


class TrackKfResult(ctypes.Structure):
    """``l3_track_kf_result_t``."""

    _fields_ = [
        ("points", ctypes.c_uint32),
        ("accepted", ctypes.c_uint32),
        ("why", ctypes.c_uint8),
    ]
```

Signatures:

```python
    # l3_track_kf.h
    "l3_track_kf_cfg_defaults": ([_P(TrackKfCfg)], None),
    "l3_track_kf_work_bytes": ([], _U32),
    "l3_track_kf_run": ([_P(TrackKfCfg), _P(ClubTrack), ctypes.c_void_p, _P(TrackKfResult)], _U32),
    "l3_track_delivery_filtered": ([_P(ClubTrack), _U32, _P(Delivery)], _U32),
    "l3_track_kf_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
```

- `tunables.py`:

```python
    _t("club", "kf.accelSigmaMps2", "float", 100.0, 5000.0, 100.0),
    _t("club", "kf.rangeSigmaM", "float", 0.005, 0.1, 0.005),
    _t("club", "kf.angleSigmaRad", "float", 0.02, 0.6, 0.02),
    _t("club", "kf.chi2Gate", "float", 2.0, 20.0, 1.0),
```

- [ ] **Step 6: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_track_kf.py tests/test_iwr6843_firmware_club_track.py tests/test_iwr6843_tunables.py -v`
Expected: all PASS. If `test_the_smoothed_track_beats_the_raw_angles` or
`test_the_filtered_delivery_reads_the_reconstruction` fails with a correct
implementation, report the measured RMS values rather than changing the bound.

- [ ] **Step 7: Commit**

```bash
git add firmware/iwr6843/l3_track_kf.h firmware/iwr6843/l3_track_kf.c firmware/iwr6843/l3_club_track.c firmware/iwr6843/makefile src/openflight/iwr6843/firmware_host.py src/openflight/iwr6843/tunables.py tests/test_iwr6843_firmware_track_kf.py
git commit -m "feat(iwr6843): club EKF with angle gating, RTS smoother and a filtered delivery"
```

---

### Task 6: Board wiring and the `reconstruct` profile stage

**Files:**
- Modify: `firmware/iwr6843/l3_profile.h:18-29`, `firmware/iwr6843/l3_profile.c:7-8,73-85`
- Modify: `firmware/iwr6843/l3_dump.c` (includes; `gClubKfWork`; the fire path near 4112-4116; RESULT near 3765; the two `gLaunch.lateFrom` lines at 3368, 3447)
- Modify: `src/openflight/iwr6843/firmware_host.py` (`PROFILE_STAGE_NAMES`)
- Test: `tests/test_iwr6843_firmware_profile.py`, `tests/test_iwr6843_firmware_board_wiring.py`

**Interfaces:**
- Consumes: `l3_track_kf_run`, `l3_track_delivery_filtered` (Task 5), `l3_ball_track_reconstruct` (Task 4).
- Produces: `L3_PROF_RECONSTRUCT` (index 8, before `L3_PROF_DSP_WAIT`), stage name `"reconstruct"`, left out of `l3_profile_frame_us`.

- [ ] **Step 1: Write the failing tests**

In `tests/test_iwr6843_firmware_profile.py`, add:

```python
def test_reconstruct_is_a_stage_but_not_a_per_frame_cost(lib):
    """It runs once per shot (the fire, RESULT): its mean would inflate the
    per-frame budget the MSS reports."""
    assert fw.PROFILE_STAGE_NAMES.index("reconstruct") == 8
    assert fw.PROFILE_STAGE_NAMES[-1] == "dspwait"
    profile = fw.Profile()
    lib.l3_profile_init(ctypes.byref(profile), 200)
    lib.l3_profile_add(ctypes.byref(profile), STAGE["residual"], 200 * 300)
    lib.l3_profile_add(ctypes.byref(profile), STAGE["reconstruct"], 200 * 900)
    lib.l3_profile_frame(ctypes.byref(profile))
    assert lib.l3_profile_frame_us(ctypes.byref(profile)) == 300
```

(Match the struct and fixture names already used in that file, e.g. `fw.Profile`
and `lib`. Read its top before writing.)

In `tests/test_iwr6843_firmware_board_wiring.py`, replace
`test_every_launch_reset_carries_the_no_late_sentinel` with:

```python
def test_no_launch_reset_names_the_removed_late_window():
    assert "lateFrom" not in SOURCE and "L3_LAUNCH_NO_LATE" not in SOURCE


def test_the_fire_reconstructs_the_club_before_its_frozen_delivery():
    """After the angle drain the club is reconstructed once and the delivery
    the shot machine freezes is the filtered one."""
    flat = " ".join(SOURCE.split())
    drain = flat.index("l3_angleQueueDrain();")
    run = flat.index("l3_track_kf_run(&gClubTrack.cfg.kf, &gClubTrack, &gClubKfWork,", drain)
    filtered = flat.index("l3_track_delivery_filtered(&gClubTrack, 8U, &gDelivery);", run)
    observe = flat.index("l3_shotObserve(teeBin, fired, impactUs);", filtered)
    assert drain < run < filtered < observe


def test_result_reconstructs_the_ball_once_before_building_the_result():
    flat = " ".join(SOURCE.split())
    gate = flat.index("if (gShot.state == L3_SHOT_RESULT && !gShotResultReady) {")
    fit = flat.index("l3_ball_track_reconstruct(&gBallTrack, &gLaunch);", gate)
    build = flat.index("l3_result_build(&gShot, &gBallTrack, &gLaunch,", gate)
    assert gate < fit < build


def test_both_reconstructions_are_profiled():
    assert SOURCE.count("l3_profileStage(L3_PROF_RECONSTRUCT,") == 2


def test_the_per_frame_paths_do_not_reconstruct():
    """Each reconstruction has exactly one call site, the two once-per-shot ones above."""
    assert SOURCE.count("l3_ball_track_reconstruct(") == 1
    assert SOURCE.count("l3_track_kf_run(") == 1
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_profile.py tests/test_iwr6843_firmware_board_wiring.py -v`
Expected: FAIL (no `reconstruct` stage; the wiring strings are missing).

- [ ] **Step 3: Profile stage**

`l3_profile.h`: insert before `L3_PROF_DSP_WAIT`:

```c
    L3_PROF_RECONSTRUCT,  /* once per shot: the club EKF at the fire, the ball fit at RESULT */
```

`l3_profile.c`: in `kStageNames`, insert `"reconstruct"` before `"dspwait"`. In
`l3_profile_frame_us`:

```c
        if (stage == L3_PROF_DSP_WAIT || stage == L3_PROF_RECONSTRUCT) {
            continue; /* dspwait is inside residual; reconstruct is once per shot */
        }
```

Update the header comment's list of stages to mention `reconstruct`.
`firmware_host.py` `PROFILE_STAGE_NAMES`: insert `"reconstruct",` before
`"dspwait",`.

- [ ] **Step 4: Board wiring in `l3_dump.c`**

1. Includes: add `#include "l3_track_kf.h"` beside `#include "l3_club_track.h"`.
2. Statics, beside `gDelivery`:

```c
/* The club reconstruction's scratch (l3_track_kf.h), ~11 KB: static, used
 * once per shot at the fire. */
static l3_track_kf_work_t gClubKfWork;
```

3. The fire path (near 4112-4116), replacing the two lines after the comment
   "Then every pending club angle...":

```c
        /* Then every pending club angle, so the shot freezes them with the
         * trajectory; the club reconstructed from them once (l3_track_kf.h),
         * and the delivery again from that. */
        l3_angleQueueDrain();
        {
            uint32_t reconstructTicks = Cycleprofiler_getTimeStamp();
            l3_track_kf_result_t kfResult;

            (void)l3_track_kf_run(&gClubTrack.cfg.kf, &gClubTrack, &gClubKfWork, &kfResult);
            (void)l3_track_delivery_filtered(&gClubTrack, 8U, &gDelivery);
            l3_profileStage(L3_PROF_RECONSTRUCT, reconstructTicks);
        }
```

4. RESULT (near 3765):

```c
    if (gShot.state == L3_SHOT_RESULT && !gShotResultReady) {
        uint32_t reconstructTicks = Cycleprofiler_getTimeStamp();

        /* Once per shot: the ball's direction from the tee (l3_ball_fit.h). */
        (void)l3_ball_track_reconstruct(&gBallTrack, &gLaunch);
        l3_profileStage(L3_PROF_RECONSTRUCT, reconstructTicks);
        l3_impactFitRun();
        l3_result_build(&gShot, &gBallTrack, &gLaunch, &gImpactFit, ++gShotId, gTrigDestBall,
                        &gShotResult);
        gShotResultReady = 1U;
    }
```

5. Delete both `gLaunch.lateFrom = L3_LAUNCH_NO_LATE;` lines (3368, 3447).

`l3_profileStage` is a static defined at `l3_dump.c:3378`. The fire path must be
below that line; confirm it is (4112 > 3378). If the RESULT block (3765) is
above any forward declaration it needs, nothing changes, since the function
is defined earlier.

- [ ] **Step 5: Run the suites**

Run: `uv run pytest tests/test_iwr6843_firmware_profile.py tests/test_iwr6843_firmware_board_wiring.py tests/test_iwr6843_firmware_board_image.py tests/test_iwr6843_firmware_sparse.py -v`
Expected: all PASS. If `test_iwr6843_firmware_profile.py` enumerates every
stage name (it loops `PROFILE_STAGE_NAMES` at ~85), it picks up `reconstruct`
automatically. If it asserts `index("dspwait") == 8`, update that to 9.

- [ ] **Step 6: Build the board image when the TI toolchain is present**

Run the firmware build as `docs/development/firmware.md` describes, and read the
MSS memory usage the linker reports. `gClubKfWork` adds about 11 KB of `.bss`.
If the MSS image no longer fits, stop and report the linker's numbers. The fix
(fewer smoothed points, or symmetric storage) is a design decision. If the
toolchain is not installed, skip this step and say so in the task report.

- [ ] **Step 7: Commit**

```bash
git add firmware/iwr6843/l3_profile.h firmware/iwr6843/l3_profile.c firmware/iwr6843/l3_dump.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_profile.py tests/test_iwr6843_firmware_board_wiring.py
git commit -m "feat(iwr6843): board reconstructs the club at the fire and the ball at RESULT, profiled"
```

---

### Task 7: Replay: reconstruct as the board does, and carry it out

**Files:**
- Modify: `src/openflight/iwr6843/firmware_replay.py` (`PointSummary` ~543, `LaunchSummary` ~382, `_point_summary` ~783, `_launch_summary` ~691, `replay_dump` near 935/1001/1360/1442)
- Test: `tests/test_iwr6843_firmware_replay.py`

**Interfaces:**
- Consumes: `l3_track_kf_run`, `l3_track_delivery_filtered`, `l3_ball_track_reconstruct`, `l3_track_find_point`, `fw.FILTER_HYP_NAMES`, `fw.BALL_FIT_WHY_NAMES`.
- Produces:
  - `PointSummary` gains `angle_confidence: float = 0.0`, `filtered_position: tuple[float, float, float] | None = None`, `filter_accepted: bool = False`, `filter_hypothesis: str = "unfiltered"`.
  - `LaunchSummary` gains `angles_accepted: int = 0`, `angle_rms_deg: float | None = None`, `angle_why: str = "none"`.
  - The JSON the viewer gets (`dump_viewer._jsonable`) carries these names.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_firmware_replay.py`:

```python
def test_a_replayed_shot_carries_both_reconstructions(lib):
    raw = synth_shot_dump(ball_speed_ms=60.0, vla_deg=12.0, hla_deg=2.0, tee_range_m=TEE_RANGE_M, n_frames=24)
    config = ReplayConfig(
        tee_bin=TEE_BIN,
        dest_bin=TEE_BIN,
        overrides={"ball.fit.teeBallHeightM": 0.152, "ball.fit.radarHeightM": 0.152},
    )
    result = replay_dump(raw, config, lib=lib)
    assert result.launch is not None
    assert result.launch.angle_why == "ok" and result.launch.angles_accepted >= 4
    assert result.launch.angle_rms_deg is not None
    fitted_ball = [p for p in result.ball_points if p.filtered_position is not None]
    assert len(fitted_ball) >= 4
    assert all(p.filter_hypothesis in fw.FILTER_HYP_NAMES for p in result.ball_points)
    fitted_club = [p for p in result.points if p.filtered_position is not None]
    assert len(fitted_club) >= 3, "the club is reconstructed over its held points"
    assert any(p.angle_confidence > 0.0 for p in result.points)


def test_the_fire_frame_reports_the_filtered_delivery(lib, swing, monkeypatch):
    """At the fire the board drains the angles, reconstructs the club once and
    freezes the filtered delivery; the replay does the same on the fired frame."""
    calls = []
    original_run = lib.l3_track_kf_run

    def spy(*args):
        calls.append("kf")
        return original_run(*args)

    monkeypatch.setattr(lib, "l3_track_kf_run", spy)
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.fired_frame is not None
    fired = result.frames[result.fired_frame]
    assert fired.delivery is not None and fired.delivery.points > 0
    assert calls.count("kf") == 2, "once at the fire, once at the end for the viewer"


def test_a_point_summary_of_an_unfiltered_point_has_no_filtered_position():
    point = fw.TrackPoint()
    point.filterHypothesis = fw.FILTER_HYP_UNFILTERED
    summary = fr._point_summary(point)  # pylint: disable=protected-access
    assert summary.filtered_position is None and summary.filter_hypothesis == "unfiltered"
```

The spy replaces the attribute on the shared ctypes library for this test
only; `monkeypatch` restores it. Keep
`test_synthetic_shot_vla_is_read_back_by_the_direction_fit` from Task 4.

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py -k "reconstruction or filtered or point_summary or direction_fit" -v`
Expected: FAIL (`LaunchSummary` has no `angle_why`).

- [ ] **Step 3: Summaries**

`PointSummary`, after `angles_valid`:

```python
    angle_confidence: float = 0.0
    # l3_track_point_t.filteredPosition; None when the point was not reconstructed.
    filtered_position: tuple[float, float, float] | None = None
    filter_accepted: bool = False
    filter_hypothesis: str = "unfiltered"
```

`_point_summary`:

```python
def _point_summary(point: fw.TrackPoint) -> PointSummary:
    reconstructed = point.filterHypothesis != fw.FILTER_HYP_UNFILTERED
    return PointSummary(
        frame=int(point.frame),
        timestamp_us=int(point.timestampUs),
        range_bin=float(point.rangeBin),
        range_m=float(point.rangeM),
        doppler_mps=float(point.dopplerAliasMps),
        confidence=float(point.confidence),
        position=(float(point.position.x), float(point.position.y), float(point.position.z)),
        angles_valid=bool(point.anglesValid),
        angle_confidence=float(point.angleConfidence),
        filtered_position=(
            (
                float(point.filteredPosition.x),
                float(point.filteredPosition.y),
                float(point.filteredPosition.z),
            )
            if reconstructed
            else None
        ),
        filter_accepted=bool(point.filterAccepted),
        filter_hypothesis=fw.FILTER_HYP_NAMES[point.filterHypothesis],
    )
```

`LaunchSummary`, after `velocity`:

```python
    angles_accepted: int = 0
    angle_rms_deg: float | None = None  # None when the direction fit kept no angles
    angle_why: str = "none"  # l3_ball_fit_why_name: why the angles are (not) valid
```

`_launch_summary`: add

```python
        angles_accepted=int(launch.anglesAccepted),
        angle_rms_deg=math.degrees(launch.angleRmsRad) if launch.anglesAccepted else None,
        angle_why=fw.BALL_FIT_WHY_NAMES[launch.angleWhy],
```

- [ ] **Step 4: Reconstruct in `replay_dump`**

1. After `angle_queue = fw.AngleQueue()` (~935):

```python
    # The club reconstruction's work area (l3_track_kf.h), as the board's gClubKfWork.
    kf_work = ctypes.create_string_buffer(lib.l3_track_kf_work_bytes())
    kf_result = fw.TrackKfResult()
```

2. After `if fired and fired_frame is None: fired_frame = frame` (~1364):

```python
        if fired and fired_frame == frame:
            # l3_considerSelfTrigger: the fire drains the club's angles (the
            # replay drained each frame already), reconstructs the club once
            # and freezes the filtered delivery.
            lib.l3_track_kf_run(
                ctypes.byref(track.cfg.kf), ctypes.byref(track), kf_work, ctypes.byref(kf_result)
            )
            lib.l3_track_delivery_filtered(ctypes.byref(track), 8, ctypes.byref(delivery))
```

3. Before `points = _confirmed_points(points, track)` (~1442):

```python
    # The viewer's trajectories. The board reconstructs the club only at the
    # fire (for its delivery) and the ball at RESULT; the replay does both over
    # every held point at the end, so the page shows the whole track even when
    # the capture ended before RESULT.
    lib.l3_track_kf_run(
        ctypes.byref(track.cfg.kf), ctypes.byref(track), kf_work, ctypes.byref(kf_result)
    )
    lib.l3_ball_track_reconstruct(ctypes.byref(ball_track), ctypes.byref(launch))
    points = _reconstructed(lib, track, points)
    ball_points = _reconstructed(lib, ball_track.core, ball_points)
```

4. The helper, beside `_confirmed_points`:

```python
def _reconstructed(lib, core, summaries: list[PointSummary]) -> list[PointSummary]:
    """The summaries with every still-held point re-read after reconstruction;
    a point rolled off the ring (or withdrawn) keeps what it was logged with."""
    index = ctypes.c_uint32()
    point = fw.TrackPoint()
    out = []
    for summary in summaries:
        held = lib.l3_track_find_point(
            ctypes.byref(core), summary.timestamp_us, ctypes.byref(index)
        ) and lib.l3_track_point(ctypes.byref(core), index.value, ctypes.byref(point))
        out.append(_point_summary(point) if held else summary)
    return out
```

Confirm `launch` is the `fw.Launch` in `replay_dump`'s scope (~1001) that
`_replay_post_frame` fills. `ball_points` must be a list in scope at that point.
Both are, per `ReplayResult(... launch=_launch_summary(launch), ball_points=ball_points ...)`.

- [ ] **Step 5: Run the replay, viewer and label suites**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py tests/test_iwr6843_labelled_replay.py tests/test_iwr6843_replay_defaults.py -v`
Expected: all PASS. The labelled-replay baselines (`label_baseline.json`) score
range and time only, so they must not move. If one moves, stop: the per-frame
path changed, which violates a global constraint.

- [ ] **Step 6: Commit**

```bash
git add src/openflight/iwr6843/firmware_replay.py tests/test_iwr6843_firmware_replay.py
git commit -m "feat(iwr6843): replay reconstructs as the board does and reports it per point"
```

---

### Task 8: The viewer draws raw and fitted together

**Files:**
- Modify: `scripts/iwr6843/dump_viewer.html` (3D card header ~144-148; `renderTraj` ~510-535; tab handlers ~858-861; state ~218)
- Test: `tests/test_iwr6843_dump_viewer.py`

**Interfaces:**
- Consumes: `firmware.points[*]` and `firmware.ball_points[*]` with `position`, `filtered_position` (null when not reconstructed), `filter_accepted`, `filter_hypothesis`, `angle_confidence` (Task 7).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_dump_viewer.py`:

```python
def _page() -> str:
    return (Path(__file__).parents[1] / "scripts" / "iwr6843" / "dump_viewer.html").read_text(
        encoding="utf-8"
    )


def test_the_trajectory_view_has_a_raw_fitted_both_toggle():
    html = _page()
    assert 'id="tabsTraj"' in html
    for value in ("both", "raw", "fitted"):
        assert f'data-t="{value}"' in html
    assert 'let trajShow = "both"' in html


def test_the_trajectory_view_draws_the_reconstruction_and_tolerates_its_absence():
    """Review Focus 5: an older payload (no filtered_position) still draws the raw track."""
    html = _page()
    body = html[html.index("function renderTraj(") : html.index("// ---------- annotate")]
    assert "filtered_position" in body
    assert ".filter((p) => p.filtered_position)" in body, "points with no reconstruction are skipped"
    assert "filter_hypothesis" in body and "angle_confidence" in body


@needs_compiler
def test_shot_points_carry_their_reconstruction_to_the_page():
    raw = synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )
    data = dv.analyze_dump(raw, dv.ViewerOptions(tee_bin=TEE_BIN, tee_range_m=TEE_RANGE_M))
    json.dumps(data, allow_nan=False)
    firmware = data["firmware"]
    for point in firmware["points"] + firmware["ball_points"]:
        assert {"filtered_position", "filter_accepted", "filter_hypothesis", "angle_confidence"} <= set(point)
        assert point["filter_hypothesis"] in fw.FILTER_HYP_NAMES
    assert "angle_why" in firmware["launch"]
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_dump_viewer.py -k "trajectory or reconstruction" -v`
Expected: the two HTML tests FAIL. The payload test passes already (Task 7);
that's fine.

- [ ] **Step 3: Toggle markup and state**

In the 3D card header, after the closing `</div>` of `#tabs3d`:

```html
            <div class="tabs" id="tabsTraj" hidden>
              <button data-t="both" class="on">Raw + fitted</button>
              <button data-t="raw">Raw</button>
              <button data-t="fitted">Fitted</button>
            </div>
```

At line ~218, after `let view3d = "mti", viewMap = "mti_db";`:

```js
let trajShow = "both";  // the trajectory view: raw angle points, the reconstruction, or both
```

Replace the `#tabs3d` handler (~858-861) with:

```js
document.querySelectorAll("#tabs3d button").forEach((b) => (b.onclick = () => {
  document.querySelectorAll("#tabs3d button").forEach((x) => x.classList.toggle("on", x === b));
  view3d = b.dataset.v; $("#tabsTraj").hidden = view3d !== "traj";
  Plotly.purge($("#p3d")); render3d();
}));
document.querySelectorAll("#tabsTraj button").forEach((b) => (b.onclick = () => {
  document.querySelectorAll("#tabsTraj button").forEach((x) => x.classList.toggle("on", x === b));
  trajShow = b.dataset.t; render3d();
}));
```

- [ ] **Step 4: `renderTraj`**

Replace the `add` helper and its callers inside `renderTraj` with:

```js
  const hover = (p) => `frame ${p.frame} · ${(p.timestamp_us / 1000).toFixed(1)} ms<br>bin ${p.range_bin.toFixed(2)} · v ${p.doppler_mps.toFixed(1)} m/s`
    + `<br>angles ${p.angles_valid ? "valid" : "boresight"} · conf ${(p.angle_confidence ?? 0).toFixed(2)}`
    + `<br>fit ${p.filter_hypothesis ?? "unfiltered"}${p.filter_accepted ? "" : " · angles not used"}`;
  // Raw: each point's own angles. A rejected angle is hollow, a floor reflection a diamond.
  const raw = (list, name, color) => {
    if (!list || !list.length) return;
    const alone = trajShow === "raw";
    traces.push({
      type: "scatter3d", mode: alone ? "lines+markers" : "markers", name: alone ? name : `${name} (raw)`,
      opacity: alone ? 1 : 0.45,
      x: list.map((p) => p.position && p.position[0]), y: list.map((p) => p.position && p.position[1]), z: list.map((p) => p.position && p.position[2]),
      marker: {
        size: 4, color: list.map((p) => (p.angles_valid ? color : C.muted)),
        symbol: list.map((p) => (p.filter_hypothesis === "image" ? "diamond"
          : (p.filter_accepted || !p.filter_hypothesis || p.filter_hypothesis === "unfiltered") ? "circle" : "circle-open")),
      },
      line: { color, width: 2 },
      text: list.map(hover), hovertemplate: `${name}<br>%{text}<br>x %{x:.3f} y %{y:.3f} z %{z:.3f} m<extra></extra>`,
    });
  };
  // Fitted: the reconstruction (l3_ball_fit / l3_track_kf); a point without one is skipped.
  const fitted = (list, name, color) => {
    const kept = (list || []).filter((p) => p.filtered_position);
    if (!kept.length) return;
    traces.push({
      type: "scatter3d", mode: "lines+markers", name: `${name} (fitted)`,
      x: kept.map((p) => p.filtered_position[0]), y: kept.map((p) => p.filtered_position[1]), z: kept.map((p) => p.filtered_position[2]),
      marker: { size: 3, color }, line: { color, width: 6 },
      text: kept.map(hover), hovertemplate: `${name} (fitted)<br>%{text}<br>x %{x:.3f} y %{y:.3f} z %{z:.3f} m<extra></extra>`,
    });
  };
  const add = (list, name, color) => {
    if (trajShow !== "fitted") raw(list, name, color);
    if (trajShow !== "raw") fitted(list, name, color);
  };
```

The calls after it (`add(F.points, "club track", C.club); ...`) stay as they are.

- [ ] **Step 5: Run the tests**

Run: `uv run pytest tests/test_iwr6843_dump_viewer.py -v`
Expected: all PASS.

- [ ] **Step 6: Check it in the browser**

Start the viewer with the browser pane: `preview_start` with
`{name: "dump-viewer"}` (from `.claude/launch.json`). Open a recording, choose
**Trajectory (golf frame)**, and check:

- The toggle appears only in that view.
- **Raw + fitted** shows faint points and a solid line for each track.
- **Fitted** hides the points; **Raw** is the old view.
- `read_console_messages` shows no errors.

Take a screenshot of one shot in **Raw + fitted** for the task report. If the
recordings folder in `launch.json` does not exist on this machine, point the
viewer at `tests/radar/recordings`:
`uv run python scripts/iwr6843/dump_viewer.py --no-browser --dir tests/radar/recordings`,
added as a new `launch.json` entry named `dump-viewer-recordings`, port 5059.

- [ ] **Step 7: Commit**

```bash
git add scripts/iwr6843/dump_viewer.html tests/test_iwr6843_dump_viewer.py
git commit -m "feat(viewer): trajectory view draws the raw angle points and the reconstruction together"
```

---

### Task 9: Evaluate on the recordings, freeze the baseline, record the deviations

**Files:**
- Create: `scripts/analysis/evaluate_trajectory_reconstruction.py`
- Create: `docs/superpowers/specs/2026-10-01-trajectory-reconstruction-baseline.json` (generated)
- Modify: `docs/superpowers/specs/2026-10-01-iwr-trajectory-reconstruction-design.md` (a "Deviations during planning" section, the baseline numbers)
- Modify: `docs/development/iwr6843-firmware-architecture.md` (the two modules), `docs/changelog.md`, `CLAUDE.md` (Key Modules line for `iwr6843/`)
- Test: `tests/test_evaluate_trajectory_reconstruction.py` (new)

**Interfaces:**
- Consumes: `fr.recording_configs`, `fr.replay_dump`, `PointSummary` and `LaunchSummary` fields (Task 7).
- Produces:
  - `scatter_about_line(points: list[tuple[float, float, float]]) -> float | None`: RMS perpendicular distance from the best-fit 3D line (SVD), None under 3 points.
  - `evaluate(directory) -> dict`.
  - A CLI: `uv run python scripts/analysis/evaluate_trajectory_reconstruction.py [--dir tests/radar/recordings] [--json out.json]`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_evaluate_trajectory_reconstruction.py`:

```python
"""The reconstruction evaluation's geometry, and that it runs over the recordings."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from openflight.iwr6843 import firmware_host as fw

SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "evaluate_trajectory_reconstruction.py"


def _module():
    spec = importlib.util.spec_from_file_location("evaluate_trajectory_reconstruction", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_points_on_a_line_have_no_scatter():
    ev = _module()
    line = [(1.0 + 0.1 * k, 0.02 * k, 0.05 * k) for k in range(6)]
    assert ev.scatter_about_line(line) == pytest.approx(0.0, abs=1e-9)


def test_scatter_is_the_rms_perpendicular_distance():
    ev = _module()
    pts = [(float(k), 0.1 if k % 2 else -0.1, 0.0) for k in range(8)]
    assert ev.scatter_about_line(pts) == pytest.approx(0.1, rel=1e-6)


def test_under_three_points_has_no_scatter():
    assert _module().scatter_about_line([(0, 0, 0), (1, 0, 0)]) is None


@pytest.mark.skipif(fw.host_compiler() is None, reason="no C compiler for the firmware modules")
def test_the_recordings_evaluate_and_the_reconstruction_is_steadier():
    report = _module().evaluate(Path("tests/radar/recordings"))
    assert report["shots"] > 0
    ball = report["ball"]
    if ball["compared"]:
        assert ball["filtered_scatter_m_median"] <= ball["raw_scatter_m_median"]
    club = report["club"]
    if club["compared"]:
        assert club["filtered_scatter_m_median"] <= club["raw_scatter_m_median"]
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run pytest tests/test_evaluate_trajectory_reconstruction.py -v`
Expected: FAIL (the script does not exist).

- [ ] **Step 3: Write the script**

Create `scripts/analysis/evaluate_trajectory_reconstruction.py`:

```python
#!/usr/bin/env python3
"""How steady the reconstructed 3D trajectories are, over recorded captures.

    uv run python scripts/analysis/evaluate_trajectory_reconstruction.py \
        [--dir tests/radar/recordings] [--json out.json]

Every ``.l3dump`` in the manifest is replayed through the compiled firmware.
For each shot's ball and club track it measures the RMS perpendicular scatter
of the raw points (each frame's own angles) and of the reconstructed points
about their own best-fit 3D line, plus the ball's fitted HLA/VLA and why.
There are no angle labels, so this measures STABILITY, not accuracy: a
reconstruction that is steadily wrong scores well here.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_replay as fr


def scatter_about_line(points) -> float | None:
    """RMS perpendicular distance of the points from their best-fit line; None under 3."""
    if len(points) < 3:
        return None
    xyz = np.asarray(points, dtype=float)
    centred = xyz - xyz.mean(axis=0)
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    along = centred @ vt[0]
    perpendicular = centred - np.outer(along, vt[0])
    return float(np.sqrt(np.mean(np.sum(perpendicular**2, axis=1))))


def _track_scatter(points) -> tuple[float | None, float | None]:
    raw = [p.position for p in points if p.position is not None and p.filtered_position is not None]
    fitted = [p.filtered_position for p in points if p.filtered_position is not None]
    return scatter_about_line(raw), scatter_about_line(fitted)


def _median(values) -> float | None:
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def _summary(pairs) -> dict:
    compared = [(r, f) for r, f in pairs if r is not None and f is not None]
    return {
        "compared": len(compared),
        "raw_scatter_m_median": _median(r for r, _ in compared),
        "filtered_scatter_m_median": _median(f for _, f in compared),
    }


def evaluate(directory: Path) -> dict:
    shots = []
    for path, config in fr.recording_configs(directory):
        result = fr.replay_dump(path.read_bytes(), config)
        launch = result.launch
        ball_raw, ball_fit = _track_scatter(result.ball_points)
        club_raw, club_fit = _track_scatter(result.points)
        shots.append(
            {
                "name": path.name,
                "ball_raw_scatter_m": ball_raw,
                "ball_filtered_scatter_m": ball_fit,
                "club_raw_scatter_m": club_raw,
                "club_filtered_scatter_m": club_fit,
                "hla_deg": launch.hla_deg if launch else None,
                "vla_deg": launch.vla_deg if launch else None,
                "angle_why": launch.angle_why if launch else "no_launch",
                "angles_accepted": launch.angles_accepted if launch else 0,
                "speed_mps": launch.speed_mps if launch else None,
            }
        )
    whys: dict[str, int] = {}
    for shot in shots:
        whys[shot["angle_why"]] = whys.get(shot["angle_why"], 0) + 1
    return {
        "shots": len(shots),
        "ball": _summary((s["ball_raw_scatter_m"], s["ball_filtered_scatter_m"]) for s in shots),
        "club": _summary((s["club_raw_scatter_m"], s["club_filtered_scatter_m"]) for s in shots),
        "angle_why_counts": whys,
        "per_shot": shots,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", type=Path, default=Path(fr.RECORDINGS_DIR))
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    report = evaluate(args.dir)
    text = json.dumps(report, indent=2)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "per_shot"}, indent=2))


if __name__ == "__main__":
    main()
```

`fr.recording_configs(directory)` is `recording_configs(directory=RECORDINGS_DIR,
*, default_tee_bin=None)` (`firmware_replay.py:2308`). If `RECORDINGS_DIR` is not
the module's constant name, use the name `recording_configs`'s default refers
to.

- [ ] **Step 4: Run the test and the evaluation**

Run: `uv run pytest tests/test_evaluate_trajectory_reconstruction.py -v`
Expected: PASS.

Run: `uv run python scripts/analysis/evaluate_trajectory_reconstruction.py --json docs/superpowers/specs/2026-10-01-trajectory-reconstruction-baseline.json`
Expected: prints the summary. Record the ball and club median raw vs filtered
scatter and the `angle_why_counts`. If `few_angles` or `scatter` dominates the
ball `angle_why_counts`, report it as a finding; do not retune in this task.

- [ ] **Step 5: Record the deviations and results in the spec and docs**

In the spec, add a section before "Open questions":

```markdown
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
8. The replay also reconstructs both tracks at the end for the viewer; the board reconstructs the
   club at the fire and the ball at RESULT.
9. The tee anchor takes the ball track origin's slant range and bearing, at teeBallHeightM - radarHeightM.
```

Then add a "Results on the recordings" section: the medians and why-counts
from the baseline JSON, and the date. In
`docs/development/iwr6843-firmware-architecture.md`, add one paragraph each for
`l3_ball_fit` and `l3_track_kf`, naming when each runs and the profile stage.
In `docs/changelog.md`, add an entry. In `CLAUDE.md`, extend the
`iwr6843/` Key Modules bullet with:
"firmware `l3_ball_fit` (tee-anchored ball direction) and `l3_track_kf` (club EKF + RTS) reconstruct the 3D tracks".

- [ ] **Step 6: Full verification**

Run each and confirm:
- `uv run pytest tests/ -q` → all PASS (the real-capture test skips without `session_logs/`).
- `uv run pylint src/openflight/ --fail-under=9` → passes.
- `uv run ruff check src/openflight/ scripts/analysis/evaluate_trajectory_reconstruction.py` → clean.
- `uv run ruff format --check src/openflight/` → clean.

- [ ] **Step 7: Commit**

```bash
git add scripts/analysis/evaluate_trajectory_reconstruction.py tests/test_evaluate_trajectory_reconstruction.py docs/ CLAUDE.md
git commit -m "docs(iwr6843): reconstruction baseline on the recordings, deviations recorded"
```

---

### Task 10 (hardware, run by the user): board timing against the budgets

Not automatable here: it needs the board.

- [ ] Flash the image built in Task 6 Step 6. Run the acceptance run from
  8b121ac7 (it prints the per-phase detect timing) and take a few swings.
- [ ] Read `triggerLog perf` and check that `perf reconstruct` has a `max` of
  1500 µs or less. That limit is the two budgets combined: ball ≤ 1000 µs plus
  club ≤ 500 µs, since the stage adds both on a shot with both. If it is over,
  lower `ball.fit.gridSteps` (10 → 8) first; that is the dominant cost.
- [ ] Record the numbers in the spec's "Results" section and commit.
