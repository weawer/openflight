"""Tests for the IWR6843 ball-leave fallback, firmware/iwr6843/l3_leave.c.

When the club rules miss (the club unseen before launch), the ball leaving
still fires the self-trigger. Before impact the band hides the ball, so the
first return to stand beyond the band's far edge, starting close to it and
stepping outward at ball speed on the next frame, is the ball: on every
labelled swing the ball goes first and the club follows 1-10 frames later.
Impact is dated by running that line back to the ball's rest bin.

The returns it watches are its own (l3_leave_targets): the trigger's floor is
learned where the club swings through, and the leaving ball stands at only
0.4-1.2 times it, so the rule reads the bins beyond the band against their
own median instead.
"""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw

WHY = {name: index for index, name in enumerate(fw.LEAVE_WHY_NAMES)}
BIN_M = 6.0 / 128
FRAME_US = 3000
EDGE = 46.0  # the band's far edge
ORIGIN = 43.0  # the ball's rest bin, the band's centre


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def leave(lib, **overrides) -> fw.Leave:
    cfg = fw.LeaveCfg()
    lib.l3_leave_cfg_defaults(ctypes.byref(cfg))
    cfg.binWidthM = BIN_M
    for name, value in overrides.items():
        setattr(cfg, name, value)
    out = fw.Leave()
    lib.l3_leave_init(ctypes.byref(out), ctypes.byref(cfg))
    return out


def frame(
    lib, state, frame_index: int, *bins: float, club: bool = True, frame_us: int = FRAME_US
) -> int:
    """One frame's targets at the given global bins, stamped frame_index * frame_us;
    ``club``: the club track is approaching on this frame."""
    targets = (fw.TargetObs * max(1, len(bins)))()
    for i, value in enumerate(bins):
        targets[i].rangeBin = value
        targets[i].peakBin = int(value)
        targets[i].timestampUs = frame_index * frame_us
    return lib.l3_leave_update(
        ctypes.byref(state), targets, len(bins), EDGE, ORIGIN, 1 if club else 0
    )


def bins_per_frame(mps: float) -> float:
    return mps * FRAME_US * 1e-6 / BIN_M


def test_defaults(lib):
    cfg = leave(lib).cfg
    assert cfg.startBins == pytest.approx(4.0)
    # A return jittering beyond a band short of the ball walked out at 20.3 m/s.
    assert cfg.minSpeedMps == pytest.approx(22.0)
    assert cfg.maxSpeedMps == pytest.approx(90.0)
    assert cfg.snr == pytest.approx(6.0)
    assert cfg.clubHoldFrames == 10


# --- the returns the rule watches ---------------------------------------------


def window(values: dict[int, float], first: int = 30, count: int = 40, noise: float = 1.0):
    """Whole-window observations, global bins first..first+count, the peak
    statistic ``noise`` everywhere but ``values`` (global bin -> peak)."""
    obs = (fw.BinObs * count)()
    for i in range(count):
        value = values.get(first + i, noise)
        obs[i].energy = obs[i].peak = obs[i].loop0 = value
    return obs


def peak_params(snr: float = 1.0) -> fw.ObsParams:
    params = fw.ObsParams()
    params.stat, params.snr, params.loopPeriodS, params.subBin = fw.STAT_PEAK, snr, 1e-4, 0
    return params


def leave_targets(lib, state, obs, *, first: int = 30, edge: float = EDGE, params=None):
    out = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    n = lib.l3_leave_targets(
        ctypes.byref(state.cfg),
        ctypes.byref(params or peak_params()),
        obs,
        first,
        len(obs),
        7,
        21_000,
        edge,
        out,
        fw.OBS_MAX_TARGETS,
        None,
    )
    return [out[i] for i in range(n)]


def test_a_return_beyond_the_edge_well_over_the_stretch_median_is_a_target(lib):
    state = leave(lib)
    found = leave_targets(lib, state, window({50: 10.0}))
    assert [t.peakBin for t in found] == [50]
    assert found[0].timestampUs == 21_000 and found[0].frame == 7


def test_the_floor_is_the_stretch_beyond_the_edge_not_the_whole_window(lib):
    """The club and the ball's ridge short of the edge, however strong, do not
    raise it: the ball at 7x the far noise stands out."""
    state = leave(lib)
    found = leave_targets(lib, state, window({36: 500.0, 44: 300.0, 45: 300.0, 50: 7.0}))
    assert [t.peakBin for t in found] == [50]


def test_a_return_under_snr_times_the_median_is_not_a_target(lib):
    state = leave(lib)
    assert leave_targets(lib, state, window({50: 5.5})) == []


def test_nothing_on_or_short_of_the_edge_is_a_target(lib):
    state = leave(lib)
    assert leave_targets(lib, state, window({40: 50.0, 46: 50.0})) == []


def test_the_trigger_snr_does_not_set_the_threshold(lib):
    """params carries the statistic; the rule's own snr sets the threshold."""
    state = leave(lib)
    found = leave_targets(lib, state, window({50: 7.0}), params=peak_params(snr=12.0))
    assert [t.peakBin for t in found] == [50]


def test_too_few_bins_beyond_the_edge_give_no_targets(lib):
    """Five bins cannot make a median: a window ending just past the band."""
    state = leave(lib)
    assert leave_targets(lib, state, window({49: 50.0}, first=20, count=32), first=20) == []


def test_a_ball_leaving_fires_on_its_second_frame_dated_back_to_its_rest_bin(lib):
    state = leave(lib)
    assert frame(lib, state, 10, 47.0) == 0
    assert state.why == WHY["started"]
    assert frame(lib, state, 11, 49.8) == 1
    assert state.why == WHY["fired"] and state.fired == 1
    # 2.8 bins a frame: the line reaches 43.0 (4 bins short of 47.0) 4/2.8 frames before frame 10.
    assert state.impactTimestampUs == pytest.approx(30_000 - 4.0 / 2.8 * FRAME_US, abs=1)
    assert state.speedMps == pytest.approx(2.8 * BIN_M / (FRAME_US * 1e-6), rel=1e-4)


def test_the_nearest_return_beyond_the_edge_starts_it_and_any_return_at_ball_speed_fires(lib):
    """The club, still short of the band, and far clutter do not matter."""
    state = leave(lib)
    frame(lib, state, 10, 38.0, 47.0, 70.0)
    assert state.startBin == pytest.approx(47.0)
    assert frame(lib, state, 11, 39.0, 70.0, 50.0) == 1


def test_a_first_return_far_beyond_the_edge_does_not_start(lib):
    state = leave(lib)
    assert frame(lib, state, 10, EDGE + 4.5) == 0
    assert state.why == WHY["far"]
    assert frame(lib, state, 11, EDGE + 4.5 + 2.8) == 0
    assert state.fired == 0


@pytest.mark.parametrize("mps", [0.0, 5.0, 18.0])
def test_a_standing_or_slow_return_does_not_fire(lib, mps):
    """A standing return beyond the ball, a walker, the club's slow follow-through."""
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    assert frame(lib, state, 11, 47.0 + bins_per_frame(mps)) == 0
    assert state.fired == 0 and state.why == WHY["slow"]


@pytest.mark.parametrize("jitter_bins", [1.0, 1.2, 1.4])
def test_range_jitter_that_does_not_fire_at_3_ms_does_not_fire_at_2_ms(lib, jitter_bins):
    """A standing return's measured range wobbles about a bin frame to frame
    (l3_leave_cfg_defaults: newBins). The rule reads a two-frame step as a
    speed, so the same wobble reads 1.5x faster at 2 ms than at 3 ms: a 1-bin
    wobble is 15.6 m/s at 3 ms but 23.4 m/s at 2 ms, over minSpeedMps (22).
    Reported 2026-10-03: with the 2 ms profile the default, the trigger fires
    on backswings. Whatever the frame period, the wobble must not fire."""
    at_3_ms = leave(lib)
    frame(lib, at_3_ms, 10, 47.0, frame_us=3000)
    assert frame(lib, at_3_ms, 11, 47.0 + jitter_bins, frame_us=3000) == 0

    at_2_ms = leave(lib)
    frame(lib, at_2_ms, 10, 47.0, frame_us=2000)
    assert frame(lib, at_2_ms, 11, 47.0 + jitter_bins, frame_us=2000) == 0, (
        f"a {jitter_bins}-bin wobble fired at 2 ms ({at_2_ms.speedMps:.1f} m/s)"
    )


def test_a_small_step_is_judged_over_at_least_2_5_ms_by_default(lib):
    cfg = fw.LeaveCfg()
    lib.l3_leave_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.minStepUs, cfg.minStepBins) == (2500, pytest.approx(1.5))


def test_at_2_ms_a_ball_stepping_1_5_bins_or_more_fires_on_the_next_frame(lib):
    """20260809_111337_707_002: the ball shows beyond the band on two frames
    only (48.55 then 50.18, 38 m/s); waiting for a third lost it."""
    state = leave(lib)
    assert frame(lib, state, 15, 48.55, frame_us=2000) == 0
    assert frame(lib, state, 16, 50.18, frame_us=2000) == 1
    assert state.speedMps == pytest.approx(1.63 * BIN_M / 2000e-6, rel=1e-3)


def test_at_2_ms_a_slow_ball_fires_on_the_second_frame_after_its_start(lib):
    """Under 1.5 bins a frame (25 m/s is 1.07) the frame in between keeps the
    start; over two frames (4 ms) it is judged as at 3 ms."""
    state = leave(lib)
    step = 25.0 * 2000e-6 / BIN_M
    assert frame(lib, state, 10, 47.0, frame_us=2000) == 0
    assert frame(lib, state, 11, 47.0 + step, frame_us=2000) == 0
    assert state.why == WHY["started"] and state.startBin == pytest.approx(47.0)
    assert frame(lib, state, 12, 47.0 + 2 * step, frame_us=2000) == 1
    assert state.speedMps == pytest.approx(25.0, rel=1e-4)
    # The line through 47.0 at 25 m/s reaches the rest bin 4 bins earlier.
    back_us = 4.0 * BIN_M / 25.0 * 1e6
    assert state.impactTimestampUs == pytest.approx(20_000 - back_us, abs=1)


def test_at_2_ms_a_wobble_over_two_frames_reads_slow(lib):
    state = leave(lib)
    frame(lib, state, 10, 47.0, frame_us=2000)
    frame(lib, state, 11, 48.0, frame_us=2000)
    assert frame(lib, state, 12, 47.6, frame_us=2000) == 0
    assert state.fired == 0 and state.why == WHY["slow"]


def test_min_step_zero_judges_the_next_frame_as_before(lib):
    state = leave(lib, minStepUs=0)
    frame(lib, state, 10, 47.0, frame_us=2000)
    assert frame(lib, state, 11, 48.0, frame_us=2000) == 1


def test_at_2_ms_the_ball_missing_on_the_frame_in_between_restarts_it(lib):
    """As at 3 ms: a frame with nothing beyond the edge forgets the start."""
    state = leave(lib)
    frame(lib, state, 10, 47.0, frame_us=2000)
    assert frame(lib, state, 11, frame_us=2000) == 0
    assert state.why == WHY["idle"] and state.started == 0


def test_a_return_moving_inward_does_not_fire(lib):
    state = leave(lib)
    frame(lib, state, 10, 48.0)
    assert frame(lib, state, 11, 46.5) == 0
    assert state.fired == 0


def test_a_step_faster_than_any_ball_does_not_fire(lib):
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    assert frame(lib, state, 11, 47.0 + bins_per_frame(95.0)) == 0
    assert state.fired == 0


def test_a_slow_step_restarts_from_the_newest_return(lib):
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    frame(lib, state, 11, 47.2)  # slow: restarts at 47.2
    assert state.startBin == pytest.approx(47.2)
    assert frame(lib, state, 12, 50.0) == 1


def test_a_frame_with_nothing_beyond_the_edge_forgets_the_start(lib):
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    assert frame(lib, state, 11, 40.0) == 0
    assert state.why == WHY["idle"] and state.started == 0
    assert frame(lib, state, 12, 52.6) == 0, "far from the edge: not a new start"


def test_nothing_on_the_band_edge_or_short_of_it_counts(lib):
    state = leave(lib)
    frame(lib, state, 10, EDGE)
    assert state.why == WHY["idle"]


def test_fires_once_until_rearmed_and_rearm_keeps_counters_and_config(lib):
    state = leave(lib, startBins=3.0)
    frame(lib, state, 10, 47.0)
    assert frame(lib, state, 11, 49.8) == 1
    assert frame(lib, state, 12, 52.6) == 0, "fired: later frames are ignored"
    lib.l3_leave_rearm(ctypes.byref(state))
    assert state.fired == 0 and state.started == 0 and state.impactTimestampUs == 0
    assert state.counters[WHY["fired"]] == 1
    assert state.cfg.startBins == pytest.approx(3.0)


def test_why_names_and_format(lib):
    assert fw.LEAVE_WHY_NAMES == (
        "none",
        "noclub",
        "idle",
        "far",
        "stood",
        "started",
        "slow",
        "fired",
    )
    for index, name in enumerate(fw.LEAVE_WHY_NAMES):
        assert lib.l3_leave_why_name(index).decode() == name
    assert lib.l3_leave_why_name(99).decode() == "?"
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    frame(lib, state, 11, 49.8)
    text = fw.c_text(lib.l3_leave_format, ctypes.byref(state), cap=240)
    assert text == "leave fired=1 why=fired start=47.00 speed=43.75 t=25714 fired_n=1"


# --- only after a swing ------------------------------------------------------
#
# A ball leaves right after the club's approach: on the rescued swings 5-8
# frames after the club was last seen. Without that, noise peaks over a quiet
# stretch's median, or clutter beyond a band that is not on the ball, can step
# outward at a ball's speed (the 2026-08-24 captures at tee 38 fired on frame 1).


def test_nothing_fires_without_a_club_approach(lib):
    state = leave(lib)
    assert frame(lib, state, 10, 47.0, club=False) == 0
    assert state.why == WHY["noclub"] and state.started == 0
    assert frame(lib, state, 11, 49.8, club=False) == 0


def test_a_club_approach_holds_the_rule_armed_for_clubHoldFrames(lib):
    state = leave(lib)
    frame(lib, state, 1, club=True)  # the club, approaching; nothing beyond the edge
    frame(lib, state, 9, 47.0, club=False)  # 8 frames on: still held
    assert state.why == WHY["started"]
    assert frame(lib, state, 10, 49.8, club=False) == 1


def test_the_hold_runs_out(lib):
    state = leave(lib)
    frame(lib, state, 1, club=True)
    for index in range(2, 12):
        frame(lib, state, index, club=False)
    assert state.why == WHY["noclub"]
    frame(lib, state, 12, 47.0, club=False)
    assert frame(lib, state, 13, 49.8, club=False) == 0


def test_rearm_forgets_the_club(lib):
    state = leave(lib)
    frame(lib, state, 1, club=True)
    lib.l3_leave_rearm(ctypes.byref(state))
    frame(lib, state, 2, 47.0, club=False)
    assert state.why == WHY["noclub"]


# --- the club near the ball ----------------------------------------------------
#
# Approaching is not enough: the band starts centred on the configured tee
# until its noise map finds the ball's ridge, and a ridge left beyond its edge
# jitters outward while the club is still on its downswing (the 2026-08-24
# captures at tee 38 fired at frame 3, launch at 8). The ball leaves only once
# the club has reached it: on the rescued swings the club was last seen 4-7.4
# bins short of the band's near edge.

LO = 40.0  # the band's near edge


def club_near(lib, active: bool, count: int, newest_bin: float) -> int:
    state = leave(lib)
    return lib.l3_leave_club_near(
        ctypes.byref(state.cfg), 1 if active else 0, count, newest_bin, LO
    )


def test_the_default_near_distance(lib):
    assert leave(lib).cfg.clubNearBins == pytest.approx(10.0)


@pytest.mark.parametrize(
    ("active", "count", "newest", "near"),
    [
        (True, 2, LO - 10.0, 1),  # at the limit
        (True, 5, LO - 4.0, 1),
        (True, 5, LO - 10.5, 0),  # still on its downswing
        (True, 1, LO - 2.0, 0),  # an acquisition, not an approach
        (False, 5, LO - 2.0, 0),  # no track
        (True, 5, LO + 1.0, 1),  # at or past the edge counts
    ],
)
def test_the_club_is_near_the_ball_when_an_approach_is_within_clubNearBins(
    lib, active, count, newest, near
):
    assert club_near(lib, active, count, newest) == near


# --- a start is something new ----------------------------------------------------
#
# With the tee set short of the ball (2026-08-24 at tee 38: band 36-41, ball at
# 43-45) the ball's standing ridge lies beyond the band's edge and jitters a
# bin or so a frame: it fired 2-5 frames before impact. A start from nothing
# must be a return with nothing within newBins of it on the frame before.


def test_the_default_new_distance(lib):
    assert leave(lib).cfg.newBins == pytest.approx(1.0)


def test_a_return_that_stood_near_itself_on_the_last_frame_does_not_start(lib):
    """The frame before is remembered even while the rule is not armed."""
    state = leave(lib)
    frame(lib, state, 8, 47.0, club=False)
    assert state.why == WHY["noclub"]
    frame(lib, state, 9, 47.3)
    assert state.why == WHY["stood"] and state.started == 0
    frame(lib, state, 10, 48.1)  # jittered 0.8 bins: still stood
    assert state.why == WHY["stood"]


def test_a_return_clear_of_everything_on_the_last_frame_starts(lib):
    state = leave(lib)
    frame(lib, state, 8, 47.0, club=False)
    frame(lib, state, 9, 48.2)
    assert state.why == WHY["started"]


def test_a_ball_leaving_the_ridge_is_new_and_fires(lib):
    state = leave(lib)
    frame(lib, state, 9, 44.0)  # the ridge, inside the band
    frame(lib, state, 10, 46.8)  # just beyond the edge, nothing near it before
    assert state.why == WHY["started"]
    assert frame(lib, state, 11, 49.6) == 1


# --- the ball's two points, for the tracker ------------------------------------


def test_a_fire_keeps_its_two_targets_for_the_ball_tracker(lib):
    """l3_ball_track_seed starts the flight from them (the tracker cannot start
    on the ball 4-6 bins out after this late fire)."""
    state = leave(lib)
    frame(lib, state, 10, 38.0, 47.0)
    assert frame(lib, state, 11, 39.0, 49.8) == 1
    first, second = state.startTarget, state.stepTarget
    assert (first.rangeBin, first.timestampUs) == (pytest.approx(47.0), 30_000)
    assert (second.rangeBin, second.timestampUs) == (pytest.approx(49.8), 33_000)


def test_a_restart_keeps_the_newest_start_target(lib):
    state = leave(lib)
    frame(lib, state, 10, 47.0)
    frame(lib, state, 11, 47.2)
    assert state.startTarget.rangeBin == pytest.approx(47.2)
    assert state.startTarget.timestampUs == 33_000


# --- the floor, for after impact -------------------------------------------------


def test_the_median_beyond_the_band_is_reported_as_the_floor(lib):
    """After impact the ball tracker scores only 16 bins beside the ball, whose
    median is no noise floor (the ball, the club, the ridge); this one, of the
    stretch the ball flies into, is: frozen, it kept 26/34 launches (a 16-bin
    median kept 14)."""
    state = leave(lib)
    floor = ctypes.c_float(-1.0)
    out = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    obs = window({50: 10.0}, noise=2.0)
    lib.l3_leave_targets(
        ctypes.byref(state.cfg),
        ctypes.byref(peak_params()),
        obs,
        30,
        len(obs),
        7,
        21_000,
        EDGE,
        out,
        fw.OBS_MAX_TARGETS,
        ctypes.byref(floor),
    )
    assert floor.value == pytest.approx(2.0)


def test_too_few_bins_leave_the_floor_untouched(lib):
    state = leave(lib)
    floor = ctypes.c_float(-1.0)
    out = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    obs = window({}, first=20, count=32)
    lib.l3_leave_targets(
        ctypes.byref(state.cfg),
        ctypes.byref(peak_params()),
        obs,
        20,
        len(obs),
        7,
        21_000,
        EDGE,
        out,
        fw.OBS_MAX_TARGETS,
        ctypes.byref(floor),
    )
    assert floor.value == pytest.approx(-1.0)
