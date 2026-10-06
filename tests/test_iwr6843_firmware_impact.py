"""Tests for the IWR6843 range-only impact, firmware/iwr6843/l3_impact.c.

The self-trigger fires on it: the club-in line fitted to the club track
(l3_impact_fit_track) crosses the ball's range within a horizon of the current
frame's time, or when an approaching club track ends near the ball: the
club's radar range at impact is 3-12 bins short of the ball's, so the crossing
alone often never comes. The geometric detector that judged the club's 3D line against
the ball's position was removed on 2026-09-30: nothing armed it on the kiosk
and it never fired on the recorded swings.
"""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw

WHY = {name: index for index, name in enumerate(fw.IMPACT_WHY_NAMES)}
FIT_WHY = {name: i for i, name in enumerate(fw.FIT_WHY_NAMES)}
CAUSE = {name: index for index, name in enumerate(fw.IMPACT_CAUSE_NAMES)}
BALL_M = 2.0


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def range_impact(lib, **overrides) -> fw.Impact:
    cfg = fw.ImpactCfg()
    lib.l3_impact_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    impact = fw.Impact()
    lib.l3_impact_init(ctypes.byref(impact), ctypes.byref(cfg))
    return impact


def club_in_estimate(time_us: float, why: str = "ok", speed_mps: float = 30.0) -> fw.FitEstimate:
    e = fw.FitEstimate()
    e.why, e.timeUs, e.sigmaUs, e.speedMps, e.points = FIT_WHY[why], time_us, 300.0, speed_mps, 4
    return e


def club_state(appended: bool, gap_m: float = 1.0, time_us: int = 0) -> fw.ImpactClub:
    """This frame's club track as the impact sees it: whether it took a point,
    its newest point ``gap_m`` short of a ball at BALL_M, and that point's time."""
    club = fw.ImpactClub()
    club.appended = int(appended)
    club.rangeM = BALL_M - gap_m
    club.timeUs = time_us
    club.ballRangeM = BALL_M
    return club


def update(lib, impact, estimate, now_us: int, club: fw.ImpactClub | None = None) -> int:
    ref = None if estimate is None else ctypes.byref(estimate)
    club_ref = None if club is None else ctypes.byref(club)
    return lib.l3_impact_update_range(ctypes.byref(impact), ref, club_ref, now_us)


def test_the_settings_are_the_horizon_the_end_distance_and_the_end_speed():
    assert [name for name, _type in fw.ImpactCfg._fields_] == ["horizonS", "endM", "endMinMps"]


def test_the_default_end_distance(lib):
    assert range_impact(lib).cfg.endM == pytest.approx(0.40)


def test_the_default_end_speed(lib):
    """Over the late backswing's downrange crossing (~17 m/s, the club-in
    fit's own floor) and under every approach that armed the end on the
    labelled swings (20.6-62 m/s)."""
    assert range_impact(lib).cfg.endMinMps == pytest.approx(20.0)


def test_the_default_horizon_value(lib):
    assert range_impact(lib).cfg.horizonS == pytest.approx(0.004)


def test_range_impact_waits_until_the_crossing_is_within_the_horizon(lib):
    impact = range_impact(lib)
    e = club_in_estimate(30_000)
    assert update(lib, impact, e, 20_000) == 0
    assert impact.why == WHY["pending"]
    assert update(lib, impact, e, 27_000) == 1
    assert impact.why == WHY["fired"]
    assert impact.impactTimestampUs == 30_000
    assert impact.offsetS == pytest.approx(0.003, abs=1e-6)


def test_a_crossing_just_passed_still_fires_and_dates_impact_before_the_frame(lib):
    impact = range_impact(lib)
    assert update(lib, impact, club_in_estimate(30_000), 32_000) == 1
    assert impact.impactTimestampUs == 30_000
    assert impact.offsetS == pytest.approx(-0.002, abs=1e-6)


def test_range_impact_long_past_is_passed_not_fired(lib):
    impact = range_impact(lib)
    assert update(lib, impact, club_in_estimate(30_000), 40_000) == 0
    assert impact.why == WHY["passed"]


@pytest.mark.parametrize("why", ["missing", "few_points", "speed_bounds"])
def test_range_impact_without_a_club_in_estimate_does_not_fire(lib, why):
    impact = range_impact(lib)
    assert update(lib, impact, club_in_estimate(30_000, why), 29_000) == 0
    assert impact.why == WHY["nodelivery"]
    assert update(lib, impact, None, 29_000) == 0
    assert impact.why == WHY["nodelivery"]


def test_a_wider_horizon_fires_earlier(lib):
    impact = range_impact(lib, horizonS=0.012)
    assert update(lib, impact, club_in_estimate(30_000), 20_000) == 1


def test_fires_once_until_rearmed_and_rearm_keeps_counters_and_config(lib):
    impact = range_impact(lib, horizonS=0.005)
    e = club_in_estimate(30_000)
    assert update(lib, impact, e, 29_000) == 1
    assert update(lib, impact, e, 30_000) == 0, "fired: later frames are ignored"
    lib.l3_impact_rearm(ctypes.byref(impact))
    assert impact.fired == 0 and impact.why == WHY["none"] and impact.impactTimestampUs == 0
    assert impact.counters[WHY["fired"]] == 1
    assert impact.cfg.horizonS == pytest.approx(0.005)
    assert update(lib, impact, e, 29_000) == 1


def test_range_impact_fires_across_the_uint32_wrap(lib):
    # The fitted crossing lies just past 2**32 (a float), now has wrapped to 0.
    impact = range_impact(lib)
    e = club_in_estimate(2.0**32 + 1024.0)
    assert lib.l3_impact_update_range(ctypes.byref(impact), ctypes.byref(e), None, 0) == 1
    assert impact.impactTimestampUs == 1024
    assert impact.offsetS == pytest.approx(0.001024, abs=1e-6)


def test_why_names_and_format(lib):
    assert fw.IMPACT_WHY_NAMES == ("none", "nodelivery", "pending", "passed", "fired")
    for index, name in enumerate(fw.IMPACT_WHY_NAMES):
        assert lib.l3_impact_why_name(index).decode() == name
    assert lib.l3_impact_why_name(99).decode() == "?"
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(30_000), 20_000)
    update(lib, impact, club_in_estimate(30_000), 27_500)
    text = fw.c_text(lib.l3_impact_format, ctypes.byref(impact), cap=240)
    assert text == (
        "impact fired=1 why=fired cause=crossing offsetms=2.50 t=30000 pending=1 passed=0 "
        "fired_n=1 armed=0"
    )


def test_the_geometric_detector_is_gone(lib):
    for gone in ("l3_impact_update", "l3_impact_closest"):
        assert not hasattr(lib, gone), gone
    for gone in ("closestM", "contact", "velocity"):
        assert gone not in {name for name, _type in fw.Impact._fields_}


def test_cause_names(lib):
    assert fw.IMPACT_CAUSE_NAMES == ("none", "crossing", "end")
    for index, name in enumerate(fw.IMPACT_CAUSE_NAMES):
        assert lib.l3_impact_cause_name(index).decode() == name
    assert lib.l3_impact_cause_name(99).decode() == "?"


# --- the approach ending near the ball ------------------------------------------
#
# The club's crossing of the ball's range is 3-12 bins (median 7.4) beyond
# where the radar last sees the club on the 34 labelled swings: at impact its
# return merges with the ball's and the club track stops taking points. The
# frame it stops, having last been seen close to the ball, is impact.


def test_a_track_that_ends_near_the_ball_fires_dated_to_its_last_point(lib):
    impact = range_impact(lib)
    e = club_in_estimate(60_000)  # the crossing is still far ahead: pending
    assert update(lib, impact, e, 24_000, club_state(True, 0.30, 24_000)) == 0
    assert impact.why == WHY["pending"] and impact.endArmed == 1
    assert update(lib, impact, e, 27_000, club_state(False)) == 1
    assert impact.why == WHY["fired"]
    assert impact.cause == CAUSE["end"]
    assert impact.impactTimestampUs == 24_000
    assert impact.offsetS == pytest.approx(-0.003, abs=1e-6)


def test_the_end_fires_when_the_track_is_released_and_its_estimate_is_gone(lib):
    """Released, the track has no points left: no club-in estimate, still the end."""
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(60_000), 24_000, club_state(True, 0.30, 24_000))
    assert update(lib, impact, None, 27_000, club_state(False)) == 1
    assert impact.cause == CAUSE["end"]


def test_a_track_that_ends_far_from_the_ball_does_not_fire(lib):
    impact = range_impact(lib)
    e = club_in_estimate(60_000)
    assert update(lib, impact, e, 24_000, club_state(True, 0.60, 24_000)) == 0
    assert impact.endArmed == 0
    assert update(lib, impact, e, 27_000, club_state(False)) == 0
    assert impact.fired == 0


def test_a_point_without_a_club_in_estimate_does_not_arm_the_end(lib):
    """A lone acquisition near the ball (clutter, a reacquired return) has no
    fitted approach behind it."""
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(60_000, "few_points"), 24_000, club_state(True, 0.1))
    assert impact.endArmed == 0
    assert update(lib, impact, None, 27_000, club_state(False)) == 0


def test_a_far_point_after_a_near_one_disarms_the_end(lib):
    impact = range_impact(lib)
    e = club_in_estimate(60_000)
    update(lib, impact, e, 24_000, club_state(True, 0.30, 24_000))
    update(lib, impact, e, 27_000, club_state(True, 0.60, 27_000))
    assert impact.endArmed == 0
    assert update(lib, impact, e, 30_000, club_state(False)) == 0


def test_the_end_is_off_with_a_zero_distance(lib):
    impact = range_impact(lib, endM=0.0)
    e = club_in_estimate(60_000)
    update(lib, impact, e, 24_000, club_state(True, 0.0, 24_000))
    assert impact.endArmed == 0
    assert update(lib, impact, e, 27_000, club_state(False)) == 0


def test_without_a_club_state_only_the_crossing_fires(lib):
    impact = range_impact(lib)
    assert update(lib, impact, club_in_estimate(30_000), 29_000) == 1
    assert impact.cause == CAUSE["crossing"]


def test_the_crossing_still_fires_on_a_frame_that_arms_the_end(lib):
    impact = range_impact(lib)
    assert update(lib, impact, club_in_estimate(30_000), 29_000, club_state(True, 0.1, 29_000))
    assert impact.cause == CAUSE["crossing"] and impact.impactTimestampUs == 30_000


def test_rearm_clears_the_armed_end(lib):
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(60_000), 24_000, club_state(True, 0.30, 24_000))
    lib.l3_impact_rearm(ctypes.byref(impact))
    assert impact.endArmed == 0 and impact.cause == CAUSE["none"]
    assert update(lib, impact, None, 27_000, club_state(False)) == 0


# --- backswing false fires (bench, 2026-10) ---------------------------------
#
# 129 fires for 36 TrackMan shots: 15 fired 0.54-0.82 s early, at takeaway,
# one with cause=end on an empty club track. The end armed on any approach
# the club-in fit accepted (17 m/s and up) within endM of the ball on either
# side, so a slow downrange crossing or a return past the ball armed it and
# the frame the track was released fired it.


def test_a_slow_approach_near_the_ball_does_not_arm_the_end(lib):
    """A backswing's downrange crossing passes the club-in fit's 17 m/s floor
    but is slower than any downswing that reached the ball."""
    impact = range_impact(lib)
    e = club_in_estimate(60_000, speed_mps=18.0)
    assert update(lib, impact, e, 24_000, club_state(True, 0.20, 24_000)) == 0
    assert impact.endArmed == 0
    assert update(lib, impact, None, 27_000, club_state(False)) == 0
    assert impact.fired == 0


def test_an_approach_at_the_end_speed_arms_the_end(lib):
    impact = range_impact(lib)
    e = club_in_estimate(60_000, speed_mps=20.0)
    update(lib, impact, e, 24_000, club_state(True, 0.20, 24_000))
    assert impact.endArmed == 1
    assert update(lib, impact, None, 27_000, club_state(False)) == 1
    assert impact.cause == CAUSE["end"]


def test_a_zero_end_speed_arms_on_any_usable_approach(lib):
    impact = range_impact(lib, endMinMps=0.0)
    update(
        lib, impact, club_in_estimate(60_000, speed_mps=17.5), 24_000, club_state(True, 0.2, 24_000)
    )
    assert impact.endArmed == 1


def test_a_point_past_the_ball_does_not_arm_the_end(lib):
    """Before impact the club is short of the ball: a return beyond it (the
    golfer, the net, the bay) is not an approach ending at the ball."""
    impact = range_impact(lib)
    e = club_in_estimate(60_000)
    assert update(lib, impact, e, 24_000, club_state(True, -0.30, 24_000)) == 0
    assert impact.endArmed == 0
    assert update(lib, impact, None, 27_000, club_state(False)) == 0
    assert impact.fired == 0


def test_a_point_at_the_ball_arms_the_end(lib):
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(60_000), 24_000, club_state(True, 0.0, 24_000))
    assert impact.endArmed == 1


def test_a_slow_point_after_a_fast_one_disarms_the_end(lib):
    """The newest approach point decides, as the distance does."""
    impact = range_impact(lib)
    update(lib, impact, club_in_estimate(60_000), 24_000, club_state(True, 0.30, 24_000))
    update(
        lib, impact, club_in_estimate(60_000, speed_mps=18.0), 27_000, club_state(True, 0.2, 27_000)
    )
    assert impact.endArmed == 0
    assert update(lib, impact, None, 30_000, club_state(False)) == 0
