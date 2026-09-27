"""Tracked flight range rate vs OPS ball speed (flight_track.py)."""

from __future__ import annotations

from pathlib import Path

import pytest

from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.flight_track import (
    BIN_M,
    FOLLOWS_BALL_RATIO,
    MPS_TO_MPH,
    FlightTrack,
    measure_flight_track,
    score_flight_track,
)
from openflight.iwr6843.monitor import read_capture_config

ROOT = Path(__file__).parents[1]
DUMPS = ROOT / "openflight_sessions" / "iwr6843"
PRE, IMPACT = 14, 6


def _adaptive_meta(flight_starts, *, step_us=2000, reason="track_lost"):
    starts = [22] * PRE + [34] * IMPACT + list(flight_starts)
    counts = [32] * PRE + [53] * IMPACT + [12] * len(flight_starts)
    return {
        "retention": {"reason": reason, "pre_frames": PRE, "planned_frames": 36},
        "range_bin_starts": tuple(starts),
        "range_bin_counts": tuple(counts),
        "frame_time_offsets_us": tuple(
            [2000 * i for i in range(PRE + IMPACT)]
            + [2000 * (PRE + IMPACT - 1) + step_us * (k + 1) for k in range(len(flight_starts))]
        ),
    }


def _mph(bins_per_frame, frame_s=0.002):
    return bins_per_frame * BIN_M / frame_s * MPS_TO_MPH


def test_outbound_windows_give_their_range_rate():
    track = measure_flight_track(_adaptive_meta([40 + 2 * k for k in range(8)]), impact_frames=6)

    assert track.flight_frames == 8
    assert track.start_bin == 46.0
    assert track.end_bin == 60.0
    assert track.range_rate_mph == pytest.approx(_mph(2.0))


def test_rate_uses_frame_times_not_frame_indices():
    """Ball-phase stride >1 retains every Nth acquisition; indices would
    overstate the rate by that factor."""
    track = measure_flight_track(
        _adaptive_meta([40 + 2 * k for k in range(8)], step_us=4000), impact_frames=6
    )

    assert track.range_rate_mph == pytest.approx(_mph(2.0) / 2)


def test_stationary_windows_read_zero():
    track = measure_flight_track(_adaptive_meta([41] * 16, reason="complete"), impact_frames=6)

    assert track.range_rate_mph == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("flight", [0, 1, 2])
def test_too_few_flight_frames_have_no_rate(flight):
    track = measure_flight_track(_adaptive_meta([40 + k for k in range(flight)]), impact_frames=6)

    assert track.flight_frames == flight
    assert track.range_rate_mph is None
    assert score_flight_track(track, 80.0)["status"] == "too_few_flight_frames"


def test_fixed_window_capture_has_no_flight_track():
    meta = _adaptive_meta([47] * 8)
    del meta["retention"]

    assert measure_flight_track(meta, impact_frames=6) is None
    assert score_flight_track(None, 80.0) is None


def _track(rate_mph):
    return FlightTrack(flight_frames=8, start_bin=46.0, end_bin=60.0, range_rate_mph=rate_mph)


@pytest.mark.parametrize(
    "ratio,status",
    [
        (1.0, "follows_ball"),
        (FOLLOWS_BALL_RATIO[0], "follows_ball"),
        (FOLLOWS_BALL_RATIO[1], "follows_ball"),
        (0.7, "not_ball"),  # post-impact clubhead speed relative to the ball
        (FOLLOWS_BALL_RATIO[1] + 0.01, "not_ball"),
        (0.0, "not_ball"),
        (-0.2, "not_ball"),
    ],
)
def test_score_band(ratio, status):
    record = score_flight_track(_track(80.0 * ratio), 80.0)

    assert record["status"] == status
    assert record["speed_ratio"] == pytest.approx(ratio)
    assert record["ops_ball_mph"] == 80.0


@pytest.mark.parametrize("ops", [None, 0.0])
def test_missing_ops_speed_is_reported_not_scored(ops):
    record = score_flight_track(_track(80.0), ops)

    assert record["status"] == "no_ops_speed"
    assert record["speed_ratio"] is None
    assert record["range_rate_mph"] == 80.0


@pytest.mark.parametrize(
    "dump,ops_mph,retention,status",
    [
        # session 180613 shot 2: retention "complete", but the window drifted
        # inward past the tee -- the selector held clutter, not the ball.
        ("iwr6843_20260927_180742_053_003.l3dump", 76.9, "complete", "not_ball"),
        # session 184327 shot 9 and 182836 shot 5: ball-rate tracks that
        # retention reported as track_lost.
        ("iwr6843_20260927_185056_218_015.l3dump", 83.9, "track_lost", "follows_ball"),
        ("iwr6843_20260927_183527_246_008.l3dump", 90.1, "track_lost", "follows_ball"),
    ],
)
def test_retention_reason_does_not_say_whether_the_ball_was_tracked(
    dump, ops_mph, retention, status
):
    meta = parse_dump((DUMPS / dump).read_bytes())[0]

    record = score_flight_track(measure_flight_track(meta, impact_frames=6), ops_mph)

    assert meta["retention"]["reason"] == retention
    assert record["status"] == status


@pytest.mark.parametrize(
    "config,pre_frames,impact_frames",
    [
        ("iwr6843_l3dump_adaptive_36f2ms_iq16.cfg", 14, 6),
        ("iwr6843_l3dump_diagnostic_24f2ms_53bin_iq16.cfg", 9, 7),
    ],
)
def test_capture_config_reports_phase_frames(config, pre_frames, impact_frames):
    summary = read_capture_config(ROOT / "config" / config)

    assert summary.pre_frames == pre_frames
    assert summary.impact_frames == impact_frames
