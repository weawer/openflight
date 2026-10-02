"""OPS-versus-IWR speed comparison records and their session statistics."""

from __future__ import annotations

import json
import math
from datetime import datetime
from types import SimpleNamespace

import pytest

from openflight.clubs import ClubType
from openflight.iwr6843.ops_compare import (
    BALL_TOLERANCE,
    CLUB_TOLERANCE,
    MIN_AGREEMENT_CONFIDENCE,
    MPS_TO_MPH,
    ErrorStats,
    OpsComparison,
    format_summary,
    group_by,
    read_session,
    read_sessions,
    summarize,
)


def _metric(value, confidence=0.8, usable=None):
    return SimpleNamespace(
        value=value, confidence=confidence, usable=(value is not None) if usable is None else usable
    )


def _onboard(ball=48.0, club=36.0, verdict="valid", ball_conf=0.9, club_conf=0.6, rng=1.6):
    metrics = {
        "ball_speed": _metric(ball, ball_conf),
        "club_speed": _metric(club, club_conf),
        "impact_range": _metric(rng, 0.5),
    }
    return SimpleNamespace(verdict=verdict, __getitem__=lambda self, k: metrics[k], metrics=metrics)


class _Onboard:
    def __init__(self, **kw):
        self._inner = _onboard(**kw)
        self.verdict = self._inner.verdict

    def __getitem__(self, key):
        return self._inner.metrics[key]


def _shot(ball=110.0, club=82.0, number=3):
    return SimpleNamespace(
        shot_number=number,
        timestamp=datetime(2026, 9, 27, 12, 0, 0),
        club=ClubType.IRON_7,
        ball_speed_mph=ball,
        club_speed_mph=club,
    )


def test_record_from_shot_keeps_both_sides_and_the_iwr_confidence():
    record = OpsComparison.from_shot(_shot(), _Onboard(), capture_format="adaptive16")
    assert (
        record.shot_number == 3
        and record.club == "7-iron"
        and record.capture_format == "adaptive16"
    )
    assert record.ops_ball_speed_mph == 110.0 and record.ops_club_speed_mph == 82.0
    assert record.iwr_ball_speed_mph == pytest.approx(48.0 * MPS_TO_MPH)
    assert record.iwr_ball_confidence == 0.9 and record.iwr_club_confidence == 0.6
    assert record.ball_delta_mph == pytest.approx(48.0 * MPS_TO_MPH - 110.0)
    assert record.ball_percent == pytest.approx(100.0 * (48.0 * MPS_TO_MPH - 110.0) / 110.0)
    assert record.impact_range_m == 1.6 and record.verdict == "valid"
    data = record.to_dict()
    assert data["club_delta_mph"] == pytest.approx(36.0 * MPS_TO_MPH - 82.0)
    assert json.dumps(data)
    assert OpsComparison.from_dict(data) == record


def test_unusable_iwr_metrics_leave_no_delta():
    record = OpsComparison.from_shot(_shot(), _Onboard(ball=None, club=20.0), capture_format=None)
    assert record.iwr_ball_speed_mph is None and record.ball_delta_mph is None
    assert record.iwr_ball_confidence is None
    assert record.club_delta_mph is not None
    zero = OpsComparison.from_shot(_shot(ball=0.0), _Onboard(), capture_format=None)
    assert zero.ball_percent is None, "no percentage of a zero reference"


def test_error_stats_and_summary():
    stats = ErrorStats.of([1.0, -1.0, 2.0, -2.0, 0.5])
    assert stats.count == 5 and stats.bias == pytest.approx(0.1)
    assert stats.mae == pytest.approx(1.3) and stats.rmse == pytest.approx(math.sqrt(10.25 / 5))
    assert stats.p95_abs == 2.0 and stats.max_abs == 2.0
    assert ErrorStats.of([]).count == 0 and ErrorStats.of([float("nan")]).bias is None
    records = [
        OpsComparison.from_shot(
            _shot(ball=110.0),
            _Onboard(ball=110.0 / MPS_TO_MPH + 1.0 / MPS_TO_MPH),
            capture_format="iq16",
        ),
        OpsComparison.from_shot(_shot(ball=100.0), _Onboard(ball=None), capture_format="iq16"),
    ]
    summary = summarize(records)
    assert summary.shots == 2 and summary.with_result == 2
    assert summary.ball.count == 1 and summary.ball.bias == pytest.approx(1.0, abs=1e-6)
    text = format_summary(summary)
    assert "2 shots" in text and "ball speed" in text and "mph" in text and "%" in text


def test_group_by_club_format_verdict_and_confidence_band():
    records = [
        OpsComparison.from_shot(_shot(), _Onboard(ball_conf=0.95), capture_format="iq16"),
        OpsComparison.from_shot(
            _shot(), _Onboard(ball_conf=0.75, verdict="partial"), capture_format="adaptive16"
        ),
        OpsComparison.from_shot(_shot(), _Onboard(ball=None), capture_format=None),
    ]
    assert set(group_by(records, "capture_format")) == {"iq16", "adaptive16", "unknown"}
    assert set(group_by(records, "verdict")) == {"valid", "partial"}
    bands = group_by(records, "ball_confidence")
    assert set(bands) == {"high", "medium", "none"}
    assert set(group_by(records, "club")) == {"7-iron"}


def test_session_files_are_read_back(tmp_path):
    record = OpsComparison.from_shot(_shot(), _Onboard(), capture_format="iq16")
    path = tmp_path / "session_1.jsonl"
    path.write_text(
        json.dumps({"type": "shot_detected", "shot_number": 3})
        + "\n"
        + json.dumps({"ts": "t", "type": "iwr_ops_comparison", **record.to_dict()})
        + "\n"
        + "garbage\n",
        encoding="utf-8",
    )
    assert list(read_session(path)) == [record]
    assert read_sessions([tmp_path]) == [record]
    assert read_sessions([tmp_path / "missing_dir"]) == []


# --- Agreement: a confidence-scaled tolerance on the IWR-minus-OPS percent ---


def _record(
    *,
    ops_ball=100.0,
    iwr_ball=None,
    ball_conf=None,
    ops_club=80.0,
    iwr_club=None,
    club_conf=None,
    verdict="valid",
):
    return OpsComparison(
        shot_number=1,
        timestamp="2026-10-02T12:00:00",
        club="7-iron",
        capture_format=None,
        verdict=verdict,
        ops_ball_speed_mph=ops_ball,
        ops_club_speed_mph=ops_club,
        iwr_ball_speed_mph=iwr_ball,
        iwr_ball_confidence=ball_conf,
        iwr_club_speed_mph=iwr_club,
        iwr_club_confidence=club_conf,
        impact_range_m=None,
    )


def test_tolerance_constants_are_pinned():
    """Placeholders until ops_validation.py has real sessions to retune them from."""
    assert (BALL_TOLERANCE.base_pct, BALL_TOLERANCE.max_pct) == (3.0, 9.0)
    assert (CLUB_TOLERANCE.base_pct, CLUB_TOLERANCE.max_pct) == (5.0, 15.0)
    assert MIN_AGREEMENT_CONFIDENCE == 0.3


@pytest.mark.parametrize(
    ("confidence", "expected"),
    [(1.0, 3.0), (0.9, 3.0 / 0.9), (0.5, 6.0), (1.0 / 3.0, 9.0), (0.3, 9.0)],
)
def test_ball_tolerance_widens_as_confidence_drops_up_to_its_cap(confidence, expected):
    assert BALL_TOLERANCE.pct(confidence) == pytest.approx(expected)


def test_tolerance_needs_the_minimum_confidence():
    with pytest.raises(ValueError, match="confidence"):
        BALL_TOLERANCE.pct(0.29)


@pytest.mark.parametrize(
    ("iwr_ball", "expected"),
    [(106.0, "agree"), (94.0, "agree"), (106.01, "disagree"), (93.99, "disagree")],
)
def test_ball_agreement_is_inclusive_at_the_tolerance(iwr_ball, expected):
    record = _record(iwr_ball=iwr_ball, ball_conf=0.5)
    assert record.ball_agreement == expected
    assert record.ball_tolerance_pct == pytest.approx(6.0)


def test_club_agreement_uses_the_club_tolerance():
    agree = _record(iwr_club=84.0, club_conf=1.0)
    disagree = _record(iwr_club=84.01, club_conf=1.0)
    assert agree.club_agreement == "agree" and agree.club_tolerance_pct == pytest.approx(5.0)
    assert disagree.club_agreement == "disagree"


@pytest.mark.parametrize(
    "overrides",
    [
        {"iwr_ball": None, "ball_conf": 0.9},  # board value missing or implausible
        {"iwr_ball": 101.0, "ball_conf": 0.9, "ops_ball": None},  # OPS missing
        {"iwr_ball": 101.0, "ball_conf": 0.29},  # below the minimum confidence
        {"iwr_ball": 101.0, "ball_conf": None},
        {"iwr_ball": 101.0, "ball_conf": 0.9, "verdict": "invalid"},
        {"iwr_ball": 101.0, "ball_conf": 0.9, "verdict": None},  # no onboard result
    ],
    ids=["no_iwr", "no_ops", "low_confidence", "no_confidence", "invalid", "no_result"],
)
def test_ball_agreement_is_unchecked_without_a_fair_comparison(overrides):
    record = _record(**overrides)
    assert record.ball_agreement == "unchecked"
    assert record.ball_tolerance_pct is None


def test_agreement_rides_in_the_dict_and_survives_a_round_trip():
    record = _record(iwr_ball=101.0, ball_conf=1.0, iwr_club=95.0, club_conf=1.0)
    data = record.to_dict()
    assert data["ball_agreement"] == "agree" and data["ball_tolerance_pct"] == pytest.approx(3.0)
    assert data["club_agreement"] == "disagree"
    assert data["club_tolerance_pct"] == pytest.approx(5.0)
    assert json.dumps(data)
    assert OpsComparison.from_dict(data) == record
