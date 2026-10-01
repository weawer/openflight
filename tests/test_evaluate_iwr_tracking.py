"""Tests for scripts/analysis/evaluate_iwr_tracking.py."""

from __future__ import annotations

import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from iwr6843_synth import synth_shot_dump

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.dump import SAMPLE_INT16_IQ, pack_dump

SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "evaluate_iwr_tracking.py"
BIN_M = 6.0 / 128


@pytest.fixture(scope="module")
def ev():
    spec = importlib.util.spec_from_file_location("evaluate_iwr_tracking", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module by name
    spec.loader.exec_module(module)
    return module


def point(frame, range_bin):
    return SimpleNamespace(frame=frame, range_bin=range_bin)


def frame_of(frame, timestamp_us, bins):
    return SimpleNamespace(
        frame=frame,
        timestamp_us=timestamp_us,
        targets=[SimpleNamespace(range_bin=b) for b in bins],
    )


def test_club_verdict_reads_the_last_points_before_the_split(ev):
    assert ev.club_verdict([point(f, 30.0 + f) for f in range(6)], 6) == "club"
    assert ev.club_verdict([point(f, 43.5 + 0.1 * f) for f in range(6)], 6) == "stuck"
    assert ev.club_verdict([point(0, 30.0), point(1, 31.0)], 6) == "few"
    after = [point(f, 30.0 + f) for f in range(4)] + [point(f, 50.0) for f in range(4, 9)]
    assert ev.club_verdict(after, 4) == "club"  # points at or after the split are ignored
    assert ev.club_verdict([point(f, 30.0 + f) for f in range(6)], None) == "club"


def test_ball_verdict_is_fifteen_percent_either_side(ev):
    assert ev.ball_verdict(None, 40.0) == "none"
    assert ev.ball_verdict(46.0, 40.0) == "ok"
    assert ev.ball_verdict(34.0, 40.0) == "ok"
    assert ev.ball_verdict(46.1, 40.0) == "wrong"
    assert ev.ball_verdict(13.2, 33.9) == "wrong"


def test_ball_present_needs_three_consecutive_points_at_the_ops_speed(ev):
    step = 40.0 * 0.002 / BIN_M
    frames = [frame_of(k, 2000 * k, [44.0, 50.0 + step * k]) for k in range(1, 4)]
    assert ev.ball_present(frames, 40.0, BIN_M) is True
    assert ev.ball_present(frames[:2], 40.0, BIN_M) is False
    assert ev.ball_present(frames, 20.0, BIN_M) is False  # twice the speed OPS saw
    gap = [frames[0], frames[1], frame_of(4, 8000, [50.0 + step * 4])]
    assert ev.ball_present(gap, 40.0, BIN_M) is False  # frame 3 missing breaks the chain
    assert ev.ball_present([], 40.0, BIN_M) is False


def test_split_frame_prefers_the_recorded_freeze(ev):
    with_freeze = SimpleNamespace(config=SimpleNamespace(post_from_frame=14), fired_frame=11)
    gate_only = SimpleNamespace(config=SimpleNamespace(post_from_frame=None), fired_frame=11)
    no_fire = SimpleNamespace(config=SimpleNamespace(post_from_frame=None), fired_frame=None)
    assert ev.split_frame(with_freeze) == 14
    assert ev.split_frame(gate_only) == 12
    assert ev.split_frame(no_fire) is None


def _outcome(ev, club, ball, present):
    return ev.Outcome("x.l3dump", club, ball, present, None, 40.0)


def test_summarize_counts_every_category(ev):
    outcomes = [
        _outcome(ev, "club", "ok", True),
        _outcome(ev, "stuck", "none", False),
        _outcome(ev, "few", "wrong", True),
    ]
    assert ev.summarize(outcomes) == {
        "captures": 3,
        "club": {"club": 1, "stuck": 1, "few": 1},
        "ball": {"ok": 1, "wrong": 1, "none": 1},
        "ball_present": 2,
    }


def test_compare_names_each_regression(ev):
    base = {
        "captures": 93,
        "club": {"club": 55, "stuck": 32, "few": 6},
        "ball": {"ok": 15, "wrong": 70, "none": 8},
        "ball_present": 23,
    }
    same = json.loads(json.dumps(base))
    assert ev.compare(same, base) == []
    worse = json.loads(json.dumps(base))
    worse["club"]["club"] = 54
    worse["ball"]["ok"] = 14
    worse["ball"]["none"] = 10
    problems = ev.compare(worse, base)
    assert len(problems) == 3
    assert ev.compare(worse, base, allow_more_none=2) == problems[:2]
    other = json.loads(json.dumps(base))
    other["captures"] = 92
    assert ev.compare(other, base) == ["captures: 92 vs baseline 93"]


def test_a_modern_session_with_a_slant_range_but_no_triggercfg_is_still_a_case(ev, tmp_path):
    """Regression: real sessions log ``tee_slant_range_m`` (no ``self_trigger``
    triggerCfg string), which the defaults dict surfaces as ``tee_range_m``.
    iter_cases must accept that as knowing the gate, not only a raw tee_bin."""
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    dump = dumps / "iwr6843_20990101_000000_000_001.l3dump"
    dump.write_bytes(synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372))
    rows = [
        {
            "type": "session_start",
            "trigger_type": "sound",
            "config": {"iwr6843": {"tee_slant_range_m": 1.372}},
        },
        {
            "type": "iwr6843_capture",
            "shot_number": 1,
            "capture_path": f"/home/pi/{dump.name}",
            "ball_speed_mph": 60.0 / 0.44704,
        },
        {"type": "shot_detected", "shot_number": 1, "club": "Driver"},
    ]
    (tmp_path / "session_x.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8"
    )
    cases = list(ev.iter_cases([tmp_path]))
    # The log holds the tape reading from the enclosure front: 1.372 m plus the array's
    # 0.30 m is 1.672 m from the antenna, bin 36 (bin 29 was the tape reading alone).
    assert len(cases) == 1 and cases[0].config.tee_bin == 36


def test_a_raw_adc_capture_is_skipped_instead_of_crashing_the_batch(ev, tmp_path):
    """Regression: a raw ADC dump (no range-FFT snapshot) has everything
    iter_cases wants (OPS speed, a known gate) but replay_dump refuses it.
    One such capture used to blow up the whole batch; it must be skipped
    the same way an unparseable header already is."""
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    dump = dumps / "iwr6843_20990101_000000_000_001.l3dump"
    cube = np.zeros((2, 36, 4, 64), dtype=complex)
    dump.write_bytes(pack_dump(cube, n_tx=3, version=3, sample_fmt=SAMPLE_INT16_IQ))
    rows = [
        {
            "type": "session_start",
            "trigger_type": "sound",
            "config": {"iwr6843": {"self_trigger": "triggerCfg 29 6.0 2"}},
        },
        {
            "type": "iwr6843_capture",
            "shot_number": 1,
            "capture_path": f"/home/pi/{dump.name}",
            "ball_speed_mph": 60.0 / 0.44704,
        },
        {"type": "shot_detected", "shot_number": 1, "club": "Driver"},
    ]
    (tmp_path / "session_x.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8"
    )
    assert list(ev.iter_cases([tmp_path])) == []


@pytest.mark.skipif(fw.host_compiler() is None, reason="no C compiler for the firmware modules")
def test_a_synthetic_shot_with_its_session_log_is_scored_end_to_end(ev, tmp_path):
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    dump = dumps / "iwr6843_20990101_000000_000_001.l3dump"
    dump.write_bytes(
        synth_shot_dump(ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=1.372)
    )
    rows = [
        {
            "type": "session_start",
            "trigger_type": "sound",
            "config": {"iwr6843": {"self_trigger": "triggerCfg 29 6.0 2"}},
        },
        {
            "type": "iwr6843_capture",
            "shot_number": 1,
            "capture_path": f"/home/pi/{dump.name}",
            "ball_speed_mph": 60.0 / 0.44704,
        },
        {"type": "shot_detected", "shot_number": 1, "club": "Driver"},
    ]
    (tmp_path / "session_x.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8"
    )
    cases = list(ev.iter_cases([tmp_path]))
    assert len(cases) == 1 and cases[0].config.tee_bin == 29
    assert cases[0].club == "Driver"
    outcome = ev.evaluate(cases[0])
    assert (outcome.club, outcome.ball, outcome.ball_present) == ("club", "ok", True)
    # The synthetic scene is level; the logged config's 10 deg pitch would tilt it and
    # (rightly) make the ball fit report the direction as uncertain, so judge HLA level.
    level = replace(cases[0], config=replace(cases[0].config, pitch_deg=0.0))
    assert ev.evaluate(level).launch_hla_deg == pytest.approx(0.0, abs=1.0)
    # The driver's 50 m/s floor sits below this 60 m/s ball: still found.
    assert ev.evaluate(cases[0], ball_hypotheses=True, fast_ball_from_club=True).ball == "ok"
    assert ev.main([str(tmp_path), "--json", str(tmp_path / "out.json")]) == 0
    written = json.loads((tmp_path / "out.json").read_text(encoding="utf-8"))
    assert written["summary"]["captures"] == 1
    assert "impact" not in written and written["outcomes"][0]["impact"] is None
    # --impact adds the impact summary and each capture's impact outcome.
    argv = [str(tmp_path), "--impact", "--band-bins", "6", "--json", str(tmp_path / "i.json")]
    assert ev.main(argv) == 0
    written = json.loads((tmp_path / "i.json").read_text(encoding="utf-8"))
    assert written["impact"]["captures"] == 1
    assert written["impact"]["club_points_in_band"] == 0
    assert written["outcomes"][0]["impact"]["name"] == dump.name


def test_the_cli_passes_the_ball_search_through(ev, monkeypatch, tmp_path):
    seen = []
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter([object()]))

    def fake_evaluate(case, *, lib=None, ball_hypotheses=None, **_tuning):
        seen.append(ball_hypotheses)
        return ev.Outcome("x", "club", "ok", True, 40.0, 40.0)

    monkeypatch.setattr(ev, "evaluate", fake_evaluate)
    assert ev.main([str(tmp_path), "--ball-hypotheses", "on"]) == 0
    assert ev.main([str(tmp_path), "--ball-hypotheses", "off"]) == 0
    assert ev.main([str(tmp_path)]) == 0
    assert seen == [True, False, None]


def args_for(ev, *extra):
    """The parsed command line, as main sees it."""
    seen = {}

    def fake_evaluate(
        case,
        *,
        lib=None,
        ball_hypotheses=None,
        tuning=None,
        fast_ball_from_club,
        band_bins=None,
        impact=False,
    ):
        seen.update(tuning=tuning, from_club=fast_ball_from_club)
        return ev.Outcome("x", "club", "ok", True, 40.0, 40.0)

    return seen, fake_evaluate


def test_no_rule_flags_means_no_tuning(ev, monkeypatch, tmp_path):
    seen, fake = args_for(ev)
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter([object()]))
    monkeypatch.setattr(ev, "evaluate", fake)
    assert ev.main([str(tmp_path)]) == 0
    assert seen == {"tuning": None, "from_club": False}


def test_the_cli_passes_the_pi_detector_rules_through(ev, monkeypatch, tmp_path):
    seen, fake = args_for(ev)
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter([object()]))
    monkeypatch.setattr(ev, "evaluate", fake)
    argv = [
        str(tmp_path),
        "--fast-ball",
        "34",
        "--fast-support",
        "0.5",
        "--min-departure-mps",
        "15",
        "--far-window-bins",
        "3",
    ]
    assert ev.main(argv) == 0
    assert seen["tuning"] == ev.fr.BallTuning(
        fast_ball_mps=34.0,
        fast_support_fraction=0.5,
        min_departure_mps=15.0,
        far_window_bins=3.0,
    )
    assert seen["from_club"] is False
    assert ev.main([str(tmp_path), "--fast-ball", "club"]) == 0
    assert seen == {"tuning": None, "from_club": True}


def test_a_bad_fast_ball_value_is_refused(ev, tmp_path):
    with pytest.raises(SystemExit, match="--fast-ball needs m/s or 'club'"):
        ev.main([str(tmp_path), "--fast-ball", "quick"])


@pytest.mark.parametrize(
    ("club", "floor"),
    [("SandWedge", 26.5), ("7i", 34.0), ("5 iron", 40.0), ("Driver", 50.0), (None, 30.0)],
)
def test_the_club_floor_is_the_pi_detectors_class_floor(ev, club, floor):
    assert ev.club_fast_ball_mps(club) == floor


def test_the_club_floor_fills_in_the_runs_tuning(ev, tmp_path):
    case = ev.Case(tmp_path / "x.l3dump", ev.fr.ReplayConfig(tee_bin=29), 40.0, club="Driver")
    assert ev.tuning_for(case, None) is None
    assert ev.tuning_for(case, None, fast_ball_from_club=True) == ev.fr.BallTuning(
        fast_ball_mps=50.0
    )
    kept = ev.fr.BallTuning(far_window_bins=3.0, fast_ball_mps=20.0)
    assert ev.tuning_for(case, kept, fast_ball_from_club=True) == ev.fr.BallTuning(
        far_window_bins=3.0, fast_ball_mps=50.0
    )


def test_band_bins_reaches_the_replay_config_only_when_given(ev, monkeypatch, tmp_path):
    configs = []

    def fake_replay(data, config, lib=None):
        configs.append(config)
        return SimpleNamespace(config=config, frames=[], points=[], fired_frame=None, launch=None)

    monkeypatch.setattr(ev.fr, "replay_dump", fake_replay)
    path = tmp_path / "x.l3dump"
    path.write_bytes(b"")
    case = ev.Case(path, ev.fr.ReplayConfig(tee_bin=29), 40.0)
    ev.evaluate(case)
    ev.evaluate(case, band_bins=6.0)
    ev.evaluate(case, band_bins=0.0)
    assert [c.band_bins for c in configs] == [None, 6.0, 0.0]
    assert configs[0] == case.config  # unset: the case's own config, unchanged


def test_evaluate_attaches_the_impact_outcome_only_when_asked(ev, monkeypatch, tmp_path):
    result = SimpleNamespace(
        config=ev.fr.ReplayConfig(tee_bin=29), frames=[], points=[], fired_frame=None, launch=None
    )
    monkeypatch.setattr(ev.fr, "replay_dump", lambda data, config, lib=None: result)
    marker = object()
    monkeypatch.setattr(ev.impact_eval, "impact_outcome", lambda name, r: (name, r, marker))
    path = tmp_path / "x.l3dump"
    path.write_bytes(b"")
    case = ev.Case(path, ev.fr.ReplayConfig(tee_bin=29), 40.0)
    assert ev.evaluate(case).impact is None
    assert ev.evaluate(case, impact=True).impact == ("x.l3dump", result, marker)


def test_the_cli_passes_band_bins_and_impact_through(ev, monkeypatch, tmp_path, capsys):
    seen = []
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter([object()]))
    impact = ev.impact_eval.ImpactOutcome("x", "consistent", ("club_in",), 100.0, -1.0, None, 0)

    def fake_evaluate(case, *, band_bins=None, impact=False, **_rest):
        seen.append((band_bins, impact))
        return ev.Outcome(
            "x", "club", "ok", True, 40.0, 40.0, impact=impact_outcome if impact else None
        )

    impact_outcome = impact
    monkeypatch.setattr(ev, "evaluate", fake_evaluate)
    assert ev.main([str(tmp_path)]) == 0
    assert "impact:" not in capsys.readouterr().out
    out = tmp_path / "o.json"
    assert ev.main([str(tmp_path), "--band-bins", "6", "--impact", "--json", str(out)]) == 0
    assert "impact:" in capsys.readouterr().out
    assert seen == [(None, False), (6.0, True)]
    written = json.loads(out.read_text(encoding="utf-8"))
    assert written["impact"] == ev.impact_eval.summarize_impact([impact])
