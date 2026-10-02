"""Tests for scripts/analysis/evaluate_iwr_tracking.py."""

from __future__ import annotations

import importlib.util
import json
import sys
from dataclasses import asdict, replace
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
    return ev.Outcome("x.l3dump", club, ball, present, present, None, 40.0)


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
        "ball_present_strict": 2,
        "ball_by_presence": {
            "present": {"ok": 1, "wrong": 1, "none": 0},
            "absent": {"ok": 0, "wrong": 0, "none": 1},
        },
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
    # 30 mm is 1.402 m from the antenna, bin 30.
    assert len(cases) == 1 and cases[0].config.tee_bin == 30


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
        return ev.Outcome("x", "club", "ok", True, True, 40.0, 40.0)

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
        net_range_m=None,
    ):
        seen.update(tuning=tuning, from_club=fast_ball_from_club)
        return ev.Outcome("x", "club", "ok", True, True, 40.0, 40.0)

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
        "--far-window-m",
        "0.14",
        "--corridor-gate",
        "off",
        "--impact-coast-ms",
        "24",
        "--max-decel",
        "150",
        "--classify-points",
        "6",
        "--recover",
        "off",
        "--recover-gate-m",
        "0.05",
        "--history-snr",
        "0.7",
    ]
    assert ev.main(argv) == 0
    assert seen["tuning"] == ev.fr.BallTuning(
        fast_ball_mps=34.0,
        fast_support_fraction=0.5,
        min_departure_mps=15.0,
        far_window_m=0.14,
        corridor_gate=False,
        impact_coast_ms=24.0,
        max_decel_mps2=150.0,
        classify_points=6,
        recover=False,
        recover_gate_m=0.05,
        history_snr=0.7,
    )
    assert ev.main([str(tmp_path), "--corridor-gate", "on", "--recover", "on"]) == 0
    assert (seen["tuning"].corridor_gate, seen["tuning"].recover) == (True, True)
    assert seen["from_club"] is False
    assert ev.main([str(tmp_path), "--fast-ball", "club"]) == 0
    assert seen == {"tuning": None, "from_club": True}


def pts(*rows):
    return [SimpleNamespace(timestamp_us=us, range_m=m) for us, m in rows]


def test_net_reached_when_the_track_arrives_on_time(ev):
    # tee 2.0 m, net 4.5 m, 50 m/s from impact 0: due at 50 ms; 3 ms frames, +-6 ms
    assert ev.net_reached(pts((47_000, 4.40), (50_000, 4.50)), 50.0, 0, 2.0, 4.5, 3000) is True
    assert ev.net_reached(pts((60_000, 4.50)), 50.0, 0, 2.0, 4.5, 3000) is False
    assert ev.net_reached(pts((30_000, 3.5)), 50.0, 0, 2.0, 4.5, 3000) is False  # never got there
    assert ev.net_reached([], None, 0, 2.0, 4.5, 3000) is None  # no launch


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
    kept = ev.fr.BallTuning(far_window_m=0.14, fast_ball_mps=20.0)
    assert ev.tuning_for(case, kept, fast_ball_from_club=True) == ev.fr.BallTuning(
        far_window_m=0.14, fast_ball_mps=50.0
    )


def test_band_bins_reaches_the_replay_config_only_when_given(ev, monkeypatch, tmp_path):
    configs = []

    def fake_replay(data, config, lib=None):
        configs.append(config)
        return SimpleNamespace(
            config=config,
            frames=[],
            points=[],
            fired_frame=None,
            launch=None,
            frozen_impact_timestamp_us=None,
        )

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
        config=ev.fr.ReplayConfig(tee_bin=29),
        frames=[],
        points=[],
        fired_frame=None,
        launch=None,
        frozen_impact_timestamp_us=None,
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
            "x", "club", "ok", True, True, 40.0, 40.0, impact=impact_outcome if impact else None
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


STEP = 40.0 * 0.003 / BIN_M  # bins per 3 ms frame at 40 m/s


def chain(frames_ms, start_bin=50.0, speed=40.0):
    """One target per listed frame (3 ms apart) on a 40 m/s line from start_bin at 0 ms."""
    return [frame_of(k, k * 3000, [start_bin + speed * (k * 0.003) / BIN_M]) for k in frames_ms]


def test_gap_tolerant_ball_present_bridges_an_impact_gap(ev):
    frames = chain([1, 5, 6])  # frames 2-4 missing: 9 ms gap
    assert ev.ball_present(frames, 40.0, BIN_M) is False  # strict: consecutive only
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is True


def test_gap_tolerant_ball_present_refuses_a_gap_longer_than_allowed(ev):
    frames = chain([1, 9, 10])  # 24 ms gap
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is False


def test_a_stationary_pair_is_not_a_ball_with_gaps_allowed(ev):
    frames = [frame_of(k, k * 3000, [50.0, 53.0]) for k in range(1, 8)]
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is False


def test_the_anchor_requires_the_chain_to_back_project_to_the_tee(ev):
    frames = chain([4, 5, 6], start_bin=50.0)  # passes bin 50 at t = 0
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000, anchor=(50.0, 0, 15_000))
    # The same chain judged against an impact 30 ms earlier does not back-project.
    assert not ev.ball_present(
        frames, 40.0, BIN_M, max_gap_us=18_000, anchor=(50.0, -30_000, 15_000)
    )


def test_ball_present_with_gaps_and_no_targets(ev):
    assert ev.ball_present([], 40.0, BIN_M, max_gap_us=18_000) is False
    assert ev.ball_present([frame_of(1, 3000, [])], 40.0, BIN_M, max_gap_us=18_000) is False


def outcome(ev, ball, present, strict=None):
    return ev.Outcome(
        name="x",
        club="club",
        ball=ball,
        ball_present=present,
        ball_present_strict=present if strict is None else strict,
        launch_mps=None,
        ops_mps=40.0,
    )


def test_summarize_splits_the_ball_verdicts_by_presence(ev):
    outs = [
        outcome(ev, "ok", True),
        outcome(ev, "none", True),
        outcome(ev, "wrong", False, strict=False),
        outcome(ev, "none", False),
    ]
    s = ev.summarize(outs)
    assert s["ball_by_presence"] == {
        "present": {"ok": 1, "wrong": 0, "none": 1},
        "absent": {"ok": 0, "wrong": 1, "none": 1},
    }
    assert s["ball_present"] == 2 and s["ball_present_strict"] == 2


def named(ev, name, ball, present, club="club"):
    return ev.Outcome(
        name=name,
        club=club,
        ball=ball,
        ball_present=present,
        ball_present_strict=present,
        launch_mps=None,
        ops_mps=40.0,
    )


def baseline_of(ev, outcomes) -> dict:
    """A baseline JSON as --json writes it: the summary and every outcome."""
    return {"summary": ev.summarize(outcomes), "outcomes": [asdict(o) for o in outcomes]}


# The legacy baseline: a, b present; c, d absent.
BASE_ROWS = [("a", "wrong", True), ("b", "none", True), ("c", "wrong", False), ("d", "ok", False)]


def base_json(ev, club="club"):
    return baseline_of(ev, [named(ev, n, b, p, club=club) for n, b, p in BASE_ROWS])


def failures(ev, lines):
    return [line for line in lines if not ev.is_informational(line)]


def test_accept_split_passes_a_strict_improvement_under_the_same_labels(ev):
    run = [
        named(ev, "a", "ok", True),
        named(ev, "b", "none", True),
        named(ev, "c", "none", False),
        named(ev, "d", "none", False),
    ]
    assert ev.accept_split(run, base_json(ev)) == []


def test_accept_split_names_each_failure_under_the_same_labels(ev):
    run = [
        named(ev, "a", "wrong", True, club="stuck"),
        named(ev, "b", "none", True),
        named(ev, "c", "wrong", False),
        named(ev, "d", "ok", False),
    ]
    problems = ev.accept_split(run, base_json(ev))
    assert problems == [
        "ball-present ok 0 not above 0",
        "ball-absent none 0 not above 0",
        "ball-absent wrong 1 not below 1",
        "club at impact 3 < baseline 4",
    ]
    assert failures(ev, problems) == problems


def test_accept_split_judges_the_run_by_the_baselines_labels(ev):
    """The run's own label differs on c (its replay saw a ball there): c is
    still judged as absent, so its 'none' counts; the difference is reported
    as information, not as a failure."""
    run = [
        named(ev, "a", "ok", True),
        named(ev, "b", "none", True),
        named(ev, "c", "none", True),  # run says present, baseline absent
        named(ev, "d", "none", False),
    ]
    lines = ev.accept_split(run, base_json(ev))
    assert lines == ["label differs: c baseline=absent run=present"]
    assert failures(ev, lines) == []


def test_a_label_flip_cannot_buy_acceptance(ev):
    """Under the run's own labels c ('wrong', now present) would leave the
    absent side with no wrong; under the baseline's it is still absent wrong."""
    run = [
        named(ev, "a", "ok", True),
        named(ev, "b", "none", True),
        named(ev, "c", "wrong", True),
        named(ev, "d", "none", False),
    ]
    lines = ev.accept_split(run, base_json(ev))
    assert failures(ev, lines) == ["ball-absent wrong 1 not below 1"]
    assert "label differs: c baseline=absent run=present" in lines


def test_accept_split_refuses_different_capture_sets(ev):
    run = [
        named(ev, "a", "ok", True),
        named(ev, "b", "none", True),
        named(ev, "c", "none", False),
        named(ev, "e", "none", False),
    ]
    problems = ev.accept_split(run, base_json(ev))
    assert len(problems) == 1 and not ev.is_informational(problems[0])
    assert "captures differ" in problems[0]
    assert "only in the run: e" in problems[0] and "only in the baseline: d" in problems[0]


def test_accept_split_refuses_duplicate_capture_names(ev):
    run = [named(ev, n, b, p) for n, b, p in BASE_ROWS] + [named(ev, "a", "ok", True)]
    problems = ev.accept_split(run, base_json(ev))
    assert problems == ["duplicate capture names in the run: a"]


@pytest.mark.parametrize(
    ("mutate", "needle"),
    [
        (lambda b: b.pop("outcomes"), "no per-capture outcomes"),
        (lambda b: b["summary"].pop("ball_by_presence"), "no ball_by_presence"),
        (lambda b: b.pop("summary"), "no summary"),
        (lambda b: b["outcomes"][0].pop("ball_present"), "no ball_present"),
    ],
)
def test_an_old_baseline_is_a_readable_problem_not_a_key_error(ev, mutate, needle):
    base = base_json(ev)
    mutate(base)
    run = [named(ev, n, b, p) for n, b, p in BASE_ROWS]
    problems = ev.accept_split(run, base)
    assert len(problems) == 1 and needle in problems[0]
    assert not ev.is_informational(problems[0])


def _run_main(ev, monkeypatch, tmp_path, run, *flags):
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter(run))
    monkeypatch.setattr(ev, "evaluate", lambda case, **_kw: case)
    return ev.main([str(tmp_path), *flags])


def test_the_cli_runs_accept_and_compare_both_and_fails_on_either(
    ev, monkeypatch, tmp_path, capsys
):
    base = tmp_path / "base.json"
    base.write_text(json.dumps(base_json(ev)), encoding="utf-8")
    same = [named(ev, n, b, p) for n, b, p in BASE_ROWS]
    code = _run_main(ev, monkeypatch, tmp_path, same, "--accept", str(base), "--compare", str(base))
    out = capsys.readouterr().out
    assert code == 1
    assert "NOT ACCEPTED: ball-present ok 0 not above 0" in out
    assert "REGRESSION" not in out  # the compare ran too and found nothing

    worse = [named(ev, n, "none", p, club="stuck") for n, _b, p in BASE_ROWS]
    code = _run_main(ev, monkeypatch, tmp_path, worse, "--accept", str(base), "--compare", str(base))
    out = capsys.readouterr().out
    assert code == 1
    assert "NOT ACCEPTED" in out and "REGRESSION club at impact" in out


def test_the_cli_prints_label_differences_without_failing(ev, monkeypatch, tmp_path, capsys):
    base = tmp_path / "base.json"
    base.write_text(json.dumps(base_json(ev)), encoding="utf-8")
    run = [
        named(ev, "a", "ok", True),
        named(ev, "b", "none", True),
        named(ev, "c", "none", True),
        named(ev, "d", "none", False),
    ]
    assert _run_main(ev, monkeypatch, tmp_path, run, "--accept", str(base)) == 0
    out = capsys.readouterr().out
    assert "NOTE: label differs: c baseline=absent run=present" in out
    assert "NOT ACCEPTED" not in out


def test_the_cli_reports_an_old_baseline_and_fails(ev, monkeypatch, tmp_path, capsys):
    base = tmp_path / "old.json"
    old = base_json(ev)
    del old["outcomes"]
    base.write_text(json.dumps(old), encoding="utf-8")
    run = [named(ev, n, b, p) for n, b, p in BASE_ROWS]
    assert _run_main(ev, monkeypatch, tmp_path, run, "--accept", str(base)) == 1
    assert "NOT ACCEPTED: " in capsys.readouterr().out


def test_a_reviewed_label_overrides_the_heuristic(ev, tmp_path, monkeypatch):
    labels = SimpleNamespace(reviewed=True, ball=(1, 2, 3))
    monkeypatch.setattr(ev, "load_labels", lambda _path: labels)
    assert ev.labelled_presence(tmp_path / "x.l3dump") is True
    labels.ball = ()
    assert ev.labelled_presence(tmp_path / "x.l3dump") is False


def test_an_unreviewed_or_broken_label_falls_back_to_the_heuristic(
    ev, tmp_path, monkeypatch, capsys
):
    monkeypatch.setattr(
        ev, "load_labels", lambda _path: SimpleNamespace(reviewed=False, ball=(1, 2, 3))
    )
    assert ev.labelled_presence(tmp_path / "x.l3dump") is None

    def broken(_path):
        raise ev.LabelError("bad file")

    monkeypatch.setattr(ev, "load_labels", broken)
    assert ev.labelled_presence(tmp_path / "x.l3dump") is None
    assert "bad file" in capsys.readouterr().err
