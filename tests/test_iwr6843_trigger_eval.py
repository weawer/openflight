"""Tests for the self-trigger evaluation (openflight.iwr6843.trigger_eval) and
the bench captures it holds the firmware to.

The bench section replays the dumps the board froze on 2026-10-03 and
2026-10-04. Fires the club-in span gate (l3_impact_fit minSpanUs) stopped
must stay stopped; the ones nothing stops yet are strict xfails, so fixing
one fails loudly and moves it to the stopped list; and the on-time
captures must keep firing on the frame they do.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr, trigger_eval as te

needs_compiler = pytest.mark.skipif(
    fw.host_compiler() is None, reason="no C compiler for the firmware modules"
)

ROOT = Path(__file__).resolve().parents[1]
BENCH = {bench.name: bench for bench in te.BENCH_SETS}


def call(appended=True, gap=None, us=0, fired=False, cause=0, ok=True, points=4, speed=30.0):
    return te._ImpactCall(  # pylint: disable=protected-access
        time_us=us,
        fit_ok=ok,
        fit_points=points,
        fit_speed_mps=speed,
        appended=appended,
        gap_m=gap if appended else None,
        point_us=us if appended else None,
        fired=fired,
        cause=cause,
    )


# --- kiosk configuration -------------------------------------------------------


def test_the_kiosk_config_is_the_trigger_alone_at_the_default_settings():
    config = te.kiosk_config(36)
    assert config.tee_bin == 36 and config.dest_bin is None and config.post_from_frame is None
    assert config.snr == te.FIRMWARE_TRIGGER_DEFAULT_SNR
    assert config.band_bins == te.TEE_BAND_DEFAULT_BINS
    assert dict(config.overrides) == {}


def test_the_kiosk_config_keeps_a_recordings_capture_settings():
    base = fr.ReplayConfig(
        tee_bin=40, dest_bin=45, loop_period_s=1e-4, pitch_deg=7.0, overrides={"fitPoints": 5}
    )
    config = te.kiosk_config(33, base)
    assert (config.loop_period_s, config.pitch_deg) == (1e-4, 7.0)
    assert config.tee_bin == 33 and config.dest_bin is None and dict(config.overrides) == {}


# --- the approach behind a fire --------------------------------------------------


def test_an_approach_run_walks_back_while_the_club_is_farther_from_the_ball():
    calls = [call(gap=0.9, us=0), call(gap=0.6, us=2000), call(gap=0.4, us=4000)]
    run = te.approach_run(calls, 2)
    assert [c.gap_m for c in run] == [0.4, 0.6, 0.9]


def test_a_point_no_farther_ends_the_run():
    calls = [call(gap=0.3, us=0), call(gap=0.6, us=2000), call(gap=0.4, us=4000)]
    assert [c.gap_m for c in te.approach_run(calls, 2)] == [0.4, 0.6]


def test_equal_distance_ends_the_run():
    calls = [call(gap=0.6, us=0), call(gap=0.6, us=2000), call(gap=0.4, us=4000)]
    assert len(te.approach_run(calls, 2)) == 2


def test_coasts_up_to_the_limit_do_not_end_the_run():
    coasts = [call(appended=False)] * te.APPROACH_MAX_COAST_FRAMES
    calls = [call(gap=0.8, us=0), *coasts, call(gap=0.4, us=6000)]
    assert len(te.approach_run(calls, len(calls) - 1)) == 2


def test_a_longer_coast_ends_the_run():
    coasts = [call(appended=False)] * (te.APPROACH_MAX_COAST_FRAMES + 1)
    calls = [call(gap=0.8, us=0), *coasts, call(gap=0.4, us=8000)]
    assert len(te.approach_run(calls, len(calls) - 1)) == 1


def test_a_run_starts_with_the_arming_point_alone():
    assert len(te.approach_run([call(gap=0.2)], 0)) == 1


def test_a_crossing_is_armed_by_its_own_frames_point():
    calls = [call(gap=0.5), call(gap=0.2, fired=True)]
    assert te._arming_index(calls, 1, "crossing") == 1  # pylint: disable=protected-access


def test_an_end_is_armed_by_the_newest_point_before_it():
    calls = [call(gap=0.5), call(gap=0.2), call(appended=False, fired=True)]
    assert te._arming_index(calls, 2, "end") == 1  # pylint: disable=protected-access


def test_a_crossing_on_a_coasted_frame_falls_back_to_the_newest_point():
    calls = [call(gap=0.3), call(appended=False, fired=True)]
    assert te._arming_index(calls, 1, "crossing") == 0  # pylint: disable=protected-access


def test_no_point_before_the_fire_has_no_arming_point():
    calls = [call(appended=False), call(appended=False, fired=True)]
    assert te._arming_index(calls, 1, "end") is None  # pylint: disable=protected-access


# --- grouping and comparing ----------------------------------------------------


def test_fire_counts_name_no_fire_none():
    outcomes = [
        te.TriggerOutcome("a", 6, "end"),
        te.TriggerOutcome("b", None, None),
        te.TriggerOutcome("c", 5, "end"),
    ]
    assert te.fire_counts(outcomes) == {"end": 2, "none": 1}


def _json(*rows):
    return [{"name": name, "fired_frame": frame, "rule": rule} for name, frame, rule in rows]


def test_identical_refs_have_no_differences():
    same = {"g": _json(("a", 6, "end"), ("b", None, None))}
    assert te.differences({"old": same, "new": same}) == []


def test_a_capture_that_changes_is_listed_with_every_refs_outcome():
    results = {
        "old": {"g": _json(("a", 6, "end"), ("b", None, None))},
        "new": {"g": _json(("a", None, None), ("b", None, None))},
    }
    assert te.differences(results) == [("g", "a", {"old": (6, "end"), "new": (None, None)})]


def test_a_capture_missing_under_one_ref_is_a_difference():
    results = {"old": {"g": _json(("a", 6, "end"))}, "new": {"g": []}}
    assert te.differences(results) == [("g", "a", {"old": (6, "end"), "new": ("missing", None)})]


def test_a_group_only_one_ref_has_is_compared_too():
    results = {"old": {"g": _json(("a", 6, "end"))}, "new": {}}
    assert [row[:2] for row in te.differences(results)] == [("g", "a")]


def test_an_unknown_group_is_refused():
    with pytest.raises(ValueError, match="unknown capture group"):
        te.evaluate(ROOT, ["nope"])


def test_every_group_is_named(tmp_path):
    names = set(te.capture_groups(tmp_path))
    assert names == {"labelled", "golfer", *BENCH}


def test_bench_statuses_come_from_the_aligned_csv_beside_the_dumps(tmp_path):
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    (tmp_path / "trackman_openflight_aligned.csv").write_text(
        "tm_sequence,iwr_dump_path,match_status\n"
        "1,set/iwr6843/a.l3dump,early_trigger\n"
        "2,,missed_busy\n"
        "3,set/iwr6843/b.l3dump,matched\n",
        encoding="utf-8",
    )
    assert te.bench_statuses(dumps) == {"a.l3dump": "early_trigger", "b.l3dump": "matched"}


def test_a_bench_folder_without_a_csv_has_no_statuses(tmp_path):
    assert te.bench_statuses(tmp_path / "iwr6843") == {}


@pytest.mark.parametrize("bench", te.BENCH_SETS, ids=lambda b: b.name)
def test_every_bench_folder_holds_dumps(bench):
    assert sorted((ROOT / bench.directory).glob("*.l3dump")), bench.directory


def test_the_cli_refuses_an_unknown_group(capsys):
    with pytest.raises(SystemExit):
        te.main(["--group", "nope"])
    assert "unknown capture group" in capsys.readouterr().err


# --- the comparison script -----------------------------------------------------


def _compare_script():
    path = ROOT / "scripts" / "analysis" / "compare_trigger_builds.py"
    spec = importlib.util.spec_from_file_location("compare_trigger_builds", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_the_report_counts_fires_per_ref_and_lists_what_changed():
    results = {
        "base": {"bench": _json(("a", 6, "end"), ("b", 5, "crossing"), ("c", None, None))},
        "HEAD": {"bench": _json(("a", None, None), ("b", 5, "crossing"), ("c", None, None))},
    }
    text = _compare_script().report(results)
    assert "base: 3 captures, crossing 1, end 1, none 1" in text
    assert "HEAD: 3 captures, crossing 1, none 2" in text
    assert "1 capture(s) differ:" in text
    assert "base=6,end  HEAD=None,None" in text


def test_the_report_says_when_nothing_differs():
    same = {"bench": _json(("a", 6, "end"))}
    assert _compare_script().report({"a": same, "b": same}).endswith("0 capture(s) differ.")


# --- the bench captures ----------------------------------------------------------
#
# The 4 Oct TrackMan session's early triggers fired 0.5-0.8 s before impact
# (TrackMan's shot time); the 3 Oct captures listed here are triggers the OPS
# found no ball for, in a session of backswings. Each dump holds 9 frames
# (18 ms) before the freeze, so only the fires that need no longer history
# reproduce: 6 of the 15 early triggers and 6 of the 18 backswing dumps.

# Stopped by minSpanUs: a new club track three points old at 2 ms frames.
STOPPED_FALSE_FIRES = (
    ("bench_2026-10-04", "iwr6843_20261004_101032_133_070.l3dump"),  # end, 24.7 m/s, 4 ms
    ("bench_2026-10-04", "iwr6843_20261004_101642_167_092.l3dump"),  # crossing, 48.6 m/s
    ("bench_2026-10-03", "iwr6843_20261003_085608_586_003.l3dump"),  # end
    ("bench_2026-10-03", "iwr6843_20261003_085710_598_006.l3dump"),  # end
    ("bench_2026-10-03", "iwr6843_20261003_085813_252_009.l3dump"),  # end
    ("bench_2026-10-03", "iwr6843_20261003_085917_549_013.l3dump"),  # crossing
)

# Not stopped by anything yet.
REMAINING_FALSE_FIRES = (
    ("bench_2026-10-04", "iwr6843_20261004_095327_512_009.l3dump", "ball-leave fallback"),
    ("bench_2026-10-04", "iwr6843_20261004_095653_582_022.l3dump", "ball-leave fallback"),
    (
        "bench_2026-10-04",
        "iwr6843_20261004_102442_904_110.l3dump",
        "end: 4 points over 10 ms at 39 m/s, a track from the window's first frame",
    ),
    (
        "bench_2026-10-03",
        "iwr6843_20261003_085843_204_011.l3dump",
        "end: 4 points over 10 ms at 36.5 m/s",
    ),
    (
        "bench_2026-10-03",
        "iwr6843_20261003_090052_735_018.l3dump",
        "crossing: 3 points over 6 ms (one coast) at 57 m/s",
    ),
)

# On time (TrackMan within ~20 ms): (fired frame, rule) as replayed.
ON_TIME_FIRES = {
    "iwr6843_20261004_095438_256_015.l3dump": (6, "crossing"),
    "iwr6843_20261004_100812_912_062.l3dump": (6, "crossing"),
    "iwr6843_20261004_100856_810_064.l3dump": (6, "crossing"),
    "iwr6843_20261004_100940_484_066.l3dump": (6, "end"),
    "iwr6843_20261004_101328_479_081.l3dump": (5, "crossing"),
    "iwr6843_20261004_101428_591_086.l3dump": (5, "crossing"),
    "iwr6843_20261004_101601_723_090.l3dump": (6, "crossing"),
    "iwr6843_20261004_102156_258_102.l3dump": (5, "crossing"),
    "iwr6843_20261004_102259_301_105.l3dump": (6, "leave"),
}


def bench_outcome(group: str, name: str) -> te.TriggerOutcome:
    bench = BENCH[group]
    raw = (ROOT / bench.directory / name).read_bytes()
    return te.replay_trigger(raw, te.kiosk_config(bench.trigger_bin), name=name)


def test_the_bench_lists_do_not_overlap():
    stopped = {name for _group, name in STOPPED_FALSE_FIRES}
    remaining = {name for _group, name, _why in REMAINING_FALSE_FIRES}
    assert not stopped & remaining
    assert not (stopped | remaining) & set(ON_TIME_FIRES)


def test_the_on_time_list_is_what_trackman_says_was_on_time():
    statuses = te.bench_statuses(ROOT / BENCH["bench_2026-10-04"].directory)
    assert all(statuses[name].startswith("matched") for name in ON_TIME_FIRES)
    early = {name for name, status in statuses.items() if status == "early_trigger"}
    listed = {n for g, n in STOPPED_FALSE_FIRES if g == "bench_2026-10-04"}
    listed |= {n for g, n, _w in REMAINING_FALSE_FIRES if g == "bench_2026-10-04"}
    assert listed <= early


@needs_compiler
@pytest.mark.parametrize(
    ("group", "name"), [pytest.param(g, n, id=n[8:30]) for g, n in STOPPED_FALSE_FIRES]
)
def test_a_stopped_bench_false_fire_stays_stopped(group, name):
    outcome = bench_outcome(group, name)
    assert outcome.fired_frame is None, outcome


@needs_compiler
@pytest.mark.parametrize(
    ("group", "name"),
    [
        pytest.param(group, name, marks=pytest.mark.xfail(strict=True, reason=why), id=name[8:30])
        for group, name, why in REMAINING_FALSE_FIRES
    ],
)
def test_a_known_bench_false_fire_does_not_fire(group, name):
    outcome = bench_outcome(group, name)
    assert outcome.fired_frame is None, outcome


@needs_compiler
@pytest.mark.parametrize(
    ("name", "expected"), [pytest.param(n, e, id=n[8:30]) for n, e in ON_TIME_FIRES.items()]
)
def test_an_on_time_bench_capture_still_fires_on_its_frame(name, expected):
    outcome = bench_outcome("bench_2026-10-04", name)
    assert (outcome.fired_frame, outcome.rule) == expected, outcome


@needs_compiler
def test_a_range_fire_reports_the_approach_that_armed_it():
    """iwr6843_20261004_100940_484_066 (matched): the END rule on a 29.7 m/s
    approach whose last point was 0.40 m short of the ball."""
    outcome = bench_outcome("bench_2026-10-04", "iwr6843_20261004_100940_484_066.l3dump")
    assert outcome.rule == "end" and outcome.club_in_points == 5
    assert outcome.club_in_speed_mps == pytest.approx(29.7, abs=0.1)
    assert outcome.gap_m == pytest.approx(0.395, abs=0.005)
    assert outcome.approach_points == 5 and outcome.approach_span_ms == pytest.approx(8.0)
    assert outcome.approach_travel_m == pytest.approx(0.244, abs=0.005)


@needs_compiler
def test_a_leave_fire_carries_no_approach():
    outcome = bench_outcome("bench_2026-10-04", "iwr6843_20261004_102259_301_105.l3dump")
    assert outcome.rule == "leave" and outcome.club_in_points is None
