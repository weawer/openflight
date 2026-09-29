"""Tests for scripts/analysis/evaluate_iwr_profiles.py: measurement quality per loop count."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from iwr6843_synth import synth_shot_dump

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.firmware_replay import ReplayConfig

SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "evaluate_iwr_profiles.py"
TEE_BIN = 29


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("evaluate_iwr_profiles", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve annotations through sys.modules
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


@pytest.fixture(scope="module")
def shot() -> bytes:
    return synth_shot_dump(
        path_deg=2.0, hla_deg=1.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=1.372
    )


def test_fewer_loops_still_measure_the_synthetic_shot_and_differences_are_reported(
    script, lib, shot
):
    evaluations = script.evaluate_loops(shot, ReplayConfig(tee_bin=TEE_BIN), [12, 6], lib=lib)
    assert [e.loops for e in evaluations] == [12, 6]
    assert all(e.fired for e in evaluations)
    full, six = evaluations
    assert full.values["ball speed m/s"] == pytest.approx(60.0, abs=3.0)
    assert six.values["ball speed m/s"] == pytest.approx(60.0, abs=4.0)
    assert six.values["club points"] >= 4 and six.values["ball points"] >= 2  # post-commit only
    table = script.format_table("shot", evaluations)
    assert table.startswith("shot: 12 loops (fired)   6 loops (fired)")
    assert "ball speed m/s" in table and "(+" in table or "(-" in table
    with pytest.raises(ValueError, match="requested"):
        script.evaluate_loops(shot, ReplayConfig(tee_bin=TEE_BIN), [16], lib=lib)


def test_truth_replaces_the_full_loop_reference(script, lib, shot):
    evaluations = script.evaluate_loops(shot, ReplayConfig(tee_bin=TEE_BIN), [12], lib=lib)
    table = script.format_table("shot", evaluations, truth={"ball speed m/s": 60.0})
    row = next(line for line in table.splitlines() if "ball speed" in line)
    assert "(" in row, "difference from the stated truth"
    assert script.format_table("x", []) == "x: nothing evaluated"


def test_script_runs_over_a_file_and_a_directory(script, shot, tmp_path, capsys):
    (tmp_path / "a.l3dump").write_bytes(shot)
    (tmp_path / "manifest.json").write_text('{"default": {"tee_bin": 29}}')
    assert (
        script.main([str(tmp_path / "a.l3dump"), "--tee-range-m", "1.372", "--loops", "12", "6"])
        == 0
    )
    out = capsys.readouterr().out
    assert out.startswith("a.l3dump: 12 loops")
    assert script.main([str(tmp_path), "--loops", "12"]) == 0
    with pytest.raises(SystemExit):
        script.main([str(tmp_path / "a.l3dump")])
    (tmp_path / "empty").mkdir()
    assert script.main([str(tmp_path / "empty"), "--tee-bin", "29"]) == 1
