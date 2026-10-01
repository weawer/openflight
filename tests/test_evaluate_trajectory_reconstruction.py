"""The reconstruction evaluation's geometry, and that it runs over the recordings."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from openflight.iwr6843 import firmware_host as fw

SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "analysis" / "evaluate_trajectory_reconstruction.py"
)


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
    # two points per x, +/-0.1 apart: the offsets are uncorrelated with x, so the best line is the x axis
    pts = [(float(k // 2), 0.1 if k % 2 else -0.1, 0.0) for k in range(8)]
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
