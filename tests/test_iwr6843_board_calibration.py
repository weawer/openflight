"""The one mapping from the loaded Calibration to what the board and the
replay are told (late-flight spec, 2026-09-29)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from openflight.iwr6843.board_calibration import BoardCalibration
from openflight.iwr6843.calibration import Calibration
from openflight.iwr6843.firmware_replay import ReplayConfig

REFERENCE = "config/iwr6843_calibration_reference.json"


def test_identity_is_zero_attitude_and_unit_elements():
    ident = BoardCalibration.identity()
    assert ident.cal_args == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert ident.elem_phase_rad == (0.0,) * 8 and ident.elem_gain == (1.0,) * 8


def test_from_calibration_maps_tilt_bias_and_elements():
    cal = Calibration.load(REFERENCE)
    board = BoardCalibration.from_calibration(cal)
    assert board.pitch_deg == pytest.approx(math.degrees(cal.tilt_rad))
    assert (board.yaw_deg, board.roll_deg, board.el_offset_deg) == (0.0, 0.0, 0.0)
    assert board.az_offset_rad == 0.0
    assert board.range_bias_m == pytest.approx(cal.range_bias_m)
    rebuilt = np.exp(-1j * np.array(board.elem_phase_rad)) / np.array(board.elem_gain)
    assert rebuilt == pytest.approx(cal.elem_correction, rel=1e-9)


def test_file_values_round_trip_to_the_json():
    raw = Calibration.load(REFERENCE).meta
    board = BoardCalibration.from_file(REFERENCE)
    assert board.elem_phase_rad == pytest.approx(tuple(raw["elem_phase_rad"]))
    assert board.elem_gain == pytest.approx(tuple(raw["elem_gain"]))


def test_replay_overrides_are_replay_config_fields():
    overrides = BoardCalibration.from_file(REFERENCE).replay_overrides()
    config = ReplayConfig(tee_bin=34, **overrides)
    assert config.elem_gain == BoardCalibration.from_file(REFERENCE).elem_gain


@pytest.mark.parametrize("gains", [(1.0,) * 7, (1.0,) * 7 + (0.0,), (1.0,) * 7 + (float("nan"),)])
def test_bad_elements_are_refused(gains):
    with pytest.raises(ValueError, match="element"):
        BoardCalibration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0,) * len(gains), gains)


def test_the_calibrated_radar_height_reaches_the_ball_fit():
    cal = Calibration.load(REFERENCE)
    cal.meta["radar_height_m"] = 0.21
    board = BoardCalibration.from_calibration(cal)
    assert board.tunable_overrides() == {"ball.fit.radarHeightM": pytest.approx(0.21)}
    config = ReplayConfig(tee_bin=34, overrides=board.tunable_overrides())
    assert config.overrides["ball.fit.radarHeightM"] == pytest.approx(0.21)


def test_a_radar_height_outside_the_tunable_bounds_is_skipped_with_a_warning(caplog):
    cal = Calibration.load(REFERENCE)
    cal.meta["radar_height_m"] = 1.5
    board = BoardCalibration.from_calibration(cal)
    with caplog.at_level("WARNING"):
        assert board.tunable_overrides() == {}
    assert "radar height" in caplog.text
