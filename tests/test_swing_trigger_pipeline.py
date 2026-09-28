"""Live pipeline diagnostics must use the firmware detector, not ball-leave replay."""

import importlib.util
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location("pipeline_swing", "scripts/iwr6843/swing_trigger.py")
swing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(swing)


def test_arm_uses_snr_without_resampling_an_absolute_threshold():
    radar = Mock()
    radar.cmd.return_value = "Done"
    swing.arm(radar, "cfg", 41, 6.0, 2)
    radar.cmd.assert_called_once_with("triggerCfg 41 6.0 2")


def test_arm_rejects_error_even_with_done():
    radar = Mock()
    radar.cmd.return_value = "Error\nDone"
    with pytest.raises(RuntimeError, match="rejected"):
        swing.arm(radar, "cfg", 41, 6.0, 2)


def test_default_config_is_the_2ms_pipeline():
    assert swing.DEFAULT_CONFIG == "config/iwr6843_l3dump_adaptive_47f2ms_53bin_a16.cfg"
