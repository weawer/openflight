"""Live club-track diagnostic capture, without the retired ball-leave replay."""

from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location("swing_trigger", "scripts/iwr6843/swing_trigger.py")
swing_trigger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(swing_trigger)


def test_windows_port_name_is_rejected_on_the_pi():
    assert "leave --port off" in swing_trigger.port_name_error("COM5", "linux")
    assert swing_trigger.port_name_error("COM5", "win32") is None
    assert swing_trigger.port_name_error("/dev/ttyUSB0", "linux") is None
    assert swing_trigger.port_name_error(None, "linux") is None


def test_diagnostics_are_saved_before_dump_and_watch_continues(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(True, b""), (True, b""), KeyboardInterrupt]
    radar.stats.return_value = "trig phase=fired tee=100 latched=1 enabled=1\nDone"
    radar.cmd.return_value = "frame=20 why=fired\nDone"
    radar.read_dump.return_value = b"raw capture"
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    events = [json.loads(line) for line in output.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["trigger_diagnostics", "capture"] * 2
    assert radar.read_dump.call_count == 2
    assert len(list(tmp_path.glob("*.l3dump"))) == 2
    assert Path(events[1]["path"]).read_bytes() == b"raw capture"
    assert "triggerLog trace" in events[0]["logs"]
    assert "triggerLog perf" in events[0]["logs"]


def test_poll_detects_latch_without_a_notice(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(False, b""), KeyboardInterrupt]
    radar.stats.return_value = "trig phase=fired tee=100 latched=1 enabled=1\nDone"
    radar.cmd.return_value = "Done"
    radar.read_dump.return_value = b"capture"
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)
    radar.read_dump.assert_called_once()


def test_untriggered_observation_saves_evidence_without_forcing_capture(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(False, b""), KeyboardInterrupt]
    radar.stats.return_value = "trig phase=watching tee=100 latched=0 enabled=1\nDone"
    radar.cmd.return_value = "why=slow\nDone"
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    radar.read_dump.assert_not_called()
    assert "why=slow" in output.getvalue()


def test_disabled_detector_is_an_error(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.return_value = (False, b"")
    radar.stats.return_value = "trig phase=off tee=0 latched=0 enabled=0\nDone"
    radar.cmd.return_value = "Done"
    with pytest.raises(RuntimeError, match="disabled"):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)
