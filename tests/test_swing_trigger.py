"""Live club-track diagnostic capture, without the retired ball-leave replay."""

from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

from openflight.iwr6843.dump import pack_dump


def health(*, active=1, latched=0, stale=0):
    return (
        f"active={active} hwa_missed=0 iq8_overrun=0 iq8_edma_err=0 "
        f"scratch_stale={stale}\ndetect dropped=0 stale=0\n"
        f"trig phase=watching tee=100 latched={latched} enabled=1\nDone"
    )


def valid_dump():
    return pack_dump(np.zeros((2, 6, 4, 8), dtype=complex), n_tx=3)


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
    radar.stats.side_effect = [health(active=0, latched=1), health()] * 2
    radar.cmd.return_value = "frame=20 why=fired\nDone"
    radar.read_dump.return_value = valid_dump()
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    events = [json.loads(line) for line in output.getvalue().splitlines()]
    assert [e["event"] for e in events] == [
        "trigger_diagnostics",
        "capture",
        "rearm_check",
        "rearmed",
    ] * 2
    assert radar.read_dump.call_count == 2
    assert len(list(tmp_path.glob("*.l3dump"))) == 2
    assert Path(events[1]["path"]).read_bytes() == valid_dump()
    assert "triggerLog trace" in events[0]["logs"]
    assert "triggerLog perf" in events[0]["logs"]


def test_poll_detects_latch_without_a_notice(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(False, b""), KeyboardInterrupt]
    radar.stats.side_effect = [health(latched=1), health()]
    radar.cmd.return_value = "Done"
    radar.read_dump.return_value = valid_dump()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)
    radar.read_dump.assert_called_once()


def test_untriggered_observation_saves_evidence_without_forcing_capture(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(False, b""), KeyboardInterrupt]
    radar.stats.return_value = health()
    radar.cmd.return_value = "why=slow\nDone"
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    radar.read_dump.assert_not_called()
    radar.cmd.assert_not_called()
    assert health() in json.loads(output.getvalue())["stats"]


def test_post_trigger_recording_is_not_interrupted_by_verbose_logs(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(True, b""), KeyboardInterrupt]
    radar.stats.side_effect = [health(active=1, latched=1), health()]
    radar.read_dump.return_value = valid_dump()
    with pytest.raises(KeyboardInterrupt):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)
    radar.cmd.assert_not_called()
    radar.read_dump.assert_called_once()


def test_disabled_detector_is_an_error(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.return_value = (False, b"")
    radar.stats.return_value = "trig phase=off tee=0 latched=0 enabled=0\nDone"
    radar.cmd.return_value = "Done"
    with pytest.raises(RuntimeError, match="disabled"):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)


@pytest.mark.parametrize("raw", [b"Error -1\nl3dump:/>", valid_dump()[:-8]])
def test_invalid_dump_is_not_counted_as_a_capture(tmp_path, raw):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(True, b""), StopIteration]
    radar.stats.return_value = health(latched=1)
    radar.cmd.return_value = "Done"
    radar.read_dump.return_value = raw
    output = io.StringIO()
    with pytest.raises(RuntimeError, match="invalid dump"):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    assert not list(tmp_path.glob("*.l3dump"))
    assert '"event": "capture"' not in output.getvalue()
    assert list(tmp_path.glob("*.invalid.bin"))[0].read_bytes() == raw


@pytest.mark.parametrize("after", [health(active=0, latched=1), health(stale=1)])
def test_failed_rearm_stops_before_counting_another_trigger(tmp_path, after):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(True, b""), StopIteration]
    radar.stats.side_effect = [health(latched=1), after]
    radar.cmd.return_value = "Done"
    radar.read_dump.return_value = valid_dump()
    output = io.StringIO()
    with pytest.raises(RuntimeError):
        swing_trigger.observe(radar, output, tmp_path, 2.0)
    assert '"event": "rearmed"' not in output.getvalue()
    assert radar.wait_trigger_notice.call_count == 1


def test_stale_frames_stop_live_observation(tmp_path):
    radar = Mock()
    radar.wait_trigger_notice.side_effect = [(False, b""), StopIteration]
    radar.stats.return_value = health(stale=531)
    radar.cmd.return_value = "Done"
    with pytest.raises(RuntimeError, match="scratch_stale=531"):
        swing_trigger.observe(radar, io.StringIO(), tmp_path, 2.0)
    radar.read_dump.assert_not_called()
