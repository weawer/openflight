"""Tests for the standalone alert wiring/timing diagnostic."""

import importlib.util
import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/hardware-test/check_ops_alert.py"
spec = importlib.util.spec_from_file_location("check_ops_alert", SCRIPT)
check_ops_alert = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_ops_alert)


def test_records_edges_and_dump_without_sending_a_capture_trigger():
    radar = MagicMock()
    gpio = MagicMock(is_pressed=False)
    records = []
    radar.last_hardware_trigger_first_byte_timestamp = 100.0
    response = "\n".join(
        json.dumps({key: value})
        for key, value in (
            ("sample_time", 10.0),
            ("trigger_time", 10.026),
            ("I", [2048] * 4096),
            ("Q", [2048] * 4096),
        )
    )

    def capture(**kwargs):
        gpio.when_pressed()
        kwargs["on_first_byte"]()
        gpio.when_released()
        return response

    radar.configure_for_internal_speed_trigger.side_effect = lambda **_: gpio.when_pressed()
    radar.wait_for_hardware_trigger.side_effect = capture
    check_ops_alert.run_test(
        radar, gpio, "internal", 1, lambda event, **fields: records.append((event, fields))
    )

    edges = [(event, fields["phase"]) for event, fields in records if event.startswith("alert_f")]
    assert edges == [("alert_falling", "setup"), ("alert_falling", "observe")]
    assert [event for event, _ in records][-5:] == [
        "alert_falling",
        "dump_start",
        "alert_rising",
        "dump",
        "capture_timing",
    ]
    assert records[-1][1]["post_trigger_ms"] == pytest.approx(110.533333)
    assert records[-2][1]["response"] == response
    assert [call.args[0] for call in radar._send_command.call_args_list] == ["Y<40", "Y?"]
    radar.trigger_capture.assert_not_called()
    radar.rearm_internal_speed_trigger.assert_not_called()


@pytest.mark.parametrize("response,event", [("", "no_dump"), ("garbage", "parse_failed")])
def test_preserves_missing_or_malformed_dump_evidence(response, event):
    radar = MagicMock()
    radar.wait_for_hardware_trigger.return_value = response
    records = []
    check_ops_alert.run_test(
        radar,
        MagicMock(),
        "internal",
        1,
        lambda kind, **fields: records.append((kind, fields)),
    )
    assert records[-2] == ("dump", {"response": response})
    assert records[-1][0] == event


def test_speed_control_drains_serial_without_waiting_for_a_dump(monkeypatch):
    radar = MagicMock()
    radar.serial.in_waiting = 10
    radar.serial.read.return_value = b'{"ALERT": "High Speed"}'
    times = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(check_ops_alert.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(check_ops_alert.time, "sleep", lambda _: None)
    records = []
    check_ops_alert.run_test(
        radar,
        MagicMock(),
        "speed",
        1,
        lambda event, **fields: records.append((event, fields)),
    )
    radar.configure_for_speed_trigger.assert_called_once()
    radar.wait_for_hardware_trigger.assert_not_called()
    assert records[-2] == ("uart", {"text": '{"ALERT": "High Speed"}'})
    assert records[-1][0] == "finished"
