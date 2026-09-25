"""Tests for the OPS243 J3 pin 1 capture-synchronization diagnostic."""

import importlib.util
import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/hardware-test/check_ops_capture_sync.py"
spec = importlib.util.spec_from_file_location("check_ops_capture_sync", SCRIPT)
capture_sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture_sync)


def test_internal_mode_records_pin_edges_and_dump_timing():
    radar = MagicMock()
    gpio = MagicMock(is_pressed=False)
    records = []
    radar.last_hardware_trigger_first_byte_timestamp = 100.0
    response = "\n".join(
        json.dumps({key: value})
        for key, value in (
            ("sample_time", 10.0),
            ("trigger_time", 10.034133333),
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

    capture_sync.run_test(
        radar=radar,
        gpio=gpio,
        mode="internal",
        duration=1,
        trigger_threshold=40,
        trigger_magnitude=600,
        pre_trigger_segments=8,
        emit=lambda event, **fields: records.append((event, fields)),
    )

    assert [call.args[0] for call in radar._send_command.call_args_list] == ["Y>0", "Y?"]
    radar.configure_for_internal_speed_trigger.assert_called_once_with(
        trigger_threshold_mph=40,
        trigger_magnitude=600,
        pre_trigger_segments=8,
    )
    edges = [(event, fields["phase"]) for event, fields in records if event.startswith("sync_")]
    assert edges == [
        ("sync_falling", "setup"),
        ("sync_falling", "observe"),
        ("sync_rising", "observe"),
    ]
    assert [event for event, _ in records][-4:] == [
        "sync_falling",
        "dump_start",
        "sync_rising",
        "capture_timing",
    ]
    assert records[-1][1]["post_trigger_ms"] == pytest.approx(102.4)


def test_speed_mode_observes_pin_without_waiting_for_dump(monkeypatch):
    radar = MagicMock()
    radar.serial.in_waiting = 0
    times = iter([0.0, 0.0, 2.0])
    monkeypatch.setattr(capture_sync.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(capture_sync.time, "sleep", lambda _: None)
    records = []

    capture_sync.run_test(
        radar=radar,
        gpio=MagicMock(is_pressed=False),
        mode="speed",
        duration=1,
        trigger_threshold=40,
        trigger_magnitude=600,
        pre_trigger_segments=8,
        emit=lambda event, **fields: records.append((event, fields)),
    )

    radar.configure_for_speed_trigger.assert_called_once()
    radar.wait_for_hardware_trigger.assert_not_called()
    assert records[-1][0] == "finished"


@pytest.mark.parametrize("response,event", [("", "no_dump"), ("garbage", "parse_failed")])
def test_internal_mode_preserves_missing_or_malformed_dump(response, event):
    radar = MagicMock()
    radar.wait_for_hardware_trigger.return_value = response
    records = []

    capture_sync.run_test(
        radar=radar,
        gpio=MagicMock(is_pressed=False),
        mode="internal",
        duration=1,
        trigger_threshold=40,
        trigger_magnitude=600,
        pre_trigger_segments=8,
        emit=lambda kind, **fields: records.append((kind, fields)),
    )

    assert records[-1][0] == event
