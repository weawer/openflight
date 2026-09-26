"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import numpy as np
import pytest

from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import TEMP_REPORT_KEYS, pack_dump


def test_send_config_rejects_missing_cli_acknowledgement(tmp_path, monkeypatch):
    """A wedged board must not be reported as configured and armed."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.send_config(str(config))


def test_stop_sensor_requires_acknowledgement_and_inactive_health(monkeypatch):
    """Shutdown must leave firmware idle rather than merely close the host UART."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    responses = iter(["sensorStop\nDone\nl3dump:/>", "stats\nactive=0\nDone\nl3dump:/>"])
    calls = []

    def fake_cmd(command, window):
        calls.append((command, window))
        return next(responses)

    monkeypatch.setattr(radar, "cmd", fake_cmd)

    radar.stop_sensor()

    assert calls == [("sensorStop", 3.0), ("stats", 2.0)]


def test_stop_sensor_rejects_firmware_that_remains_active(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    responses = iter(["sensorStop\nDone\n", "stats\nactive=1\nDone\n"])
    monkeypatch.setattr(radar, "cmd", lambda *_args: next(responses))

    with pytest.raises(RuntimeError, match="remained active"):
        radar.stop_sensor()


def test_send_config_flushes_previous_mmwave_profile_when_config_omits_flush(tmp_path, monkeypatch):
    """Repeated startup must not exhaust the firmware's mmWave profile slots."""
    config = tmp_path / "radar.cfg"
    config.write_text("dfeDataOutputMode 1\nsensorStart\n", encoding="utf-8")
    commands = []
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)

    def command(line, *_args, **_kwargs):
        commands.append(line)
        if line == "stats":
            return "active=1\nDone\n"
        return "Done\n"

    monkeypatch.setattr(radar, "cmd", command)

    radar.send_config(str(config))

    assert commands == [
        "sensorStop",
        "flushCfg",
        "dfeDataOutputMode 1",
        "sensorStart",
        "stats",
    ]


def test_send_config_waits_for_sensor_to_become_active(tmp_path, monkeypatch):
    """sensorStart may acknowledge before RF calibration and HWA startup finish."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    statuses = iter(
        (
            "active=0 calib=0x0 hwa_frames=0\nDone\n",
            "active=0 calib=0x1ffe hwa_frames=0\nDone\n",
            "active=1 calib=0x1ffe hwa_frames=2\nDone\n",
        )
    )
    commands = []
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)

    def command(line, *_args, **_kwargs):
        commands.append(line)
        return next(statuses) if line == "stats" else "Done\n"

    monkeypatch.setattr(radar, "cmd", command)

    radar.send_config(str(config))

    assert commands == ["sensorStop", "flushCfg", "sensorStart", "stats", "stats", "stats"]


class FakeSerial:
    """Serial double that exposes the in_waiting/read/write pieces read_dump uses."""

    def __init__(self, payload: bytes):
        self.payload = bytearray(payload)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.payload)

    def reset_input_buffer(self):
        pass

    def write(self, data: bytes):
        self.writes.append(data)

    def read(self, nbytes: int):
        nbytes = min(nbytes, len(self.payload))
        chunk = self.payload[:nbytes]
        del self.payload[:nbytes]
        return bytes(chunk)


def test_read_dump_waits_for_cli_ready_after_binary_payload():
    raw = pack_dump(np.ones((2, 6, 4, 7), dtype=complex), n_tx=3, version=3)

    class ChunkedSerial:
        def __init__(self):
            self.chunks = [bytearray(b"l3dump\r\n" + raw), bytearray(b"Done\r\nl3dump:/>")]
            self.writes = []
            self.delay_next_chunk = False

        @property
        def in_waiting(self):
            if self.delay_next_chunk:
                return 0
            return len(self.chunks[0]) if self.chunks else 0

        def reset_input_buffer(self):
            return None

        def write(self, value):
            self.writes.append(value)

        def read(self, count):
            if self.delay_next_chunk:
                self.delay_next_chunk = False
                return b""
            if not self.chunks:
                return b""
            chunk = self.chunks[0]
            data = bytes(chunk[:count])
            del chunk[:count]
            if not chunk:
                self.chunks.pop(0)
                if self.chunks:
                    self.delay_next_chunk = True
            return data

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = ChunkedSerial()

    assert radar.read_dump(timeout_s=0.1) == raw
    assert radar.ser.chunks == []
    assert radar.ser.writes == [b"l3dump\n"]


def test_read_shadow_dump_returns_decisions_tied_to_binary_frames():
    raw = pack_dump(np.ones((1, 6, 4, 7), dtype=complex), n_tx=3, version=3)

    class FakeSerial:
        def __init__(self):
            self.payload = bytearray(
                b"l3shadow\r\n"
                b"SHD f=0 c=31,42 s=31 w=25,12 q=900 n=120 ok=1 a=0\r\n"
                + raw
                + b"Done\r\nl3dump:/>"
            )
            self.writes = []

        def reset_input_buffer(self):
            pass

        def write(self, data):
            self.writes.append(data)

        @property
        def in_waiting(self):
            return len(self.payload)

        def read(self, count):
            data = bytes(self.payload[:count])
            del self.payload[:count]
            return data

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial()

    capture, decisions = radar.read_shadow_dump(timeout_s=0.1)

    assert capture == raw
    assert decisions == [
        {
            "frame": 0,
            "c0": 31,
            "c1": 42,
            "selected": 31,
            "proposed_start": 25,
            "proposed_bins": 12,
            "confidence_q8": 900,
            "noise": 120,
            "accepted": 1,
            "ambiguous": 0,
        }
    ]
    assert radar.ser.writes == [b"l3shadow\n"]


def test_read_dump_reports_firmware_restart_error_after_binary_payload():
    raw = pack_dump(np.ones((1, 3, 4, 4), dtype=complex), n_tx=3, version=3)
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"l3dump\r\n" + raw + b"Error: RF restart failed\r\n")

    with pytest.raises(RuntimeError, match="RF restart failed"):
        radar.read_dump(timeout_s=0.1)


def test_read_dump_sizes_v5_header_extension():
    report = {key: index + 40 for index, key in enumerate(TEMP_REPORT_KEYS)}
    raw = pack_dump(
        np.zeros((2, 4, 4, 8), dtype=complex),
        n_tx=2,
        version=5,
        temperature_report=report,
    )
    serial = FakeSerial(b"cli echo\r\n" + raw + b"trailing cli noise")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = serial

    dump = radar.read_dump(timeout_s=0.1)

    assert dump == raw
    assert serial.writes == [b"l3dump\n"]


class _NoticeSerial:
    """Serial double that hands out queued CLI chunks one read at a time."""

    def __init__(self, chunks: list[bytes]):
        self.chunks = list(chunks)
        self.read_sizes: list[int] = []

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, count: int) -> bytes:
        self.read_sizes.append(count)
        return self.chunks.pop(0) if self.chunks else b""


def _notice_radar(chunks: list[bytes]) -> IWR6843Radar:
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _NoticeSerial(chunks)
    return radar


def test_trigger_notice_in_one_read_is_reported():
    assert _notice_radar([b"Triggered\n"]).wait_trigger_notice() == (True, b"")


def test_trigger_notice_split_across_reads_is_reported_on_the_second():
    radar = _notice_radar([b"...Trig", b"gered\n"])

    found, pending = radar.wait_trigger_notice()
    assert found is False
    found, pending = radar.wait_trigger_notice(pending)
    assert found is True
    assert pending == b""


def test_idle_port_blocks_on_a_single_byte_read():
    """read(1) returns as soon as a byte arrives; no poll interval adds latency."""
    radar = _notice_radar([])

    assert radar.wait_trigger_notice() == (False, b"")
    assert radar.ser.read_sizes == [1]


def test_unrelated_cli_text_is_trimmed_to_a_split_word_tail():
    radar = _notice_radar([b"stats frames=12 wraps=3 active=1\n"])

    found, pending = radar.wait_trigger_notice()

    assert found is False
    assert len(pending) == len(b"Triggered") - 1


@pytest.mark.parametrize("prefix,suffix", [(b"Triggered\n", b""), (b"Trig", b"gered\n")])
def test_trigger_in_command_reply_survives_until_listener(prefix, suffix):
    class CommandSerial(FakeSerial):
        def write(self, data):
            super().write(data)
            self.payload.extend(b"Done\nl3dump:/>" + prefix)

    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = CommandSerial(b"")
    assert "Done" in radar.cmd("triggerCfg 14 1000 2")
    radar.ser.payload.extend(suffix)
    found, pending = radar.wait_trigger_notice()
    assert found
    assert radar.wait_trigger_notice(pending)[0] is False


def test_notice_received_with_dump_trailer_is_preserved():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"")
    radar._wait_for_dump_cli_ready(b"Done\nl3dump:/>Trig", timeout_s=0.1)
    radar.ser.payload.extend(b"gered\n")
    assert radar.wait_trigger_notice()[0]


def test_watch_script_releases_a_trigger_in_the_arming_reply(monkeypatch):
    import runpy
    import sys
    from unittest.mock import Mock, PropertyMock

    script = runpy.run_path("scripts/iwr6843/watch_trigger.py")
    main = script["main"]
    radar = Mock()
    radar.cmd.side_effect = ["Done\n", "Done\nTriggered\n", "Done\n"]
    type(radar.ser).in_waiting = PropertyMock(side_effect=KeyboardInterrupt)
    monkeypatch.setitem(main.__globals__, "IWR6843Radar", lambda **_kwargs: radar)
    monkeypatch.setitem(main.__globals__, "tee_local_bin", lambda *_args: 14)
    monkeypatch.setattr(sys, "argv", ["watch_trigger.py"])

    main()

    radar.release_sparse_freeze.assert_called_once()
    radar.close.assert_called_once()


def test_adaptive_header_split_keeps_shadow_decisions(monkeypatch):
    raw = pack_dump(
        np.ones((1, 36, 4, 12), dtype=complex), n_tx=3, version=9,
        sample_fmt=4, frame_period_us=2000, range_bin_starts=[30],
        range_bin_counts=[12], frame_time_offsets_us=[0],
        temperature_report={key: 40 for key in TEMP_REPORT_KEYS},
        retention=dict(reason="complete", pre_frames=1, planned_frames=1),
    )
    prefix = b"SHD f=0 c=31,42 s=31 w=25,12 q=900 n=120 ok=1 a=0\r\n"
    radar = _notice_radar([prefix + raw[:20], raw[20:44], raw[44:] + b"Done\r\nl3dump:/>"])
    monkeypatch.setattr(radar.ser, "reset_input_buffer", lambda: None, raising=False)
    monkeypatch.setattr(radar.ser, "write", lambda _: None, raising=False)
    received, decisions = radar.read_shadow_dump(timeout_s=0.1)
    assert received == raw
    assert len(decisions) == 1
    assert decisions[0]["selected"] == 31
