"""Tests for the IWR6843 CLI and dump serial contract."""

from __future__ import annotations

import time

import numpy as np
import pytest

from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import TEMP_REPORT_KEYS, pack_dump
from openflight.iwr6843.monitor import SELF_TRIGGER_DEFAULT_SNR


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


def test_trigger_log_reads_the_front_end_floor_and_configuration(monkeypatch):
    """Two lines since the range gate's 128-record log went with it."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    reply = (
        "triggerLog\ntrig frames=4000 floor=812.0 thr=812.0 traced=3\n"
        "trigcfg tee=38 snr=1.00 approach=12 past=3 stat=peak\n"
        "Done\nl3dump:/>"
    )

    def fake_cmd(command, window):
        calls.append((command, window))
        return reply

    monkeypatch.setattr(radar, "cmd", fake_cmd)

    assert radar.trigger_log() == reply
    assert calls == [("triggerLog", 2.0)]


def test_tee_scan_sends_the_bin_range(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    monkeypatch.setattr(
        radar, "cmd", lambda command, window: calls.append((command, window)) or "Done\n"
    )

    radar.tee_scan(8, 13)

    assert calls == [("ball scan 8 13", 4.0)]


def test_stop_sensor_rejects_firmware_that_remains_active(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    responses = iter(["sensorStop\nDone\n", "stats\nactive=1\nDone\n"])
    monkeypatch.setattr(radar, "cmd", lambda *_args: next(responses))

    with pytest.raises(RuntimeError, match="remained active"):
        radar.stop_sensor()


def test_send_config_streams_given_lines_instead_of_the_file(tmp_path, monkeypatch):
    """The monitor's tee-relative rewrite reaches the board, not the file's windows."""
    config = tmp_path / "radar.cfg"
    config.write_text("phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1\nsensorStart\n", "utf-8")
    commands = []
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)
    monkeypatch.setattr(
        radar,
        "cmd",
        lambda line, *_a, **_k: commands.append(line) or ("active=1\nDone\n"),
    )

    radar.send_config(
        str(config),
        lines=[
            "% comment",
            "",
            "captureCfg adaptive 1 9 4",
            "phaseCaptureCfg 20 53 9 25 53 7 25 53 51 8 1",
            "sensorStart",
        ],
    )

    assert commands == [
        "debugCfg 0",
        "sensorStop",
        "flushCfg",
        "captureCfg adaptive 1 9 4",
        "phaseCaptureCfg 20 53 9 25 53 7 25 53 51 8 1",
        "sensorStart",
        "stats",
    ]


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
        "debugCfg 0",
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

    assert commands == [
        "debugCfg 0",
        "sensorStop",
        "flushCfg",
        "sensorStart",
        "stats",
        "stats",
        "stats",
    ]


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


class _ResettingSerial(FakeSerial):
    """Like the real port: reset_input_buffer discards every unread byte."""

    def __init__(self, payload: bytes, reply: bytes = b"Done\nl3dump:/>"):
        super().__init__(payload)
        self.reply = reply

    def reset_input_buffer(self):
        self.payload.clear()

    def write(self, data: bytes):
        super().write(data)
        self.payload.extend(self.reply)


def _command_radar(payload: bytes, reply: bytes = b"Done\nl3dump:/>") -> IWR6843Radar:
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _ResettingSerial(payload, reply)
    return radar


def test_trigger_already_waiting_before_a_command_is_not_discarded():
    """A notice queued before cmd() must still reach the listener; else the ring stays frozen."""
    radar = _command_radar(b"Triggered\n")

    assert "Done" in radar.cmd("stats")
    assert radar.wait_trigger_notice()[0] is True


@pytest.mark.parametrize("prefix,suffix", [(b"Triggered\n", b""), (b"Trig", b"gered\n")])
def test_trigger_inside_a_command_reply_survives_until_the_listener(prefix, suffix):
    radar = _command_radar(b"", reply=b"Done\nl3dump:/>" + prefix)

    assert "Done" in radar.cmd("triggerCfg 14 1000 2")
    radar.ser.payload.extend(suffix)
    found, pending = radar.wait_trigger_notice()

    assert found is True
    assert radar.wait_trigger_notice(pending)[0] is False


def test_a_remembered_trigger_is_reported_only_once():
    radar = _command_radar(b"Triggered\n")
    radar.cmd("stats")

    assert radar.wait_trigger_notice()[0] is True
    assert radar.wait_trigger_notice()[0] is False


def test_stale_command_output_without_a_trigger_is_still_dropped():
    radar = _command_radar(b"frames=1 active=1\nDone\n")

    reply = radar.cmd("stats")

    assert "frames=1" not in reply
    assert radar.wait_trigger_notice()[0] is False


class _ChunkedReplySerial:
    """Delivers the reply one chunk per read, like a byte-by-byte firmware write."""

    def __init__(self, chunks: list[bytes]):
        self.pending = [bytearray(chunk) for chunk in chunks]
        self.chunks: list[bytearray] = []
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def reset_input_buffer(self):
        self.chunks.clear()

    def write(self, data: bytes):
        self.writes.append(data)
        # The reply exists only once the command has been sent.
        self.chunks.extend(self.pending)
        self.pending = []

    def read(self, nbytes: int):
        if not self.chunks:
            return b""
        chunk = self.chunks[0]
        out = bytes(chunk[:nbytes])
        del chunk[:nbytes]
        if not chunk:
            self.chunks.pop(0)
        return out


def _chunked_radar(chunks: list[bytes]) -> IWR6843Radar:
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _ChunkedReplySerial(chunks)
    return radar


def test_reply_split_inside_the_error_line_is_read_through_to_the_prompt():
    """'Error: stop the senso' must not end the reply: the rest is not the next command's."""
    radar = _chunked_radar(
        [
            b"captureFormat iq8\nError: stop the senso",
            b"r before captureFormat\nError -1\nl3dump:/>",
        ]
    )

    reply = radar.cmd("captureFormat iq8", 0.5)

    assert reply.endswith("Error -1\nl3dump:/>")
    assert "stop the sensor before captureFormat" in reply
    assert radar.ser.in_waiting == 0


def test_bytes_after_the_prompt_are_not_part_of_the_reply():
    """A debug line streaming right behind the prompt must not be returned half-arrived."""
    radar = _chunked_radar(
        [b"debugCfg 1\ntrig phase=watching tee=1 bin=14\nDone\nl3dump:/>trig phase=track"]
    )

    reply = radar.cmd("debugCfg 1", 0.5)

    assert reply.endswith("l3dump:/>")
    assert "phase=track" not in reply


def test_notice_behind_the_prompt_is_still_remembered_for_the_listener():
    radar = _chunked_radar([b"stats\nactive=1\nDone\nl3dump:/>Triggered\n"])

    reply = radar.cmd("stats", 0.5)

    assert "Triggered" not in reply
    assert radar.wait_trigger_notice()[0] is True


def test_reply_ends_at_the_prompt_after_done_not_at_a_prompt_before_it():
    radar = _chunked_radar([b"stats\nframes=1 active=1\nDone\n", b"l3dump:/>"])

    reply = radar.cmd("stats", 0.5)

    assert reply == "stats\nframes=1 active=1\nDone\nl3dump:/>"


def test_debug_line_between_done_and_the_prompt_stays_in_the_reply():
    radar = _chunked_radar(
        [b"debugCfg 1\ntrig phase=watching tee=1 bin=14 latched=0\nDone\n", b"l3dump:/>"]
    )

    reply = radar.cmd("debugCfg 1", 0.5)

    assert "trig phase=watching" in reply
    assert reply.endswith("l3dump:/>")


def test_reply_without_a_prompt_returns_after_a_quiet_period_not_the_window():
    radar = _chunked_radar([b"'19' is not recognized as a CLI command\n"])
    started = time.monotonic()

    reply = radar.cmd("stats", 2.0)

    assert "not recognized" in reply
    assert 0.08 < time.monotonic() - started < 0.6


def test_notice_arriving_with_the_dump_trailer_is_preserved():
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = FakeSerial(b"")

    radar._wait_for_dump_cli_ready(b"Done\nl3dump:/>Trig", timeout_s=0.1)
    radar.ser.payload.extend(b"gered\n")

    assert radar.wait_trigger_notice()[0] is True


def test_reconfiguring_forgets_a_trigger_from_the_previous_session(tmp_path, monkeypatch):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = _command_radar(b"Triggered\n", reply=b"active=1\nDone\nl3dump:/>")
    monkeypatch.setattr(radar, "drain_stale_output", lambda: 0)

    radar.send_config(str(config))

    assert radar.wait_trigger_notice()[0] is False


def test_watch_script_releases_a_trigger_in_the_arming_reply(monkeypatch):
    import runpy
    import sys
    from unittest.mock import Mock, PropertyMock

    main = runpy.run_path("scripts/iwr6843/watch_trigger.py")["main"]
    radar = Mock()
    calls: list[str] = []

    def _cmd(line, window=1.5):
        del window
        calls.append(line)
        if line.startswith("triggerCfg"):
            return "Done\nTriggered\n"
        return "Done\n"

    radar.cmd.side_effect = _cmd
    radar.release_sparse_freeze.side_effect = lambda: calls.append("release")
    radar.club_track.side_effect = lambda: calls.append("triggerLog track") or "clubtrack\nDone\n"
    type(radar.ser).in_waiting = PropertyMock(side_effect=KeyboardInterrupt)
    monkeypatch.setitem(main.__globals__, "IWR6843Radar", lambda **_kwargs: radar)
    monkeypatch.setitem(main.__globals__, "self_trigger_bin", lambda *_args: 14)
    monkeypatch.setitem(
        main.__globals__, "measure_trigger_level", lambda *_args, **_kwargs: (200000.0, 1200000.0)
    )
    monkeypatch.setattr(sys, "argv", ["watch_trigger.py", "--snr", "6"])

    main()

    # The club track that fired is printed before the ring is released.
    assert calls == [
        "debugCfg 1",
        "triggerCfg 14 6.0 1",
        "debugCfg 0",
        "triggerLog track",
        "release",
        "debugCfg 1",
        "debugCfg 0",
    ]
    radar.close.assert_called_once()


def test_watch_script_arms_above_the_measured_tee_floor(monkeypatch):
    """A person at the desk is ~2e5; level 1000 arms on them and fires."""
    import runpy
    import sys
    from unittest.mock import Mock, PropertyMock

    main = runpy.run_path("scripts/iwr6843/watch_trigger.py")["main"]
    radar = Mock()
    calls: list[str] = []

    def _cmd(line, window=1.5):
        del window
        calls.append(line)
        return "Done\n"

    radar.cmd.side_effect = _cmd
    type(radar.ser).in_waiting = PropertyMock(side_effect=KeyboardInterrupt)
    monkeypatch.setitem(main.__globals__, "IWR6843Radar", lambda **_kwargs: radar)
    monkeypatch.setitem(main.__globals__, "self_trigger_bin", lambda *_args: 14)
    monkeypatch.setitem(
        main.__globals__, "measure_trigger_level", lambda *_args, **_kwargs: (200000.0, 1200000.0)
    )
    monkeypatch.setattr(sys, "argv", ["watch_trigger.py"])

    main()

    assert calls[:2] == ["debugCfg 1", f"triggerCfg 14 {SELF_TRIGGER_DEFAULT_SNR} 1"]


def test_watch_script_has_no_range_gate_options():
    """--hits was the removed range gate's track-frame count."""
    import runpy
    import sys

    main = runpy.run_path("scripts/iwr6843/watch_trigger.py")["main"]
    sys_argv = sys.argv
    try:
        sys.argv = ["watch_trigger.py", "--hits", "2"]
        with pytest.raises(SystemExit):
            main()
    finally:
        sys.argv = sys_argv


class _PyserialShortRead:
    """``read(n)`` waits out the port timeout unless ``n`` bytes are already buffered.

    That is pyserial's contract. A stats reply is a few hundred bytes, so
    ``read(512)`` costs the whole 0.3s even after ``Done`` has arrived.
    """

    def __init__(self, reply: bytes, timeout: float = 0.3):
        self.reply = reply
        self.timeout = timeout
        self.pending = bytearray()
        self.elapsed = 0.0

    @property
    def in_waiting(self) -> int:
        return len(self.pending)

    def write(self, data: bytes) -> None:
        del data
        self.pending = bytearray(self.reply)

    def read(self, nbytes: int) -> bytes:
        if nbytes > len(self.pending):
            self.elapsed += self.timeout
        nbytes = min(nbytes, len(self.pending))
        chunk = bytes(self.pending[:nbytes])
        del self.pending[:nbytes]
        return chunk


def test_background_floor_collects_eight_samples_inside_two_seconds():
    """A 2s empty-lane sample must survive the 0.3s port timeout on each stats."""
    from openflight.iwr6843.monitor import measure_trigger_level

    port = _PyserialShortRead(b"trig phase=tee-low tee=180000 latched=0 enabled=1\nDone\nl3dump:/>")
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = port
    radar._trigger_pending = b""

    def pause(seconds: float) -> None:
        port.elapsed += seconds

    floor, level = measure_trigger_level(
        radar,
        14,
        clock=lambda: port.elapsed,
        pause=pause,
    )

    assert floor == pytest.approx(180000.0)
    assert level == pytest.approx(SELF_TRIGGER_DEFAULT_SNR * 180000.0), (
        "threshold is floor x the default snr"
    )
    # The 2s window is the pauses between readings. A timeout on each stats
    # pushes the sixth sample past that window.
    assert port.elapsed == pytest.approx(2.0)


def test_reading_the_frozen_capture_consumes_its_remembered_trigger():
    """The notice names the capture the readback takes; it must not fire again after rearm."""
    radar = _command_radar(b"Triggered\n")
    radar.cmd("stats")
    radar.ser.reply = b""

    try:
        radar.read_dump(timeout_s=0.05, stall_tolerance_s=0.01)
    except (RuntimeError, TimeoutError):
        pass

    assert radar.wait_trigger_notice()[0] is False


def test_shot_result_parses_the_packet_only_once_the_machine_reached_result(monkeypatch):
    """The firmware always prints a packet; ``ready=`` says whether it is this shot's."""
    from openflight.iwr6843 import firmware_host as fw

    radar = IWR6843Radar.__new__(IWR6843Radar)
    packet = bytearray(fw.RESULT_PACKET_BYTES)
    packet[0] = 2  # version
    packet[4] = 9  # shot id
    packet[92] = 2  # verdict valid (after 2 u32, 9 floats, 2 u32, 9 floats, 1 u32)
    hex_text = packet.hex()
    replies = {
        "triggerLog result": (
            "triggerLog result\nresult v2 shot=9 verdict=valid valid=0x0 quality=0x0 impact=0 "
            f"source=none club=0 ball=0 smash=0.00 ready=1\n  ball_speed=- conf=0.00 flags=none\n"
            f"packet {hex_text[:164]}\npacket+ {hex_text[164:]}\nDone\nl3dump:/>"
        ),
        "triggerLog shot": (
            "triggerLog shot\nshot state=ready since=1 impact=- source=none "
            "origin=0.00,0.00,0.00 club=0 post=0 transitions=1\nDone\nl3dump:/>"
        ),
        "triggerLog perf": "triggerLog perf\nperf frames=12 total=400us clock=200\nDone\nl3dump:/>",
        "triggerLog track": "triggerLog track\nclubtrack active=0 ...\nDone\nl3dump:/>",
    }
    calls = []

    def fake_cmd(command, window):
        calls.append((command, window))
        return replies[command]

    monkeypatch.setattr(radar, "cmd", fake_cmd)
    result = radar.shot_result()
    assert result is not None and result.shot_id == 9 and result.verdict == "valid"
    assert "shot state=ready" in radar.shot_status()
    assert "perf frames=12" in radar.perf()
    assert "clubtrack" in radar.club_track()
    assert [c[0] for c in calls] == [
        "triggerLog result",
        "triggerLog shot",
        "triggerLog perf",
        "triggerLog track",
    ]
    replies["triggerLog result"] = replies["triggerLog result"].replace("ready=1", "ready=0")
    assert radar.shot_result() is None


def test_set_tee_band_sends_the_track_cfg_sub_mode(monkeypatch):
    """The tee band rides trackCfg: the firmware CLI table is at CLI_MAX_CMD."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    monkeypatch.setattr(
        radar, "cmd", lambda command, window: calls.append((command, window)) or "Done\n"
    )

    radar.set_tee_band(6.0)
    radar.set_tee_band(2.5)
    radar.set_tee_band(0.0)

    assert [command for command, _ in calls] == [
        "trackCfg impactFit 6",
        "trackCfg impactFit 2.5",
        "trackCfg impactFit 0",
    ]


@pytest.mark.parametrize(
    ("reply", "match"),
    [
        ("Error: trackCfg impactFit <bandBins 0..64>\n", "config rejected"),
        ("", "did not acknowledge"),
    ],
)
def test_set_tee_band_requires_done(monkeypatch, reply, match):
    """Older firmware without the sub-mode, or a wedged board, must not pass silently."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: reply)

    with pytest.raises(RuntimeError, match=match):
        radar.set_tee_band(6.0)


def test_set_tee_band_reports_an_acknowledged_band(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "Done\n")

    assert radar.set_tee_band(6.0) is True
    assert radar.set_tee_band(0.0) is True


@pytest.mark.parametrize(
    "reply",
    [
        "Error: trackCfg <minBin> <maxBin> ...\n",
        "'trackCfg' is not recognized as a CLI command\n",
    ],
)
def test_clearing_the_band_on_firmware_without_it_is_not_an_error(monkeypatch, reply):
    """Older firmware has no band to clear: 0 there is a no-op, reported as False."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: reply)

    assert radar.set_tee_band(0.0) is False
    with pytest.raises(RuntimeError):
        radar.set_tee_band(6.0)


def test_clearing_the_band_on_a_silent_board_still_fails(monkeypatch):
    """No reply at all is a wedged board, not old firmware."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.set_tee_band(0.0)


def test_set_tee_band_can_carry_the_club_in_min_span(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    monkeypatch.setattr(
        radar, "cmd", lambda command, window: calls.append((command, window)) or "Done\n"
    )

    assert radar.set_tee_band(6.0, 5500) is True
    assert radar.set_tee_band(0.0, 0) is True

    assert [command for command, _ in calls] == [
        "trackCfg impactFit 6 5500",
        "trackCfg impactFit 0 0",
    ]


@pytest.mark.parametrize(
    "reply",
    [
        "Error: trackCfg impactFit <bandBins 0..64>\n",
        "'trackCfg' is not recognized as a CLI command\n",
    ],
)
def test_firmware_without_the_min_span_refuses_it_as_false(monkeypatch, reply):
    """1.0.2 and older take one value: the caller decides whether that matters."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: reply)

    assert radar.set_tee_band(6.0, 5500) is False


def test_a_silent_board_fails_with_the_min_span_too(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.set_tee_band(6.0, 5500)


def test_set_ball_snr_sends_the_track_cfg_sub_mode(monkeypatch):
    """The ball tracker's snr rides trackCfg beside the band; 0 is the firmware default."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    monkeypatch.setattr(
        radar, "cmd", lambda command, window: calls.append((command, window)) or "Done\n"
    )

    assert radar.set_ball_snr(4.5) is True
    assert radar.set_ball_snr(0.0) is True

    assert [command for command, _ in calls] == ["trackCfg ballSnr 4.5", "trackCfg ballSnr 0"]


@pytest.mark.parametrize(
    "reply",
    [
        "Error: trackCfg <loopPeriodS> <rangeResM> ...\n",
        "'trackCfg' is not recognized as a CLI command\n",
    ],
)
def test_restoring_the_ball_snr_default_on_firmware_without_it_is_not_an_error(monkeypatch, reply):
    """Older firmware always uses its default: asking for it is a no-op (False),
    asking for anything else is an error."""
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: reply)

    assert radar.set_ball_snr(0.0) is False
    with pytest.raises(RuntimeError):
        radar.set_ball_snr(4.0)


def test_restoring_the_ball_snr_on_a_silent_board_still_fails(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_args, **_kwargs: "")

    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.set_ball_snr(0.0)


def test_set_radar_cal_and_elements_send_the_track_cfg_sub_modes(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    calls = []
    monkeypatch.setattr(radar, "cmd", lambda command, window: calls.append(command) or "Done\n")

    assert radar.set_radar_cal((10.4, 0.0, 0.0, 0.12, 0.0, 0.066)) is True
    assert radar.set_elements((0.28, -0.43) + (0.0,) * 6, (0.95, 1.01) + (1.0,) * 6)

    assert calls[0] == "trackCfg cal 10.4 0 0 0.12 0 0.066"
    assert calls[1:] == [
        "trackCfg elem 0 0.28 0.95",
        "trackCfg elem 1 -0.43 1.01",
        *[f"trackCfg elem {i} 0 1" for i in range(2, 8)],
    ]


@pytest.mark.parametrize(
    "reply", ["Error: trackCfg <loopPeriodS> ...\n", "'trackCfg' is not recognized as a CLI command\n"]
)
def test_calibration_refused_by_old_firmware_returns_false_for_any_values(monkeypatch, reply):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_a, **_k: reply)
    assert radar.set_radar_cal((0.0,) * 6) is False
    assert radar.set_elements((0.0,) * 8, (1.0,) * 8) is False
    assert radar.set_radar_cal((10.4, 0, 0, 0, 0, 0.066)) is False
    assert radar.set_elements((0.28,) + (0.0,) * 7, (0.95,) + (1.0,) * 7) is False


def test_calibration_on_a_silent_board_fails(monkeypatch):
    radar = IWR6843Radar.__new__(IWR6843Radar)
    monkeypatch.setattr(radar, "cmd", lambda *_a, **_k: "")
    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.set_elements((0.28,) + (0.0,) * 7, (0.95,) + (1.0,) * 7)
    with pytest.raises(RuntimeError, match="did not acknowledge"):
        radar.set_radar_cal((10.4, 0, 0, 0, 0, 0.066))
