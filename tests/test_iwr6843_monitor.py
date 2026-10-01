"""Tests for GPIO-triggered TI capture and OPS shot correlation."""

from __future__ import annotations

import inspect
import logging
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import openflight.iwr6843.monitor as iwr_monitor
from openflight.iwr6843.board_calibration import BoardCalibration
from openflight.iwr6843.dsp_link import DETECT_CORES, DspLinkError
from openflight.iwr6843.dump import pack_dump
from openflight.iwr6843.firmware_version import FirmwareVersion
from openflight.iwr6843.monitor import (
    DEFAULT_IWR6843_CONFIG,
    DETECT_CORE,
    MSS_SCORED_CAPTURE_FORMATS,
    SELF_TRIGGER_DEFAULT_SNR,
    SELF_TRIGGER_OFF_COMMAND,
    TEE_BAND_DEFAULT_BINS,
    IWR6843CaptureMonitor,
    SelfTriggerConfig,
    measure_trigger_level,
    read_capture_config,
    tee_global_bin,
    tx_order_from_config,
)
from openflight.iwr6843.sparse import SparseCapture


def test_default_iwr6843_config_is_adaptive16():
    path = Path(DEFAULT_IWR6843_CONFIG)
    assert path.name == "iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg"
    text = path.read_text(encoding="utf-8")
    assert "captureFormat adaptive16" in text


# A capture running and the self-trigger not latched: nothing to rearm.
ARMED_STATS = "frames=10 wraps=0 active=1\ntrig phase=watching tee=1 latched=0 enabled=1\nDone\n"
# Frozen on a self-trigger: the readback that should have rearmed it failed.
FROZEN_STATS = "frames=10 wraps=0 active=0\ntrig phase=fired tee=1 latched=1 enabled=1\nDone\n"
# Stopped and not latched: l3release refuses (l3_awaitFrozenRing), only a restart runs it.
STOPPED_STATS = "frames=10 wraps=0 active=0\ntrig phase=fired tee=1 latched=0 enabled=1\nDone\n"


class FakeRadar:
    """Small transport double with a complete L3 dump."""

    port = "/dev/fake-iwr6843"

    def __init__(self, raw: bytes, error: Exception | None = None):
        self.raw = raw
        self.error = error
        self.configs = []
        self.closed = False
        self.read_started_at = None
        self.shutdown_events = []
        self.ball_snrs = []
        self.stats_replies: list[str | Exception] = []
        self.stats_reads = 0
        self.releases = 0
        self.release_error: Exception | None = None
        self.config_errors: list[Exception] = []
        self.version_reply: FirmwareVersion | None | Exception = FirmwareVersion(
            version="1.2.3", git="abc123", variant="hybrid-cadence", built="2026-09-30T00:00:00Z"
        )

    def stats(self) -> str:
        """Armed unless a test queued other replies (or errors) first."""
        self.stats_reads += 1
        reply = self.stats_replies.pop(0) if self.stats_replies else ARMED_STATS
        if isinstance(reply, Exception):
            raise reply
        return reply

    def release_sparse_freeze(self) -> None:
        if self.release_error is not None:
            raise self.release_error
        self.releases += 1

    def firmware_version(self):
        if isinstance(self.version_reply, Exception):
            raise self.version_reply
        return self.version_reply

    def send_config(self, path: str, lines=None):
        if self.config_errors:
            raise self.config_errors.pop(0)
        self.configs.append(path)

    def set_tee_band(self, bins: float) -> bool:
        """The monitor sends the band (0 included) at every start."""
        return True

    def set_ball_snr(self, snr: float) -> bool:
        """The monitor sends the ball snr (0: the firmware default) at every start."""
        self.ball_snrs.append(snr)
        return True

    def set_radar_cal(self, args):
        """The monitor sends the board calibration (identity included) at every start."""
        return True

    def set_elements(self, phases, gains):
        return True

    def read_dump(self):
        self.read_started_at = time.monotonic()
        if self.error is not None:
            raise self.error
        return self.raw

    def close(self):
        self.shutdown_events.append("close")
        self.closed = True

    def stop_sensor(self):
        self.shutdown_events.append("sensorStop")


class FakeButton:
    """gpiozero-compatible button double."""

    def __init__(self, pin, pull_up, bounce_time):
        self.pin = pin
        self.pull_up = pull_up
        self.bounce_time = bounce_time
        self.when_pressed = None
        self.closed = False

    def close(self):
        self.closed = True


def _temperature_report() -> dict[str, int]:
    return {
        "device_time_ms": 123456,
        "rx0_c": 42,
        "rx1_c": 43,
        "rx2_c": 44,
        "rx3_c": 45,
        "tx0_c": 46,
        "tx1_c": 47,
        "tx2_c": 48,
        "pm_c": 49,
        "dig0_c": 50,
        "dig1_c": 51,
    }


def _raw_dump(temperature_report: dict[str, int] | None = None) -> bytes:
    cube = np.zeros((2, 4, 4, 8), dtype=complex)
    return pack_dump(
        cube,
        n_tx=2,
        version=5 if temperature_report is not None else 3,
        frame_period_us=6000,
        temperature_report=temperature_report,
    )


def test_capture_config_reports_physical_timing_and_format(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text(
        "\n".join(
            (
                "profileCfg 0 60.0 7 3 38 0 0 100 1 128 4000 0 0 30",
                "chirpCfg 0 0 0 0 0 0 0 1",
                "chirpCfg 1 1 0 0 0 0 0 2",
                "chirpCfg 2 2 0 0 0 0 0 4",
                "frameCfg 0 2 12 0 2 1 0",
                "captureFormat iq16",
                "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1",
            )
        )
        + "\n",
        encoding="utf-8",
    )

    summary = read_capture_config(config)

    assert summary.n_tx == 3
    assert summary.loops == 12
    assert summary.frame_period_s == pytest.approx(0.002)
    assert summary.chirp_period_s == pytest.approx(45e-6)
    assert summary.loop_period_s == pytest.approx(135e-6)
    assert summary.capture_format == "iq16"


def test_capture_monitor_accepts_iq8_self_trigger_and_says_so(tmp_path, caplog):
    """IQ8 rings are read by the firmware's detect path (int8 times the frame
    scale), so the monitor no longer refuses them; it notes the firmware need."""
    config = tmp_path / "radar.cfg"
    config.write_text(
        "captureFormat iq8\nphaseCaptureCfg 20 53 14 32 53 10 47 53 64 12 1\n",
        encoding="utf-8",
    )
    radar = SelfTriggerRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        self_trigger=SelfTriggerConfig(tee_bin=1, snr=2.0),
    )

    with caplog.at_level(logging.INFO):
        monitor.start(armed=False)
    try:
        assert radar.configs == [str(config)]
        assert radar.commands[0][0] == "triggerCfg 1 2.0 1"
        assert any("IQ8 detect" in record.getMessage() for record in caplog.records)
    finally:
        monitor.stop()


def test_capture_monitor_matches_gpio_edge_to_ops_impact(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = FakeRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        save_dumps=True,
    )
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge + 0.012, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.trigger_timestamp == edge
    assert capture.path is not None and capture.path.read_bytes() == _raw_dump()
    assert radar.configs == [str(config)]
    assert monitor._button.bounce_time is None  # pylint: disable=protected-access

    monitor.stop()
    assert radar.closed


def test_capture_monitor_keeps_valid_raw_in_memory_without_writing_dump(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    raw = _raw_dump()
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(raw),
        button_factory=FakeButton,
    )
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.raw == raw
    assert capture.path is None
    assert not (tmp_path / "dumps").exists()
    monitor.stop()


def test_capture_monitor_notifies_trigger_observers(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    raw = _raw_dump()
    observed = []
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(raw),
        button_factory=FakeButton,
        trigger_observers=[observed.append],
    )
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert observed == [edge]
    monitor.stop()


def test_capture_monitor_records_temperature_report_from_dump_header(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    report = _temperature_report()
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(_raw_dump(temperature_report=report)),
        button_factory=FakeButton,
    )
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.temperature_report == report
    monitor.stop()


def test_capture_monitor_can_configure_before_arming_gpio(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = FakeRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
    )
    monitor.start(armed=False)

    edge = time.time()
    assert not monitor.notify_trigger(edge)
    assert monitor._button.when_pressed is None  # pylint: disable=protected-access

    monitor.arm()
    assert monitor._button.when_pressed == monitor.notify_trigger  # pylint: disable=protected-access
    assert monitor.notify_trigger(edge)
    assert monitor.capture_for_shot(edge, timeout_s=1.0).valid
    monitor.stop()


def test_capture_monitor_finishes_active_dump_before_closing_serial(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")

    class BlockingRadar(FakeRadar):
        def __init__(self, raw):
            super().__init__(raw)
            self.read_started = threading.Event()
            self.release_read = threading.Event()
            self.closed_before_read_finished = False

        def read_dump(self):
            self.read_started.set()
            self.release_read.wait(timeout=1.0)
            return self.raw

        def close(self):
            self.closed_before_read_finished = not self.release_read.is_set()
            # Unblock the old close-before-join implementation so this test fails fast.
            self.release_read.set()
            super().close()

    radar = BlockingRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
    )
    monitor.start()
    assert monitor.notify_trigger(time.time())
    assert radar.read_started.wait(timeout=0.5)

    stopper = threading.Thread(target=monitor.stop)
    stopper.start()
    time.sleep(0.05)
    radar.release_read.set()
    stopper.join(timeout=1.0)

    assert not stopper.is_alive()
    assert not radar.closed_before_read_finished
    assert radar.closed
    assert radar.shutdown_events == ["sensorStop", "close"]


def test_capture_monitor_closes_serial_when_sensor_stop_fails(tmp_path):
    """A failed firmware stop must not leak the host serial descriptor."""
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")

    class StopFailingRadar(FakeRadar):
        def stop_sensor(self):
            self.shutdown_events.append("sensorStop")
            raise RuntimeError("firmware remained active")

    radar = StopFailingRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
    )
    monitor.start()

    monitor.stop()

    assert radar.shutdown_events == ["sensorStop", "close"]
    assert radar.closed


def test_capture_monitor_discards_stale_false_trigger(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(_raw_dump()),
        button_factory=FakeButton,
        match_tolerance_s=0.1,
    )
    monitor.start()
    assert monitor.notify_trigger(100.0)

    assert monitor.capture_for_shot(101.0, timeout_s=0.1) is None
    monitor.stop()


def test_capture_monitor_returns_quickly_when_matching_trigger_is_absent(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(_raw_dump()),
        button_factory=FakeButton,
        match_tolerance_s=0.1,
    )
    monitor.start()

    start = time.monotonic()
    capture = monitor.capture_for_shot(time.time() - 1.0, timeout_s=1.0)

    assert capture is None
    assert time.monotonic() - start < 0.2
    monitor.stop()


def test_capture_monitor_surfaces_dump_failure_without_hanging(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(b"", error=OSError("serial disconnected")),
        button_factory=FakeButton,
    )
    monitor.start()
    edge = time.time()
    assert monitor.notify_trigger(edge)

    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None
    assert not capture.valid
    assert capture.error == "serial disconnected"
    monitor.stop()


def test_capture_monitor_closes_serial_when_gpio_setup_fails(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = FakeRadar(_raw_dump())

    def failing_button(*_args, **_kwargs):
        raise RuntimeError("GPIO unavailable")

    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=failing_button,
    )

    try:
        monitor.start()
    except RuntimeError as error:
        assert str(error) == "GPIO unavailable"
    else:
        raise AssertionError("expected GPIO setup to fail")
    assert radar.shutdown_events == ["sensorStop", "close"]
    assert radar.closed


class SelfTriggerRadar(FakeRadar):
    """FakeRadar that also speaks the CLI and reports firmware notices."""

    def __init__(self, raw: bytes, *, cmd_reply: str = "Done\n"):
        super().__init__(raw)
        self.cmd_reply = cmd_reply
        self.commands: list[tuple[str, str]] = []
        self.notices: list[bytes] = []
        self.sparse = None
        self.sparse_error: Exception | None = None
        self.result = None
        self.result_error: Exception | None = None
        self.result_reads = 0
        self.core_requests: list[str | None] = []

    def cmd(self, line: str, window: float = 1.5) -> str:
        del window
        self.commands.append((line, threading.current_thread().name))
        return self.cmd_reply

    def detect_core(self, core=None):
        """Refuses what the driver refuses; kept out of ``commands`` so the
        triggerCfg assertions read as before (DetectCoreRadar logs it)."""
        if core is not None and core not in DETECT_CORES:
            raise ValueError(f"core must be one of {DETECT_CORES}, got {core!r}")
        self.core_requests.append(core)
        return SimpleNamespace(requested=core or "dss", active="dss")

    def shot_result(self):
        self.result_reads += 1
        if self.result_error is not None:
            raise self.result_error
        return self.result

    def wait_trigger_notice(self, pending: bytes = b"") -> tuple[bool, bytes]:
        if not self.notices:
            time.sleep(0.002)
            return False, pending
        pending += self.notices.pop(0)
        if b"Triggered" in pending:
            return True, b""
        return False, pending

    def read_sparse(self, planner):
        del planner
        if self.sparse_error is not None:
            raise self.sparse_error
        return self.sparse


def _self_trigger_monitor(tmp_path, radar, **kwargs) -> IWR6843CaptureMonitor:
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    return IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        self_trigger=SelfTriggerConfig(tee_bin=12, snr=6.0),
        **kwargs,
    )


def _wait_until(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()


def test_self_trigger_notice_starts_the_shot_listeners(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    heard = []
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.add_trigger_observer(heard.append)
    monitor.start(armed=False)
    monitor.arm()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert monitor._button is None  # pylint: disable=protected-access
    assert capture is not None and capture.valid
    assert len(heard) == 1
    assert heard[0] == capture.trigger_timestamp
    monitor.stop()


def test_self_trigger_capture_carries_the_firmware_result(tmp_path):
    """The result packet is read before the readback (which rearms the ring
    and resets it) and rides on the capture for the shot pipeline."""
    radar = SelfTriggerRadar(_raw_dump())
    radar.result = SimpleNamespace(shot_id=3, verdict="valid", club_points=4, ball_points=9)
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False)
    monitor.arm()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.onboard_result is radar.result
    assert radar.result_reads == 1
    assert radar.read_started_at is not None
    monitor.stop()


def test_unreadable_firmware_result_never_costs_the_capture(tmp_path, caplog):
    radar = SelfTriggerRadar(_raw_dump())
    radar.result_error = RuntimeError("serial timeout")
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False)
    monitor.arm()
    radar.notices.append(b"Triggered\n")

    with caplog.at_level(logging.WARNING):
        capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.onboard_result is None
    assert any("Onboard result unreadable" in r.getMessage() for r in caplog.records)
    monitor.stop()


def test_gpio_captures_do_not_ask_for_a_firmware_result(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    radar = FakeRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
    )
    monitor.start()
    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)
    monitor.stop()

    assert capture is not None and capture.valid
    assert capture.onboard_result is None


def test_self_trigger_config_is_sent_before_the_worker_owns_the_port(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False)
    monitor.stop()

    assert radar.commands[0] == ("triggerCfg 12 6.0 1", threading.current_thread().name)


def test_measure_trigger_level_reads_p95_and_stops_if_the_probe_is_rejected():
    class _Radar:
        def __init__(self, reply: str):
            self.reply = reply
            self.commands: list[str] = []

        def cmd(self, line: str, window: float = 1.5) -> str:
            del window
            self.commands.append(line)
            if line == "stats":
                return "trig phase=tee-low tee=200000 latched=0 enabled=1\nDone\n"
            return self.reply

        def stats(self) -> str:
            return self.cmd("stats")

    clock = {"t": 0.0}
    radar = _Radar("Done\n")
    floor, level = measure_trigger_level(
        radar,
        14,
        clock=lambda: clock["t"],
        pause=lambda seconds: clock.__setitem__("t", clock["t"] + seconds),
    )

    assert radar.commands[0] == f"triggerCfg 14 {SELF_TRIGGER_DEFAULT_SNR} 1", (
        "the real arm, at the default snr"
    )
    assert floor == pytest.approx(200000.0)
    assert level == pytest.approx(SELF_TRIGGER_DEFAULT_SNR * 200000.0), "threshold = floor x snr"

    rejected = _Radar("Error: trigger power\n")
    with pytest.raises(RuntimeError, match="background probe rejected"):
        measure_trigger_level(rejected, 14, clock=lambda: 0.0, pause=lambda _seconds: None)


def test_rejected_self_trigger_config_fails_start_and_releases_the_radar(tmp_path):
    radar = SelfTriggerRadar(_raw_dump(), cmd_reply="Error: trigger bin\n")
    monitor = _self_trigger_monitor(tmp_path, radar)

    with pytest.raises(RuntimeError, match="self-trigger rejected"):
        monitor.start(armed=False)
    assert radar.closed
    assert "sensorStop" in radar.shutdown_events


def test_notice_while_disarmed_releases_the_ring_and_never_captures(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    heard = []
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.add_trigger_observer(heard.append)
    monitor.start(armed=False)
    radar.notices.append(b"Triggered\n")
    assert _wait_until(lambda: radar.releases == 1)

    monitor.arm()
    capture = monitor.capture_for_shot(None, timeout_s=0.2)

    assert capture is None
    assert heard == []
    assert radar.read_started_at is None
    monitor.stop()


class _FlakyReleaseRadar(SelfTriggerRadar):
    """Release fails ``failures`` times before succeeding."""

    def __init__(self, raw: bytes, failures: int):
        super().__init__(raw)
        self.failures = failures
        self.attempts = 0

    def release_sparse_freeze(self) -> None:
        self.attempts += 1
        if self.attempts <= self.failures:
            raise RuntimeError("l3sparse release timed out")
        self.releases += 1


def test_a_failed_release_is_retried_until_the_ring_rearms(tmp_path, monkeypatch):
    """One failed release must not leave the firmware frozen for the rest of the session."""
    monkeypatch.setattr(iwr_monitor, "_LISTENER_ERROR_BACKOFF_S", 0.0)
    radar = _FlakyReleaseRadar(_raw_dump(), failures=2)
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False)
    radar.notices.append(b"Triggered\n")

    assert _wait_until(lambda: radar.releases == 1)
    assert radar.attempts == 3
    monitor.stop()


def test_a_rejected_notice_while_armed_releases_the_ring(tmp_path, monkeypatch):
    """A duplicate/busy rejection still froze the firmware; somebody must release it."""
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    monkeypatch.setattr(monitor, "notify_trigger", lambda *_args, **_kwargs: False)
    radar.notices.append(b"Triggered\n")

    assert _wait_until(lambda: radar.releases == 1)
    assert radar.read_started_at is None
    monitor.stop()


def test_an_accepted_notice_is_not_released(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert radar.releases == 0
    monitor.stop()


def test_notice_split_across_reads_still_triggers(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    radar.notices.extend([b"Trig", b"gered\n"])

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    monitor.stop()


def test_submitted_job_runs_on_the_capture_worker(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    ran = []
    done = threading.Event()

    def job(job_radar):
        ran.append((job_radar, threading.current_thread().name))
        done.set()

    assert monitor.submit("probe", job)
    assert done.wait(1.0)
    assert ran == [(radar, "iwr6843-capture")]
    monitor.stop()


def test_submit_is_refused_when_the_monitor_is_not_running(tmp_path):
    monitor = _self_trigger_monitor(tmp_path, SelfTriggerRadar(_raw_dump()))

    assert monitor.submit("probe", lambda _radar: None) is False


def test_other_profile_turns_the_trigger_off_and_back_on_even_on_failure(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    done = threading.Event()

    def job(_radar):
        try:
            monitor.run_on_other_profile(lambda _r: (_ for _ in ()).throw(OSError("retune")))
        finally:
            done.set()

    monitor.submit("late-window", job)
    assert done.wait(1.0)
    monitor.stop()

    lines = [line for line, _thread in radar.commands]
    assert lines == ["triggerCfg 12 6.0 1", SELF_TRIGGER_OFF_COMMAND, "triggerCfg 12 6.0 1"]


def test_trigger_during_a_serial_job_is_rejected(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=FakeRadar(_raw_dump()),
        button_factory=FakeButton,
    )
    monitor.start()
    started = threading.Event()
    release = threading.Event()

    def job(_radar):
        started.set()
        release.wait(1.0)

    monitor.submit("late-window", job)
    assert started.wait(1.0)
    try:
        assert monitor.notify_trigger(time.time()) is False
    finally:
        release.set()
    monitor.stop()


def test_job_waits_behind_an_in_flight_capture(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    order = []

    class SlowRadar(FakeRadar):
        def read_dump(self):
            time.sleep(0.05)
            order.append("capture")
            return super().read_dump()

    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=SlowRadar(_raw_dump()),
        button_factory=FakeButton,
    )
    monitor.start()
    done = threading.Event()
    assert monitor.notify_trigger(time.time())
    monitor.submit("late-window", lambda _radar: (order.append("job"), done.set()))

    assert done.wait(1.0)
    assert order == ["capture", "job"]
    monitor.stop()


def test_sparse_capture_carries_its_noise_floor(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    radar.sparse = SparseCapture(
        raw=_raw_dump(), noise_power=4.5, requested_cells=10, sent_cells=10
    )
    monitor = _self_trigger_monitor(tmp_path, radar, slice_planner=lambda _summary: None)
    monitor.start()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.noise_power == 4.5
    assert radar.read_started_at is None
    monitor.stop()


def test_sparse_rejected_before_freeze_falls_back_to_full_dump(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    radar.sparse = None
    monitor = _self_trigger_monitor(tmp_path, radar, slice_planner=lambda _summary: None)
    monitor.start()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.noise_power is None
    assert radar.read_started_at is not None
    monitor.stop()


def test_sparse_failure_after_freeze_is_a_capture_error_not_a_fallback(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    radar.sparse_error = TimeoutError("ILS1 packet incomplete")
    monitor = _self_trigger_monitor(tmp_path, radar, slice_planner=lambda _summary: None)
    monitor.start()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=1.0)

    assert capture is not None
    assert not capture.valid
    assert "ILS1" in capture.error
    assert radar.read_started_at is None
    monitor.stop()


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"tee_bin": -1, "snr": 6.0}, "bin"),
        ({"tee_bin": 3, "snr": 0.5}, "snr"),
        ({"tee_bin": 3, "snr": float("nan")}, "snr"),
    ],
)
def test_self_trigger_config_rejects_values_the_firmware_would_misread(kwargs, message):
    with pytest.raises(ValueError, match=message):
        SelfTriggerConfig(**kwargs)


def test_self_trigger_command_is_the_firmware_triggercfg_line():
    assert SelfTriggerConfig(tee_bin=14, snr=6.0).command == "triggerCfg 14 6.0 1"
    # Zero is the firmware's "off"; the on-line never sends it.
    assert SELF_TRIGGER_OFF_COMMAND == "triggerCfg 0 0 0"


def _cfg(tmp_path, *lines: str):
    path = tmp_path / "capture.cfg"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_capture_config_summary_reads_masks_and_first_window(tmp_path):
    path = _cfg(
        tmp_path,
        "chirpCfg 0 0 0 0 0 0 0 1",
        "chirpCfg 1 1 0 0 0 0 0 4",
        "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1",
    )

    summary = read_capture_config(path)

    assert summary.chirp_tx_masks == ("1", "4")
    assert summary.first_window_start == 20
    assert summary.first_window_bins == 53
    assert tx_order_from_config(path) == "normal"


def test_tee_global_bin_is_the_absolute_fft_bin_inside_the_first_window(tmp_path):
    path = _cfg(tmp_path, "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1")

    # 1.575 m / (6 m / 128) = bin 33.6 -> 34: global, not 34 - 20 = 14 as the
    # window offset. The firmware speaks global bins everywhere now.
    assert tee_global_bin(1.575, path) == 34


def test_self_trigger_bin_is_two_short_of_the_ball_for_a_face_tape(tmp_path):
    """Stock tee 1.575 m from the front → array 1.875 m → ball bin 40 → watch 38."""
    from openflight.iwr6843.monitor import self_trigger_bin

    path = _cfg(tmp_path, "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1")
    assert self_trigger_bin(1.575, path) == 38


@pytest.mark.parametrize("tee_m", [0.5, 4.0])
def test_tee_outside_the_first_window_is_an_error(tmp_path, tee_m):
    path = _cfg(tmp_path, "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1")

    with pytest.raises(ValueError, match="outside the first capture window"):
        tee_global_bin(tee_m, path)


def test_tee_bin_needs_a_capture_window(tmp_path):
    with pytest.raises(ValueError, match="no phaseCaptureCfg"):
        tee_global_bin(1.5, _cfg(tmp_path, "sensorStart"))


def test_listener_serial_error_does_not_kill_the_worker(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    failures = {"left": 1}
    original = radar.wait_trigger_notice

    def flaky(pending=b""):
        if failures["left"]:
            failures["left"] -= 1
            raise OSError("device reports readiness to read but returned no data")
        return original(pending)

    radar.wait_trigger_notice = flaky
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start()
    radar.notices.append(b"Triggered\n")

    capture = monitor.capture_for_shot(None, timeout_s=2.0)

    assert capture is not None and capture.valid
    monitor.stop()


# --- read order: l3track, then l3sparse, then l3dump ------------------------

from openflight.iwr6843.driver import UnsupportedCommand  # noqa: E402
from openflight.iwr6843.sparse import OnboardTrack  # noqa: E402

_ONBOARD = OnboardTrack(True, 40, 960.0, 49.0, 0.2, 0.001, 0.03)


class SparseRadar(FakeRadar):
    """Records which read path the monitor took."""

    def __init__(self, raw: bytes, *, tracked=None, sparse=None):
        super().__init__(raw)
        self.tracked = tracked
        self.sparse = sparse
        self.calls = []

    def read_tracked(self):
        self.calls.append("l3track")
        if isinstance(self.tracked, Exception):
            raise self.tracked
        return self.tracked

    def read_sparse(self, planner):
        self.calls.append("l3sparse")
        assert callable(planner)
        if self.sparse is None:
            return None
        raw, noise = self.sparse
        return SparseCapture(raw=raw, noise_power=noise, requested_cells=1, sent_cells=1)

    def read_dump(self):
        self.calls.append("l3dump")
        return super().read_dump()


def _monitor(tmp_path, radar, *, onboard=True, planner=True):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    return IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        slice_planner=(lambda _summary: []) if planner else None,
        onboard_tracking=onboard,
    )


def test_firmware_tracked_cells_are_read_first(tmp_path):
    radar = SparseRadar(b"full", tracked=(b"tracked", 2.5, _ONBOARD), sparse=(b"sparse", 1.0))
    monitor = _monitor(tmp_path, radar)

    raw, noise, track = monitor._read_capture()  # pylint: disable=protected-access

    assert (raw, noise, track) == (b"tracked", 2.5, _ONBOARD)
    assert radar.calls == ["l3track"]


def test_old_firmware_turns_onboard_tracking_off_for_the_session(tmp_path):
    radar = SparseRadar(b"full", tracked=UnsupportedCommand("not recognized"), sparse=(b"s", 1.0))
    monitor = _monitor(tmp_path, radar)

    first = monitor._read_capture()  # pylint: disable=protected-access
    second = monitor._read_capture()  # pylint: disable=protected-access

    assert first == (b"s", 1.0, None) == second
    assert monitor.onboard_tracking is False
    assert radar.calls == ["l3track", "l3sparse", "l3sparse"]


def test_firmware_refusal_falls_back_to_host_planned_cells(tmp_path):
    """No trackCfg or IQ8 storage: try l3sparse, keep trying l3track later."""
    radar = SparseRadar(b"full", tracked=None, sparse=(b"s", 1.0))
    monitor = _monitor(tmp_path, radar)

    assert monitor._read_capture() == (b"s", 1.0, None)  # pylint: disable=protected-access
    assert monitor.onboard_tracking is True
    assert radar.calls == ["l3track", "l3sparse"]


def test_full_dump_is_the_last_resort(tmp_path):
    radar = SparseRadar(_raw_dump(), tracked=None, sparse=None)
    monitor = _monitor(tmp_path, radar)

    raw, noise, track = monitor._read_capture()  # pylint: disable=protected-access

    assert raw == _raw_dump() and noise is None and track is None
    assert radar.calls == ["l3track", "l3sparse", "l3dump"]


def test_onboard_tracking_off_skips_l3track(tmp_path):
    radar = SparseRadar(b"full", tracked=(b"t", 1.0, _ONBOARD), sparse=(b"s", 1.0))
    monitor = _monitor(tmp_path, radar, onboard=False)

    assert monitor._read_capture() == (b"s", 1.0, None)  # pylint: disable=protected-access
    assert radar.calls == ["l3sparse"]


def test_broken_track_stream_fails_the_capture_instead_of_falling_back(tmp_path):
    """After the firmware rearms, l3sparse would read a different window."""
    radar = SparseRadar(b"full", tracked=RuntimeError("l3track cell packet ended early"))
    monitor = _monitor(tmp_path, radar)
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and not capture.valid
    assert "ended early" in capture.error
    assert radar.calls == ["l3track"]
    monitor.stop()


def test_capture_carries_the_firmware_track(tmp_path):
    radar = SparseRadar(b"full", tracked=(_raw_dump(), 0.0, _ONBOARD))
    monitor = _monitor(tmp_path, radar)
    monitor.start()

    edge = time.time()
    assert monitor.notify_trigger(edge)
    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and capture.valid
    assert capture.onboard_track == _ONBOARD
    monitor.stop()


# Data UART rate for `l3dump` readback (firmware/iwr6843/l3_dump.c).
_DUMP_BAUD = 1_041_667
_DUMP_BYTES_PER_SECOND = _DUMP_BAUD / 10  # 8N1: 10 bits on the wire per byte
_MAX_CAPTURE_FRAMES = 64  # firmware L3_MAX_CAPTURE_FRAMES cap
# Raw IQ16 samples per frame: 3 TX x 12 loops x 4 RX x 53 bins x 2 B.
_RAW_FRAME_BYTES = 3 * 12 * 4 * 53 * 2
# On-the-wire overhead (see firmware/iwr6843/dump_format.h and l3_dump.c):
# l3_dump_header_t (20 B) + l3_temperature_report_t (24 B) once per dump,
# plus a 4 B per-frame descriptor and a 2 B per-frame IQ8 scale byte pair.
_DUMP_HEADER_BYTES = 20 + 24
_PER_FRAME_OVERHEAD_BYTES = 4 + 2
_REQUIRED_MARGIN_S = 2.0


def test_dump_fallback_timeout_covers_the_frame_cap():
    """The host's shot-matching deadline must outlast the slowest possible
    `l3dump` diagnostic fallback (the full 64-frame cap) with margin, so a
    future profile can't silently outgrow it."""
    default = (
        inspect.signature(IWR6843CaptureMonitor.capture_for_shot).parameters["timeout_s"].default
    )

    worst_case_bytes = _DUMP_HEADER_BYTES + _MAX_CAPTURE_FRAMES * (
        _RAW_FRAME_BYTES + _PER_FRAME_OVERHEAD_BYTES
    )
    worst_case_s = worst_case_bytes / _DUMP_BYTES_PER_SECOND

    assert default >= worst_case_s + _REQUIRED_MARGIN_S, (
        f"timeout {default}s leaves under {_REQUIRED_MARGIN_S}s over a "
        f"{worst_case_s:.2f}s worst-case {_MAX_CAPTURE_FRAMES}-frame dump"
    )


# --- cadence acceptance soak: stats parsing, no hardware ---------------------


def _load_cadence_soak():
    """Import the hardware-test script's parser without a serial port."""
    import importlib.util
    from pathlib import Path

    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "hardware-test"
        / "iwr6843_cadence_soak.py"
    )
    spec = importlib.util.spec_from_file_location("iwr6843_cadence_soak", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cadence_soak_parses_firmware_stats():
    """The real `stats` line (firmware/iwr6843/l3_dump.c, l3_cli_stats),
    including the compound `plan=`/`used=` fields the regex partially
    matches -- those extra matches must not break parsing."""
    soak = _load_cadence_soak()

    line = (
        "frames=112345 wraps=0 active=1 calib=0x0 rf_faults=0 "
        "hwa_frames=112345 hwa_out=112345 hwa_rearms=112344 hwa_rearm_err=0 "
        "hwa_missed=10 freeze_req=0 freeze_done=0 freeze_to=0 "
        "format=iq8 plan=51pre/0post loops=12 used=100/200\n"
        "iq8_packed=112345 iq8_overrun=0 iq8_clipped=0 pending=0 pre_seen=112345 "
        "post_kept=0 post_seen=0 stride=1 iq8_edma_done=112345 iq8_edma_err=0 "
        "iq8_edma_wait=0 iq8_busy=0/1 iq8_scale=7\n"
    )
    stats = soak.parse_stats(line)

    assert stats["hwa_frames"] == 112345
    assert stats["hwa_missed"] == 10
    assert stats["iq8_overrun"] == 0
    assert stats["iq8_edma_err"] == 0


def test_cadence_soak_reports_rearm_latency_against_the_frame_period():
    soak = _load_cadence_soak()
    stats = soak.parse_stats("hwa_frames=10 rearm_last_us=120 rearm_max_us=310 rearm_timed=9\n")

    summary = soak.rearm_summary(stats, 0.002)

    assert summary == "rearm_last_us=120 rearm_max_us=310 (15.5% of the 2000 us frame) timed=9"


def test_cadence_soak_rearm_latency_is_optional_for_older_firmware():
    soak = _load_cadence_soak()

    assert soak.rearm_summary(soak.parse_stats("hwa_frames=10\n"), 0.003) is None


def test_cadence_soak_fails_closed_on_missing_field():
    """A firmware image that doesn't report a required counter (e.g. an
    older build without L3_IQ8_EDMA_PACK) must not be silently treated as
    zero missed/overrun/error counts."""
    soak = _load_cadence_soak()

    stats = soak.parse_stats("hwa_frames=100 hwa_missed=0\n")

    missing = [f for f in soak.REQUIRED_STAT_FIELDS if f not in stats]
    assert missing == ["iq8_overrun", "iq8_edma_err"]


def test_cadence_soak_frame_period_from_cfg(tmp_path):
    """Frame periodicity is read from the profile, not hardcoded -- the
    wide/iq16 default (3 ms) and dense/iq8 profile (2 ms) differ."""
    soak = _load_cadence_soak()
    cfg = tmp_path / "profile.cfg"
    cfg.write_text("dfeDataOutputMode 1\nframeCfg 0 2 12 0 3 1 0\nsensorStart\n")

    assert soak.frame_period_s(str(cfg)) == pytest.approx(0.003)


def test_cadence_soak_miss_rate_threshold_exceeds_baseline():
    """The pass/fail threshold must be a materiality band above the
    recorded baseline, not equal to it (any real miss would then fail)."""
    soak = _load_cadence_soak()

    assert soak.MAX_MISS_RATE > soak.BASELINE_MISS_RATE


def test_self_trigger_does_not_allocate_a_gpio(tmp_path):
    monitor = _self_trigger_monitor(tmp_path, SelfTriggerRadar(_raw_dump()))
    monitor._button_factory = lambda *a, **k: pytest.fail("self-trigger allocated GPIO")
    monitor.start()
    monitor.stop()


def test_tracker_configuration_precedes_trigger_and_listener(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False, onboard_track_config="trackCfg 0.000135 0.046875 4 1 1.6")
    monitor.stop()
    assert [command for command, _ in radar.commands] == [
        "trackCfg 0.000135 0.046875 4 1 1.6",
        "triggerCfg 12 6.0 1",
    ]
    assert all(thread == threading.current_thread().name for _, thread in radar.commands)
    assert monitor.onboard_tracking


class TeeBandRadar(FakeRadar):
    """Records the config and the tee band in the order the monitor sends them."""

    def __init__(
        self,
        raw: bytes,
        band_error: Exception | None = None,
        band_supported: bool = True,
        cal_supported: bool = True,
    ):
        super().__init__(raw)
        self.events: list[tuple[str, object]] = []
        self.band_error = band_error
        self.band_supported = band_supported
        self.cal_supported = cal_supported

    def send_config(self, path: str, lines=None):
        super().send_config(path, lines)
        self.events.append(("config", path))

    def set_tee_band(self, bins: float) -> bool:
        self.events.append(("band", bins))
        if self.band_error is not None:
            raise self.band_error
        return self.band_supported

    def set_radar_cal(self, args):
        self.events.append(("cal", tuple(args)))
        return self.cal_supported

    def set_elements(self, phases, gains):
        self.events.append(("elem", tuple(phases)))
        return self.cal_supported


def _tee_band_monitor(tmp_path, radar, **kwargs):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    return IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        **kwargs,
    )


@pytest.mark.parametrize("bins", [6.0, 0.5, 64.0])
def test_tee_band_is_sent_after_the_config(tmp_path, bins):
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar, tee_band_bins=bins)

    monitor.start(armed=False)
    try:
        assert radar.events[:2] == [("config", str(tmp_path / "radar.cfg")), ("band", bins)]
    finally:
        monitor.stop()


def test_tee_band_off_is_still_sent_so_a_restart_clears_a_stale_band(tmp_path):
    """The firmware keeps the band across sensorStart: a Pi restarted without
    the flag must send 0, or the band a previous run set stays live."""
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar, tee_band_bins=0.0)

    monitor.start(armed=False)
    try:
        assert monitor.tee_band_bins == 0.0
        assert radar.events[:2] == [("config", str(tmp_path / "radar.cfg")), ("band", 0.0)]
    finally:
        monitor.stop()


def test_tee_band_off_on_older_firmware_is_logged_and_start_continues(tmp_path, caplog):
    radar = TeeBandRadar(_raw_dump(), band_supported=False)
    monitor = _tee_band_monitor(tmp_path, radar, tee_band_bins=0.0)

    with caplog.at_level(logging.INFO, logger="openflight.iwr6843.monitor"):
        monitor.start(armed=False)
    try:
        assert monitor._running  # pylint: disable=protected-access
        assert any(
            r.levelno == logging.INFO and "tee band" in r.getMessage() for r in caplog.records
        )
    finally:
        monitor.stop()


@pytest.mark.parametrize("bins", [-0.5, -6.0, float("nan"), 64.5, float("inf")])
def test_tee_band_must_be_within_the_firmware_limits(tmp_path, bins):
    with pytest.raises(ValueError, match="tee band"):
        _tee_band_monitor(tmp_path, TeeBandRadar(_raw_dump()), tee_band_bins=bins)


def test_rejected_tee_band_stops_the_configured_sensor(tmp_path):
    radar = TeeBandRadar(_raw_dump(), band_error=RuntimeError("config rejected"))
    monitor = _tee_band_monitor(tmp_path, radar, tee_band_bins=6.0)

    with pytest.raises(RuntimeError, match="config rejected"):
        monitor.start(armed=False)

    assert radar.shutdown_events[0] == "sensorStop"
    assert radar.closed


def test_ball_snr_restores_the_firmware_default_when_unset(tmp_path):
    """The firmware keeps the ball snr across sensorStart: without the flag
    the Pi sends 0 so a stale value from a previous run is cleared."""
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar)

    monitor.start(armed=False)
    try:
        assert monitor.ball_snr is None
        assert radar.ball_snrs == [0.0]
    finally:
        monitor.stop()


def test_ball_snr_is_sent_when_set(tmp_path):
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar, ball_snr=4.5)

    monitor.start(armed=False)
    try:
        assert radar.ball_snrs == [4.5]
    finally:
        monitor.stop()


@pytest.mark.parametrize("snr", [0.0, 0.5, -1.0, float("nan"), float("inf")])
def test_ball_snr_must_be_at_least_the_floor(tmp_path, snr):
    with pytest.raises(ValueError, match="ball snr"):
        _tee_band_monitor(tmp_path, TeeBandRadar(_raw_dump()), ball_snr=snr)


def test_ball_snr_default_on_older_firmware_is_logged_and_start_continues(tmp_path, caplog):
    class OldRadar(TeeBandRadar):
        def set_ball_snr(self, snr: float) -> bool:
            self.ball_snrs.append(snr)
            return False

    radar = OldRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar)

    with caplog.at_level(logging.INFO, logger="openflight.iwr6843.monitor"):
        monitor.start(armed=False)
    try:
        assert monitor._running  # pylint: disable=protected-access
        assert any("ball snr" in r.getMessage() for r in caplog.records)
    finally:
        monitor.stop()


def test_tee_band_is_on_at_the_firmware_default_width_by_default(tmp_path):
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar)

    monitor.start(armed=False)
    try:
        assert TEE_BAND_DEFAULT_BINS == 6.0
        assert radar.events[1] == ("band", TEE_BAND_DEFAULT_BINS)
    finally:
        monitor.stop()


def test_board_calibration_is_sent_after_band_and_ball_snr_before_the_trigger(tmp_path):
    radar = TeeBandRadar(_raw_dump())
    board = BoardCalibration.from_file("config/iwr6843_calibration_reference.json")
    monitor = _tee_band_monitor(tmp_path, radar, board_calibration=board)
    monitor.start(armed=False)
    try:
        kinds = [kind for kind, _ in radar.events]
        assert kinds.index("band") < kinds.index("cal") < kinds.index("elem")
        assert radar.events[kinds.index("cal")][1] == board.cal_args
        assert monitor.calibration_applied is True
    finally:
        monitor.stop()


def test_no_calibration_sends_identity_so_a_restart_clears_it(tmp_path):
    radar = TeeBandRadar(_raw_dump())
    monitor = _tee_band_monitor(tmp_path, radar)
    monitor.start(armed=False)
    try:
        cal = next(args for kind, args in radar.events if kind == "cal")
        assert cal == BoardCalibration.identity().cal_args
    finally:
        monitor.stop()


def test_old_firmware_refusing_a_real_calibration_warns_doubts_and_continues(tmp_path, caplog):
    radar = TeeBandRadar(_raw_dump(), cal_supported=False)
    board = BoardCalibration.from_file("config/iwr6843_calibration_reference.json")
    monitor = _tee_band_monitor(tmp_path, radar, board_calibration=board)
    with caplog.at_level(logging.WARNING, logger="openflight.iwr6843.monitor"):
        monitor.start(armed=False)
    try:
        assert monitor._running  # pylint: disable=protected-access
        assert monitor.calibration_applied is False
        assert any("calibration" in r.getMessage() for r in caplog.records)
    finally:
        monitor.stop()


def test_old_firmware_refusing_identity_is_not_a_warning_or_a_doubt(tmp_path, caplog):
    radar = TeeBandRadar(_raw_dump(), cal_supported=False)
    monitor = _tee_band_monitor(tmp_path, radar)
    with caplog.at_level(logging.WARNING, logger="openflight.iwr6843.monitor"):
        monitor.start(armed=False)
    try:
        assert monitor.calibration_applied is True
        assert not [r for r in caplog.records if r.levelno >= logging.WARNING]
    finally:
        monitor.stop()


class _FakePacket:
    """Stands in for a ShotResultPacket: records whether it was doubted."""

    shot_id = 1
    verdict = "ok"
    club_points = 0
    ball_points = 0
    doubted = False

    def with_onboard_angles_doubted(self):
        doubted = _FakePacket()
        doubted.doubted = True
        return doubted


@pytest.mark.parametrize("applied, doubted", [(True, False), (False, True)])
def test_onboard_result_is_doubted_only_when_the_board_is_uncalibrated(tmp_path, applied, doubted):
    radar = TeeBandRadar(_raw_dump())
    radar.shot_result = _FakePacket
    monitor = _tee_band_monitor(tmp_path, radar)
    monitor.calibration_applied = applied
    result = monitor._read_onboard_result()  # pylint: disable=protected-access
    assert result.doubted is doubted


# --- every capture ends with the board armed ------------------------------------
#
# 2026-09-30 17:43: a self-trigger fired, l3dump answered 18 bytes, the capture
# failed, and nothing released or restarted the board: it sat frozen and none
# of the next swings fired. After every capture the worker checks the board
# (``stats``: a capture running, the self-trigger not latched) and rearms it:
# l3release, else the radar restarted as start() configured it, retried until
# it runs.


def _started(tmp_path, radar, *, self_trigger=True, **kwargs) -> IWR6843CaptureMonitor:
    if self_trigger:
        monitor = _self_trigger_monitor(tmp_path, radar, **kwargs)
        monitor.start(armed=False)
    else:
        config = tmp_path / "radar.cfg"
        config.write_text("sensorStart\n", encoding="utf-8")
        monitor = IWR6843CaptureMonitor(
            config_path=config,
            output_dir=tmp_path / "dumps",
            radar=radar,
            button_factory=FakeButton,
            **kwargs,
        )
        monitor.start(armed=False)
    monitor.arm()
    return monitor


def _self_triggered_capture(monitor, radar):
    radar.notices.append(b"Triggered\n")
    return monitor.capture_for_shot(None, timeout_s=1.0)


def test_a_failed_dump_releases_the_frozen_ring(tmp_path):
    radar = SelfTriggerRadar(b"x" * 18)
    radar.stats_replies = [FROZEN_STATS]
    monitor = _started(tmp_path, radar)

    capture = _self_triggered_capture(monitor, radar)

    assert capture is not None and not capture.valid
    assert "short IWR6843 dump" in capture.error
    assert _wait_until(lambda: radar.releases == 1)
    assert len(radar.configs) == 1, "released, not restarted"
    monitor.stop()


def test_a_board_that_refuses_the_release_is_restarted_as_it_was_started(tmp_path):
    """Stopped and not latched, l3release refuses: the whole start-up runs again
    (config, band, ball snr, calibration, triggerCfg) on the worker thread."""
    radar = SelfTriggerRadar(b"x" * 18)
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("IWR6843 did not release the frozen ring: Error")
    monitor = _started(tmp_path, radar)

    _self_triggered_capture(monitor, radar)

    assert _wait_until(lambda: len(radar.configs) == 2)
    assert _wait_until(
        lambda: len([c for c, _t in radar.commands if c.startswith("triggerCfg")]) == 2
    )
    arms = [thread for line, thread in radar.commands if line == "triggerCfg 12 6.0 1"]
    assert arms[1] == "iwr6843-capture", "the worker owns the port after start"
    assert radar.ball_snrs == [0.0, 0.0]
    monitor.stop()


def test_restart_hooks_run_after_a_restart(tmp_path):
    """What the server sets after start (the ball detector's ball cfg) is set again."""
    radar = SelfTriggerRadar(b"x" * 18)
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("refused")
    monitor = _self_trigger_monitor(tmp_path, radar)
    hooked = []
    monitor.add_restart_hook(hooked.append)
    monitor.start(armed=False)
    monitor.arm()

    _self_triggered_capture(monitor, radar)

    assert _wait_until(lambda: hooked == [radar])
    monitor.stop()


def test_a_capture_that_left_the_board_armed_is_not_disturbed(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _started(tmp_path, radar)

    capture = _self_triggered_capture(monitor, radar)

    assert capture is not None and capture.valid
    assert _wait_until(lambda: radar.stats_reads >= 1), "checked"
    time.sleep(0.05)
    assert radar.releases == 0 and len(radar.configs) == 1
    monitor.stop()


def test_unreadable_stats_count_as_not_armed(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    radar.stats_replies = [RuntimeError("serial timeout")]
    monitor = _started(tmp_path, radar)

    _self_triggered_capture(monitor, radar)

    assert _wait_until(lambda: radar.releases == 1)
    monitor.stop()


def test_a_failing_restart_is_retried_until_the_board_runs(tmp_path, monkeypatch):
    monkeypatch.setattr(iwr_monitor, "_REARM_RETRY_BACKOFF_S", 0.0)
    radar = SelfTriggerRadar(b"x" * 18)
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("refused")
    monitor = _started(tmp_path, radar)
    radar.config_errors = [RuntimeError("config rejected"), RuntimeError("config rejected")]

    _self_triggered_capture(monitor, radar)

    assert _wait_until(lambda: len(radar.configs) == 2)
    assert radar.config_errors == []
    monitor.stop()


def test_a_failed_gpio_capture_rearms_the_board_too(tmp_path):
    """Sound-triggered, a failed l3dump can leave the capture stopped as well."""
    radar = FakeRadar(_raw_dump(), error=RuntimeError("IWR6843 dump stalled"))
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("refused")
    monitor = _started(tmp_path, radar, self_trigger=False)
    edge = time.time()
    assert monitor.notify_trigger(edge)

    capture = monitor.capture_for_shot(edge, timeout_s=1.0)

    assert capture is not None and not capture.valid
    assert _wait_until(lambda: len(radar.configs) == 2)
    monitor.stop()


def test_an_edge_while_the_board_is_being_rearmed_is_rejected_as_busy(tmp_path):
    release = threading.Event()

    class SlowRestart(FakeRadar):
        def send_config(self, path: str, lines=None):
            if self.configs:  # the restart, not the start
                release.wait(timeout=1.0)
            super().send_config(path, lines)

    radar = SlowRestart(_raw_dump(), error=RuntimeError("stalled"))
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("refused")
    monitor = _started(tmp_path, radar, self_trigger=False)
    assert monitor.notify_trigger(time.time())
    try:
        assert _wait_until(lambda: monitor._rearming)  # pylint: disable=protected-access
        assert monitor.notify_trigger(time.time() + 1.0) is False
    finally:
        release.set()
    assert _wait_until(lambda: len(radar.configs) == 2)
    monitor.stop()


# --- the self-trigger's detect core (trackCfg detectCore) ----------------------


class DetectCoreRadar(SelfTriggerRadar):
    """SelfTriggerRadar whose ``trackCfg detectCore`` lands in the same
    command list as triggerCfg, so the order between the two is visible."""

    def __init__(self, raw: bytes, *, takes: str | None = None, error: Exception | None = None):
        super().__init__(raw)
        self.takes = takes  # what the board reports as requested; None: what was asked
        self.core_error = error

    def detect_core(self, core=None):
        status = super().detect_core(core)
        self.commands.append((f"trackCfg detectCore {core}", threading.current_thread().name))
        if self.core_error is not None:
            raise self.core_error
        return SimpleNamespace(requested=self.takes or status.requested, active=status.active)


def _core_monitor(tmp_path, radar, capture_format: str | None, *, self_trigger=True):
    config = tmp_path / "radar.cfg"
    lines = ([f"captureFormat {capture_format}"] if capture_format else []) + ["sensorStart"]
    config.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
        self_trigger=SelfTriggerConfig(tee_bin=12, snr=6.0) if self_trigger else None,
    )


def _lines(radar) -> list[str]:
    return [line for line, _thread in radar.commands]


MSS_WARNING = "the MSS scores the self-trigger's bins"


def test_the_monitor_asks_for_a_core_the_driver_knows():
    """The firmware dropped the mss choice (2026-10-01); a name the driver
    refuses would stop every kiosk start."""
    assert DETECT_CORE in DETECT_CORES


def test_the_fake_radar_refuses_a_core_the_driver_would_refuse():
    with pytest.raises(ValueError, match="core must be one of"):
        SelfTriggerRadar(_raw_dump()).detect_core("mss")


@pytest.mark.parametrize("capture_format", ["iq16", "adaptive16", "compact16", "iq8", None])
def test_the_dss_is_chosen_before_arming_on_every_profile(tmp_path, capture_format):
    """dss clears a latched MSS; at 2 ms an armed MSS detector starves the
    CLI (2026-10-01 rig), so it is sent before triggerCfg, not after."""
    radar = DetectCoreRadar(_raw_dump())
    monitor = _core_monitor(tmp_path, radar, capture_format)
    monitor.start(armed=False)

    assert monitor.detect_core == "dss"
    assert _lines(radar) == ["trackCfg detectCore dss", "triggerCfg 12 6.0 1"]
    monitor.stop()


@pytest.mark.parametrize("capture_format", MSS_SCORED_CAPTURE_FORMATS)
def test_a_profile_the_dss_cannot_read_warns_that_the_mss_scores(tmp_path, caplog, capture_format):
    monitor = _core_monitor(tmp_path, DetectCoreRadar(_raw_dump()), capture_format)
    with caplog.at_level(logging.WARNING, logger="openflight"):
        monitor.start(armed=False)

    warnings = [r.getMessage() for r in caplog.records if MSS_WARNING in r.getMessage()]
    assert len(warnings) == 1 and capture_format in warnings[0]
    monitor.stop()


@pytest.mark.parametrize("capture_format", ["iq16", None])
def test_a_profile_the_dss_reads_does_not_warn(tmp_path, caplog, capture_format):
    monitor = _core_monitor(tmp_path, DetectCoreRadar(_raw_dump()), capture_format)
    with caplog.at_level(logging.WARNING, logger="openflight"):
        monitor.start(armed=False)

    assert not [r for r in caplog.records if MSS_WARNING in r.getMessage()]
    monitor.stop()


def test_without_a_self_trigger_no_core_is_chosen_and_nothing_warns(tmp_path, caplog):
    """The detect core only scores the self-trigger's bins."""
    radar = DetectCoreRadar(_raw_dump())
    monitor = _core_monitor(tmp_path, radar, "adaptive16", self_trigger=False)
    with caplog.at_level(logging.WARNING, logger="openflight"):
        monitor.start(armed=False)

    assert monitor.detect_core is None
    assert radar.commands == []
    assert not [r for r in caplog.records if MSS_WARNING in r.getMessage()]
    monitor.stop()


def test_firmware_without_the_command_still_arms_and_says_so(tmp_path, caplog):
    """An image that predates detectCore scores on its MSS; nothing to choose."""
    radar = DetectCoreRadar(
        _raw_dump(), error=DspLinkError("trackCfg detectCore: no detect line in 'Done'")
    )
    monitor = _core_monitor(tmp_path, radar, "iq16")
    with caplog.at_level(logging.WARNING, logger="openflight"):
        monitor.start(armed=False)

    assert monitor.detect_core is None
    assert _lines(radar) == ["trackCfg detectCore dss", "triggerCfg 12 6.0 1"]
    assert any("did not take trackCfg detectCore dss" in r.getMessage() for r in caplog.records)
    monitor.stop()


def test_a_board_that_reports_another_core_fails_start_and_never_arms(tmp_path):
    radar = DetectCoreRadar(_raw_dump(), takes="verify")
    monitor = _core_monitor(tmp_path, radar, "iq16")

    with pytest.raises(RuntimeError, match="detect core dss not taken: requested=verify"):
        monitor.start(armed=False)
    assert "triggerCfg 12 6.0 1" not in _lines(radar), "never armed on the wrong core"


def test_a_restart_chooses_the_dss_again_before_arming(tmp_path):
    """A board found stopped is restarted on the worker thread; the firmware
    keeps a latch across sensorStart, so dss is sent again ahead of triggerCfg."""
    radar = DetectCoreRadar(b"x" * 18)
    radar.stats_replies = [STOPPED_STATS]
    radar.release_error = RuntimeError("refused")
    monitor = _core_monitor(tmp_path, radar, "iq16")
    monitor.start(armed=False)
    monitor.arm()

    _self_triggered_capture(monitor, radar)

    assert _wait_until(lambda: _lines(radar).count("triggerCfg 12 6.0 1") == 2)
    assert _lines(radar) == [
        "trackCfg detectCore dss",
        "triggerCfg 12 6.0 1",
        "trackCfg detectCore dss",
        "triggerCfg 12 6.0 1",
    ]
    restart_thread = [t for line, t in radar.commands if line == "trackCfg detectCore dss"][1]
    assert restart_thread == "iwr6843-capture"
    assert monitor.detect_core == "dss"
    monitor.stop()
