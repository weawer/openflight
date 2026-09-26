"""Tests for GPIO-triggered TI capture and OPS shot correlation."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from openflight.iwr6843.dump import pack_dump
from openflight.iwr6843.monitor import (
    SELF_TRIGGER_OFF_COMMAND,
    IWR6843CaptureMonitor,
    SelfTriggerConfig,
    read_capture_config,
    tee_local_bin,
    tx_order_from_config,
)
from openflight.iwr6843.sparse import SparseCapture


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

    def send_config(self, path: str):
        self.configs.append(path)

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


def test_capture_monitor_rejects_iq8_self_trigger_before_configuring_hardware(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text(
        "captureFormat iq8\nphaseCaptureCfg 20 53 14 32 53 10 47 53 64 12 1\n",
        encoding="utf-8",
    )
    radar = FakeRadar(_raw_dump())
    monitor = IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        self_trigger=SelfTriggerConfig(local_bin=1, level=2.0, hits=2),
    )

    with pytest.raises(ValueError, match="IQ8.*self-trigger"):
        monitor.start()

    assert radar.configs == []


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
        self.releases = 0
        self.sparse = None
        self.sparse_error: Exception | None = None

    def cmd(self, line: str, window: float = 1.5) -> str:
        del window
        self.commands.append((line, threading.current_thread().name))
        return self.cmd_reply

    def wait_trigger_notice(self, pending: bytes = b"") -> tuple[bool, bytes]:
        if not self.notices:
            time.sleep(0.002)
            return False, pending
        pending += self.notices.pop(0)
        if b"Triggered" in pending:
            return True, b""
        return False, pending

    def release_sparse_freeze(self) -> None:
        self.releases += 1

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
        self_trigger=SelfTriggerConfig(local_bin=12, level=1000.0, hits=2),
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


def test_self_trigger_config_is_sent_before_the_worker_owns_the_port(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor.start(armed=False)
    monitor.stop()

    assert radar.commands[0] == ("triggerCfg 12 1000.0 2", threading.current_thread().name)


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
    assert lines == ["triggerCfg 12 1000.0 2", SELF_TRIGGER_OFF_COMMAND, "triggerCfg 12 1000.0 2"]


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
        ({"local_bin": -1, "level": 1000.0, "hits": 2}, "bin"),
        ({"local_bin": 3, "level": 0.0, "hits": 2}, "level"),
        ({"local_bin": 3, "level": 1000.0, "hits": 0}, "hits"),
    ],
)
def test_self_trigger_config_rejects_values_the_firmware_would_misread(kwargs, message):
    with pytest.raises(ValueError, match=message):
        SelfTriggerConfig(**kwargs)


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


def test_tee_local_bin_is_relative_to_the_first_window(tmp_path):
    path = _cfg(tmp_path, "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1")

    # 1.575 m / (6 m / 128) = bin 33.6 -> 34; 34 - 20 = 14.
    assert tee_local_bin(1.575, path) == 14


@pytest.mark.parametrize("tee_m", [0.5, 4.0])
def test_tee_outside_the_first_window_is_an_error(tmp_path, tee_m):
    path = _cfg(tmp_path, "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1")

    with pytest.raises(ValueError, match="outside the first capture window"):
        tee_local_bin(tee_m, path)


def test_tee_bin_needs_a_capture_window(tmp_path):
    with pytest.raises(ValueError, match="no phaseCaptureCfg"):
        tee_local_bin(1.5, _cfg(tmp_path, "sensorStart"))


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
        "triggerCfg 12 1000.0 2",
    ]
    assert all(thread == threading.current_thread().name for _, thread in radar.commands)
    assert monitor.onboard_tracking


def test_rejected_self_trigger_is_released(tmp_path):
    radar = SelfTriggerRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    monitor._running = monitor._armed = True
    monitor._last_edge_timestamp = time.time()
    radar.notices.append(b"Triggered\n")
    monitor._listen_for_self_trigger()
    assert radar.releases == 1


def test_failed_release_is_retried_without_another_notice(tmp_path):
    class RetryRadar(SelfTriggerRadar):
        def release_sparse_freeze(self):
            self.releases += 1
            if self.releases == 1:
                raise OSError("temporary failure")

    radar = RetryRadar(_raw_dump())
    monitor = _self_trigger_monitor(tmp_path, radar)
    radar.notices.append(b"Triggered\n")
    try:
        monitor._listen_for_self_trigger()
    except OSError:
        pass
    monitor._listen_for_self_trigger()
    assert radar.releases == 2
