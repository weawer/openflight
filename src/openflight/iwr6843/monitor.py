"""GPIO-triggered IWR6843 L3 capture and OPS-shot correlation."""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from openflight.gpio_factory import ensure_lgpio_pin_factory
from openflight.iwr6843.driver import IWR6843Radar, UnsupportedCommand
from openflight.iwr6843.dump import HEADER, parse_header, payload_nbytes
from openflight.iwr6843.sparse import OnboardTrack, SlicePlanner
from openflight.iwr6843.tracking import RANGE_SPAN_M

logger = logging.getLogger(__name__)

_GRACEFUL_DUMP_SHUTDOWN_S = 12.0
# Pause after a serial error in the self-trigger listener so a dead port
# logs a warning twice a second instead of spinning.
_LISTENER_ERROR_BACKOFF_S = 0.5


@dataclass(frozen=True)
class CaptureConfigSummary:
    """The few cfg fields the host needs outside the firmware."""

    chirp_tx_masks: tuple[str, ...]
    first_window_start: int | None
    first_window_bins: int | None
    loops: int | None = None
    frame_period_s: float | None = None
    chirp_period_s: float | None = None
    capture_format: str | None = None

    @property
    def n_tx(self) -> int:
        """Number of distinct transmitters enabled by the chirp sequence."""
        return len(set(self.chirp_tx_masks))

    @property
    def loop_period_s(self) -> float | None:
        """Elapsed time between successive chirps from the same transmitter."""
        if self.chirp_period_s is None or not self.n_tx:
            return None
        return self.chirp_period_s * self.n_tx


def read_capture_config(config_path: str | Path) -> CaptureConfigSummary:
    """Parse chirp TX masks and the first saved range window from a cfg."""
    masks: list[str] = []
    window: tuple[int, int] | None = None
    loops: int | None = None
    frame_period_s: float | None = None
    chirp_period_s: float | None = None
    capture_format: str | None = None
    with Path(config_path).open(encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            fields = line.split()
            if line.startswith("chirpCfg"):
                masks.append(line.rsplit(maxsplit=1)[-1])
            elif line.startswith("phaseCaptureCfg") and window is None:
                window = (int(fields[1]), int(fields[2]))
            elif line.startswith("profileCfg"):
                chirp_period_s = (float(fields[3]) + float(fields[5])) * 1e-6
            elif line.startswith("frameCfg"):
                loops = int(fields[3])
                frame_period_s = float(fields[5]) * 1e-3
            elif line.startswith("captureFormat"):
                capture_format = fields[1].lower()
    return CaptureConfigSummary(
        chirp_tx_masks=tuple(masks),
        first_window_start=window[0] if window else None,
        first_window_bins=window[1] if window else None,
        loops=loops,
        frame_period_s=frame_period_s,
        chirp_period_s=chirp_period_s,
        capture_format=capture_format,
    )


def tx_order_from_config(config_path: str | Path) -> str:
    """Infer the vertical physical TX order from chirp masks in a cfg."""
    masks = list(read_capture_config(config_path).chirp_tx_masks)
    if masks == ["1", "4"]:
        return "normal"
    if masks == ["4", "1"]:
        return "reversed"
    if masks == ["1", "2", "4"]:
        return "normal"
    raise ValueError(f"IWR6843 config must contain chirp TX masks 1/4, 4/1, or 1/2/4, got {masks}")


def tee_local_bin(tee_range_m: float, config_path: str | Path, fft_size: int = 128) -> int:
    """Tee bin inside the cfg's first saved window.

    Raises when the tee falls outside that window: the firmware would watch a
    bin that never sees the ball.
    """
    summary = read_capture_config(config_path)
    if summary.first_window_start is None or summary.first_window_bins is None:
        raise ValueError(f"{config_path} has no phaseCaptureCfg")
    absolute = int(round(tee_range_m / (RANGE_SPAN_M / fft_size)))
    local = absolute - summary.first_window_start
    if not 0 <= local < summary.first_window_bins:
        raise ValueError(
            f"tee at {tee_range_m:.2f} m (bin {absolute}) is outside the first capture "
            f"window, bins {summary.first_window_start}-"
            f"{summary.first_window_start + summary.first_window_bins - 1}"
        )
    return local


@dataclass(frozen=True)
class SelfTriggerConfig:
    """Firmware ``triggerCfg``: freeze when the ball leaves the tee bin."""

    local_bin: int
    level: float
    hits: int

    def __post_init__(self) -> None:
        if self.local_bin < 0:
            raise ValueError(f"self-trigger bin must be >= 0, got {self.local_bin}")
        if not math.isfinite(self.level) or self.level <= 0.0:
            raise ValueError(f"self-trigger level must be > 0, got {self.level}")
        if self.hits < 1:
            # triggerCfg treats 0 hits as "off", which would leave the host
            # waiting for a notice that never comes.
            raise ValueError(f"self-trigger hits must be >= 1, got {self.hits}")

    @property
    def command(self) -> str:
        """CLI line that arms this trigger."""
        return f"triggerCfg {self.local_bin} {self.level} {self.hits}"


# hits=0 disables the firmware trigger (see l3_cli_triggerCfg).
SELF_TRIGGER_OFF_COMMAND = "triggerCfg 0 0 0"


@dataclass(frozen=True)
class _SerialJob:
    """Work that needs the radar serial port, run on the capture worker."""

    name: str
    run: Callable[[IWR6843Radar], None]


_STOP = object()


@dataclass(frozen=True)
class IWR6843Capture:
    """One GPIO edge and its completed L3 dump."""

    sequence: int
    trigger_timestamp: float
    completed_timestamp: float
    dump_duration_s: float
    raw: bytes | None
    path: Path | None
    error: str | None = None
    temperature_report: dict[str, int] | None = None
    noise_power: float | None = None
    onboard_track: OnboardTrack | None = None

    @property
    def valid(self) -> bool:
        """Whether a complete dump was captured."""
        return self.raw is not None and self.error is None


class IWR6843CaptureMonitor:
    """Capture TI rolling-buffer dumps on the same sound edge used by OPS.

    The GPIO callback only timestamps and queues the edge. Serial transfer is
    handled on a dedicated thread because one 768 KiB dump takes several
    seconds at the firmware UART rate.

    That worker is the only code that touches the radar serial port once the
    monitor is running. Other serial work (the late-window retune) is queued
    with :meth:`submit` so it cannot interleave with a capture or with the
    self-trigger listener.
    """

    def __init__(
        self,
        *,
        config_path: str | Path,
        output_dir: str | Path,
        port: str | None = None,
        gpio_pin: int = 17,
        radar: IWR6843Radar | None = None,
        button_factory: Callable | None = None,
        match_tolerance_s: float = 0.75,
        save_dumps: bool = False,
        trigger_observers: list[Callable[[float], None]] | None = None,
        slice_planner: SlicePlanner | None = None,
        self_trigger: SelfTriggerConfig | None = None,
        onboard_tracking: bool = False,
    ):
        self.config_path = Path(config_path)
        self.output_dir = Path(output_dir).expanduser()
        self.gpio_pin = gpio_pin
        self.match_tolerance_s = match_tolerance_s
        self.save_dumps = save_dumps
        self.radar = radar or IWR6843Radar(port=port)
        self._button_factory = button_factory
        self._button = None
        self._running = False
        self._armed = False
        self._capture_active = False
        self._job_active = False
        self._edge_pending = False
        self._sequence = 0
        self._last_edge_timestamp = 0.0
        self._events: queue.Queue = queue.Queue()
        self._captures: deque[IWR6843Capture] = deque()
        self._condition = threading.Condition()
        self._worker: threading.Thread | None = None
        self._trigger_observers = list(trigger_observers or [])
        self.slice_planner = slice_planner
        self.self_trigger = self_trigger
        # Firmware picks the cells itself (l3track). Cleared if it cannot.
        self.onboard_tracking = onboard_tracking
        self._adaptive_retention = False
        self._trigger_notice = b""
        self._release_pending = False

    @property
    def watch_self_trigger(self) -> bool:
        """True when the firmware trigger replaces the GPIO edge."""
        return self.self_trigger is not None

    @property
    def port(self) -> str:
        """Connected TI serial port."""
        return self.radar.port

    def start(self, *, armed: bool = True, onboard_track_config: str | None = None) -> None:
        """Configure the radar and GPIO, optionally arming trigger capture."""
        if self._running:
            return
        if not self.config_path.is_file():
            raise FileNotFoundError(f"IWR6843 config not found: {self.config_path}")
        config = read_capture_config(self.config_path)
        self._adaptive_retention = config.capture_format == "adaptive16"
        if self.self_trigger is not None and config.capture_format == "iq8":
            raise ValueError("IQ8 capture does not support the IWR6843 self-trigger")
        if self.save_dumps:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        configured = False
        try:
            self.radar.send_config(str(self.config_path))
            configured = True
            # Before the worker starts: after that only the worker may talk
            # to the radar.
            if onboard_track_config is not None:
                self._configure_onboard_tracking(onboard_track_config)
            self._apply_self_trigger()

            if not self.watch_self_trigger:
                button_factory = self._button_factory
                if button_factory is None:
                    # Must precede the first gpiozero device: on a Pi 5 gpiozero's
                    # own auto-detection fails outright. See gpio_factory.
                    ensure_lgpio_pin_factory()

                    from gpiozero import (
                        Button,  # pylint: disable=import-error,import-outside-toplevel
                    )

                    button_factory = Button
                # No gpiozero debounce: lgpio delays delivery by the debounce interval,
                # which previously cost the first 50 ms of ball flight.
                self._button = button_factory(self.gpio_pin, pull_up=False, bounce_time=None)
            self._running = True
            self._worker = threading.Thread(
                target=self._capture_loop,
                name="iwr6843-capture",
                daemon=True,
            )
            self._worker.start()
            if armed:
                self.arm()
        except Exception:
            self._running = False
            if self._button is not None:
                self._button.close()
                self._button = None
            if configured:
                self._stop_sensor_and_close()
            else:
                self.radar.close()
            raise
        logger.info(
            "[IWR6843] Configured on BCM%d using %s (%s%s%s)",
            self.gpio_pin,
            self.port,
            self.config_path.name,
            ", armed" if self._armed else ", waiting for OPS",
            f", self-trigger {self.self_trigger.command!r}" if self.self_trigger else "",
        )

    def _configure_onboard_tracking(self, command: str) -> bool:
        """Hand the rig limits to the firmware tracker; True when it accepts them.

        Optional: older firmware has no ``trackCfg``, and any failure here only
        means the host keeps planning cells over ``l3sparse``.
        """
        self.onboard_tracking = False
        try:
            reply = self.radar.cmd(command, 2.0)
        except Exception as error:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] trackCfg failed (%s); the host will plan cells", error)
            return False
        if "Error" in reply or "Done" not in reply:
            logger.info(
                "[IWR6843] Firmware has no on-chip tracker (%s); the host will plan cells",
                reply.strip() or "no reply",
            )
            return False
        self.onboard_tracking = True
        logger.info("[IWR6843] On-chip tracker armed: %s", command)
        return True

    def _apply_self_trigger(self) -> None:
        """Send ``triggerCfg`` for the configured self-trigger, if any."""
        if self.self_trigger is None:
            return
        reply = self.radar.cmd(self.self_trigger.command, 2.0)
        if "Error" in reply or "Done" not in reply:
            raise RuntimeError(f"IWR6843 self-trigger rejected: {reply.strip()}")

    def _disable_self_trigger(self) -> None:
        """Stop the firmware trigger before a profile it was not tuned for."""
        if self.self_trigger is None:
            return
        reply = self.radar.cmd(SELF_TRIGGER_OFF_COMMAND, 2.0)
        if "Error" in reply or "Done" not in reply:
            raise RuntimeError(f"IWR6843 self-trigger disable rejected: {reply.strip()}")

    def arm(self) -> None:
        """Accept triggers after the OPS trigger path is fully initialized."""
        if not self._running:
            raise RuntimeError("cannot arm an IWR6843 monitor that is not running")
        if self._armed:
            return
        # Attach while logically disarmed so a line already high from OPS
        # startup cannot synchronously create a false capture. The self-trigger
        # path listens for the firmware line instead of this pin; notices that
        # arrive while disarmed are consumed and released by the worker.
        if not self.watch_self_trigger:
            self._button.when_pressed = self.notify_trigger
        self._armed = True
        logger.info(
            "[IWR6843] Armed on %s",
            "firmware self-trigger" if self.watch_self_trigger else f"BCM{self.gpio_pin}",
        )

    def add_trigger_observer(self, observer: Callable[[float], None]) -> None:
        """Notify one more listener when a trigger is accepted."""
        self._trigger_observers.append(observer)

    def submit(self, name: str, job: Callable[[IWR6843Radar], None]) -> bool:
        """Queue serial work behind any capture. False when not running.

        The job owns its own error reporting; the worker only logs what escapes.
        """
        if not self._running:
            return False
        self._events.put(_SerialJob(name=name, run=job))
        return True

    def run_on_other_profile(self, job: Callable[[IWR6843Radar], None]) -> None:
        """Run ``job`` (which retunes and restores the radar) with the trigger off.

        Call from a submitted job. The self-trigger is re-applied afterwards
        even when ``job`` fails, so the impact profile keeps its trigger.
        """
        self._disable_self_trigger()
        try:
            job(self.radar)
        finally:
            self._apply_self_trigger()

    def notify_trigger(self, timestamp: float | None = None) -> bool:
        """Queue a trigger edge without doing serial work in the callback."""
        if not self._running or not self._armed:
            return False
        edge_timestamp = time.time() if timestamp is None else float(timestamp)
        with self._condition:
            # Reject acoustic ringing, a second edge while the UART dump is in
            # flight, and any edge while another profile is loaded. The OPS
            # side makes the same shot wait.
            if (
                self._capture_active
                or self._job_active
                or self._edge_pending
                or edge_timestamp - self._last_edge_timestamp < 0.1
            ):
                logger.debug("[IWR6843] Ignoring duplicate/busy trigger edge")
                return False
            self._last_edge_timestamp = edge_timestamp
            self._edge_pending = True
            self._events.put_nowait(edge_timestamp)
            self._condition.notify_all()
        for observer in self._trigger_observers:
            try:
                observer(edge_timestamp)
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Trigger observer failed", exc_info=True)
        return True

    def _validate_dump(self, raw: bytes) -> dict:
        if len(raw) < HEADER.size:
            raise ValueError(f"short IWR6843 dump: {len(raw)} bytes")
        metadata = parse_header(raw)
        expected = metadata["header_nbytes"] + payload_nbytes(metadata, raw)
        if len(raw) != expected:
            raise ValueError(f"short IWR6843 dump: {len(raw)} bytes, expected {expected}")
        return metadata

    def _capture_path(self, sequence: int, trigger_timestamp: float) -> Path:
        timestamp = datetime.fromtimestamp(trigger_timestamp).strftime("%Y%m%d_%H%M%S_%f")[:-3]
        return self.output_dir / f"iwr6843_{timestamp}_{sequence:03d}.l3dump"

    def _listen_for_self_trigger(self) -> None:
        """Block briefly on the CLI for the firmware's ``Triggered`` line.

        The read returns as soon as a byte arrives, so the trigger reaches the
        OPS within about a millisecond of the notice instead of a poll period.
        """
        if not self._release_pending:
            found, self._trigger_notice = self.radar.wait_trigger_notice(self._trigger_notice)
            if not found:
                return
            if self.notify_trigger():
                return
            self._release_pending = True
            logger.info("[IWR6843] Releasing an unaccepted self-trigger capture")
        self.radar.release_sparse_freeze()
        self._release_pending = False

    def _next_event(self):
        """Next queued edge/job/stop. Listens for the self-trigger while idle."""
        if not self.watch_self_trigger:
            return self._events.get()
        while self._running:
            try:
                return self._events.get_nowait()
            except queue.Empty:
                pass
            try:
                self._listen_for_self_trigger()
            except Exception:  # pylint: disable=broad-exception-caught
                # Keep the worker alive: it still owns captures and jobs.
                logger.warning("[IWR6843] Self-trigger listener error", exc_info=True)
                time.sleep(_LISTENER_ERROR_BACKOFF_S)
        return _STOP

    def _read_capture(self) -> tuple[bytes, float | None, OnboardTrack | None]:
        """Read one frozen capture, preferring the least serial traffic.

        Firmware-tracked cells (``l3track``), then host-planned cells
        (``l3sparse``), then the full ring (``l3dump``). Each step falls back
        only when the firmware refused before streaming.
        """
        if self._adaptive_retention:
            return self.radar.read_dump(), None, None
        if self.onboard_tracking:
            try:
                tracked = self.radar.read_tracked()
            except UnsupportedCommand:
                logger.warning("[IWR6843] Firmware has no l3track; the host will plan cells")
                self.onboard_tracking = False
                tracked = None
            if tracked is not None:
                raw, noise_power, track = tracked
                logger.info(
                    "[IWR6843] Firmware track: %s",
                    f"{track.slope_bins:.0f} bins/s, {track.n_inliers} inliers"
                    if track.found
                    else "no ball",
                )
                if noise_power is not None and noise_power <= 0:
                    noise_power = None
                return raw, noise_power, track
        if self.slice_planner is not None:
            sparse = self.radar.read_sparse(self.slice_planner)
            if sparse is not None:
                if sparse.truncated:
                    logger.warning(
                        "[IWR6843] Sparse capture carried %d of %d planned cells",
                        sparse.sent_cells,
                        sparse.requested_cells,
                    )
                return sparse.raw, sparse.noise_power, None
        return self.radar.read_dump(), None, None

    def _capture_loop(self) -> None:
        while self._running:
            event = self._next_event()
            if event is None or event is _STOP or not self._running:
                break
            if isinstance(event, _SerialJob):
                self._run_job(event)
            else:
                self._capture(float(event))

    def _run_job(self, job: _SerialJob) -> None:
        with self._condition:
            self._job_active = True
        start = time.monotonic()
        try:
            job.run(self.radar)
        except Exception:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] Serial job %s failed", job.name, exc_info=True)
        finally:
            with self._condition:
                self._job_active = False
                self._condition.notify_all()
            logger.info(
                "[IWR6843] Serial job %s finished in %.2fs", job.name, time.monotonic() - start
            )

    def _capture(self, edge_timestamp: float) -> None:
        with self._condition:
            self._edge_pending = False
            self._capture_active = True
            self._sequence += 1
            sequence = self._sequence
        start = time.time()
        raw = None
        path = None
        error = None
        metadata = None
        noise_power = None
        onboard_track = None
        try:
            logger.info("[IWR6843] Trigger #%d: reading track samples", sequence)
            raw, noise_power, onboard_track = self._read_capture()
            metadata = self._validate_dump(raw)
            if self.save_dumps:
                path = self._capture_path(sequence, edge_timestamp)
                path.write_bytes(raw)
            retention = metadata.get("retention")
            if retention and retention["reason"] != "complete":
                raise ValueError(f"adaptive retention stopped: {retention['reason']}")
        except Exception as exc:  # pylint: disable=broad-exception-caught
            error = str(exc)
            raw = None
            logger.warning("[IWR6843] Capture #%d failed: %s", sequence, exc, exc_info=True)
        completed = time.time()
        capture = IWR6843Capture(
            sequence=sequence,
            trigger_timestamp=edge_timestamp,
            completed_timestamp=completed,
            dump_duration_s=completed - start,
            raw=raw,
            path=path,
            error=error,
            temperature_report=(
                metadata.get("temperature_report") if metadata is not None else None
            ),
            noise_power=noise_power,
            onboard_track=onboard_track,
        )
        with self._condition:
            self._capture_active = False
            self._captures.append(capture)
            self._condition.notify_all()
        logger.info(
            "[IWR6843] Capture #%d complete: %s in %.2fs",
            sequence,
            f"{len(raw)} bytes" if raw is not None else error,
            capture.dump_duration_s,
        )

    def capture_for_shot(
        self,
        impact_timestamp: float | None,
        *,
        timeout_s: float = 12.0,
    ) -> IWR6843Capture | None:
        """Consume the capture nearest an OPS impact timestamp."""
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if impact_timestamp is None and self._captures:
                    return self._captures.popleft()

                if impact_timestamp is not None:
                    cutoff = impact_timestamp - self.match_tolerance_s
                    while self._captures and self._captures[0].trigger_timestamp < cutoff:
                        stale = self._captures.popleft()
                        logger.warning(
                            "[IWR6843] Discarding unmatched capture #%d (edge %.3f, shot %.3f)",
                            stale.sequence,
                            stale.trigger_timestamp,
                            impact_timestamp,
                        )
                    matches = [
                        capture
                        for capture in self._captures
                        if abs(capture.trigger_timestamp - impact_timestamp)
                        <= self.match_tolerance_s
                    ]
                    if matches:
                        selected = min(
                            matches,
                            key=lambda capture: abs(capture.trigger_timestamp - impact_timestamp),
                        )
                        self._captures.remove(selected)
                        return selected

                    matching_capture_active = abs(
                        self._last_edge_timestamp - impact_timestamp
                    ) <= self.match_tolerance_s and (self._capture_active or self._edge_pending)
                    if (
                        time.time() > impact_timestamp + self.match_tolerance_s
                        and not matching_capture_active
                    ):
                        return None

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)

    def stop(self) -> None:
        """Drain active capture, stop firmware, then release host resources."""
        if not self._running:
            return
        self._armed = False
        self._running = False
        if self._button is not None:
            self._button.when_pressed = None
            self._button.close()
            self._button = None
        self._events.put_nowait(None)
        if self._worker is not None:
            # Preserve a complete debug dump and its trailing CLI prompt before
            # issuing sensorStop. Closing early strands firmware mid-transfer.
            self._worker.join(timeout=_GRACEFUL_DUMP_SHUTDOWN_S)
            if self._worker.is_alive():
                logger.warning(
                    "[IWR6843] Active dump did not finish within %.1fs; "
                    "forcing serial close (board reset may be required)",
                    _GRACEFUL_DUMP_SHUTDOWN_S,
                )
                self.radar.close()
                self._worker.join(timeout=2.0)
            else:
                self._stop_sensor_and_close()
            self._worker = None
        else:
            self._stop_sensor_and_close()
        logger.info("[IWR6843] Capture monitor stopped")

    def _stop_sensor_and_close(self) -> None:
        """Best-effort firmware stop that never leaks the serial descriptor."""
        try:
            self.radar.stop_sensor()
            logger.info("[IWR6843] Firmware capture stopped and verified inactive")
        except Exception:  # pylint: disable=broad-exception-caught
            logger.warning(
                "[IWR6843] Firmware did not stop cleanly; board reset may be required",
                exc_info=True,
            )
        finally:
            self.radar.close()


__all__ = [
    "SELF_TRIGGER_OFF_COMMAND",
    "CaptureConfigSummary",
    "IWR6843Capture",
    "IWR6843CaptureMonitor",
    "SelfTriggerConfig",
    "read_capture_config",
    "tee_local_bin",
    "tx_order_from_config",
]
