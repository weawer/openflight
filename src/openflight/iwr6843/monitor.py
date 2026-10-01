"""GPIO-triggered IWR6843 L3 capture and OPS-shot correlation."""

from __future__ import annotations

import logging
import math
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from openflight.gpio_factory import ensure_lgpio_pin_factory
from openflight.iwr6843.board_calibration import BoardCalibration
from openflight.iwr6843.driver import IWR6843Radar, UnsupportedCommand
from openflight.iwr6843.dsp_link import DspLinkError
from openflight.iwr6843.dump import HEADER, parse_header, payload_nbytes
from openflight.iwr6843.firmware_version import FirmwareVersion
from openflight.iwr6843.self_trigger import (
    FIRMWARE_TRIGGER_DEFAULT_BIN,
    FIRMWARE_TRIGGER_DEFAULT_SNR,
    FLOOR_PAUSE_S,
    FLOOR_SAMPLE_S,
    TEE_BAND_DEFAULT_BINS,
    check_ball_snr,
    level_above_floor,
    tee_power_from_stats,
)
from openflight.iwr6843.sparse import OnboardTrack, SlicePlanner
from openflight.iwr6843.tracking import RANGE_SPAN_M, same_tx_loop_period_s

logger = logging.getLogger(__name__)

_GRACEFUL_DUMP_SHUTDOWN_S = 12.0
# Pause after a serial error in the self-trigger listener so a dead port
# logs a warning twice a second instead of spinning.
_LISTENER_ERROR_BACKOFF_S = 0.5
# Between attempts to rearm the board after a capture (l3release, else a restart).
_REARM_RETRY_BACKOFF_S = 0.5
# The board's state in a ``stats`` reply: a capture running, the trigger latched.
_STATS_ACTIVE = re.compile(r"\bactive=(\d+)")
_STATS_LATCHED = re.compile(r"\blatched=(\d+)")
# The firmware's limit for "trackCfg impactFit" (L3_IMPACT_FIT_MAX_BAND_BINS).
TEE_BAND_MAX_BINS = 64.0


@dataclass(frozen=True)
class CaptureConfigSummary:
    """The few cfg fields the host needs outside the firmware."""

    chirp_tx_masks: tuple[str, ...]
    first_window_start: int | None
    first_window_bins: int | None
    chirp_period_s: float | None = None
    loops: int | None = None
    frame_period_s: float | None = None
    capture_format: str | None = None

    @property
    def n_tx(self) -> int:
        """Transmitters the chirp sequence enables, one chirp each per loop."""
        return len(set(self.chirp_tx_masks))

    @property
    def loop_period_s(self) -> float | None:
        """Same-TX chirp interval, or None when the cfg has no profile or chirps."""
        if self.chirp_period_s is None or not self.n_tx:
            return None
        return same_tx_loop_period_s(self.n_tx, self.chirp_period_s)


def read_capture_config(config_path: str | Path) -> CaptureConfigSummary:
    """Parse chirp TX masks, physical timing, storage format and the first saved window."""
    masks: list[str] = []
    window: tuple[int, int] | None = None
    chirp_period_s: float | None = None
    loops: int | None = None
    frame_period_s: float | None = None
    capture_format: str | None = None
    with Path(config_path).open(encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            fields = line.split()
            if line.startswith("chirpCfg"):
                masks.append(fields[-1])
            elif line.startswith("profileCfg"):
                # idleTime + rampEndTime, both in microseconds.
                chirp_period_s = (float(fields[3]) + float(fields[5])) * 1e-6
            elif line.startswith("frameCfg"):
                # numLoops, then framePeriodicity in milliseconds.
                loops = int(fields[3])
                frame_period_s = float(fields[5]) * 1e-3
            elif line.startswith("captureFormat"):
                capture_format = fields[1].lower()
            elif line.startswith("phaseCaptureCfg") and window is None:
                window = (int(fields[1]), int(fields[2]))
    return CaptureConfigSummary(
        chirp_tx_masks=tuple(masks),
        first_window_start=window[0] if window else None,
        first_window_bins=window[1] if window else None,
        chirp_period_s=chirp_period_s,
        loops=loops,
        frame_period_s=frame_period_s,
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


def tee_global_bin(tee_range_m: float, config_path: str | Path, fft_size: int = 128) -> int:
    """The tee's global range-FFT bin, checked to lie in the cfg's first window.

    ``tee_range_m`` is range from the antenna array. Raises when the tee falls
    outside the first capture window: the firmware would watch bins it never
    captures.
    """
    absolute = int(round(tee_range_m / (RANGE_SPAN_M / fft_size)))
    return check_first_window_bin(
        absolute, config_path, f"tee at {tee_range_m:.2f} m (bin {absolute})"
    )


def check_first_window_bin(global_bin: int, config_path: str | Path, what: str = "") -> int:
    """``global_bin`` when the cfg's first capture window holds it, else ValueError.

    A trigger aimed outside that window would watch bins it never captures.
    """
    summary = read_capture_config(config_path)
    if summary.first_window_start is None or summary.first_window_bins is None:
        raise ValueError(f"{config_path} has no phaseCaptureCfg")
    end = summary.first_window_start + summary.first_window_bins
    if not summary.first_window_start <= global_bin < end:
        raise ValueError(
            f"{what or f'bin {global_bin}'} is outside the first capture "
            f"window, bins {summary.first_window_start}-{end - 1}"
        )
    return global_bin


# Impact and ball windows start this many bins short of the tee, so the club's
# last approach and the ball leaving are both kept. l3_adaptive_cfg_defaults'
# marginBins: the firmware's lock-follow uses the same margin.
CAPTURE_MARGIN_BINS = 4
_PHASE_CAPTURE = "phaseCaptureCfg"
_ADAPTIVE_CAPTURE = "captureCfg adaptive"


def _window_start(start: int, width: int, fft_size: int) -> int:
    """A window start kept inside the FFT, as l3_adaptive_clip does."""
    return max(0, min(start, fft_size - width))


def tee_relative_config(lines: list[str], tee_bin: int, fft_size: int = 128) -> list[str]:
    """The cfg lines with the impact and ball windows placed on the tee.

    The shipped profiles hard-code those windows for one tee (bins 32/47 for
    1.57 m); a nearer tee falls short of the impact window and the capture
    keeps neither the club at impact nor the ball leaving. This mirrors
    l3_adaptive_windows for a ball on the tee: impact and post start
    CAPTURE_MARGIN_BINS short of it, late half a window further out. The pre
    window, widths, frame counts and stride stay as configured. Unless the
    cfg already sets it, ``captureCfg adaptive`` is added so a ball lock
    elsewhere moves the windows to the ball; its approach is the configured
    pre window's, so a lock on the tee changes nothing.

    Raises ValueError without exactly one well-formed phaseCaptureCfg, or
    when the tee is outside its pre window.
    """
    indices = [i for i, line in enumerate(lines) if line.strip().startswith(_PHASE_CAPTURE)]
    if len(indices) != 1:
        raise ValueError(f"expected one {_PHASE_CAPTURE} line, found {len(indices)}")
    index = indices[0]
    fields = lines[index].split()[1:]
    if len(fields) != 11:
        raise ValueError(f"{_PHASE_CAPTURE} needs 11 values, got {len(fields)}")
    values = [int(field) for field in fields]
    pre_start, pre_bins, impact_bins, post_bins = values[0], values[1], values[4], values[7]
    if not pre_start <= tee_bin < pre_start + pre_bins:
        raise ValueError(
            f"tee bin {tee_bin} is outside the pre-impact window, bins "
            f"{pre_start}-{pre_start + pre_bins - 1}"
        )
    near = tee_bin - CAPTURE_MARGIN_BINS
    post_start = _window_start(near, post_bins, fft_size)
    values[3] = _window_start(near, impact_bins, fft_size)
    values[6] = post_start
    values[8] = _window_start(post_start + post_bins // 2, post_bins, fft_size)
    rewritten = list(lines)
    rewritten[index] = " ".join([_PHASE_CAPTURE, *(str(value) for value in values)])
    if not any(line.strip().startswith(_ADAPTIVE_CAPTURE) for line in lines):
        rewritten.insert(
            index, f"{_ADAPTIVE_CAPTURE} 1 {tee_bin - pre_start} {CAPTURE_MARGIN_BINS}"
        )
    return rewritten


def _monotonic() -> float:
    """Clock for the startup sample. Tests replace this so startup does not sleep."""
    return time.monotonic()


def _pause(seconds: float) -> None:
    """Wait between background samples. Tests replace this so startup does not sleep."""
    time.sleep(seconds)


# Default RF profile for the kiosk and live IWR scripts: adaptive16 keeps the
# wide 53-bin IQ16 processing windows and retains a 141 ms movie (24/7/16).
DEFAULT_IWR6843_CONFIG = "config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg"
# A return short of the tee counts as a club target at this multiple of the
# firmware's running noise floor. The board's triggerLog trace shows the snr
# real swings and idle frames reach; tune from that.
SELF_TRIGGER_DEFAULT_SNR = FIRMWARE_TRIGGER_DEFAULT_SNR
# The global bin the trigger watches without --iwr6843-self-trigger-bin.
SELF_TRIGGER_DEFAULT_BIN = FIRMWARE_TRIGGER_DEFAULT_BIN
# Bins short of the ball the trigger (and the ball search it arms) is aimed at
# by default. The watch region has to reach just short of where the club
# reaches the ball, and the ball tracker only accepts a return within its origin gate of the arm
# bin: on 38 labelled swings the ball was best tracked 2 bins short of its rest
# bin (470/552 ball points) and lost outright from 4-5 bins short.
SELF_TRIGGER_TEE_LEAD_BINS = 2


def self_trigger_bin(tee_from_front_m: float, config_path: str | Path, fft_size: int = 128) -> int:
    """``triggerCfg`` bin for a tee tape from the enclosure front.

    Adds the array depth, then aims ``SELF_TRIGGER_TEE_LEAD_BINS`` short of the
    ball — the same default the kiosk uses without ``--iwr6843-self-trigger-bin``.
    """
    from openflight.iwr6843.calibration import antenna_range_m

    ball = tee_global_bin(antenna_range_m(tee_from_front_m), config_path, fft_size)
    watched = ball - SELF_TRIGGER_TEE_LEAD_BINS
    return check_first_window_bin(watched, config_path, f"self-trigger bin {watched}")


@dataclass(frozen=True)
class SelfTriggerConfig:
    """Firmware ``triggerCfg``: freeze when the club track reaches the tee.

    ``triggerCfg`` turns the self-trigger on around ``tee_bin``: per frame the
    firmware extracts club targets of at least ``snr`` times its running noise
    floor from the bins short of it, the club track follows the club through
    them, and its range-only impact freezes the capture when the club's line
    is predicted to cross the tee's range. (The range gate that fired from its
    own short track was removed on 2026-09-30.) The watch depth keeps its
    firmware default; ``triggerLog track`` on the board reports the track.
    """

    tee_bin: int  # global range-FFT bin (tee_global_bin), not a window offset
    snr: float

    def __post_init__(self) -> None:
        if self.tee_bin < 0:
            raise ValueError(f"self-trigger bin must be >= 0, got {self.tee_bin}")
        if not math.isfinite(self.snr) or self.snr < 1.0:
            # Below the floor itself every frame would be a candidate.
            raise ValueError(f"self-trigger snr must be >= 1, got {self.snr}")

    @property
    def command(self) -> str:
        """CLI line that arms this trigger: ``<bin> <snr> <on>``."""
        return f"triggerCfg {self.tee_bin} {self.snr} 1"


# on=0 disables the firmware trigger (see l3_cli_triggerCfg).
SELF_TRIGGER_OFF_COMMAND = "triggerCfg 0 0 0"

# The self-trigger's bins are scored on the DSS (``trackCfg detectCore dss``,
# the firmware's boot default): ~6.5x faster than the MSS (2026-10-01 rig:
# 570 us mean, 1383 us max a frame at 2 ms, against ~2.6 ms on the MSS at
# 3 ms). The DSS only reads a plain IQ16 ring; on these formats the firmware
# counts every frame ineligible and the MSS scores it.
MSS_SCORED_CAPTURE_FORMATS = ("iq8", "compact16", "adaptive16")
DETECT_CORE = "dss"


def measure_trigger_level(
    radar: IWR6843Radar,
    tee_bin: int,
    *,
    snr: float = SELF_TRIGGER_DEFAULT_SNR,
    clock: Callable[[], float] | None = None,
    pause: Callable[[float], None] | None = None,
) -> tuple[float, float]:
    """Arm at ``snr`` over the firmware's floor and return ``(p95 floor, threshold)``.

    The firmware owns the noise floor: once armed its ``stats`` report it as
    ``tee=`` (in the detection statistic's units), so this samples that for
    two seconds and reports the p95 with the ``floor x snr`` threshold the
    detector applies. The arm is the real one, so the detector is left armed.
    The lane must stay empty: a latch during the sample is a failed startup,
    not a floor.
    """
    now = _monotonic if clock is None else clock
    wait = _pause if pause is None else pause
    probe = SelfTriggerConfig(tee_bin=tee_bin, snr=snr).command
    reply = radar.cmd(probe, 2.0)
    if "Error" in reply or "Done" not in reply:
        raise RuntimeError(f"IWR6843 background probe rejected: {reply.strip()}")
    samples: list[float] = []
    deadline = now() + FLOOR_SAMPLE_S
    while now() < deadline:
        health = radar.stats()
        if "latched=1" in health:
            raise RuntimeError(
                "background sample latched the trigger; keep the lane empty and retry"
            )
        tee = tee_power_from_stats(health)
        if tee is not None and tee > 0.0:
            samples.append(tee)
        wait(FLOOR_PAUSE_S)
    try:
        floor, _margin_level = level_above_floor(samples)
    except ValueError as exc:
        raise RuntimeError(f"IWR6843 background sample failed: {exc}") from exc
    return floor, floor * snr


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
    # The firmware's own shot result (shot_result.ShotResultPacket), read before
    # the readback rearms the ring; None on firmware without it or without a
    # RESULT this shot.
    onboard_result: object | None = None

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
        tee_range_m: float | None = None,
        tee_band_bins: float = TEE_BAND_DEFAULT_BINS,
        ball_snr: float | None = None,
        board_calibration: BoardCalibration | None = None,
    ):
        # "not <=" also refuses NaN.
        if not 0.0 <= tee_band_bins <= TEE_BAND_MAX_BINS:
            raise ValueError(
                f"tee band must be 0..{TEE_BAND_MAX_BINS:g} bins (0 = off), got {tee_band_bins}"
            )
        # Width in range bins of the band near the ball that the firmware's
        # club and ball trackers ignore (placed by the firmware on the
        # noisiest idle bins near the tee); 0 turns it off.
        self.tee_band_bins = float(tee_band_bins)
        # The ball tracker's extraction snr, apart from the trigger's; None
        # sends 0, the firmware's own default.
        self.ball_snr = None if ball_snr is None else check_ball_snr(ball_snr)
        # Sent to the board at every start (identity when None); False until
        # the firmware has acknowledged it.
        self.board_calibration = board_calibration or BoardCalibration.identity()
        self.calibration_applied = False
        self.config_path = Path(config_path)
        # What the board reported after ``trackCfg detectCore dss`` at the
        # last (re)start; None without a self-trigger or on older firmware.
        self.detect_core: str | None = None
        # With the tee known, the impact and ball windows are placed on it
        # (tee_relative_config) instead of the cfg's fixed ones.
        self.tee_range_m = tee_range_m
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
        self._trigger_notice = b""
        # A frozen ring nobody will read. Retried until the release succeeds.
        self._release_pending = False
        # After every capture the board is left armed (_ensure_armed); edges
        # while that runs are refused as busy. A restart repeats start()'s
        # configuration (kept here) and then runs the restart hooks.
        self._rearming = False
        self._sensor_configured = False
        self._config_lines: list[str] | None = None
        self._onboard_track_config: str | None = None
        self._restart_hooks: list[Callable[[IWR6843Radar], None]] = []
        # Read once at start; None when the image predates ``stats version``.
        self.firmware_version: FirmwareVersion | None = None

    def _tee_relative_config_lines(self) -> list[str] | None:
        """The cfg with its windows on the tee, or None to send the file as it is."""
        if self.tee_range_m is None:
            return None
        tee_bin = tee_global_bin(self.tee_range_m, self.config_path)
        lines = tee_relative_config(
            self.config_path.read_text(encoding="utf-8").splitlines(), tee_bin
        )
        logger.info(
            "[IWR6843] Capture windows on the tee (bin %d): %s",
            tee_bin,
            next(line for line in lines if line.startswith(_PHASE_CAPTURE)),
        )
        return lines

    @property
    def watch_self_trigger(self) -> bool:
        """True when the firmware trigger replaces the GPIO edge."""
        return self.self_trigger is not None

    @property
    def port(self) -> str:
        """Connected TI serial port."""
        return self.radar.port

    def _log_firmware_version(self) -> None:
        """Record the flashed image; a failed query never blocks startup."""
        try:
            self.firmware_version = self.radar.firmware_version()
        except RuntimeError as exc:
            logger.warning("[IWR6843] Could not read firmware version: %s", exc)
            return
        if self.firmware_version is None:
            logger.warning(
                "[IWR6843] Flashed firmware predates stats version; "
                "reflash a versioned release from firmware/releases/"
            )
        else:
            logger.info("[IWR6843] Firmware %s", self.firmware_version)

    def start(self, *, armed: bool = True, onboard_track_config: str | None = None) -> None:
        """Configure the radar and GPIO, optionally arming trigger capture.

        ``onboard_track_config`` is the ``trackCfg`` line for the firmware
        tracker. It is sent here, before the worker thread owns the port,
        because a command from another thread would race the self-trigger
        listener's reads.
        """
        if self._running:
            return
        if not self.config_path.is_file():
            raise FileNotFoundError(f"IWR6843 config not found: {self.config_path}")
        config_lines = self._tee_relative_config_lines()
        if self.watch_self_trigger:
            # The detect path reads IQ8 rings too (int8 times the frame scale)
            # since the ring readers took a component width; older firmware
            # would score garbage, so say which format is in use.
            capture_format = read_capture_config(self.config_path).capture_format
            if capture_format == "iq8":
                logger.info("[IWR6843] Self-trigger on an IQ8 ring: needs firmware with IQ8 detect")
            elif capture_format in ("compact16", "adaptive16"):
                logger.info(
                    "[IWR6843] %s capture: the detect path reads the IQ16 scratch and L3 keeps "
                    "the retained windows; needs firmware with the compact formats",
                    capture_format,
                )
            if capture_format in MSS_SCORED_CAPTURE_FORMATS:
                logger.warning(
                    "[IWR6843] %s capture: the DSS cannot read this ring, so the MSS scores "
                    "the self-trigger's bins (~6.5x slower; it overran a 3 ms frame and "
                    "cannot keep up at 2 ms). Use a captureFormat iq16 profile for the DSS",
                    capture_format,
                )
        if self.save_dumps:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        self._log_firmware_version()
        self._config_lines = config_lines
        self._onboard_track_config = onboard_track_config
        self._sensor_configured = False
        try:
            # Before the worker starts: after that only the worker may talk
            # to the radar.
            self._configure_radar()

            # The self-trigger listens for the firmware line; the pin stays
            # free so a stray sound-gate edge cannot start a second capture.
            if not self.watch_self_trigger:
                self._button = self._open_button()
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
            if self._sensor_configured:
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

    def _configure_radar(self) -> None:
        """The radar as start() leaves it: the cfg, the tee band, the ball snr,
        the board calibration, the onboard tracker and the self-trigger.

        Also how _ensure_armed restarts a board a failed capture left stopped,
        so the two cannot drift apart. Raises on failure; _sensor_configured
        says whether the cfg got as far as starting the sensor (which then
        needs stopping).
        """
        self.radar.send_config(str(self.config_path), lines=self._config_lines)
        self._sensor_configured = True
        # Always sent, 0 included: the firmware keeps the band across
        # sensorStart, so a restart without the flag must clear it.
        if not self.radar.set_tee_band(self.tee_band_bins):
            logger.info(
                "[IWR6843] Firmware has no tee band (trackCfg impactFit); "
                "nothing to clear with the band off"
            )
        # Always sent, the default (0) included, for the same reason.
        if not self.radar.set_ball_snr(0.0 if self.ball_snr is None else self.ball_snr):
            logger.info(
                "[IWR6843] Firmware has no ball snr setting (trackCfg ballSnr); "
                "it uses its own default"
            )
        # Always sent, identity included: the firmware keeps it across
        # sensorStart. Sent before triggerCfg, which copies it into the tracks.
        board = self.board_calibration
        applied = self.radar.set_radar_cal(board.cal_args)
        applied = self.radar.set_elements(board.elem_phase_rad, board.elem_gain) and applied
        # Firmware without the sub-modes already runs the identity, so an
        # identity refusal loses nothing.
        self.calibration_applied = applied or board.is_identity
        if not self.calibration_applied:
            logger.warning(
                "[IWR6843] Firmware has no trackCfg cal/elem, so the calibration was not "
                "applied: onboard launch angles, club path and angle of attack are "
                "uncalibrated and will not be used this session"
            )
        if self._onboard_track_config is not None:
            self._configure_onboard_tracking(self._onboard_track_config)
        # Before triggerCfg: an armed detector latched on the MSS cannot keep
        # up at 2 ms, starves the CLI, and the board stops answering.
        self._apply_detect_core()
        self._apply_self_trigger()

    def add_restart_hook(self, hook: Callable[[IWR6843Radar], None]) -> None:
        """Run ``hook(radar)`` on the worker after a restart that rearmed the
        board: what was set after start (the ball detector's ``ball cfg``) is
        set again. A failing hook is logged and does not undo the rearm."""
        self._restart_hooks.append(hook)

    def _board_armed(self) -> bool:
        """A capture running and, with the self-trigger, not latched.

        Unreadable stats count as not armed. Firmware whose stats carry no
        ``active=`` field cannot say, and is left alone.
        """
        try:
            text = self.radar.stats()
        except Exception as exc:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] Board state unreadable after the capture: %s", exc)
            return False
        active = _STATS_ACTIVE.search(text)
        if active is None:
            return True
        latched = _STATS_LATCHED.search(text)
        frozen = self.watch_self_trigger and latched is not None and latched.group(1) == "1"
        return active.group(1) == "1" and not frozen

    def _restart_radar(self) -> None:
        """start()'s configuration again, then the restart hooks."""
        self._configure_radar()
        for hook in self._restart_hooks:
            try:
                hook(self.radar)
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Restart hook failed", exc_info=True)
        logger.warning("[IWR6843] Radar restarted to rearm it after a capture")

    def _rearm(self) -> None:
        """l3release (it stops at a frame boundary and rearms), else a restart.

        The firmware refuses l3release with nothing frozen and nothing running
        (l3_awaitFrozenRing), which a readback that stopped the capture and
        then failed leaves behind; only a restart runs that board again.
        Raises when the restart fails.
        """
        try:
            self.radar.release_sparse_freeze()
        except Exception as exc:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] l3release refused (%s): restarting the radar", exc)
        else:
            if self._board_armed():
                logger.info("[IWR6843] Rearmed after the capture (l3release)")
                return
            logger.warning("[IWR6843] Still not armed after l3release: restarting the radar")
        self._restart_radar()

    def _ensure_armed(self) -> None:
        """Leave the board armed after a capture, whatever its readback did.

        2026-09-30: an l3dump that answered 18 bytes left the board frozen and
        nothing rearmed it, so no later swing fired. Retried until the board
        runs or the monitor stops; edges meanwhile are refused as busy.
        """
        if self._board_armed():
            return
        with self._condition:
            self._rearming = True
        try:
            while self._running:
                try:
                    self._rearm()
                    return
                except Exception:  # pylint: disable=broad-exception-caught
                    logger.warning("[IWR6843] Rearm failed; retrying", exc_info=True)
                    time.sleep(_REARM_RETRY_BACKOFF_S)
        finally:
            with self._condition:
                self._rearming = False

    def _open_button(self):
        """The trigger-pin input, from the injected factory or gpiozero."""
        button_factory = self._button_factory
        if button_factory is None:
            # Must precede the first gpiozero device: on a Pi 5 gpiozero's
            # own auto-detection fails outright. See gpio_factory.
            ensure_lgpio_pin_factory()

            from gpiozero import Button  # pylint: disable=import-error,import-outside-toplevel

            button_factory = Button
        # No gpiozero debounce: lgpio delays delivery by the debounce interval,
        # which previously cost the first 50 ms of ball flight.
        return button_factory(self.gpio_pin, pull_up=False, bounce_time=None)

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

    def _apply_detect_core(self) -> None:
        """Send ``trackCfg detectCore dss`` when the self-trigger is used.

        dss is the firmware's boot default, but three DSS failures in a row
        latch the MSS for every frame until dss is chosen again, and the
        firmware keeps that across sensorStart: sent at every (re)start so an
        armed board never begins on a latched MSS. Firmware without the
        command (it scores on the MSS) is left as it is.
        """
        self.detect_core = None
        if self.self_trigger is None:
            return
        try:
            status = self.radar.detect_core(DETECT_CORE)
        except DspLinkError as error:
            logger.warning(
                "[IWR6843] Firmware did not take trackCfg detectCore %s (%s); the "
                "self-trigger's bins are scored where this image scores them",
                DETECT_CORE,
                error,
            )
            return
        if status.requested != DETECT_CORE:
            raise RuntimeError(
                f"IWR6843 detect core {DETECT_CORE} not taken: requested={status.requested}"
            )
        self.detect_core = status.requested
        logger.info(
            "[IWR6843] Self-trigger detect core: %s (active=%s)", status.requested, status.active
        )

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
                or self._rearming
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
            if not found or self.notify_trigger():
                return
            # Disarmed, busy or a duplicate: the firmware froze its ring and
            # waits for l3sparse, but no capture will ask for it.
            logger.info("[IWR6843] Releasing an unaccepted self-trigger capture")
            self._release_pending = True
        # Raises on failure; the caller backs off and this retries next pass.
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

    def _read_onboard_result(self):
        """The firmware's result packet for this shot, or None.

        Read before the readback because l3track/l3sparse rearm the ring at
        the end, which resets the result. A failure here never costs the
        capture: it is logged and the shot goes on without onboard metrics.
        """
        try:
            result = self.radar.shot_result()
        except Exception as exc:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] Onboard result unreadable: %s", exc)
            return None
        if result is not None and not self.calibration_applied:
            result = result.with_onboard_angles_doubted()
        if result is not None:
            logger.info(
                "[IWR6843] Onboard result: shot %d %s, %d club / %d ball points",
                result.shot_id,
                result.verdict,
                result.club_points,
                result.ball_points,
            )
        return result

    def _read_capture(self) -> tuple[bytes, float | None, OnboardTrack | None]:
        """Read one frozen capture, preferring the least serial traffic.

        Firmware-tracked cells (``l3track``), then host-planned cells
        (``l3sparse``), then the full ring (``l3dump``). Each step falls back
        only when the firmware refused before streaming.
        """
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
        onboard_result = self._read_onboard_result() if self.watch_self_trigger else None
        try:
            logger.info("[IWR6843] Trigger #%d: reading track samples", sequence)
            raw, noise_power, onboard_track = self._read_capture()
            metadata = self._validate_dump(raw)
            if self.save_dumps:
                path = self._capture_path(sequence, edge_timestamp)
                path.write_bytes(raw)
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
            onboard_result=onboard_result,
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
        # Always, success or not: a readback that failed can leave the ring
        # frozen or the capture stopped, and then nothing fires again.
        self._ensure_armed()

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
    "SELF_TRIGGER_DEFAULT_BIN",
    "SELF_TRIGGER_TEE_LEAD_BINS",
    "DEFAULT_IWR6843_CONFIG",
    "DETECT_CORE",
    "MSS_SCORED_CAPTURE_FORMATS",
    "SELF_TRIGGER_DEFAULT_SNR",
    "TEE_BAND_DEFAULT_BINS",
    "TEE_BAND_MAX_BINS",
    "SELF_TRIGGER_OFF_COMMAND",
    "CaptureConfigSummary",
    "IWR6843Capture",
    "IWR6843CaptureMonitor",
    "SelfTriggerConfig",
    "check_first_window_bin",
    "measure_trigger_level",
    "read_capture_config",
    "self_trigger_bin",
    "tee_global_bin",
    "tx_order_from_config",
]
