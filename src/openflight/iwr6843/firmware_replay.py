"""Replay recorded IWR6843 captures through the firmware's own trigger and club track.

A ``.l3dump`` holds the range-FFT ring the firmware froze around a swing:
every pre-trigger frame the self-trigger scored, with the same bins, loops
and ordering. This module reduces each frame to the per-bin observations
``l3_dump.c`` computes on the R4F (``l3_verticalResidual``: burst-MTI
residual energy, strongest loop, loop 0 and the lag-1 loop autocorrelation,
over the vertical TX pair and every RX) and feeds them, frame by frame, to
the compiled C observation layer, self-trigger and club track. What comes
back is what the board would have decided and logged for that capture, so a
change to the C can be judged against every recorded swing before it is
flashed.

Bins are GLOBAL range-FFT bins throughout, as in the firmware. The residual
port accumulates in float64 where the firmware uses float32; the observations
agree to float32 rounding, which no threshold in the detector resolves.
"""

from __future__ import annotations

import ctypes
import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.dump import is_range_snapshot, parse_dump
from openflight.iwr6843.tracking import RANGE_SPAN_M, same_tx_loop_period_s

# The firmware's defaults for the CLI-configured trigger parameters
# (monitor.SelfTriggerConfig); the C owns the rest.
DEFAULT_SNR = 6.0
DEFAULT_TRACK_FRAMES = 2
DEFAULT_FFT_SIZE = 128
# The lag-1 Doppler readout aliases at wavelength / (4 T); at 135 us that
# is about +/- 9 m/s, so a clubhead reads as a speed uniformly over the span.
FLOOR_SHIFT = 2  # L3_TRIG_FLOOR_SHIFT

_OBS_DTYPE = np.dtype(
    [("energy", "<f4"), ("peak", "<f4"), ("loop0", "<f4"), ("r1Re", "<f4"), ("r1Im", "<f4")]
)


def vertical_tx_indices(n_tx: int) -> tuple[int, ...]:
    """The transmitters ``l3_verticalResidual`` sums: all of them, less the
    azimuth element (TX1) of a three-TX loop."""
    if n_tx < 1:
        raise ValueError(f"a loop needs at least one transmitter, got {n_tx}")
    return tuple(tx for tx in range(n_tx) if not (n_tx == 3 and tx == 1))


def range_windowed(frame_bins: np.ndarray, window: str, valid_bins: int) -> np.ndarray:
    """One frame's ``[..., bins]`` through the firmware's range window.

    ``"hann"`` is the periodic Hann window in time, applied exactly as the
    3-tap kernel ``X[k] - (X[k-1] + X[k+1]) / 2`` on the 128-point range FFT
    (``l3_iq16_stats.h``). Bins on the edge of the frame's ``valid_bins`` have
    one neighbour and stay unwindowed, as on the board; bins past it are
    padding and are returned as they are.
    """
    if window not in fw.RANGE_WINDOW_NAMES:
        raise ValueError(f"window must be one of {sorted(fw.RANGE_WINDOW_NAMES)}, got {window!r}")
    if window == "none" or valid_bins < 3:
        return frame_bins
    out = frame_bins.astype(np.complex128)
    source = frame_bins[..., :valid_bins].astype(np.complex128)
    out[..., 1 : valid_bins - 1] = source[..., 1:-1] - 0.5 * (source[..., :-2] + source[..., 2:])
    return out


def bin_observation_table(  # pylint: disable=too-many-arguments
    cube: np.ndarray,
    frame: int,
    first_local: int,
    count: int,
    n_tx: int,
    *,
    window: str = "none",
    valid_bins: int | None = None,
) -> np.ndarray:
    """``l3_verticalResidual`` for ``count`` bins from local bin ``first_local``.

    ``cube`` is a parsed dump, ``[frames, chirps, rx, bins]`` with chirp c
    from transmitter ``c % n_tx`` of loop ``c // n_tx``. ``window`` is the
    firmware's ``trackCfg window``; ``valid_bins`` is the frame's bin count
    (the cube's width when None), which decides the unwindowed edge bins.
    Returns a structured array with fields energy, peak, loop0, r1Re and r1Im,
    one row per bin.
    """
    chirps = cube.shape[1]
    if chirps % n_tx:
        raise ValueError(f"{chirps} chirps per frame is not a whole number of {n_tx}-TX loops")
    loops = chirps // n_tx
    if count <= 0 or first_local < 0 or first_local + count > cube.shape[-1]:
        raise ValueError(f"bins {first_local}..{first_local + count - 1} outside the frame")
    width = cube.shape[-1] if valid_bins is None else valid_bins
    data = range_windowed(cube[frame], window, width)[:, :, first_local : first_local + count]
    data = data.reshape(loops, n_tx, cube.shape[2], count)[:, list(vertical_tx_indices(n_tx))]
    residual = data - data.mean(axis=0, keepdims=True)
    power = residual.real**2 + residual.imag**2
    loop_power = power.sum(axis=(1, 2))  # [loops, bins]
    lag1 = (residual[1:] * np.conj(residual[:-1])).sum(axis=(0, 1, 2)) if loops > 1 else 0.0
    table = np.zeros(count, dtype=_OBS_DTYPE)
    table["energy"] = loop_power.sum(axis=0)
    table["peak"] = loop_power.max(axis=0)
    table["loop0"] = loop_power[0]
    table["r1Re"] = np.real(lag1)
    table["r1Im"] = np.imag(lag1)
    return table


def bin_observations(  # pylint: disable=too-many-arguments
    cube: np.ndarray,
    frame: int,
    first_local: int,
    count: int,
    n_tx: int,
    *,
    window: str = "none",
    valid_bins: int | None = None,
) -> ctypes.Array:
    """:func:`bin_observation_table` as a ctypes array of ``count`` :class:`BinObs`,
    ready for ``l3_trig_update`` and ``l3_obs_extract``."""
    table = bin_observation_table(
        cube, frame, first_local, count, n_tx, window=window, valid_bins=valid_bins
    )
    return (fw.BinObs * count).from_buffer_copy(table.tobytes())


def channel_snapshot(
    cube: np.ndarray,
    frame: int,
    local_bin: int,
    n_tx: int,
    *,
    lag1_phase_rad: float,
    radial_velocity_mps: float,
    chirp_period_s: float,
) -> fw.AngleSnapshot:
    """``l3_channelSnapshot``: every (tx, rx) channel at one bin, its burst-MTI
    residual summed over the loops with the target's per-loop Doppler phase
    unwound, ready for ``l3_angle_estimate``."""
    chirps, n_rx = cube.shape[1], cube.shape[2]
    loops = chirps // n_tx
    data = cube[frame, :, :, local_bin].reshape(loops, n_tx, n_rx)
    residual = data - data.mean(axis=0, keepdims=True)
    rotor = np.exp(-1j * lag1_phase_rad * np.arange(loops))
    summed = (residual * rotor[:, None, None]).sum(axis=0)  # [tx, rx]
    snap = fw.AngleSnapshot()
    fw_lib = _default_library()
    fw_lib.l3_angle_snapshot_init(ctypes.byref(snap), n_tx, n_rx)
    for tx in range(snap.ntx):
        for rx in range(snap.nrx):
            value = summed[tx, rx]
            snap.channel[tx * snap.nrx + rx] = fw.Cpx(float(value.real), float(value.imag))
    snap.lag1PhaseRad = lag1_phase_rad
    snap.radialVelocityMps = radial_velocity_mps
    snap.chirpPeriodS = chirp_period_s
    return snap


def static_channel_snapshot(
    cube: np.ndarray, frame: int, local_bin: int, n_tx: int, *, chirp_period_s: float
) -> fw.AngleSnapshot:
    """``l3_channelSnapshotStatic``: the raw samples of every channel at one bin
    summed over the loops, for a stationary target such as the ball on its tee."""
    chirps, n_rx = cube.shape[1], cube.shape[2]
    loops = chirps // n_tx
    summed = cube[frame, :, :, local_bin].reshape(loops, n_tx, n_rx).sum(axis=0)
    snap = fw.AngleSnapshot()
    _default_library().l3_angle_snapshot_init(ctypes.byref(snap), n_tx, n_rx)
    for tx in range(snap.ntx):
        for rx in range(snap.nrx):
            value = summed[tx, rx]
            snap.channel[tx * snap.nrx + rx] = fw.Cpx(float(value.real), float(value.imag))
    snap.lag1PhaseRad = 0.0
    snap.radialVelocityMps = 0.0
    snap.chirpPeriodS = chirp_period_s
    return snap


BALL_ANGLE_MIN_PEAK_RATIO = 3.0  # L3_BALL_ANGLE_MIN_PEAK_RATIO

_LIBRARY: ctypes.CDLL | None = None


def _default_library() -> ctypes.CDLL:
    """The compiled firmware modules, built once per process."""
    global _LIBRARY  # pylint: disable=global-statement
    if _LIBRARY is None:
        _LIBRARY = fw.build_firmware_library()
    return _LIBRARY


def frame_window(meta: dict, frame: int) -> tuple[int, int]:
    """(global bin of local bin 0, valid bins) of one frame of a parsed dump."""
    starts = meta.get("range_bin_starts")
    counts = meta.get("range_bin_counts")
    start = starts[frame] if starts else meta.get("range_bin_start", 0)
    count = counts[frame] if counts else meta["n_samples"]
    return int(start), int(count)


# Frame period assumed for dumps whose header predates the field (version < 3):
# shot.DEFAULT_FRAME_PERIOD_S, the 12 ms frames that firmware ran.
FALLBACK_FRAME_PERIOD_US = 12000


def frame_timestamps_us(meta: dict) -> tuple[int, ...]:
    """Microseconds of each frame from the oldest retained one, as the firmware counts them.

    Timed formats carry per-frame offsets; older ones a frame period; the
    oldest neither, and get the 12 ms fallback so the fits have a time axis.
    """
    offsets = meta.get("frame_time_offsets_us")
    if offsets:
        return tuple(int(offset) for offset in offsets)
    period = int(meta.get("frame_period_us", 0)) or FALLBACK_FRAME_PERIOD_US
    return tuple(frame * period for frame in range(meta["n_frames"]))


@dataclass(frozen=True)
class RetainReplay:
    """The adaptive16 retention plan to mirror: slot widths per phase and the policy."""

    pre_bins: int = 16
    impact_bins: int = 24
    post_bins: int = 16
    impact_frames: int = 7  # first post frames stored at impact_bins
    enabled: bool = True  # False: compact16's centred windows
    approach_bins: int | None = None  # l3_retain_cfg_t overrides; None keeps the defaults
    approach_margin_bins: int | None = None
    impact_bias_bins: int | None = None
    ball_search_lead_bins: int | None = None
    ball_follow_lead_bins: int | None = None
    spin_frames: int | None = None


@dataclass(frozen=True)
class RetainSummary:
    """The window one frame would have kept, and whether the frame's point fell inside it."""

    start: int
    bins: int
    priority: str
    why: str
    point_bin: float | None  # the club or ball point appended this frame
    covered: bool | None  # None when no point was appended

    @property
    def end(self) -> int:
        return self.start + self.bins


@dataclass(frozen=True)
class BallTuning:
    """``l3_ball_track_cfg_t`` overrides for one replay; None keeps the firmware default.

    The Pi detector's ball rules, as firmware switches to judge on captures:
    ``fast_ball_mps`` / ``fast_support_fraction`` are its fastest-credible
    selection among the hypotheses, ``min_departure_mps`` its hard speed floor
    (set on both the legacy acquisition and the hypotheses) and
    ``far_window_bins`` its separate far range window.
    """

    fast_ball_mps: float | None = None
    fast_support_fraction: float | None = None
    min_departure_mps: float | None = None
    far_window_bins: float | None = None

    def apply(self, cfg: fw.BallTrackCfg) -> None:
        """Write the set overrides into a ball-track configuration."""
        if self.fast_ball_mps is not None:
            cfg.hyps.fastBallMps = self.fast_ball_mps
        if self.fast_support_fraction is not None:
            cfg.hyps.fastSupportFraction = self.fast_support_fraction
        if self.min_departure_mps is not None:
            cfg.minDepartureMps = self.min_departure_mps
            cfg.hyps.minDepartureMps = self.min_departure_mps
        if self.far_window_bins is not None:
            cfg.hyps.farWindowBins = self.far_window_bins


@dataclass(frozen=True)
class ReplayConfig:
    """What ``triggerCfg`` and the capture profile would have told the board."""

    tee_bin: int  # global; the destination without a locked ball
    snr: float = DEFAULT_SNR
    track_frames: int = DEFAULT_TRACK_FRAMES
    stat: str = "peak"  # "peak" or "energy"
    subbin: str = "parabolic"  # how targets read their sub-bin range: "parabolic" or "centroid"
    range_window: str = "none"  # "trackCfg window": "none" or "hann"
    dest_bin: int | None = None  # a locked ball's global bin; None uses the tee
    loop_period_s: float | None = None  # None: n_tx x the shipped chirp period
    fft_size: int = DEFAULT_FFT_SIZE
    stop_at_fire: bool = False  # True: ignore frames after the trigger fires, as the board does
    # Radar calibration: "trackCfg cal" values in the firmware's units.
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    roll_deg: float = 0.0
    azimuth_offset_rad: float = 0.0
    elevation_offset_deg: float = 0.0
    range_bias_m: float = 0.0
    # Geometric impact detector: armed lets it end the replay like the gate.
    impact_armed: bool = False
    # Frames after the trigger fires go to the ball tracker, as the board's
    # post movie does; a locked ball at dest_bin makes the shot require one.
    post_impact: bool = True
    # With a locked ball (dest_bin), read its direction from the static return
    # in the first frame, as the firmware does from the ball detector's lock,
    # so the destination is a 3D position rather than a point on boresight.
    ball_angles: bool = True
    # Treat this frame as the first post-impact frame whatever the gate does:
    # for captures the sound trigger froze, the plan's first post slot IS
    # impact, so the ball tracker can be judged on its own. None: the gate.
    post_from_frame: int | None = None
    # Retention mirror (l3_retain.c): the IQ16 bins each frame would have
    # kept in an adaptive16 capture, judged against the points the trackers
    # appended. None replays without it.
    retain: RetainReplay | None = None
    # The ball search: True/False sets l3_ball_track_cfg_t.useHypotheses for
    # this replay; None keeps the firmware default.
    ball_hypotheses: bool | None = None
    # The Pi detector's rules as ball-track overrides; None keeps the defaults.
    ball_tuning: BallTuning | None = None
    # Run the joint club/ball path search (l3_joint_search) in parallel with
    # the legacy ball tracker, host-only.  Results are included in the
    # ReplayResult as joint_ball_points and joint_club_points.
    joint_search: bool = False

    @property
    def destination(self) -> int:
        """The global bin the club is judged against: the locked ball, else the tee."""
        return self.tee_bin if self.dest_bin is None else self.dest_bin


@dataclass(frozen=True)
class TargetSummary:
    """One extracted target, copied out of the C structure."""

    range_bin: float
    peak_bin: int
    snr: float
    coherence: float
    doppler_mps: float
    confidence: float


@dataclass(frozen=True)
class AngleSummary:
    azimuth_deg: float | None
    elevation_deg: float | None
    azimuth_coherence: float
    elevation_peak_ratio: float
    confidence: float = 0.0  # l3_angle_confidence: 0..1


@dataclass(frozen=True)
class DeliverySummary:
    """``l3_delivery_t`` copied out: the club's velocity vector and its metrics."""

    points: int
    speed_mps: float
    radial_speed_mps: float
    path_deg: float | None
    attack_deg: float | None
    residual_m: float
    confidence: float
    velocity: tuple[float, float, float]


@dataclass(frozen=True)
class LaunchSummary:
    """``l3_launch_t`` copied out: ball speed and launch angles at impact."""

    points: int
    speed_mps: float
    radial_speed_mps: float
    hla_deg: float | None
    vla_deg: float | None
    residual_m: float
    confidence: float
    velocity: tuple[float, float, float]


@dataclass(frozen=True)
class ReplayFrame:
    """What one frame did to the trigger, the track and the impact detector."""

    frame: int
    timestamp_us: int
    first_bin: int  # global bin of the first scored observation
    count: int  # observations scored; 0 when the window missed the tee
    floor: float  # the trigger's floor after this frame
    trig_state: str
    fired: bool
    targets: tuple[TargetSummary, ...]
    track_why: str
    track_bin: float | None  # the point appended this frame, global sub-bin
    angle: AngleSummary | None = None  # for the point appended this frame
    delivery: DeliverySummary | None = None
    impact_why: str = "none"
    shot_state: str = "waiting_for_ball"
    ball_why: str = "none"  # the ball tracker's verdict on a post-impact frame
    ball_bin: float | None = None  # the ball point appended this frame
    retain: RetainSummary | None = None  # the retention mirror's window for this frame
    ball_hypotheses: tuple[HypothesisSummary, ...] = ()  # active after this frame


@dataclass(frozen=True)
class HypothesisSummary:
    """One ball hypothesis after a frame: its id and (frame, global bin) points."""

    id: int
    points: tuple[tuple[int, float], ...]


@dataclass(frozen=True)
class PointSummary:
    """One appended trajectory point, copied out of the C structure."""

    frame: int
    timestamp_us: int
    range_bin: float
    range_m: float
    doppler_mps: float
    confidence: float
    # l3_track_point_t.position: the point in the calibrated radar frame, metres.
    position: tuple[float, float, float] | None = None
    angles_valid: bool = False


@dataclass
class ReplayResult:
    """One capture through the firmware modules, with the whole trajectory kept."""

    config: ReplayConfig
    frames: list[ReplayFrame]
    points: list[PointSummary]  # every appended point, beyond the C ring's depth
    fired_frame: int | None  # the range gate
    geometric_frame: int | None  # the geometric impact detector's fire
    impact_timestamp_us: int | None  # interpolated impact time from the geometry
    delivery: DeliverySummary | None  # at the end of the replay
    impact_status: str  # l3_impact_format at the end of the replay
    launch: LaunchSummary | None  # from the ball tracker, when a flight was confirmed
    ball_points: list[PointSummary]
    ball_angle: AngleSummary | None  # the locked ball's measured direction, when trusted
    shot_status: str  # l3_shot_format at the end
    ball_status: str  # l3_ball_track_format_status at the end
    track_counters: dict[str, int]
    trig_counters: dict[str, int]
    speed_mps: float
    fit_slope_bins_per_s: float
    fit_residual_bins: float
    status: str  # l3_track_format_status at the end of the replay
    trigger_summary: str  # l3_trig_format_summary at the end of the replay
    trig: fw.Trig = field(repr=False)
    track: fw.ClubTrack = field(repr=False)
    impact: fw.Impact = field(repr=False)
    shot: fw.Shot = field(repr=False)
    ball_track: fw.BallTrack = field(repr=False)
    # Joint search results (populated only when config.joint_search is True)
    joint_ball_points: list[PointSummary] = field(default_factory=list)
    joint_club_points: list[PointSummary] = field(default_factory=list)
    joint_counters: dict[str, int] = field(default_factory=dict)
    joint_confirmed: bool = False

    @property
    def retain_windows(self) -> list[RetainSummary]:
        return [frame.retain for frame in self.frames if frame.retain is not None]

    @property
    def retain_coverage(self) -> tuple[int, int]:
        """(points inside their frame's retained window, points appended) over the replay."""
        judged = [w for w in self.retain_windows if w.covered is not None]
        return sum(1 for w in judged if w.covered), len(judged)

    @property
    def retain_bins_saved(self) -> tuple[int, int]:
        """(bins retained, bins processed) over the frames the mirror judged."""
        kept = sum(w.bins for w in self.retain_windows)
        processed = sum(frame.count for frame in self.frames if frame.retain is not None)
        return kept, processed

    @property
    def acquisitions(self) -> int:
        """Times the track started from nothing; one per swing is the goal."""
        return self.track_counters["acquired"]

    @property
    def longest_run(self) -> int:
        """Most consecutive frames that each appended a point: the trajectory's continuity."""
        best = run = 0
        previous = None
        for point in self.points:
            run = run + 1 if previous is not None and point.frame == previous + 1 else 1
            best = max(best, run)
            previous = point.frame
        return best

    @property
    def approach_fraction(self) -> float:
        """Share of consecutive point pairs that moved toward the destination (rising bin)."""
        pairs = list(zip(self.points, self.points[1:], strict=False))
        if not pairs:
            return 0.0
        return sum(1 for a, b in pairs if b.range_bin > a.range_bin) / len(pairs)


_TRIG_COUNTERS = (
    "frames",
    "cand",
    "acq",
    "adv",
    "jump",
    "miss",
    "lost",
    "lowcoh",
    "slowdop",
    "young",
    "slow",
    "short",
    "fired",
)


def _target_summary(target: fw.TargetObs) -> TargetSummary:
    return TargetSummary(
        range_bin=float(target.rangeBin),
        peak_bin=int(target.peakBin),
        snr=float(target.snr),
        coherence=float(target.coherence),
        doppler_mps=float(target.dopplerAliasMps),
        confidence=float(target.confidence),
    )


def _angle_summary(obs: fw.AngleObs) -> AngleSummary:
    return AngleSummary(
        azimuth_deg=math.degrees(obs.azimuthRad) if obs.azimuthValid else None,
        elevation_deg=math.degrees(obs.elevationRad) if obs.elevationValid else None,
        azimuth_coherence=float(obs.azimuthCoherence),
        elevation_peak_ratio=float(obs.elevationPeakRatio),
        confidence=float(obs.confidence),
    )


def _delivery_summary(delivery: fw.Delivery) -> DeliverySummary | None:
    if not delivery.speedValid:
        return None
    return DeliverySummary(
        points=int(delivery.points),
        speed_mps=float(delivery.speedMps),
        radial_speed_mps=float(delivery.radialSpeedMps),
        path_deg=math.degrees(delivery.pathRad) if delivery.pathValid else None,
        attack_deg=math.degrees(delivery.attackRad) if delivery.attackValid else None,
        residual_m=float(delivery.residualM),
        confidence=float(delivery.confidence),
        velocity=(
            float(delivery.velocity.x),
            float(delivery.velocity.y),
            float(delivery.velocity.z),
        ),
    )


def _launch_summary(launch: fw.Launch) -> LaunchSummary | None:
    if not launch.speedValid:
        return None
    return LaunchSummary(
        points=int(launch.points),
        speed_mps=float(launch.speedMps),
        radial_speed_mps=float(launch.radialSpeedMps),
        hla_deg=math.degrees(launch.hlaRad) if launch.hlaValid else None,
        vla_deg=math.degrees(launch.vlaRad) if launch.vlaValid else None,
        residual_m=float(launch.residualM),
        confidence=float(launch.confidence),
        velocity=(float(launch.velocity.x), float(launch.velocity.y), float(launch.velocity.z)),
    )


def _estimate_angles(
    lib: ctypes.CDLL,
    cal: fw.RadarCal,
    cube: np.ndarray,
    frame: int,
    window_start: int,
    n_tx: int,
    hit: fw.TargetObs,
    radial_velocity_mps: float,
    chirp_period_s: float,
) -> tuple[fw.AngleObs | None, int]:
    """One target's angles as the board would estimate them; (obs, flags)."""
    snapshot = channel_snapshot(
        cube,
        frame,
        int(hit.peakBin) - window_start,
        n_tx,
        lag1_phase_rad=float(hit.dopplerPhaseRad),
        radial_velocity_mps=radial_velocity_mps,
        chirp_period_s=chirp_period_s,
    )
    obs = fw.AngleObs()
    if not lib.l3_angle_estimate(ctypes.byref(cal), ctypes.byref(snapshot), ctypes.byref(obs)):
        return None, 0
    flags = (fw.ANGLE_AZIMUTH if obs.azimuthValid else 0) | (
        fw.ANGLE_ELEVATION if obs.elevationValid else 0
    )
    return obs, flags


def _radar_cal(lib: ctypes.CDLL, config: ReplayConfig) -> fw.RadarCal:
    cal = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(cal), fw.CAL_MAX_VIRTUAL)
    cal.radarPitchRad = math.radians(config.pitch_deg)
    cal.radarYawRad = math.radians(config.yaw_deg)
    cal.radarRollRad = math.radians(config.roll_deg)
    cal.azimuthOffsetRad = config.azimuth_offset_rad
    cal.elevationOffsetRad = math.radians(config.elevation_offset_deg)
    cal.rangeBiasM = config.range_bias_m
    return cal


def _point_summary(point: fw.TrackPoint) -> PointSummary:
    return PointSummary(
        frame=int(point.frame),
        timestamp_us=int(point.timestampUs),
        range_bin=float(point.rangeBin),
        range_m=float(point.rangeM),
        doppler_mps=float(point.dopplerAliasMps),
        confidence=float(point.confidence),
        position=(float(point.position.x), float(point.position.y), float(point.position.z)),
        angles_valid=bool(point.anglesValid),
    )


def _retain_cfg(lib: ctypes.CDLL, retain: RetainReplay) -> fw.RetainCfg:
    cfg = fw.RetainCfg()
    lib.l3_retain_cfg_defaults(ctypes.byref(cfg))
    cfg.enabled = 1 if retain.enabled else 0
    for field_name, value in (
        ("approachBins", retain.approach_bins),
        ("approachMarginBins", retain.approach_margin_bins),
        ("impactBiasBins", retain.impact_bias_bins),
        ("ballSearchLeadBins", retain.ball_search_lead_bins),
        ("ballFollowLeadBins", retain.ball_follow_lead_bins),
        ("spinFrames", retain.spin_frames),
    ):
        if value is not None:
            setattr(cfg, field_name, value)
    if lib.l3_retain_cfg_check(ctypes.byref(cfg)) != 0:
        raise ValueError(f"the firmware rejects this retention configuration: {retain}")
    return cfg


def _retain_window(  # pylint: disable=too-many-arguments
    lib: ctypes.CDLL,
    cfg: fw.RetainCfg,
    *,
    shot: fw.Shot,
    locked: bool,
    destination: int,
    track: fw.ClubTrack,
    ball_track: fw.BallTrack,
    post: bool,
    post_index: int,
    process_start: int,
    process_bins: int,
    retain_bins: int,
) -> fw.RetainWindow:
    """``l3_retain_window`` for the coming frame from the trackers' predictions,
    as the firmware's rearm task would call it before the frame lands."""
    state = fw.RetainState()
    state.shotState = shot.state
    state.ballLocked = 1 if locked else 0
    state.ballBin = float(destination)
    state.clubActive = 1 if track.active else 0
    state.clubBin = lib.l3_retain_predict(track.lastBin, track.velocityBinsPerFrame)
    state.postFrame = 1 if post else 0
    state.postIndex = post_index
    state.ballTrackConfirmed = 1 if ball_track.confirmed else 0
    core = ball_track.core
    state.ballTrackBin = lib.l3_retain_predict(core.lastBin, core.velocityBinsPerFrame)
    out = fw.RetainWindow()
    lib.l3_retain_window(
        ctypes.byref(cfg),
        ctypes.byref(state),
        process_start,
        process_bins,
        retain_bins,
        ctypes.byref(out),
    )
    return out


def _attach_retention(
    frames: list[ReplayFrame], windows: dict[int, fw.RetainWindow]
) -> list[ReplayFrame]:
    """Pair each frame with the window decided for it and the point it appended."""
    out = []
    for frame in frames:
        window = windows.get(frame.frame)
        if window is None:
            out.append(frame)
            continue
        point_bin = frame.ball_bin if frame.ball_bin is not None else frame.track_bin
        out.append(replace(frame, retain=_retain_summary(window, point_bin)))
    return out


def _retain_summary(window: fw.RetainWindow, point_bin: float | None) -> RetainSummary:
    covered = None
    if point_bin is not None:
        covered = window.start <= point_bin < window.start + window.bins
    return RetainSummary(
        start=int(window.start),
        bins=int(window.bins),
        priority=fw.RETAIN_PRIORITY_NAMES[window.priority],
        why=fw.RETAIN_WHY_NAMES[window.why],
        point_bin=point_bin,
        covered=covered,
    )


def replay_dump(
    raw: bytes, config: ReplayConfig, *, lib: ctypes.CDLL | None = None
) -> ReplayResult:
    """Run one capture's frames through the trigger, target extraction and club track.

    Mirrors ``l3_considerSelfTrigger``: per frame the watch region around the
    destination, one observation per bin, ``l3_trig_update`` (which adapts the
    floor), then the same observations as ranked targets into
    ``l3_track_update``. The dump must be a range-FFT snapshot.
    """
    lib = lib or _default_library()
    meta, cube = parse_dump(raw)
    if not is_range_snapshot(meta):
        raise ValueError("replay needs a range-FFT snapshot dump, not raw ADC samples")
    if config.stat not in fw.STAT_NAMES:
        raise ValueError(f"stat must be one of {sorted(fw.STAT_NAMES)}, got {config.stat!r}")
    if config.subbin not in fw.SUBBIN_NAMES:
        raise ValueError(f"subbin must be one of {sorted(fw.SUBBIN_NAMES)}, got {config.subbin!r}")
    if config.range_window not in fw.RANGE_WINDOW_NAMES:
        raise ValueError(
            f"range_window must be one of {sorted(fw.RANGE_WINDOW_NAMES)}, "
            f"got {config.range_window!r}"
        )
    n_tx = int(meta["n_tx"])
    loop_period_s = config.loop_period_s or same_tx_loop_period_s(n_tx)
    timestamps = frame_timestamps_us(meta)

    trig_cfg = fw.TrigCfg()
    lib.l3_trig_cfg_defaults(ctypes.byref(trig_cfg))
    trig_cfg.teeBin = config.tee_bin
    trig_cfg.snr = config.snr
    trig_cfg.trackFrames = config.track_frames
    trig_cfg.stat = fw.STAT_NAMES[config.stat]
    if lib.l3_trig_cfg_check(ctypes.byref(trig_cfg)) != 0:
        raise ValueError(f"the firmware rejects this trigger configuration: {config}")
    trig = fw.Trig()
    lib.l3_trig_init(ctypes.byref(trig), ctypes.byref(trig_cfg), loop_period_s)

    track_cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
    track_cfg.binWidthM = RANGE_SPAN_M / config.fft_size
    track_cfg.velocitySpanMps = 2.0 * fw.OBS_WAVELENGTH_M / (4.0 * loop_period_s)
    cal = _radar_cal(lib, config)
    track_cfg.cal = cal
    track = fw.ClubTrack()
    lib.l3_track_init(ctypes.byref(track), ctypes.byref(track_cfg))

    impact_cfg = fw.ImpactCfg()
    lib.l3_impact_cfg_defaults(ctypes.byref(impact_cfg))
    impact = fw.Impact()
    lib.l3_impact_init(ctypes.byref(impact), ctypes.byref(impact_cfg))
    delivery = fw.Delivery()
    ball_position = fw.Vec3()
    bin_width_m = RANGE_SPAN_M / config.fft_size
    chirp_period_s = loop_period_s / n_tx

    shot_cfg = fw.ShotCfg()
    lib.l3_shot_cfg_defaults(ctypes.byref(shot_cfg))
    shot_cfg.requireBall = 1 if config.dest_bin is not None else 0
    shot_cfg.ballTrackFrames = int(meta["n_frames"])
    shot = fw.Shot()
    lib.l3_shot_init(ctypes.byref(shot), ctypes.byref(shot_cfg))
    ball_cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(ball_cfg))
    ball_cfg.core.binWidthM = bin_width_m
    ball_cfg.core.velocitySpanMps = track_cfg.velocitySpanMps
    ball_cfg.core.cal = cal
    if config.ball_hypotheses is not None:
        ball_cfg.useHypotheses = 1 if config.ball_hypotheses else 0
    if config.ball_tuning is not None:
        config.ball_tuning.apply(ball_cfg)
    ball_track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(ball_track), ctypes.byref(ball_cfg))
    launch = fw.Launch()
    ball_points: list[PointSummary] = []
    ball_floor = ctypes.c_float(0.0)  # the post window's own floor, as gBallFloor
    # Joint search (host-only, optional)
    joint: fw.Joint | None = None
    joint_targets = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    joint_floor = ctypes.c_float(0.0)
    if config.joint_search:
        joint_cfg_s = fw.JointCfg()
        lib.l3_joint_cfg_defaults(ctypes.byref(joint_cfg_s))
        joint_cfg_s.binWidthM = bin_width_m
        joint_cfg_s.velocitySpanMps = track_cfg.velocitySpanMps
        joint_cfg_s.cal = cal
        joint = fw.Joint()
        lib.l3_joint_init(ctypes.byref(joint), ctypes.byref(joint_cfg_s))
    # The destination's direction: the locked ball's static return, else boresight.
    ball_azimuth = 0.0
    ball_elevation = 0.0
    ball_angle: AngleSummary | None = None
    if config.dest_bin is not None and config.ball_angles and meta["n_frames"] > 0:
        first_start, first_count = frame_window(meta, 0)
        if first_start <= config.destination < first_start + first_count:
            static_obs = fw.AngleObs()
            static_snap = static_channel_snapshot(
                cube, 0, config.destination - first_start, n_tx, chirp_period_s=chirp_period_s
            )
            if (
                lib.l3_angle_estimate(
                    ctypes.byref(cal), ctypes.byref(static_snap), ctypes.byref(static_obs)
                )
                and static_obs.elevationValid
                and static_obs.elevationPeakRatio >= BALL_ANGLE_MIN_PEAK_RATIO
            ):
                ball_azimuth = float(static_obs.azimuthRad) if static_obs.azimuthValid else 0.0
                ball_elevation = float(static_obs.elevationRad)
                ball_angle = _angle_summary(static_obs)

    params = fw.ObsParams(
        trig_cfg.stat, trig_cfg.snr, loop_period_s, fw.SUBBIN_NAMES[config.subbin]
    )
    targets = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    first_local = ctypes.c_uint32()
    count = ctypes.c_uint32()
    destination = config.destination

    frames: list[ReplayFrame] = []
    points: list[PointSummary] = []
    fired_frame: int | None = None
    geometric_frame: int | None = None
    retain_cfg = _retain_cfg(lib, config.retain) if config.retain is not None else None
    retain_windows: dict[int, fw.RetainWindow] = {}
    post_index = 0
    club_at_impact: tuple[float, float, float] | None = None
    for frame in range(int(meta["n_frames"])):
        ended = fired_frame is not None or (config.impact_armed and geometric_frame is not None)
        forced = config.post_from_frame is not None and frame >= config.post_from_frame
        if config.stop_at_fire and ended and not forced:
            break
        window_start, window_bins = frame_window(meta, frame)
        timestamp_us = timestamps[frame]
        retain_window = None
        if retain_cfg is not None and config.retain is not None:
            # Decided before the frame lands, from the previous frame's state,
            # as the rearm task decides where the next slot looks.
            post = (ended or forced) and config.post_impact
            if post:
                retain_bins = (
                    config.retain.impact_bins
                    if post_index < config.retain.impact_frames
                    else config.retain.post_bins
                )
            else:
                retain_bins = config.retain.pre_bins
            retain_window = _retain_window(
                lib,
                retain_cfg,
                shot=shot,
                locked=config.dest_bin is not None,
                destination=destination,
                track=track,
                ball_track=ball_track,
                post=post,
                post_index=post_index,
                process_start=window_start,
                process_bins=window_bins,
                retain_bins=retain_bins,
            )
            retain_windows[frame] = retain_window
            if post:
                post_index += 1
        if forced and frame == config.post_from_frame:
            # The recorded freeze is the impact: (re)arm the ball tracker at
            # the destination as IMPACT would have, whatever the gate did
            # earlier, and let the machine follow. Points gathered before it
            # belong to the gate's early fire, not to the flight.
            ball_points.clear()
            lib.l3_frames_observe(
                ctypes.byref(cal),
                destination * bin_width_m,
                ball_azimuth,
                ball_elevation,
                ctypes.byref(ball_position),
            )
            lib.l3_ball_track_arm(
                ctypes.byref(ball_track),
                float(destination),
                ctypes.byref(ball_position),
                timestamp_us,
            )
            if joint is not None:
                _joint_arm_from_track(lib, joint, track, timestamp_us)
            forced_in = fw.ShotInput()
            forced_in.ballLocked = 1 if config.dest_bin is not None else 0
            forced_in.ballPosition = ball_position
            forced_in.gateFired = 1
            forced_in.impactTimestampUs = timestamp_us
            forced_in.delivery = ctypes.pointer(delivery)
            forced_in.club = ctypes.pointer(track)
            lib.l3_shot_update(ctypes.byref(shot), ctypes.byref(forced_in), frame)
        if (ended or forced) and config.post_impact:
            # The board's post movie: the ball tracker, not the trigger, with
            # the club track carried on beside it. The club's speed is its
            # approach's, read before the follow-through joins the track.
            if club_at_impact is None:
                club_at_impact = _club_fit(lib, track)
            frames.append(
                _replay_post_frame(
                    lib,
                    cube,
                    frame,
                    timestamp_us,
                    window_start,
                    window_bins,
                    n_tx,
                    params,
                    ball_floor,
                    targets,
                    cal,
                    chirp_period_s,
                    ball_track,
                    launch,
                    shot,
                    ball_position,
                    ball_points,
                    fw.TRIG_STATE_NAMES[trig.state],
                    track,
                    points,
                    range_window=config.range_window,
                )
            )
            if joint is not None:
                _joint_post_frame(
                    lib,
                    joint,
                    cube,
                    frame,
                    timestamp_us,
                    window_start,
                    window_bins,
                    n_tx,
                    params,
                    joint_floor,
                    joint_targets,
                    range_window=config.range_window,
                )
            continue
        in_window = lib.l3_trig_region(
            ctypes.byref(trig_cfg),
            destination,
            window_start,
            window_bins,
            ctypes.byref(first_local),
            ctypes.byref(count),
        )
        if not in_window:
            frames.append(
                ReplayFrame(
                    frame,
                    timestamp_us,
                    window_start,
                    0,
                    float(trig.floor),
                    fw.TRIG_STATE_NAMES[trig.state],
                    False,
                    (),
                    fw.TRACK_WHY_NAMES[track.why],
                    None,
                    None,
                    None,
                    fw.IMPACT_WHY_NAMES[impact.why],
                )
            )
            continue
        first_bin = window_start + first_local.value
        obs = bin_observations(
            cube,
            frame,
            first_local.value,
            count.value,
            n_tx,
            window=config.range_window,
            valid_bins=window_bins,
        )
        fired = bool(
            lib.l3_trig_update(ctypes.byref(trig), frame, destination, first_bin, obs, count.value)
        )
        found = lib.l3_obs_extract(
            ctypes.byref(params),
            frame,
            timestamp_us,
            first_bin,
            obs,
            count.value,
            trig.floor,
            targets,
            fw.OBS_MAX_TARGETS,
        )
        appended = lib.l3_track_update(ctypes.byref(track), targets, found, frame, timestamp_us)
        track_bin = None
        angle = None
        newest = fw.TrackPoint()
        if appended and track.lastTargetIndex < found and track.count > 1:
            # As the board does: angles for the associated target only, the
            # track's range-rate resolving the TDM alias, so a track's first
            # point (no range rate yet) stays range-only.
            lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
            obs_angle, flags = _estimate_angles(
                lib,
                cal,
                cube,
                frame,
                window_start,
                n_tx,
                targets[track.lastTargetIndex],
                float(newest.radialVelocityMps),
                chirp_period_s,
            )
            if obs_angle is not None:
                lib.l3_track_set_angles(
                    ctypes.byref(track), obs_angle.azimuthRad, obs_angle.elevationRad, flags
                )
                angle = _angle_summary(obs_angle)
        if appended:
            lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
            points.append(_point_summary(newest))
            track_bin = float(newest.rangeBin)
        lib.l3_track_delivery(ctypes.byref(track), 8, ctypes.byref(delivery))
        lib.l3_frames_observe(
            ctypes.byref(cal),
            destination * bin_width_m,
            ball_azimuth,
            ball_elevation,
            ctypes.byref(ball_position),
        )
        geometric = lib.l3_impact_update(
            ctypes.byref(impact), ctypes.byref(delivery), ctypes.byref(ball_position), 1
        )
        if geometric and geometric_frame is None:
            geometric_frame = frame
        if fired:
            fired_frame = frame
        # The shot machine, as l3_shotObserve feeds it; IMPACT arms the ball tracker.
        shot_in = fw.ShotInput()
        shot_in.ballLocked = 1 if config.dest_bin is not None else 0
        shot_in.ballPosition = ball_position
        shot_in.clubActive = track.active
        shot_in.clubPoints = track.count
        shot_in.gateFired = 1 if fired else 0
        shot_in.geometricFired = 1 if (geometric and config.impact_armed) else 0
        shot_in.impactTimestampUs = (
            int(impact.impactTimestampUs) if (geometric and config.impact_armed) else timestamp_us
        )
        shot_in.delivery = ctypes.pointer(delivery)
        shot_in.club = ctypes.pointer(track)
        if (
            lib.l3_shot_update(ctypes.byref(shot), ctypes.byref(shot_in), frame)
            == fw.SHOT_STATE_NAMES.index("impact")
            and shot.impactFrame == frame
        ):
            lib.l3_ball_track_arm(
                ctypes.byref(ball_track),
                float(destination),
                ctypes.byref(ball_position),
                shot_in.impactTimestampUs,
            )
            if joint is not None:
                _joint_arm_from_track(lib, joint, track, shot_in.impactTimestampUs)
        frames.append(
            ReplayFrame(
                frame,
                timestamp_us,
                first_bin,
                count.value,
                float(trig.floor),
                fw.TRIG_STATE_NAMES[trig.state],
                fired,
                tuple(_target_summary(targets[i]) for i in range(found)),
                fw.TRACK_WHY_NAMES[track.why],
                track_bin,
                angle,
                _delivery_summary(delivery),
                fw.IMPACT_WHY_NAMES[impact.why],
                fw.SHOT_STATE_NAMES[shot.state],
            )
        )

    if retain_cfg is not None:
        frames = _attach_retention(frames, retain_windows)
    if joint is not None:
        lib.l3_joint_finish(ctypes.byref(joint))
    club_speed, club_slope, club_residual = club_at_impact or _club_fit(lib, track)
    joint_ball_pts, joint_club_pts = _joint_collect_points(joint) if joint is not None else ([], [])
    return ReplayResult(
        config=config,
        frames=frames,
        points=points,
        fired_frame=fired_frame,
        geometric_frame=geometric_frame,
        impact_timestamp_us=int(impact.impactTimestampUs) if impact.fired else None,
        delivery=_delivery_summary(delivery),
        impact_status=fw.c_text(lib.l3_impact_format, ctypes.byref(impact), cap=240),
        launch=_launch_summary(launch),
        ball_points=ball_points,
        ball_angle=ball_angle,
        shot_status=fw.c_text(lib.l3_shot_format, ctypes.byref(shot), cap=240),
        ball_status=fw.c_text(lib.l3_ball_track_format_status, ctypes.byref(ball_track), cap=240),
        track_counters={name: int(track.counters[i]) for i, name in enumerate(fw.TRACK_WHY_NAMES)},
        trig_counters={name: int(trig.counters[i]) for i, name in enumerate(_TRIG_COUNTERS)},
        joint_ball_points=joint_ball_pts,
        joint_club_points=joint_club_pts,
        joint_counters=(
            {name: int(joint.counters[i]) for i, name in enumerate(fw.JOINT_CNT_NAMES)}
            if joint is not None
            else {}
        ),
        joint_confirmed=bool(joint.ballConfirmed) if joint is not None else False,
        speed_mps=club_speed,
        fit_slope_bins_per_s=club_slope,
        fit_residual_bins=club_residual,
        status=fw.c_text(lib.l3_track_format_status, ctypes.byref(track), destination),
        trigger_summary=fw.c_text(lib.l3_trig_format_summary, ctypes.byref(trig), cap=400),
        trig=trig,
        track=track,
        impact=impact,
        shot=shot,
        ball_track=ball_track,
    )


def _joint_arm_from_track(lib, joint: fw.Joint, track: fw.ClubTrack, timestamp_us: int) -> None:
    """Arm the joint search from the current club track at the moment of impact."""
    if joint.seedValid or track.count == 0:
        return
    newest = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
    speed_mps, _slope, _residual = _club_fit(lib, track)
    kin = fw.JointKin()
    kin.rangeBin = newest.rangeBin
    kin.speedMps = speed_mps
    kin.timestampUs = timestamp_us
    lib.l3_joint_arm(ctypes.byref(joint), ctypes.byref(kin), timestamp_us)


def _joint_post_frame(  # pylint: disable=too-many-arguments
    lib,
    joint: fw.Joint,
    cube,
    frame: int,
    timestamp_us: int,
    window_start: int,
    window_bins: int,
    n_tx: int,
    params: fw.ObsParams,
    joint_floor: ctypes.c_float,
    joint_targets: ctypes.Array,
    *,
    range_window: str = "none",
) -> None:
    """Feed one post-impact frame to the joint search."""
    count = min(window_bins, fw.TRIG_MAX_BINS)
    obs = bin_observations(cube, frame, 0, count, n_tx, window=range_window, valid_bins=window_bins)
    joint_params = fw.ObsParams(params.stat, DEFAULT_SNR, params.loopPeriodS, params.subBin)
    lib.l3_obs_floor_update(ctypes.byref(joint_floor), params.stat, obs, count, FLOOR_SHIFT)
    floor = joint_floor.value
    found = lib.l3_obs_extract(
        ctypes.byref(joint_params),
        frame,
        timestamp_us,
        window_start,
        obs,
        count,
        floor,
        joint_targets,
        fw.OBS_MAX_TARGETS,
    )
    lib.l3_joint_update(ctypes.byref(joint), frame, timestamp_us, joint_targets, found)


def _joint_collect_points(joint: fw.Joint) -> tuple[list[PointSummary], list[PointSummary]]:
    """Read all finalized club and ball points from the joint search output."""
    ball_pts: list[PointSummary] = []
    club_pts: list[PointSummary] = []
    for i in range(int(joint.ballCount)):
        pt = joint.ballPoints[i]
        ball_pts.append(
            PointSummary(
                frame=0,
                timestamp_us=int(pt.timestampUs),
                range_bin=float(pt.rangeBin),
                range_m=float(pt.rangeBin) * float(joint.cfg.binWidthM),
                doppler_mps=float(pt.speedMps),
                confidence=0.0,
            )
        )
    for i in range(int(joint.clubCount)):
        pt = joint.clubPoints[i]
        club_pts.append(
            PointSummary(
                frame=0,
                timestamp_us=int(pt.timestampUs),
                range_bin=float(pt.rangeBin),
                range_m=float(pt.rangeBin) * float(joint.cfg.binWidthM),
                doppler_mps=float(pt.speedMps),
                confidence=0.0,
            )
        )
    return ball_pts, club_pts


def _club_fit(lib, track: fw.ClubTrack) -> tuple[float, float, float]:
    """The club track's (speed m/s, range-rate slope bins/s, fit residual bins)."""
    slope = ctypes.c_float()
    residual = ctypes.c_float()
    used = lib.l3_track_fit(
        ctypes.byref(track), fw.TRACK_POINTS, ctypes.byref(slope), ctypes.byref(residual)
    )
    return (
        float(lib.l3_track_speed_mps(ctypes.byref(track), fw.TRACK_POINTS)),
        float(slope.value) if used else 0.0,
        float(residual.value) if used else 0.0,
    )


def _replay_post_frame(  # pylint: disable=too-many-arguments,too-many-locals
    lib,
    cube,
    frame,
    timestamp_us,
    window_start,
    window_bins,
    n_tx,
    params,
    ball_floor,
    targets,
    cal,
    chirp_period_s,
    ball_track,
    launch,
    shot,
    ball_position,
    ball_points,
    trig_state,
    track,
    points,
    *,
    range_window: str = "none",
) -> ReplayFrame:
    """``l3_considerBallTrack``: the whole window as targets against the post
    window's own floor; the club track followed through them (the stronger of
    the two tracks after impact) and the ball tracker beside it, angles for
    the ball point, the launch fit and the shot machine's post-impact
    transitions."""
    count = min(window_bins, fw.TRIG_MAX_BINS)
    obs = bin_observations(cube, frame, 0, count, n_tx, window=range_window, valid_bins=window_bins)
    ball_params = fw.ObsParams(params.stat, ball_track.cfg.snr, params.loopPeriodS, params.subBin)
    lib.l3_obs_floor_update(ctypes.byref(ball_floor), params.stat, obs, count, FLOOR_SHIFT)
    floor = ball_floor.value
    found = lib.l3_obs_extract(
        ctypes.byref(ball_params),
        frame,
        timestamp_us,
        window_start,
        obs,
        count,
        floor,
        targets,
        fw.OBS_MAX_TARGETS,
    )
    track_bin = None
    if lib.l3_track_follow(ctypes.byref(track), targets, found, frame, timestamp_us):
        newest = fw.TrackPoint()
        lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
        points.append(_point_summary(newest))
        track_bin = float(newest.rangeBin)
    appended = lib.l3_ball_track_update_joint(
        ctypes.byref(ball_track), targets, found, frame, timestamp_us, track.lastTargetIndex
    )
    ball_bin = None
    angle = None
    if appended:
        newest = fw.TrackPoint()
        core = ball_track.core
        lib.l3_track_point(ctypes.byref(core), core.count - 1, ctypes.byref(newest))
        if core.count > 1 and ball_track.lastTargetIndex < found:
            hit = targets[ball_track.lastTargetIndex]
            if hit is not None:
                obs_angle, flags = _estimate_angles(
                    lib,
                    cal,
                    cube,
                    frame,
                    window_start,
                    n_tx,
                    hit,
                    float(newest.radialVelocityMps),
                    chirp_period_s,
                )
                if obs_angle is not None:
                    lib.l3_ball_track_set_angles(
                        ctypes.byref(ball_track),
                        obs_angle.azimuthRad,
                        obs_angle.elevationRad,
                        flags,
                    )
                    angle = _angle_summary(obs_angle)
        lib.l3_track_point(ctypes.byref(core), core.count - 1, ctypes.byref(newest))
        ball_points.append(_point_summary(newest))
        ball_bin = float(newest.rangeBin)
    _hypothesis_angles(
        lib, cal, cube, frame, window_start, n_tx, targets, found, ball_track, chirp_period_s
    )
    lib.l3_ball_track_launch(ctypes.byref(ball_track), ctypes.byref(launch))
    shot_in = fw.ShotInput()
    shot_in.ballPosition = ball_position
    shot_in.postFrame = 1
    shot_in.ballTrackDone = ball_track.done
    if lib.l3_shot_update(
        ctypes.byref(shot), ctypes.byref(shot_in), frame
    ) == fw.SHOT_STATE_NAMES.index("solve"):
        shot_in.solved = 1
        lib.l3_shot_update(ctypes.byref(shot), ctypes.byref(shot_in), frame)
    return ReplayFrame(
        frame,
        timestamp_us,
        window_start,
        count,
        float(floor),
        trig_state,
        False,
        tuple(_target_summary(targets[i]) for i in range(found)),
        fw.TRACK_WHY_NAMES[track.why],
        track_bin,
        angle,
        None,
        "none",
        fw.SHOT_STATE_NAMES[shot.state],
        fw.BALL_TRACK_WHY_NAMES[ball_track.why],
        ball_bin,
        retain=None,
        ball_hypotheses=_hypothesis_summaries(ball_track),
    )


def _hypothesis_angles(  # pylint: disable=too-many-arguments
    lib, cal, cube, frame, window_start, n_tx, targets, found, ball_track, chirp_period_s
) -> None:
    """Angles for every ball-hypothesis point appended this frame, as the board
    estimates them, so the chosen ball keeps angles on its early points."""
    hyps = ball_track.hyps
    rate, at, residual = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
    for index in range(fw.BALL_HYP_MAX):
        hyp = hyps.hyp[index]
        if not hyp.active or hyp.lastTargetIndex >= found:
            continue
        newest = hyp.points[hyp.count - 1]
        radial = (
            rate.value * hyps.cfg.binWidthM
            if lib.l3_ball_hyp_fit(
                ctypes.byref(hyp),
                newest.timestampUs,
                ctypes.byref(rate),
                ctypes.byref(at),
                ctypes.byref(residual),
            )
            else 0.0
        )
        obs_angle, flags = _estimate_angles(
            lib,
            cal,
            cube,
            frame,
            window_start,
            n_tx,
            targets[hyp.lastTargetIndex],
            radial,
            chirp_period_s,
        )
        if obs_angle is not None:
            lib.l3_ball_hyps_set_angles(
                ctypes.byref(hyps), index, obs_angle.azimuthRad, obs_angle.elevationRad, flags
            )


def _hypothesis_summaries(ball_track) -> tuple[HypothesisSummary, ...]:
    """The active ball hypotheses after a frame, for the viewer."""
    out = []
    for index in range(fw.BALL_HYP_MAX):
        hyp = ball_track.hyps.hyp[index]
        if hyp.active:
            out.append(
                HypothesisSummary(
                    int(hyp.id),
                    tuple(
                        (int(hyp.points[k].frame), float(hyp.points[k].rangeBin))
                        for k in range(hyp.count)
                    ),
                )
            )
    return tuple(out)


def replay_file(
    path: str | Path, config: ReplayConfig, *, lib: ctypes.CDLL | None = None
) -> ReplayResult:
    """:func:`replay_dump` on a ``.l3dump`` file."""
    return replay_dump(Path(path).read_bytes(), config, lib=lib)


RECORDINGS_DIR = Path(__file__).resolve().parents[3] / "tests" / "radar" / "recordings"
MANIFEST_NAME = "manifest.json"
EXPECT_KEY = "expect"


@dataclass(frozen=True)
class Expectation:
    """What a recording is expected to produce: ranges, not exact values.

    Keys of the manifest's ``expect`` entry: ``impact_frame`` [lo, hi] (the
    range gate), ``geometric_frame`` [lo, hi], ``club_points_min``,
    ``club_direction`` ("approaching"), ``acquisitions_max``,
    ``ball_origin_bin`` [lo, hi] (the first ball point), ``ball_speed_mps``
    [lo, hi], ``club_speed_mps`` [lo, hi], ``fires`` (true/false),
    ``club_last_bin`` [lo, hi] (the club track's last point at or before the
    gate fired: short of the ball when the club, not a return beside the
    ball, was tracked).
    """

    impact_frame: tuple[int, int] | None = None
    geometric_frame: tuple[int, int] | None = None
    club_points_min: int | None = None
    club_direction: str | None = None
    acquisitions_max: int | None = None
    ball_origin_bin: tuple[float, float] | None = None
    ball_speed_mps: tuple[float, float] | None = None
    club_speed_mps: tuple[float, float] | None = None
    fires: bool | None = None
    club_last_bin: tuple[float, float] | None = None

    @classmethod
    def from_manifest(cls, raw: dict) -> Expectation:
        known = {f for f in cls.__dataclass_fields__}  # pylint: disable=no-member
        unknown = set(raw) - known
        if unknown:
            raise ValueError(f"unknown expectation keys {sorted(unknown)}; known: {sorted(known)}")
        values = dict(raw)
        for key in (
            "impact_frame",
            "geometric_frame",
            "ball_origin_bin",
            "ball_speed_mps",
            "club_speed_mps",
            "club_last_bin",
        ):
            if key in values:
                lo, hi = values[key]
                values[key] = (lo, hi)
        return cls(**values)

    def check(self, result: ReplayResult) -> list[str]:
        """The expectations the result fails, as readable reasons; empty is a pass."""
        failures: list[str] = []

        def in_range(name: str, value, bounds) -> None:
            if bounds is None:
                return
            if value is None:
                failures.append(f"{name}: none, expected {bounds[0]}..{bounds[1]}")
            elif not bounds[0] <= value <= bounds[1]:
                failures.append(f"{name}: {value:.2f} outside {bounds[0]}..{bounds[1]}")

        if self.fires is not None and (result.fired_frame is not None) != self.fires:
            failures.append(f"fires: {result.fired_frame is not None}, expected {self.fires}")
        in_range("impact_frame", result.fired_frame, self.impact_frame)
        in_range("geometric_frame", result.geometric_frame, self.geometric_frame)
        if self.club_points_min is not None and len(result.points) < self.club_points_min:
            failures.append(f"club_points: {len(result.points)} < {self.club_points_min}")
        if self.club_direction == "approaching" and result.approach_fraction < 0.75:
            failures.append(
                f"club_direction: approach fraction {result.approach_fraction:.2f} < 0.75"
            )
        if self.acquisitions_max is not None and result.acquisitions > self.acquisitions_max:
            failures.append(f"acquisitions: {result.acquisitions} > {self.acquisitions_max}")
        if self.club_last_bin is not None:
            held = [
                p
                for p in result.points
                if result.fired_frame is None or p.frame <= result.fired_frame
            ]
            in_range("club_last_bin", held[-1].range_bin if held else None, self.club_last_bin)
        first_ball = result.ball_points[0].range_bin if result.ball_points else None
        in_range("ball_origin_bin", first_ball, self.ball_origin_bin)
        in_range(
            "ball_speed_mps",
            None if result.launch is None else result.launch.speed_mps,
            self.ball_speed_mps,
        )
        in_range(
            "club_speed_mps",
            None if result.delivery is None else result.delivery.speed_mps,
            self.club_speed_mps,
        )
        return failures


def recording_expectations(directory: str | Path = RECORDINGS_DIR) -> dict[str, Expectation]:
    """The ``expect`` entry of each recording in the manifest, by file name;
    the ``default`` entry's expectations apply to files without their own."""
    directory = Path(directory)
    manifest_path = directory / MANIFEST_NAME
    if not manifest_path.exists():
        return {}
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    default = manifest.get("default", {}).get(EXPECT_KEY, {})
    out: dict[str, Expectation] = {}
    for path in sorted(directory.glob("*.l3dump")):
        raw = {**default, **manifest.get(path.name, {}).get(EXPECT_KEY, {})}
        if raw:
            out[path.name] = Expectation.from_manifest(raw)
    return out


def recording_configs(
    directory: str | Path = RECORDINGS_DIR, *, default_tee_bin: int | None = None
) -> list[tuple[Path, ReplayConfig]]:
    """Every ``.l3dump`` in a recordings directory with its replay configuration.

    ``manifest.json`` beside the dumps maps each file name to the keyword
    arguments of :class:`ReplayConfig` (``tee_bin`` at least, ``dest_bin``
    when a ball was locked); a ``"default"`` entry supplies the rest. A dump
    without an entry and without a default tee bin is an error, because a
    guessed tee watches the wrong stretch of air.
    """
    directory = Path(directory)
    manifest: dict = {}
    manifest_path = directory / MANIFEST_NAME
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    default = dict(manifest.get("default", {}))
    if default_tee_bin is not None:
        default["tee_bin"] = default_tee_bin
    configs: list[tuple[Path, ReplayConfig]] = []
    for path in sorted(directory.glob("*.l3dump")):
        entry = {**default, **manifest.get(path.name, {})}
        entry.pop("notes", None)
        entry.pop(EXPECT_KEY, None)
        if "tee_bin" not in entry:
            raise ValueError(
                f"{path.name}: no tee bin. Give --tee-bin or --tee-range-m on the command line, "
                f'or put {{"default": {{"tee_bin": 34}}}} in {directory / MANIFEST_NAME} '
                "(34 is 1.59 m on the 128-point FFT)."
            )
        configs.append((path, ReplayConfig(**entry)))
    return configs


def _delivery_line(result: ReplayResult) -> str:
    d = result.delivery
    if d is None:
        return "delivery: none"
    path = "-" if d.path_deg is None else f"{d.path_deg:+.1f} deg"
    attack = "-" if d.attack_deg is None else f"{d.attack_deg:+.1f} deg"
    geometric = "-" if result.geometric_frame is None else str(result.geometric_frame)
    return (
        f"delivery: {d.points} points, speed {d.speed_mps:.1f} m/s (radial {d.radial_speed_mps:.1f}), "
        f"path {path}, attack {attack}, residual {1000 * d.residual_m:.1f} mm, "
        f"confidence {d.confidence:.2f}; geometric impact frame {geometric}"
    )


def _point_angles(result: ReplayResult, point: PointSummary) -> str:
    for frame in result.frames:
        if frame.frame == point.frame and frame.angle is not None:
            az = "-" if frame.angle.azimuth_deg is None else f"{frame.angle.azimuth_deg:+.1f}"
            el = "-" if frame.angle.elevation_deg is None else f"{frame.angle.elevation_deg:+.1f}"
            return f" az={az} el={el} aconf={frame.angle.confidence:.2f}"
    return ""


def _ball_angle_line(result: ReplayResult) -> str:
    angle = result.ball_angle
    if angle is None:
        return (
            "ball direction: boresight (no locked ball, or its static return did not stand clear)"
        )
    az = "-" if angle.azimuth_deg is None else f"{angle.azimuth_deg:+.1f} deg"
    el = "-" if angle.elevation_deg is None else f"{angle.elevation_deg:+.1f} deg"
    return f"ball direction: az {az}, el {el} (peak ratio {angle.elevation_peak_ratio:.1f})"


def _launch_line(result: ReplayResult) -> str:
    launch = result.launch
    if launch is None:
        return "launch: none"
    hla = "-" if launch.hla_deg is None else f"{launch.hla_deg:+.1f} deg"
    vla = "-" if launch.vla_deg is None else f"{launch.vla_deg:+.1f} deg"
    return (
        f"launch: {launch.points} points, ball speed {launch.speed_mps:.1f} m/s "
        f"(radial {launch.radial_speed_mps:.1f}), hla {hla}, vla {vla}, "
        f"residual {1000 * launch.residual_m:.1f} mm, confidence {launch.confidence:.2f}"
    )


def _retention_line(result: ReplayResult) -> str:
    covered, judged = result.retain_coverage
    kept, processed = result.retain_bins_saved
    reasons: dict[str, int] = {}
    for window in result.retain_windows:
        reasons[window.why] = reasons.get(window.why, 0) + 1
    why = " ".join(f"{name}={count}" for name, count in sorted(reasons.items()))
    share = f"{100.0 * kept / processed:.0f}%" if processed else "-"
    missed = [
        f"{w.why}@{w.point_bin:.1f}!in[{w.start},{w.end})"
        for w in result.retain_windows
        if w.covered is False
    ]
    tail = f"; missed {', '.join(missed[:4])}" if missed else ""
    return (
        f"retention: {covered}/{judged} points inside the kept window, "
        f"{kept}/{processed} bins kept ({share}); {why}{tail}"
    )


def format_report(result: ReplayResult, *, name: str = "", points: bool = False) -> str:
    """A human-readable verdict on one capture: continuity first, then the detail."""
    fired = "no fire" if result.fired_frame is None else f"fired frame {result.fired_frame}"
    lines = [
        f"{name or 'capture'}: {fired}, {len(result.points)} points, "
        f"longest run {result.longest_run}, acquisitions {result.acquisitions}, "
        f"coasted {result.track_counters['coasted']}, dropped {result.track_counters['dropped']}, "
        f"approach {100.0 * result.approach_fraction:.0f}%, "
        f"speed {result.speed_mps:.1f} m/s, fit residual {result.fit_residual_bins:.2f} bins",
        f"  {result.status}",
        f"  {result.trigger_summary}",
        "  " + _delivery_line(result),
        f"  {result.impact_status}",
        "  " + _launch_line(result),
        "  " + _ball_angle_line(result),
        f"  {result.shot_status}",
        f"  {result.ball_status}",
    ]
    if result.retain_windows:
        lines.append("  " + _retention_line(result))
    if points:
        distance = result.config.destination
        for point in result.points:
            lines.append(
                f"  p frame={point.frame} t={point.timestamp_us / 1000.0:.1f}ms "
                f"bin={point.range_bin:.2f} dist={distance - point.range_bin:.1f} "
                f"range={point.range_m:.3f} vd={point.doppler_mps:.2f} "
                f"conf={point.confidence:.2f}" + _point_angles(result, point)
            )
        for point in result.ball_points:
            lines.append(
                f"  b frame={point.frame} t={point.timestamp_us / 1000.0:.1f}ms "
                f"bin={point.range_bin:.2f} range={point.range_m:.3f} vd={point.doppler_mps:.2f} "
                f"conf={point.confidence:.2f}" + _point_angles(result, point)
            )
    return "\n".join(lines)


__all__ = [
    "DEFAULT_FFT_SIZE",
    "MANIFEST_NAME",
    "RECORDINGS_DIR",
    "DEFAULT_SNR",
    "DEFAULT_TRACK_FRAMES",
    "EXPECT_KEY",
    "FALLBACK_FRAME_PERIOD_US",
    "AngleSummary",
    "BallTuning",
    "Expectation",
    "DeliverySummary",
    "HypothesisSummary",
    "LaunchSummary",
    "PointSummary",
    "ReplayConfig",
    "ReplayFrame",
    "ReplayResult",
    "RetainReplay",
    "RetainSummary",
    "TargetSummary",
    "bin_observation_table",
    "bin_observations",
    "channel_snapshot",
    "format_report",
    "static_channel_snapshot",
    "frame_timestamps_us",
    "frame_window",
    "recording_configs",
    "recording_expectations",
    "replay_dump",
    "replay_file",
    "vertical_tx_indices",
]
