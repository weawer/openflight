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
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_host as fw, tunables
from openflight.iwr6843.dump import clutter_map_struct, is_range_snapshot, parse_dump
from openflight.iwr6843.self_trigger import BALL_SNR_MAX, check_ball_snr
from openflight.iwr6843.tracking import RANGE_SPAN_M, same_tx_loop_period_s

# The CLI-configured trigger parameters the recorded captures were made
# with (monitor.SelfTriggerConfig then); the C owns the rest. A replay
# reproduces a recording, so these stay put when the Pi's defaults move
# (self_trigger.FIRMWARE_TRIGGER_DEFAULT_*).
DEFAULT_SNR = 6.0
# The ball tracker's extraction snr then (l3_ball_track_cfg_defaults' old 3);
# the board's default is now self_trigger.FIRMWARE_BALL_DEFAULT_SNR.
DEFAULT_BALL_SNR = 3.0
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


def bin_observation_table(
    cube: np.ndarray, frame: int, first_local: int, count: int, n_tx: int
) -> np.ndarray:
    """``l3_verticalResidual`` for ``count`` bins from local bin ``first_local``.

    ``cube`` is a parsed dump, ``[frames, chirps, rx, bins]`` with chirp c
    from transmitter ``c % n_tx`` of loop ``c // n_tx``. Returns a structured
    array with fields energy, peak, loop0, r1Re and r1Im, one row per bin.
    """
    chirps = cube.shape[1]
    if chirps % n_tx:
        raise ValueError(f"{chirps} chirps per frame is not a whole number of {n_tx}-TX loops")
    loops = chirps // n_tx
    if count <= 0 or first_local < 0 or first_local + count > cube.shape[-1]:
        raise ValueError(f"bins {first_local}..{first_local + count - 1} outside the frame")
    data = cube[frame, :, :, first_local : first_local + count]
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


def bin_observations(
    cube: np.ndarray, frame: int, first_local: int, count: int, n_tx: int
) -> ctypes.Array:
    """:func:`bin_observation_table` as a ctypes array of ``count`` :class:`BinObs`,
    ready for ``l3_trig_update`` and ``l3_obs_extract``."""
    table = bin_observation_table(cube, frame, first_local, count, n_tx)
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
# Shot states before impact is declared: the impact fit runs only after them.
PRE_IMPACT_SHOT_STATES = ("waiting_for_ball", "ready", "club_acquire", "club_track")

_LIBRARY: ctypes.CDLL | None = None


def _default_library() -> ctypes.CDLL:
    """The compiled firmware modules, built once per process."""
    global _LIBRARY  # pylint: disable=global-statement
    if _LIBRARY is None:
        _LIBRARY = fw.build_firmware_library()
    return _LIBRARY


def post_frame_count(meta: dict) -> int | None:
    """Post-impact frames the board's capture plan held (impact + ball frames).

    From the adaptive retention report when present; else from the first
    frame whose window start differs from the pre-impact window's; None when
    the dump cannot tell (one window throughout, no report).
    """
    n_frames = int(meta["n_frames"])
    retention = meta.get("retention")
    if retention:
        return n_frames - int(retention["pre_frames"])
    starts = meta.get("range_bin_starts")
    if starts:
        for index, start in enumerate(starts):
            if start != starts[0]:
                return n_frames - index
    return None


def freeze_frame(meta: dict) -> int | None:
    """The first post-impact frame of a saved capture: the frame the board froze on.

    ``n_frames`` less ``post_frame_count``, so a capture without a retention
    report (before v8) still has one where its plan switches windows; None
    when the dump cannot tell.
    """
    post = post_frame_count(meta)
    return None if post is None else int(meta["n_frames"]) - post


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
    ``far_window_m`` its separate far range window.
    """

    fast_ball_mps: float | None = None
    fast_support_fraction: float | None = None
    min_departure_mps: float | None = None
    far_window_m: float | None = None
    corridor_gate: bool | None = None
    impact_coast_ms: float | None = None
    max_decel_mps2: float | None = None
    classify_points: int | None = None
    recover: bool | None = None
    recover_gate_m: float | None = None
    history_snr: float | None = None

    def apply(self, cfg: fw.BallTrackCfg) -> None:
        """Write the set overrides into a ball-track configuration."""
        hyps = cfg.hyps
        if self.fast_ball_mps is not None:
            hyps.fastBallMps = self.fast_ball_mps
        if self.fast_support_fraction is not None:
            hyps.fastSupportFraction = self.fast_support_fraction
        if self.min_departure_mps is not None:
            cfg.minDepartureMps = self.min_departure_mps
            hyps.minDepartureMps = self.min_departure_mps
        if self.far_window_m is not None:
            hyps.farWindowM = self.far_window_m
        if self.corridor_gate is not None:
            hyps.corridorGate = 1 if self.corridor_gate else 0
        if self.impact_coast_ms is not None:
            hyps.impactCoastUs = round(self.impact_coast_ms * 1000)
        if self.max_decel_mps2 is not None:
            hyps.maxDecelMps2 = self.max_decel_mps2
        if self.classify_points is not None:
            hyps.classifyPoints = self.classify_points
        if self.recover is not None:
            cfg.recover = 1 if self.recover else 0
        if self.recover_gate_m is not None:
            cfg.rec.gateM = self.recover_gate_m
        if self.history_snr is not None:
            cfg.historySnr = self.history_snr


@dataclass(frozen=True)
class ReplayConfig:
    """What ``triggerCfg`` and the capture profile would have told the board."""

    tee_bin: int  # global; the destination without a locked ball
    snr: float = DEFAULT_SNR
    stat: str = "peak"  # "peak" or "energy"
    subbin: str = "parabolic"  # how targets read their sub-bin range: "parabolic" or "centroid"
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
    # The tee band's total width in bins (l3_band.h): placed on the noisiest
    # idle bins near the destination (l3_band_place) and frozen while a club
    # track is active; targets inside it are dropped before any tracker sees
    # them (before impact the club keeps only targets short of it). None or
    # 0: no band, as the recordings were made (the board's own default,
    # l3_impact_fit_cfg_defaults, is 6 bins).
    band_bins: float | None = None
    # "trackCfg ballSnr": the ball tracker's extraction snr, apart from the
    # trigger's ``snr``; None is the recordings' (DEFAULT_BALL_SNR).
    ball_snr: float | None = None
    # Element calibration ("trackCfg elem"), physical order: None is identity,
    # as the recordings were made. board_calibration.replay_overrides fills it.
    elem_phase_rad: tuple[float, ...] | None = None
    elem_gain: tuple[float, ...] | None = None
    # Firmware config constants (tunables.py) set over the defaults; applied last, so they win.
    overrides: Mapping[str, float] = field(default_factory=dict)

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
    angles_accepted: int = 0
    angle_rms_deg: float | None = None  # None when the direction fit kept no angles
    angle_why: str = "none"  # l3_ball_fit_why_name: why the angles are (not) valid


@dataclass(frozen=True)
class TrackEstimateSummary:
    """One track's impact estimate (l3_fit_estimate_t)."""

    why: str
    points: int
    time_us: float | None  # None unless the estimate was kept, dropped or too uncertain
    sigma_us: float | None
    speed_mps: float


@dataclass(frozen=True)
class ImpactFitSummary:
    """The three estimates and their fusion (l3_impact_fit_t)."""

    verdict: str
    impact_us: float | None
    spread_us: float
    refined_minus_trigger_us: float | None
    dropped: str | None
    no_lock: bool
    tracks: dict[str, TrackEstimateSummary]


def apply_impact_fit(shot: fw.Shot, fit: fw.ImpactFit) -> None:
    """``l3_impactFitRun``'s last step: a verdict other than none, with a
    time, replaces the shot's frozen impact time (``fw.round_us``, the C's
    ``l3_round_us``)."""
    if fit.verdict != fw.FIT_VERDICT_NAMES.index("none") and fit.impactUs > 0.0:
        shot.impactTimestampUs = fw.round_us(fit.impactUs)


_SHOT_RESULT = fw.SHOT_STATE_NAMES.index("result")


def _run_impact_fit(  # pylint: disable=too-many-arguments
    lib: ctypes.CDLL,
    fit_cfg: fw.ImpactFitCfg,
    shot: fw.Shot,
    track: fw.ClubTrack,
    ball_track: fw.BallTrack,
    ball_range_m: float,
    no_lock: bool,
    fit: fw.ImpactFit,
) -> int:
    """``l3_impactFitRun``: club in as the shot froze it, club out after the
    impact frame, ball out; measured against the frozen impact time, which a
    verdict then replaces on the shot (``apply_impact_fit``). Returns the
    frozen time."""
    club_in_list = fw.FitList(
        ctypes.cast(shot.clubTrajectory, ctypes.POINTER(fw.TrackPoint)), shot.clubPoints
    )
    club_out = fw.FitSpan()
    lib.l3_fit_span_after(ctypes.byref(track), shot.impactFrame, ctypes.byref(club_out))
    ball_out = fw.FitSpan(ctypes.pointer(ball_track.core), 0, ball_track.core.count)
    frozen_us = int(shot.impactTimestampUs)
    lib.l3_impact_fit_run(
        ctypes.byref(fit_cfg),
        ctypes.byref(club_in_list),
        ctypes.byref(club_out),
        ctypes.byref(ball_out),
        ball_range_m,
        1 if no_lock else 0,
        frozen_us,
        ctypes.byref(fit),
    )
    apply_impact_fit(shot, fit)
    return frozen_us


def _impact_fit_summary(fit: fw.ImpactFit) -> ImpactFitSummary:
    tracks = {}
    for index, name in enumerate(fw.FIT_TRACK_NAMES):
        e = fit.track[index]
        why = fw.FIT_WHY_NAMES[e.why]
        timed = fw.fit_track_timed(why)
        tracks[name] = TrackEstimateSummary(
            why,
            int(e.points),
            float(e.timeUs) if timed else None,
            float(e.sigmaUs) if timed else None,
            float(e.speedMps),
        )
    verdict = fw.FIT_VERDICT_NAMES[fit.verdict]
    decided = fw.fit_verdict_decided(verdict)
    return ImpactFitSummary(
        verdict=verdict,
        impact_us=float(fit.impactUs) if decided else None,
        spread_us=float(fit.spreadUs),
        refined_minus_trigger_us=float(fit.refinedMinusTriggerUs) if decided else None,
        dropped=(
            fw.FIT_TRACK_NAMES[fit.droppedTrack]
            if fit.droppedTrack < len(fw.FIT_TRACK_NAMES)
            else None
        ),
        no_lock=bool(fit.noLock),
        tracks=tracks,
    )


@dataclass(frozen=True)
class ReplayFrame:
    """What one frame did to the trigger, the track and the range-only impact."""

    frame: int
    timestamp_us: int
    first_bin: int  # global bin of the first scored observation
    count: int  # observations scored; 0 when the window missed the tee
    floor: float  # the trigger front end's floor after this frame
    fired: bool  # the self-trigger fired on this frame
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
    # Range bins the board scores (l3_verticalResidual) on this frame: its
    # cost is ~73 us a bin (triggerLog perf, 2026-09-30) against a 3 ms frame.
    scored_bins: int = 0
    # Club channel snapshots queued on this frame's decision path for the
    # angle task (l3_angle_queue.h), where the angle estimate used to run.
    angle_snapshots: int = 0


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
    angle_confidence: float = 0.0
    # l3_track_point_t.filteredPosition; None when the point was not reconstructed.
    filtered_position: tuple[float, float, float] | None = None
    filter_accepted: bool = False
    filter_hypothesis: str = "unfiltered"


@dataclass
class ReplayResult:
    """One capture through the firmware modules, with the whole trajectory kept."""

    config: ReplayConfig
    frames: list[ReplayFrame]
    points: list[PointSummary]  # every appended point, beyond the C ring's depth
    # The self-trigger's fire, the frame the board freezes on: the club
    # track's range-only impact.
    fired_frame: int | None
    impact_timestamp_us: int | None  # the range impact's crossing time, when it fired
    delivery: DeliverySummary | None  # at the end of the replay
    impact_status: str  # l3_impact_format (the range impact) at the end of the replay
    launch: LaunchSummary | None  # from the ball tracker, when a flight was confirmed
    ball_points: list[PointSummary]
    ball_angle: AngleSummary | None  # the locked ball's measured direction, when trusted
    shot_status: str  # l3_shot_format at the end
    ball_status: str  # l3_ball_track_format_status at the end
    track_counters: dict[str, int]
    speed_mps: float
    fit_slope_bins_per_s: float
    fit_residual_bins: float
    status: str  # l3_track_format_status at the end of the replay
    trigger_summary: str  # l3_trig_format_summary (the front end's floor) at the end
    trig: fw.Trig = field(repr=False)
    track: fw.ClubTrack = field(repr=False)
    impact: fw.Impact = field(repr=False)
    shot: fw.Shot = field(repr=False)
    ball_track: fw.BallTrack = field(repr=False)
    band: tuple[float, float] | None = None  # as last placed
    # The noise map (l3_band_noise_t averages) the band was last placed from,
    # and the first frame the band froze on an acquired club (None: never).
    band_noise: tuple[float, ...] = ()
    band_frozen_frame: int | None = None
    range_frame: int | None = None  # the range-only impact's fire
    leave_frame: int | None = None  # the ball-leave fallback's fire
    leave_status: str = ""  # l3_leave_format at the end of the replay
    impact_fit: ImpactFitSummary | None = None
    impact_fit_status: str = ""
    # The shot's impact time as IMPACT froze it, before the fit refined
    # shot.impactTimestampUs (apply_impact_fit); None without an impact.
    frozen_impact_timestamp_us: int | None = None
    # Frames the shot machine tracked the ball for before SOLVE, and whether
    # the dump said (retention report / window change) or it fell back to all.
    ball_track_frames: int = 0
    ball_track_frames_known: bool = False
    recovered_frames: tuple[int, ...] = ()  # frames the backward pass added at adoption

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


def recovered_frames(verdict: fw.BallHypVerdict) -> tuple[int, ...]:
    """The frames the backward pass added at adoption: bit k of recoveredMask
    is frame recoveredFirstFrame + k."""
    return tuple(
        int(verdict.recoveredFirstFrame) + bit
        for bit in range(32)
        if verdict.recoveredMask >> bit & 1
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
        angles_accepted=int(launch.anglesAccepted),
        angle_rms_deg=math.degrees(launch.angleRmsRad) if launch.anglesAccepted else None,
        angle_why=fw.BALL_FIT_WHY_NAMES[launch.angleWhy],
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
    *,
    track_rate_mps: float | None = None,
) -> tuple[fw.AngleObs | None, int]:
    """One target's angles as the board would estimate them; (obs, flags).

    ``track_rate_mps`` (the fitted range rate), when given, replaces the radial
    velocity that picks the TDM branch; the measured lag-1 phase stays the rotor.
    """
    if track_rate_mps:
        radial_velocity_mps = track_rate_mps
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


def drain_angle_queue(
    lib: ctypes.CDLL, queue: fw.AngleQueue, cal: fw.RadarCal, track: fw.ClubTrack
) -> AngleSummary | None:
    """Every pending club angle onto its point (``l3_angleQueueDrain``); the
    summary of the last one applied, or None when none was."""
    job = fw.AngleJob()
    applied = None
    while lib.l3_angle_queue_pop(ctypes.byref(queue), ctypes.byref(job)):
        obs = fw.AngleObs()
        if (
            lib.l3_angle_queue_apply(
                ctypes.byref(queue),
                ctypes.byref(cal),
                ctypes.byref(job),
                ctypes.byref(track),
                ctypes.byref(obs),
            )
            == 1
        ):
            applied = _angle_summary(obs)
    return applied


def _radar_cal(lib: ctypes.CDLL, config: ReplayConfig) -> fw.RadarCal:
    cal = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(cal), fw.CAL_MAX_VIRTUAL)
    cal.radarPitchRad = math.radians(config.pitch_deg)
    cal.radarYawRad = math.radians(config.yaw_deg)
    cal.radarRollRad = math.radians(config.roll_deg)
    cal.azimuthOffsetRad = config.azimuth_offset_rad
    cal.elevationOffsetRad = math.radians(config.elevation_offset_deg)
    cal.rangeBiasM = config.range_bias_m
    if config.elem_phase_rad is not None and config.elem_gain is not None:
        for index, (phase, gain) in enumerate(zip(config.elem_phase_rad, config.elem_gain)):
            if lib.l3_cal_set_element(ctypes.byref(cal), index, gain, phase) != 0:
                raise ValueError(f"element {index}: gain must be positive, got {gain}")
    return cal


def _point_summary(point: fw.TrackPoint) -> PointSummary:
    reconstructed = point.filterHypothesis != fw.FILTER_HYP_UNFILTERED
    return PointSummary(
        frame=int(point.frame),
        timestamp_us=int(point.timestampUs),
        range_bin=float(point.rangeBin),
        range_m=float(point.rangeM),
        doppler_mps=float(point.dopplerAliasMps),
        confidence=float(point.confidence),
        position=(float(point.position.x), float(point.position.y), float(point.position.z)),
        angles_valid=bool(point.anglesValid),
        angle_confidence=float(point.angleConfidence),
        filtered_position=(
            (
                float(point.filteredPosition.x),
                float(point.filteredPosition.y),
                float(point.filteredPosition.z),
            )
            if reconstructed
            else None
        ),
        filter_accepted=bool(point.filterAccepted),
        filter_hypothesis=fw.FILTER_HYP_NAMES[point.filterHypothesis],
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
    if config.ball_snr is not None:
        check_ball_snr(config.ball_snr)
    if (config.elem_phase_rad is None) != (config.elem_gain is None) or (
        config.elem_phase_rad is not None
        and (len(config.elem_phase_rad) != 8 or len(config.elem_gain) != 8)
    ):
        raise ValueError("element calibration needs 8 element phases and 8 gains")
    tunables.check_overrides(config.overrides)
    n_tx = int(meta["n_tx"])
    loop_period_s = config.loop_period_s or same_tx_loop_period_s(n_tx)
    timestamps = frame_timestamps_us(meta)

    trig_cfg = fw.TrigCfg()
    lib.l3_trig_cfg_defaults(ctypes.byref(trig_cfg))
    trig_cfg.teeBin = config.tee_bin
    trig_cfg.snr = config.snr
    trig_cfg.stat = fw.STAT_NAMES[config.stat]
    tunables.apply_overrides(config.overrides, "trig", trig_cfg)
    if lib.l3_trig_cfg_check(ctypes.byref(trig_cfg)) != 0:
        raise ValueError(f"the firmware rejects this trigger configuration: {config}")
    trig = fw.Trig()
    lib.l3_trig_init(ctypes.byref(trig), ctypes.byref(trig_cfg))

    track_cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
    track_cfg.binWidthM = RANGE_SPAN_M / config.fft_size
    track_cfg.velocitySpanMps = 2.0 * fw.OBS_WAVELENGTH_M / (4.0 * loop_period_s)
    cal = _radar_cal(lib, config)
    track_cfg.cal = cal
    tunables.apply_overrides(config.overrides, "club", track_cfg)
    track = fw.ClubTrack()
    lib.l3_track_init(ctypes.byref(track), ctypes.byref(track_cfg))
    # The club's pending angles (l3_angle_queue.h), as the board queues them.
    angle_queue = fw.AngleQueue()
    lib.l3_angle_queue_init(ctypes.byref(angle_queue))
    # The club reconstruction's work area (l3_track_kf.h): the replay and viewer run the
    # club filter; the board does not.
    kf_work = ctypes.create_string_buffer(lib.l3_track_kf_work_bytes())
    kf_result = fw.TrackKfResult()

    impact_cfg = fw.ImpactCfg()
    lib.l3_impact_cfg_defaults(ctypes.byref(impact_cfg))
    # The range-only impact: the self-trigger (l3_impact_update_range).
    impact = fw.Impact()
    lib.l3_impact_init(ctypes.byref(impact), ctypes.byref(impact_cfg))
    # The ball-leave fallback (l3_leave_update): fires when the club rules miss.
    leave_cfg = fw.LeaveCfg()
    lib.l3_leave_cfg_defaults(ctypes.byref(leave_cfg))
    leave_cfg.binWidthM = RANGE_SPAN_M / config.fft_size
    leave = fw.Leave()
    lib.l3_leave_init(ctypes.byref(leave), ctypes.byref(leave_cfg))
    destination = config.destination
    fit_cfg = fw.ImpactFitCfg()
    lib.l3_impact_fit_cfg_defaults(ctypes.byref(fit_cfg))
    fit_cfg.binWidthM = RANGE_SPAN_M / config.fft_size
    fit_cfg.bandBins = 0.0 if config.band_bins is None else config.band_bins
    tunables.apply_overrides(config.overrides, "fit", fit_cfg)
    # The tee band: re-placed on every idle pre-impact frame from the noise
    # map (l3_band_place), frozen while the club track is active, kept through
    # impact and the post frames -- as l3_considerSelfTrigger.
    band_enabled = fit_cfg.bandBins > 0
    band = fw.Band()
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    if "clutter_map" in meta:
        # The board's map as it stood at the dump (version 10): the capture
        # alone has too few idle frames to learn it. The replay feeds it on as
        # the board did, so the ring's idle frames count twice (at 1/16).
        noise = clutter_map_struct(meta["clutter_map"])
    band_frozen = False
    band_frozen_frame: int | None = None
    # The impact fit, run once when the shot first reaches RESULT (as the
    # board's l3_impactFitRun beside l3_result_build); reset until then.
    fit = fw.ImpactFit()
    lib.l3_impact_fit_reset(ctypes.byref(fit))
    fitted_frozen_us: int | None = None  # the frozen time the fit replaced
    range_frame: int | None = None
    leave_frame: int | None = None
    club_reader = fw.fit_reader(lib, "l3_fit_span_point")
    delivery = fw.Delivery()
    ball_position = fw.Vec3()
    bin_width_m = RANGE_SPAN_M / config.fft_size
    chirp_period_s = loop_period_s / n_tx
    frame_us = int(meta.get("frame_period_us") or FALLBACK_FRAME_PERIOD_US)

    shot_cfg = fw.ShotCfg()
    lib.l3_shot_cfg_defaults(ctypes.byref(shot_cfg))
    shot_cfg.requireBall = 1 if config.dest_bin is not None else 0
    known_post = post_frame_count(meta)
    shot_cfg.ballTrackFrames = known_post if known_post is not None else int(meta["n_frames"])
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
    # The board overrides only the extraction snr (gBallSnr); so does this.
    ball_cfg.snr = DEFAULT_BALL_SNR if config.ball_snr is None else config.ball_snr
    tunables.apply_overrides(config.overrides, "ball", ball_cfg)
    ball_track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(ball_track), ctypes.byref(ball_cfg))
    launch = fw.Launch()
    ball_points: list[PointSummary] = []
    ball_floor = ctypes.c_float(0.0)  # the post window's own floor, as gBallFloor
    # The scan plan (l3_scan.h): which bins a frame scores with the band on.
    scan_cfg = fw.ScanCfg()
    lib.l3_scan_cfg_defaults(ctypes.byref(scan_cfg))
    map_cursor = ctypes.c_uint32(0)
    # The fallback's median beyond the band: the post window's frozen floor.
    leave_floor = ctypes.c_float(0.0)
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

    frames: list[ReplayFrame] = []
    points: list[PointSummary] = []
    fired_frame: int | None = None
    retain_cfg = _retain_cfg(lib, config.retain) if config.retain is not None else None
    retain_windows: dict[int, fw.RetainWindow] = {}
    post_index = 0
    club_at_impact: tuple[float, float, float] | None = None
    for frame in range(int(meta["n_frames"])):
        forced = config.post_from_frame is not None and frame >= config.post_from_frame
        # A sound-triggered recording's freeze is the impact: no self-trigger,
        # however early it fires, makes the frames before it post-impact.
        early = config.post_from_frame is not None and not forced
        ended = not early and fired_frame is not None
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
            _arm_ball(
                lib, ball_track, fit_cfg, track, band, destination, ball_position, timestamp_us
            )
            forced_in = fw.ShotInput()
            forced_in.ballLocked = 1 if config.dest_bin is not None else 0
            forced_in.ballPosition = ball_position
            # The shot machine's inputs are the impact detectors'; an impact
            # the replay is given enters through the range-only one.
            forced_in.rangeFired = 1
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
                    track,
                    points,
                    band,
                    destination,
                    bin_width_m,
                    frame_us,
                    scan_cfg=scan_cfg if band_enabled else None,
                    frozen_floor=_post_floor(leave_floor, trig) if band_enabled else None,
                )
            )
            if fitted_frozen_us is None and shot.state == _SHOT_RESULT:
                fitted_frozen_us = _run_impact_fit(
                    lib,
                    fit_cfg,
                    shot,
                    track,
                    ball_track,
                    destination * bin_width_m,
                    config.dest_bin is None,
                    fit,
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
        # The band first: the scan plan and the trigger's region follow it.
        if band_enabled and not band_frozen:
            lib.l3_band_place(
                ctypes.byref(noise),
                float(destination),
                fit_cfg.bandSearchBins,
                fit_cfg.bandBins,
                ctypes.byref(band),
            )
        elif not band_enabled:
            band.valid = 0
        scan = None
        if band_enabled:
            scan = _scan_pre_impact(
                lib,
                cube,
                frame,
                timestamp_us,
                window_start,
                window_bins,
                n_tx,
                params,
                trig,
                destination,
                band,
                targets,
                scan_cfg,
                region=(first_bin, count.value),
                idle=not track.active,
                map_cursor=map_cursor,
                leave=leave,
                leave_track=track,
                leave_floor=leave_floor,
                clutter=noise,
                clutter_sigmas=fit_cfg.clutterSigmas,
            )
            found = scan.found
            first_bin, scored_region = scan.region.first, scan.region.count
            scored_bins = scan.scored
        else:
            obs = bin_observations(cube, frame, first_local.value, count.value, n_tx)
            lib.l3_trig_observe(ctypes.byref(trig), frame, destination, first_bin, obs, count.value)
            found, _, _ = _pre_impact_club_targets(
                lib,
                cube,
                frame,
                timestamp_us,
                window_start,
                window_bins,
                n_tx,
                params,
                float(trig.floor),
                band,
                targets,
                trigger_region=(first_bin, obs, count.value),
                band_enabled=False,
            )
            scored_region = count.value
            scored_bins = count.value
        left = bool(leave.fired) and leave_frame is None
        if left:
            leave_frame = frame
        appended = lib.l3_track_update(ctypes.byref(track), targets, found, frame, timestamp_us)
        if band_enabled:
            # An active club track freezes the band where it stands; an idle
            # frame thaws it and feeds the whole window it scored to the map.
            if track.active:
                if band_frozen_frame is None and not band_frozen:
                    band_frozen_frame = frame
                band_frozen = True
            else:
                band_frozen = False
                if scan is not None:
                    # Idle: every span this frame scored feeds the map.
                    for span in scan.map_spans:
                        lib.l3_band_noise_update_span(
                            ctypes.byref(noise),
                            params.stat,
                            window_start,
                            scan.window_count,
                            span.first,
                            _span_obs(scan.window_obs, window_start, span),
                            span.count,
                        )
        track_bin = None
        angle = None
        angle_snapshots = 0
        newest = fw.TrackPoint()
        if appended and track.lastTargetIndex < found and track.count > 1:
            # As the board does: the associated target's channels, queued by
            # the point's timestamp for the angle task (l3_angle_queue.h), the
            # track's range-rate resolving the TDM alias, so a track's first
            # point (no range rate yet) stays range-only. The replay drains at
            # once, as a board whose angle task is idle; a fire drains anyway.
            lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
            hit = targets[track.lastTargetIndex]
            snapshot = channel_snapshot(
                cube,
                frame,
                int(hit.peakBin) - window_start,
                n_tx,
                lag1_phase_rad=float(hit.dopplerPhaseRad),
                radial_velocity_mps=float(newest.radialVelocityMps),
                chirp_period_s=chirp_period_s,
            )
            lib.l3_angle_queue_push(
                ctypes.byref(angle_queue), int(newest.timestampUs), ctypes.byref(snapshot)
            )
            angle_snapshots = 1
            angle = drain_angle_queue(lib, angle_queue, cal, track)
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
        club_span = fw.FitSpan(ctypes.pointer(track), 0, track.count)
        club_in = fw.FitEstimate()
        lib.l3_impact_fit_track(
            ctypes.byref(fit_cfg),
            fw.FIT_CLUB_IN,
            club_reader,
            ctypes.byref(club_span),
            track.count,
            destination * bin_width_m,
            ctypes.byref(club_in),
        )
        club_now = fw.ImpactClub()
        club_now.appended = 1 if appended else 0
        if appended:
            club_now.rangeM = float(newest.rangeM)
            club_now.timeUs = int(newest.timestampUs)
        club_now.ballRangeM = destination * bin_width_m
        ranged = lib.l3_impact_update_range(
            ctypes.byref(impact), ctypes.byref(club_in), ctypes.byref(club_now), timestamp_us
        )
        if ranged and range_frame is None:
            range_frame = frame
        # l3_considerSelfTrigger: the range-only impact fires. A
        # sound-triggered recording's impact is post_from_frame, so nothing
        # before it fires.
        fired = bool(ranged or left) and not early
        if fired and fired_frame is None:
            fired_frame = frame
        # The shot machine, as l3_shotObserve feeds it; IMPACT arms the ball tracker.
        shot_in = fw.ShotInput()
        shot_in.ballLocked = 1 if config.dest_bin is not None else 0
        shot_in.ballPosition = ball_position
        shot_in.clubActive = track.active
        shot_in.clubPoints = track.count
        shot_in.rangeFired = 1 if fired else 0
        if fired and ranged:
            shot_in.impactTimestampUs = int(impact.impactTimestampUs)
        elif fired:
            shot_in.impactTimestampUs = int(leave.impactTimestampUs)
        else:
            shot_in.impactTimestampUs = timestamp_us
        shot_in.delivery = ctypes.pointer(delivery)
        shot_in.club = ctypes.pointer(track)
        if (
            lib.l3_shot_update(ctypes.byref(shot), ctypes.byref(shot_in), frame)
            == fw.SHOT_STATE_NAMES.index("impact")
            and shot.impactFrame == frame
        ):
            _arm_ball(
                lib,
                ball_track,
                fit_cfg,
                track,
                band,
                destination,
                ball_position,
                shot_in.impactTimestampUs,
            )
        if fired and left and ball_track.armed:
            # l3_considerSelfTrigger: the fallback's late fire (the club's rule
            # may have fired too) seeds the flight with the ball's two points.
            if lib.l3_ball_track_seed(
                ctypes.byref(ball_track),
                ctypes.byref(leave.startTarget),
                ctypes.byref(leave.stepTarget),
            ):
                for index in range(ball_track.core.count):
                    point = fw.TrackPoint()
                    lib.l3_track_point(ctypes.byref(ball_track.core), index, ctypes.byref(point))
                    ball_points.append(_point_summary(point))
        frames.append(
            ReplayFrame(
                frame,
                timestamp_us,
                first_bin,
                scored_region,
                float(trig.floor),
                fired,
                tuple(_target_summary(targets[i]) for i in range(found)),
                fw.TRACK_WHY_NAMES[track.why],
                track_bin,
                angle,
                _delivery_summary(delivery),
                fw.IMPACT_WHY_NAMES[impact.why],
                fw.SHOT_STATE_NAMES[shot.state],
                scored_bins=scored_bins,
                angle_snapshots=angle_snapshots,
            )
        )

    if retain_cfg is not None:
        frames = _attach_retention(frames, retain_windows)
    club_speed, club_slope, club_residual = club_at_impact or _club_fit(lib, track)
    impact_declared = fw.SHOT_STATE_NAMES[shot.state] not in PRE_IMPACT_SHOT_STATES
    if not impact_declared:
        frozen_impact_us = None
    elif fitted_frozen_us is not None:
        frozen_impact_us = fitted_frozen_us
    else:
        frozen_impact_us = int(shot.impactTimestampUs)
    # The viewer's trajectories. The board does not reconstruct the club (its
    # frozen delivery is the unfiltered one); the replay does, for the viewer
    # only. The ball is reconstructed once, as the board does at RESULT, over
    # every held point at the end, so the page shows the whole track even when
    # the capture ended before RESULT.
    lib.l3_track_kf_run(
        ctypes.byref(track.cfg.kf), ctypes.byref(track), kf_work, ctypes.byref(kf_result)
    )
    lib.l3_ball_track_reconstruct(ctypes.byref(ball_track), ctypes.byref(launch))
    points = _reconstructed(lib, track, points)
    ball_points = _reconstructed(lib, ball_track.core, ball_points)
    points = _confirmed_points(points, track)
    return ReplayResult(
        config=config,
        frames=frames,
        points=points,
        fired_frame=fired_frame,
        impact_timestamp_us=int(impact.impactTimestampUs) if impact.fired else None,
        delivery=_delivery_summary(delivery),
        impact_status=fw.c_text(lib.l3_impact_format, ctypes.byref(impact), cap=240),
        launch=_launch_summary(launch),
        ball_points=ball_points,
        ball_angle=ball_angle,
        shot_status=fw.c_text(lib.l3_shot_format, ctypes.byref(shot), cap=240),
        ball_status=fw.c_text(lib.l3_ball_track_format_status, ctypes.byref(ball_track), cap=240),
        track_counters={name: int(track.counters[i]) for i, name in enumerate(fw.TRACK_WHY_NAMES)},
        band=(float(band.loBin), float(band.hiBin)) if band.valid else None,
        band_noise=tuple(float(v) for v in noise.avg[: noise.count]),
        band_frozen_frame=band_frozen_frame,
        range_frame=range_frame,
        leave_frame=leave_frame,
        leave_status=fw.c_text(lib.l3_leave_format, ctypes.byref(leave), cap=240),
        impact_fit=_impact_fit_summary(fit) if fitted_frozen_us is not None else None,
        impact_fit_status=fw.c_text(lib.l3_impact_fit_format, ctypes.byref(fit), cap=240),
        frozen_impact_timestamp_us=frozen_impact_us,
        ball_track_frames=int(shot_cfg.ballTrackFrames),
        ball_track_frames_known=known_post is not None,
        recovered_frames=recovered_frames(ball_track.verdict),
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


def _banded_window_targets(  # pylint: disable=too-many-arguments
    lib,
    cube,
    frame: int,
    timestamp_us: int,
    window_start: int,
    window_bins: int,
    n_tx: int,
    params: fw.ObsParams,
    band: fw.Band,
    targets: ctypes.Array,
    *,
    floor: float | None = None,
    running_floor: ctypes.c_float | None = None,
    keep_short: bool = False,
    leave: fw.Leave | None = None,
    leave_track: fw.ClubTrack | None = None,
) -> tuple[int, int, float, ctypes.Array]:
    """The whole frame window as targets with the tee band removed:
    ``l3_obs_extract`` over local bins ``0..count`` (global first bin
    ``window_start``, so a target's ``peakBin - window_start`` is its local
    bin), then ``l3_band_filter`` -- or, with ``keep_short``,
    ``l3_band_keep_short``, which also drops everything beyond the band. A
    disabled band removes nothing. The floor is either given (the trigger's,
    before impact) or a running one updated from this window first (the post
    window's own, as gBallFloor). Returns (targets kept, bins scored, floor,
    the scored bins' observations). ``leave`` reads the same observations
    for its own targets, as l3_preImpactClubTargets feeds l3_leave_update:
    the band's far edge and its centre (the ball's rest bin), armed by
    ``leave_track`` near the band (l3_leave_club_near); no band, no update."""
    if (floor is None) == (running_floor is None):
        raise ValueError("give exactly one of floor and running_floor")
    count = min(window_bins, fw.TRIG_MAX_BINS)
    obs = bin_observations(cube, frame, 0, count, n_tx)
    if running_floor is not None:
        lib.l3_obs_floor_update(ctypes.byref(running_floor), params.stat, obs, count, FLOOR_SHIFT)
        floor = running_floor.value
    found = lib.l3_obs_extract(
        ctypes.byref(params),
        frame,
        timestamp_us,
        window_start,
        obs,
        count,
        floor,
        targets,
        fw.OBS_MAX_TARGETS,
    )
    if leave is not None and band.valid:
        leaving = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
        n_leaving = lib.l3_leave_targets(
            ctypes.byref(leave.cfg),
            ctypes.byref(params),
            obs,
            window_start,
            count,
            frame,
            timestamp_us,
            band.hiBin,
            leaving,
            fw.OBS_MAX_TARGETS,
        )
        lib.l3_leave_update(
            ctypes.byref(leave),
            leaving,
            n_leaving,
            band.hiBin,
            0.5 * (band.loBin + band.hiBin),
            _leave_club_near(lib, leave, leave_track, band),
        )
    keep = lib.l3_band_keep_short if keep_short else lib.l3_band_filter
    found = keep(ctypes.byref(band), targets, found)
    return found, count, float(floor), obs


def _leave_club_near(lib, leave: fw.Leave, track: fw.ClubTrack | None, band: fw.Band) -> int:
    """l3_leave_club_near for the club track as it stood after the last frame,
    as l3_preImpactClubTargets asks it."""
    if track is None or track.count == 0:
        return 0
    newest = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
    return lib.l3_leave_club_near(
        ctypes.byref(leave.cfg), track.active, track.count, newest.rangeBin, band.loBin
    )


@dataclass(frozen=True)
class _PreScan:
    """One band-on pre-impact frame as the scan plan scored it."""

    found: int  # club targets kept (short of the band)
    region: fw.Span  # the trigger's region, clipped to short of the band
    map_spans: tuple[fw.Span, ...]  # what an idle frame feeds the noise map
    window_obs: ctypes.Array  # per-bin observations over the window (each bin's own)
    window_count: int
    scored: int  # distinct bins the board scores this frame


def _span_obs(window_obs: ctypes.Array, window_start: int, span: fw.Span) -> ctypes.Array:
    """The observations of a span, as a C array starting at its first bin."""
    first = span.first - window_start
    return (fw.BinObs * max(1, span.count))(*window_obs[first : first + span.count])


def _scan_pre_impact(  # pylint: disable=too-many-arguments,too-many-locals
    lib,
    cube,
    frame: int,
    timestamp_us: int,
    window_start: int,
    window_bins: int,
    n_tx: int,
    params: fw.ObsParams,
    trig,
    destination: int,
    band: fw.Band,
    targets: ctypes.Array,
    scan_cfg: fw.ScanCfg,
    *,
    region: tuple[int, int],
    idle: bool,
    map_cursor: ctypes.c_uint32,
    leave: fw.Leave,
    leave_track: fw.ClubTrack,
    leave_floor: ctypes.c_float,
    clutter: fw.BandNoise,
    clutter_sigmas: float,
) -> _PreScan:
    """l3_preImpactClubTargets with the band on: the scan plan (l3_scan_pre)
    scores the trigger region clipped to short of the band, the club's
    approach short of it and the fallback's stretch beyond it, plus on an idle
    frame a chunk of the band's interior for the noise map. Each bin's
    observation is its own, so the replay computes the window once and reads
    the spans out of it; ``scored`` is what the board computes."""
    window_count = min(window_bins, fw.TRIG_MAX_BINS)
    window_obs = bin_observations(cube, frame, 0, window_count, n_tx)
    region_span, club, leave_span = fw.Span(), fw.Span(), fw.Span()
    lib.l3_scan_pre(
        ctypes.byref(scan_cfg),
        window_start,
        window_count,
        region[0],
        region[1],
        ctypes.byref(band),
        ctypes.byref(region_span),
        ctypes.byref(club),
        ctypes.byref(leave_span),
    )
    chunk = fw.Span()
    if idle:
        lib.l3_scan_map_chunk(
            ctypes.byref(scan_cfg),
            window_start,
            window_count,
            ctypes.byref(band),
            ctypes.byref(map_cursor),
            ctypes.byref(chunk),
        )
    spans = (fw.Span * 4)(region_span, club, leave_span, chunk)
    scored = lib.l3_scan_count(spans, 4)
    if region_span.count > 0:
        lib.l3_trig_observe(
            ctypes.byref(trig),
            frame,
            destination,
            region_span.first,
            _span_obs(window_obs, window_start, region_span),
            region_span.count,
        )
    if band.valid and leave_span.count > 0:
        leaving = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
        n_leaving = lib.l3_leave_targets(
            ctypes.byref(leave.cfg),
            ctypes.byref(params),
            _span_obs(window_obs, window_start, leave_span),
            leave_span.first,
            leave_span.count,
            frame,
            timestamp_us,
            band.hiBin,
            leaving,
            fw.OBS_MAX_TARGETS,
            ctypes.byref(leave_floor),
        )
        lib.l3_leave_update(
            ctypes.byref(leave),
            leaving,
            n_leaving,
            band.hiBin,
            0.5 * (band.loBin + band.hiBin),
            _leave_club_near(lib, leave, leave_track, band),
        )
    found = 0
    if club.count > 0:
        found = lib.l3_obs_extract(
            ctypes.byref(params),
            frame,
            timestamp_us,
            club.first,
            _span_obs(window_obs, window_start, club),
            club.count,
            float(trig.floor),
            targets,
            fw.OBS_MAX_TARGETS,
        )
        found = lib.l3_band_keep_short(ctypes.byref(band), targets, found)
        # The scene at address is not the club (l3_band_clutter_filter).
        found = lib.l3_band_clutter_filter(ctypes.byref(clutter), clutter_sigmas, targets, found)
    map_spans = tuple(span for span in (club, leave_span, chunk) if span.count > 0)
    return _PreScan(found, region_span, map_spans, window_obs, window_count, scored)


def _pre_impact_club_targets(  # pylint: disable=too-many-arguments
    lib,
    cube,
    frame: int,
    timestamp_us: int,
    window_start: int,
    window_bins: int,
    n_tx: int,
    params: fw.ObsParams,
    floor: float,
    band: fw.Band,
    targets: ctypes.Array,
    *,
    trigger_region: tuple[int, ctypes.Array, int],
    band_enabled: bool,
    leave: fw.Leave | None = None,
    leave_track: fw.ClubTrack | None = None,
) -> tuple[int, ctypes.Array | None, int]:
    """The club track's targets on a pre-impact frame: (how many, the
    whole-window observations scored, their count) -- (n, None, 0) when the
    whole window was not scored.

    Band off: the trigger region's own observations (``trigger_region`` is
    its (global first bin, observations, count)), extracted against the
    trigger's floor -- the view the club track always had. Band on
    (``band_enabled``, whether or not a band is placed yet): the trigger
    region would be mostly band, so the club reads the whole window against
    the same floor and keeps only targets short of the band
    (``l3_band_keep_short``): the club approaches the ball, so nothing in the
    band or beyond it is the club before impact. Those whole-window
    observations also feed the band's noise map on idle frames."""
    if band_enabled:
        found, window_count, _, window_obs = _banded_window_targets(
            lib,
            cube,
            frame,
            timestamp_us,
            window_start,
            window_bins,
            n_tx,
            params,
            band,
            targets,
            floor=floor,
            keep_short=True,
            leave=leave,
            leave_track=leave_track,
        )
        return found, window_obs, window_count
    first_bin, obs, count = trigger_region
    found = lib.l3_obs_extract(
        ctypes.byref(params),
        frame,
        timestamp_us,
        first_bin,
        obs,
        count,
        floor,
        targets,
        fw.OBS_MAX_TARGETS,
    )
    return found, None, 0


def _ball_arm_bin(band: fw.Band, destination: int) -> float:
    """Where the ball tracker is armed (as l3_ballArmBin): the band's far edge
    as it stands, else the destination."""
    return float(band.hiBin) if band.valid else float(destination)


def _arm_ball(  # pylint: disable=too-many-arguments,too-many-positional-arguments
    lib, ball_track, fit_cfg, track, band, destination, ball_position, gate_us
) -> None:
    """l3_shotObserve's arm: the anchor from the tee and the club, then the track."""
    anchor = fw.BallAnchor()
    lib.l3_ball_track_anchor(
        ctypes.byref(ball_track),
        float(destination),
        _ball_arm_bin(band, destination),
        int(gate_us),
        ctypes.byref(fit_cfg),
        ctypes.byref(track),
        ctypes.byref(anchor),
    )
    lib.l3_ball_track_arm(
        ctypes.byref(ball_track), ctypes.byref(anchor), ctypes.byref(ball_position)
    )


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


def _follow_ctx(  # pylint: disable=too-many-arguments
    lib, shot, ball_track, band, destination, bin_width_m, frame_us, ball_claim
) -> fw.FollowCtx:
    """The scene after impact for l3_track_follow, as l3_considerBallTrack builds it."""
    ctx = fw.FollowCtx()
    ctx.bandValid = band.valid
    ctx.bandHiBin = band.hiBin
    ctx.originBin = float(destination)
    ctx.impactTimestampUs = shot.impactTimestampUs
    delivery = shot.delivery
    # No approach measured (the capture began with the club in the band): the
    # fastest club bounds re-acquisition instead.
    approach_mps = (
        delivery.radialSpeedMps if delivery.speedValid else fw.TRACK_FOLLOW_UNKNOWN_APPROACH_MPS
    )
    ctx.approachBinsPerS = approach_mps / bin_width_m
    ctx.approachKnown = 1 if delivery.speedValid else 0
    ball_rate = float(lib.l3_track_recent_rate(ctypes.byref(ball_track.core)))
    ctx.ballBinsPerS = ball_rate if ball_rate > 0.0 else 0.0  # a receding "ball" is no rate
    ctx.ballClaimIndex = ball_claim
    ctx.frameUs = frame_us
    return ctx


def _follow_club(  # pylint: disable=too-many-arguments
    lib, track, targets, found, frame, timestamp_us, follow, points
) -> float | None:
    """One post-impact frame of ``l3_track_follow``, kept in ``points``: an
    appended point is logged; a tentative point the frame withdrew (the track's
    total fell) is taken off the log again. Returns the appended bin."""
    total = int(track.total)
    appended = lib.l3_track_follow(
        ctypes.byref(track), targets, found, frame, timestamp_us, ctypes.byref(follow)
    )
    if int(track.total) < total and points:
        points.pop()
    if not appended:
        return None
    newest = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(track), track.count - 1, ctypes.byref(newest))
    points.append(_point_summary(newest))
    return float(newest.rangeBin)


def _reconstructed(lib, core, summaries: list[PointSummary]) -> list[PointSummary]:
    """The summaries with every still-held point re-read after reconstruction;
    a point rolled off the ring (or withdrawn) keeps what it was logged with.
    Held points are found by timestamp, so this assumes timestamps are unique."""
    index = ctypes.c_uint32()
    point = fw.TrackPoint()
    out = []
    for summary in summaries:
        held = lib.l3_track_find_point(
            ctypes.byref(core), summary.timestamp_us, ctypes.byref(index)
        ) and lib.l3_track_point(ctypes.byref(core), index.value, ctypes.byref(point))
        out.append(_point_summary(point) if held else summary)
    return out


def _confirmed_points(points: list[PointSummary], track) -> list[PointSummary]:
    """The logged points less a newest point still tentative when the capture
    ends: nothing confirmed it, so it is not a club point
    (``l3_fit_span_after`` leaves it out of club out on the board)."""
    return points[:-1] if track.tentative and points else points


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
    track,
    points,
    band,
    destination,
    bin_width_m,
    frame_us,
    *,
    scan_cfg: fw.ScanCfg | None = None,
    frozen_floor: float | None = None,
) -> ReplayFrame:
    """``l3_considerBallTrack``: the whole window as targets against the post
    window's own floor -- with the band on, the scan plan's 16 bins following
    the ball against the floor frozen at impact; the ball tracker first, then the club track followed
    through the scene the ball leaves (the ball's claim and rate, the band it
    coasts across), angles for the ball point, the launch fit and the shot
    machine's post-impact transitions."""
    ball_params = fw.ObsParams(
        params.stat,
        lib.l3_ball_track_extract_snr(ctypes.byref(ball_track.cfg), ball_track.cfg.snr),
        params.loopPeriodS,
        params.subBin,
    )
    if scan_cfg is not None and frozen_floor is not None and band.valid:
        found, count, floor = _scan_post_impact(
            lib,
            cube,
            frame,
            timestamp_us,
            window_start,
            window_bins,
            n_tx,
            ball_params,
            band,
            targets,
            scan_cfg,
            ball_track,
            track,
            frozen_floor,
        )
        ball_floor.value = floor
    else:
        found, count, floor, _ = _banded_window_targets(
            lib,
            cube,
            frame,
            timestamp_us,
            window_start,
            window_bins,
            n_tx,
            ball_params,
            band,
            targets,
            running_floor=ball_floor,
        )
    # The ball first: its claim and rate tell the club what it is not.
    appended = lib.l3_ball_track_update_joint(
        ctypes.byref(ball_track), targets, found, frame, timestamp_us, fw.TRACK_NO_TARGET
    )
    ball_claim = ball_track.lastTargetIndex if appended else fw.TRACK_NO_TARGET
    follow = _follow_ctx(
        lib, shot, ball_track, band, destination, bin_width_m, frame_us, ball_claim
    )
    track_bin = _follow_club(lib, track, targets, found, frame, timestamp_us, follow, points)
    # The club's claim reaches the ball's history now (l3_considerBallTrack).
    lib.l3_ball_track_note_club(ctypes.byref(ball_track), frame, track.lastTargetIndex)
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
                    track_rate_mps=(
                        lib.l3_track_recent_rate(ctypes.byref(core)) * bin_width_m or None
                    ),
                )
                if obs_angle is not None:
                    lib.l3_ball_track_set_angles(
                        ctypes.byref(ball_track),
                        obs_angle.azimuthRad,
                        obs_angle.elevationRad,
                        flags,
                        float(obs_angle.confidence),
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
        scored_bins=count,
    )


def _post_floor(leave_floor: ctypes.c_float, trig) -> float:
    """The post window's frozen floor (as l3_considerSelfTrigger freezes
    gBallFloor at impact): the fallback's median beyond the band, the noise of
    the stretch the ball flies into; without one (too few bins beyond the band
    in the window) the trigger's floor, the same statistic on the approach."""
    return float(leave_floor.value) if leave_floor.value > 0.0 else float(trig.floor)


def _predicted_bin(core, frame: int) -> float:
    """A track's range at ``frame`` from its last point and its rate."""
    return float(core.lastBin) + float(core.velocityBinsPerFrame) * (frame - core.lastFrame)


def _scan_post_impact(  # pylint: disable=too-many-arguments,too-many-locals
    lib,
    cube,
    frame: int,
    timestamp_us: int,
    window_start: int,
    window_bins: int,
    n_tx: int,
    params: fw.ObsParams,
    band: fw.Band,
    targets: ctypes.Array,
    scan_cfg: fw.ScanCfg,
    ball_track: fw.BallTrack,
    club_track: fw.ClubTrack,
    frozen_floor: float,
) -> tuple[int, int, float]:
    """l3_considerBallTrack with the band on: the scan plan's post windows
    (l3_scan_post) -- postBins following the ball, from just beyond the band
    until it is tracked, and postClubBins following the club, just beyond the
    band until it is taken -- merged (l3_scan_merge) and each extracted against
    the floor frozen at impact (the fallback's median beyond the band: a
    16-bin window's own median is the ball, the club and the ridge). Returns
    (targets kept, bins scored, floor)."""
    count = min(window_bins, fw.TRIG_MAX_BINS)
    ball_core = ball_track.core
    club_live = bool(club_track.active) and club_track.count > 0
    ball_span, club_span = fw.Span(), fw.Span()
    lib.l3_scan_post(
        ctypes.byref(scan_cfg),
        window_start,
        count,
        ctypes.byref(band),
        1 if ball_core.active else 0,
        _predicted_bin(ball_core, frame),
        1 if club_live else 0,
        _predicted_bin(club_track, frame) if club_live else 0.0,
        ctypes.byref(ball_span),
        ctypes.byref(club_span),
    )
    spans = (fw.Span * 2)()
    n_spans = lib.l3_scan_merge(ball_span, club_span, spans)
    window_obs = bin_observations(cube, frame, 0, count, n_tx)
    found = 0
    scored = 0
    for index in range(n_spans):
        span = spans[index]
        scored += span.count
        # As the board: each span's targets follow the last span's in the buffer.
        extracted = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
        got = lib.l3_obs_extract(
            ctypes.byref(params),
            frame,
            timestamp_us,
            span.first,
            _span_obs(window_obs, window_start, span),
            span.count,
            frozen_floor,
            extracted,
            fw.OBS_MAX_TARGETS - found,
        )
        for i in range(got):
            targets[found + i] = extracted[i]
        found += got
    found = lib.l3_band_filter(ctypes.byref(band), targets, found)
    return found, scored, frozen_floor


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
            track_rate_mps=radial or None,
        )
        if obs_angle is not None:
            lib.l3_ball_hyps_set_angles(
                ctypes.byref(hyps),
                index,
                obs_angle.azimuthRad,
                obs_angle.elevationRad,
                flags,
                float(obs_angle.confidence),
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
    self-trigger's fire), ``club_points_min``,
    ``club_direction`` ("approaching"), ``acquisitions_max``,
    ``ball_origin_bin`` [lo, hi] (the first ball point), ``ball_speed_mps``
    [lo, hi], ``club_speed_mps`` [lo, hi], ``fires`` (true/false),
    ``club_last_bin`` [lo, hi] (the club track's last point at or before the
    self-trigger fired: short of the ball when the club, not a return beside
    the ball, was tracked).
    """

    impact_frame: tuple[int, int] | None = None
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
    return [
        (path, ReplayConfig(**recording_entry(path, default_tee_bin=default_tee_bin)))
        for path in sorted(directory.glob("*.l3dump"))
    ]


def recording_entry(path: str | Path, *, default_tee_bin: int | None = None) -> dict:
    """One dump's manifest keyword arguments for :class:`ReplayConfig`.

    The manifest's ``default`` entry merged with the file's own, without the
    ``notes`` and ``expect`` entries. Raises ``ValueError`` when no tee bin
    results. ``recording_configs`` and the dump viewer share this so a page and
    a test replay the same way.
    """
    path = Path(path)
    manifest: dict = {}
    manifest_path = path.parent / MANIFEST_NAME
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    default = dict(manifest.get("default", {}))
    if default_tee_bin is not None:
        default["tee_bin"] = default_tee_bin
    entry = {**default, **manifest.get(path.name, {})}
    entry.pop("notes", None)
    entry.pop(EXPECT_KEY, None)
    if "tee_bin" not in entry:
        raise ValueError(
            f"{path.name}: no tee bin. Give --tee-bin or --tee-range-m on the command line, "
            f'or put {{"default": {{"tee_bin": 34}}}} in {path.parent / MANIFEST_NAME} '
            "(34 is 1.59 m on the 128-point FFT)."
        )
    return entry


def _delivery_line(result: ReplayResult) -> str:
    d = result.delivery
    if d is None:
        return "delivery: none"
    path = "-" if d.path_deg is None else f"{d.path_deg:+.1f} deg"
    attack = "-" if d.attack_deg is None else f"{d.attack_deg:+.1f} deg"
    return (
        f"delivery: {d.points} points, speed {d.speed_mps:.1f} m/s (radial {d.radial_speed_mps:.1f}), "
        f"path {path}, attack {attack}, residual {1000 * d.residual_m:.1f} mm, "
        f"confidence {d.confidence:.2f}"
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
        f"residual {1000 * launch.residual_m:.1f} mm, confidence {launch.confidence:.2f}, "
        f"why={launch.angle_why} angles={launch.angles_accepted}"
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
        f"  {result.impact_fit_status}",
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
    "BALL_SNR_MAX",
    "DEFAULT_BALL_SNR",
    "DEFAULT_SNR",
    "EXPECT_KEY",
    "FALLBACK_FRAME_PERIOD_US",
    "AngleSummary",
    "BallTuning",
    "Expectation",
    "post_frame_count",
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
    "freeze_frame",
    "frame_window",
    "recording_configs",
    "recording_expectations",
    "replay_dump",
    "replay_file",
    "vertical_tx_indices",
]
