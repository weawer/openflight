"""Host build of the pure-C firmware decision modules, driven through ctypes.

``firmware/iwr6843/l3_observation.c`` (per-bin residual observations to
ranked targets), ``l3_trigger.c`` (the self-trigger) and ``l3_club_track.c``
(the persistent club trajectory) have no hardware dependencies, so the same
sources the R4F runs are compiled with the host C compiler and called from
Python. The ctypes structures here mirror the C headers field for field; a
layout change on one side without the other reads garbage (or crashes), which
is exactly what the ctypes test suites catch.

The replay harness (``firmware_replay``) and the firmware test suites share
this one binding so a signature or layout change is made in one place.
"""

# ctypes mirrors are data, not behaviour.
# pylint: disable=too-few-public-methods
from __future__ import annotations

import ctypes
import hashlib
import importlib.util
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

FIRMWARE_DIR = Path(__file__).resolve().parents[3] / "firmware" / "iwr6843"
HOST_SOURCES = (
    "l3_text.c",
    "l3_frames.c",
    "l3_angle.c",
    "l3_observation.c",
    "l3_band.c",
    "l3_trigger.c",
    "l3_club_track.c",
    "l3_track_kf.c",
    "l3_impact_fit.c",
    "l3_ball_anchor.c",
    "l3_launch.c",
    "l3_ball_hyp.c",
    "l3_ball_recover.c",
    "l3_impact.c",
    "l3_leave.c",
    "l3_scan.c",
    "l3_shot.c",
    "l3_ball_track.c",
    "l3_ball_fit.c",
    "l3_result.c",
    "l3_profile.c",
    "l3_adaptive.c",
    "l3_iq8.c",
    "l3_retain.c",
    "l3_iq16_stats.c",
    "l3_bin_score.c",
    "l3_channels.c",
    "l3_angle_queue.c",
    "l3_dsp_ipc.c",
    "l3_detect_core.c",
    "l3_timing.c",
    "l3_window.c",
)

# l3_observation.h
OBS_MAX_BINS = 64
BAND_NOISE_BINS = 64
BAND_NOISE_MIN_UPDATES = 8
OBS_MAX_TARGETS = 8
OBS_WAVELENGTH_M = 0.00484
OBS_FLOOR_MIN = 1.0
STAT_ENERGY, STAT_PEAK = 0, 1
STAT_NAMES = {"energy": STAT_ENERGY, "peak": STAT_PEAK}
SUBBIN_CENTROID, SUBBIN_PARABOLIC = 0, 1
SUBBIN_NAMES = {"centroid": SUBBIN_CENTROID, "parabolic": SUBBIN_PARABOLIC}

# l3_trigger.h
TRIG_MAX_BINS = 64
TRIG_TRACE_DEPTH = 64

# l3_frames.h / l3_angle.h
CAL_MAX_VIRTUAL = 8
ANGLE_MAX_TX, ANGLE_MAX_RX = 3, 4
ANGLE_MAX_CHANNELS = ANGLE_MAX_TX * ANGLE_MAX_RX
ANGLE_GRID_STEPS = 161

# l3_observation.h anglesValid bits
ANGLE_AZIMUTH, ANGLE_ELEVATION = 1, 2

# l3_impact.h
IMPACT_WHY_NAMES = ("none", "nodelivery", "pending", "passed", "fired")
IMPACT_CAUSE_NAMES = ("none", "crossing", "end")
LEAVE_CLUB_MIN_POINTS = 2  # l3_leave.h
LEAVE_WHY_NAMES = ("none", "noclub", "idle", "far", "stood", "started", "slow", "fired")

# l3_impact_fit.h
FIT_MAX_POINTS = 8
FIT_NO_TRACK = 0xFF
FIT_TRACK_NAMES = ("club_in", "club_out", "ball_out")
FIT_CLUB_IN, FIT_CLUB_OUT, FIT_BALL_OUT = 0, 1, 2
FIT_WHY_NAMES = (
    "ok",
    "missing",
    "few_points",
    "wrong_direction",
    "speed_bounds",
    "physics",
    "nonfinite",
    "dropped",
    "uncertain",
)
FIT_VERDICT_NAMES = ("none", "single_track", "consistent", "inconsistent")
# The whys whose estimate keeps its time and sigma (l3_impact_fit_format prints them).
_FIT_TIMED_WHYS = frozenset({"ok", "dropped", "uncertain"})


def fit_track_timed(why: str) -> bool:
    """Whether a track estimate with this why carries a time and sigma."""
    return why in _FIT_TIMED_WHYS


def fit_verdict_decided(verdict: str) -> bool:
    """Whether a fit verdict carries an impact time: any known one but none."""
    return verdict in FIT_VERDICT_NAMES and verdict != "none"


_UINT32_WRAP = 2**32


def round_us(us: float) -> int:
    """``l3_round_us``: a float time in microseconds rounded half up to a
    uint32 timestamp. 0 when not finite, not positive or past a second wrap;
    a time in [2**32, 2**33) folds back by 2**32. The fraction is taken after
    truncation, as the C does, so every float32 input rounds identically."""
    if not math.isfinite(us) or not us > 0.0 or us >= 2 * _UINT32_WRAP:
        return 0
    if us >= _UINT32_WRAP:
        us -= _UINT32_WRAP
    whole = int(us)
    return whole + 1 if us - whole >= 0.5 else whole


# l3_shot.h
SHOT_STATE_NAMES = (
    "waiting_for_ball",
    "ready",
    "club_acquire",
    "club_track",
    "impact",
    "ball_track",
    "solve",
    "result",
)
SHOT_IMPACT_GATE, SHOT_IMPACT_GEOMETRY, SHOT_IMPACT_RANGE = 1, 2, 4

# l3_ball_track.h
BALL_TRACK_WHY_NAMES = (
    "none",
    "unarmed",
    "nocandidate",
    "acquired",
    "confirmed",
    "tooslow",
    "toofast",
    "tracked",
    "coasted",
    "lost",
    "searching",
)

# l3_result.h
RESULT_VERSION = 2
RESULT_METRICS = 9
RESULT_V1_PACKET_BYTES = 100
RESULT_PACKET_BYTES = 164
RESULT_METRIC_NAMES = (
    "ball_speed",
    "vertical_launch",
    "horizontal_launch",
    "club_speed",
    "club_path",
    "angle_of_attack",
    "spin_rate",
    "spin_axis",
    "impact_range",
)
RESULT_VERDICT_NAMES = ("invalid", "partial", "valid")
MEAS_VALID, MEAS_MEASURED, MEAS_RADIAL_ONLY, MEAS_IMPLAUSIBLE, MEAS_FALLBACK = 1, 2, 4, 8, 16
QUALITY_FLAGS = {
    "ball_locked": 1,
    "club_track": 2,
    "impact_identified": 4,
    "ball_from_origin": 8,
    "club_continuous": 16,
    "ball_continuous": 32,
    "speeds_plausible": 64,
    "residuals_ok": 128,
    "angles_plausible": 256,
    "smash_plausible": 512,
    "geometric_impact": 1024,
    "impact_uncertain": 2048,  # a warning: the impact fit's tracks disagreed
    # a warning: the ball's launch was slower than the club's approach
    "ball_slower_than_club": 4096,
}

# l3_iq8.h
IQ8_PATH_CPU, IQ8_PATH_EDMA, IQ8_PATH_DUMP = 0, 1, 2
IQ8_PATH_NAMES = {"cpu": IQ8_PATH_CPU, "edma": IQ8_PATH_EDMA, "dump": IQ8_PATH_DUMP}

# l3_retain.h
RETAIN_PRIORITY_NAMES = ("low", "track", "ball", "impact", "spin")
RETAIN_WHY_NAMES = (
    "centred",
    "tee",
    "ball",
    "club",
    "approach",
    "impact",
    "ballsearch",
    "ballfollow",
)

# l3_profile.h
PROFILE_STAGE_NAMES = (
    "residual",
    "trigger",
    "extract",
    "clubtrack",
    "angle",
    "impact",
    "balldetect",
    "balltrack",
    "reconstruct",
    "dspwait",
)

# l3_window.h
WINDOW_MAX_SAMPLES = 256
RANGE_WINDOW_NONE = 0
RANGE_WINDOW_HANN = 1
RANGE_WINDOW_NAMES = ("none", "hann")

# l3_club_track.h
TRACK_POINTS = 32
TRACK_NO_TARGET = 0xFFFFFFFF
TRACK_CANDIDATES = 4
TRACK_CANDIDATE_MAX_GAP_FRAMES = 2
# No approach measured at impact: the fastest club (l3_impact_fit clubMaxMps).
TRACK_FOLLOW_UNKNOWN_APPROACH_MPS = 70.0
# A tentative point is confirmed by the next point this far downrange of it.
TRACK_TENTATIVE_ADVANCE_BINS = 1.0
TRACK_WHY_NAMES = ("none", "acquired", "associated", "coasted", "dropped", "idle", "released")

# l3_ball_hyp.h
BALL_HYP_MAX = 4
BALL_HYP_POINTS = 8
BALL_HYP_NONE = 0xFFFFFFFF

# l3_ball_recover.h
BALL_HISTORY_FRAMES = 24
BALL_HISTORY_TARGETS = 6


class BinObs(ctypes.Structure):
    """One range bin of one frame: ``l3_bin_obs_t``."""

    _fields_ = [
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("r1Re", ctypes.c_float),
        ("r1Im", ctypes.c_float),
    ]


class ObsParams(ctypes.Structure):
    """``l3_obs_params_t``."""

    _fields_ = [
        ("stat", ctypes.c_uint32),
        ("snr", ctypes.c_float),
        ("loopPeriodS", ctypes.c_float),
        ("subBin", ctypes.c_uint32),
    ]


class TargetObs(ctypes.Structure):
    """``l3_target_obs_t``: one extracted target, global sub-bin range."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("timestampUs", ctypes.c_uint32),
        ("peakBin", ctypes.c_uint8),
        ("rangeBin", ctypes.c_float),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("stat", ctypes.c_float),
        ("snr", ctypes.c_float),
        ("coherence", ctypes.c_float),
        ("r1Re", ctypes.c_float),
        ("r1Im", ctypes.c_float),
        ("dopplerPhaseRad", ctypes.c_float),
        ("dopplerAliasMps", ctypes.c_float),
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
        ("anglesValid", ctypes.c_uint8),
        ("confidence", ctypes.c_float),
    ]


class Band(ctypes.Structure):
    """``l3_band_t``: the tee band, both edges inside."""

    _fields_ = [
        ("valid", ctypes.c_uint8),
        ("loBin", ctypes.c_float),
        ("hiBin", ctypes.c_float),
    ]


class BandNoise(ctypes.Structure):
    """``l3_band_noise_t``: per-bin EMA of the trigger statistic on idle frames."""

    _fields_ = [
        ("firstBin", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("updates", ctypes.c_uint32),
        ("avg", ctypes.c_float * BAND_NOISE_BINS),
        ("seen", ctypes.c_uint8 * BAND_NOISE_BINS),
        ("dev", ctypes.c_float * BAND_NOISE_BINS),
    ]


class Vec3(ctypes.Structure):
    """``l3_vec3_t``."""

    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Spherical(ctypes.Structure):
    """``l3_spherical_t``: range, azimuth (right positive), elevation (up positive)."""

    _fields_ = [
        ("rangeM", ctypes.c_float),
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
    ]


class RadarCal(ctypes.Structure):
    """``l3_radar_cal_t``."""

    _fields_ = [
        ("virtualElements", ctypes.c_uint32),
        ("correctionRe", ctypes.c_float * CAL_MAX_VIRTUAL),
        ("correctionIm", ctypes.c_float * CAL_MAX_VIRTUAL),
        ("azimuthOffsetRad", ctypes.c_float),
        ("elevationOffsetRad", ctypes.c_float),
        ("radarPitchRad", ctypes.c_float),
        ("radarYawRad", ctypes.c_float),
        ("radarRollRad", ctypes.c_float),
        ("rangeBiasM", ctypes.c_float),
    ]


class Cpx(ctypes.Structure):
    """``l3_cpx_t``."""

    _fields_ = [("re", ctypes.c_float), ("im", ctypes.c_float)]


class AngleSnapshot(ctypes.Structure):
    """``l3_angle_snapshot_t``: tx-major channel values at one target's bin."""

    _fields_ = [
        ("ntx", ctypes.c_uint32),
        ("nrx", ctypes.c_uint32),
        ("channel", Cpx * ANGLE_MAX_CHANNELS),
        ("lag1PhaseRad", ctypes.c_float),
        ("radialVelocityMps", ctypes.c_float),
        ("chirpPeriodS", ctypes.c_float),
    ]


# l3_angle_queue.h
L3_ANGLE_QUEUE_DEPTH = 12


class AngleJob(ctypes.Structure):
    """``l3_angle_job_t``: a club point's snapshot, keyed by its timestamp."""

    _fields_ = [("timestampUs", ctypes.c_uint32), ("snapshot", AngleSnapshot)]


class AngleQueue(ctypes.Structure):
    """``l3_angle_queue_t``: the club's pending angles."""

    _fields_ = [
        ("jobs", AngleJob * L3_ANGLE_QUEUE_DEPTH),
        ("head", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("queued", ctypes.c_uint32),
        ("done", ctypes.c_uint32),
        ("stale", ctypes.c_uint32),
        ("failed", ctypes.c_uint32),
        ("dropped", ctypes.c_uint32),
    ]


class ChannelFrame(ctypes.Structure):
    """``l3_channel_frame_t``: a detect frame for the per-channel loops."""

    _fields_ = [
        ("base", ctypes.POINTER(ctypes.c_uint8)),
        ("binCount", ctypes.c_uint32),
        ("cb", ctypes.c_uint32),
        ("scale", ctypes.c_float),
        ("ntx", ctypes.c_uint32),
        ("nrx", ctypes.c_uint32),
        ("loops", ctypes.c_uint32),
        ("loopPeriodS", ctypes.c_float),
    ]


class AngleObs(ctypes.Structure):
    """``l3_angle_obs_t``."""

    _fields_ = [
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
        ("azimuthCoherence", ctypes.c_float),
        ("elevationPeakRatio", ctypes.c_float),
        ("chirpPhaseRad", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("azimuthValid", ctypes.c_uint8),
        ("elevationValid", ctypes.c_uint8),
    ]


class TrigCfg(ctypes.Structure):
    """``l3_trig_cfg_t``: the self-trigger front end's watch region and threshold."""

    _fields_ = [
        ("teeBin", ctypes.c_uint32),
        ("snr", ctypes.c_float),
        ("approachBins", ctypes.c_uint32),
        ("pastBins", ctypes.c_uint32),
        ("stat", ctypes.c_uint32),
    ]


class TrigTrace(ctypes.Structure):
    """``l3_trig_trace_t``: the region's strongest bin of one frame."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("gap", ctypes.c_uint16),
        ("bin", ctypes.c_uint8),
        ("dest", ctypes.c_uint8),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("floor", ctypes.c_float),
        ("threshold", ctypes.c_float),
        ("coherencePct", ctypes.c_uint8),
    ]


class Trig(ctypes.Structure):
    """``l3_trig_t``: the self-trigger front end (floor, trace, max-hold)."""

    _fields_ = [
        ("cfg", TrigCfg),
        ("floor", ctypes.c_float),
        ("frames", ctypes.c_uint32),
        ("traceQuiet", ctypes.c_uint32),
        ("traceNext", ctypes.c_uint32),
        ("traceCount", ctypes.c_uint32),
        ("trace", TrigTrace * TRIG_TRACE_DEPTH),
        ("maxFirstBin", ctypes.c_uint32),
        ("maxBins", ctypes.c_uint32),
        ("maxStat", ctypes.c_float * TRIG_MAX_BINS),
        ("maxFrame", ctypes.c_uint32 * TRIG_MAX_BINS),
    ]


# l3_club_track.h L3_FILTER_HYP_*
FILTER_HYP_NAMES = ("none", "direct", "image", "ambiguous", "unfiltered")
FILTER_HYP_UNFILTERED = FILTER_HYP_NAMES.index("unfiltered")


# l3_track_kf.h
TRACK_KF_WHY_NAMES = ("none", "ok", "few_points", "diverged")


class TrackKfResult(ctypes.Structure):
    """``l3_track_kf_result_t``."""

    _fields_ = [
        ("points", ctypes.c_uint32),
        ("accepted", ctypes.c_uint32),
        ("why", ctypes.c_uint8),
    ]


class TrackKfCfg(ctypes.Structure):
    """``l3_track_kf_cfg_t``."""

    _fields_ = [
        ("accelSigmaMps2", ctypes.c_float),
        ("rangeSigmaM", ctypes.c_float),
        ("angleSigmaRad", ctypes.c_float),
        ("minAngleConfidence", ctypes.c_float),
        ("chi2Gate", ctypes.c_float),
        ("initPositionSigmaM", ctypes.c_float),
        ("initVelocitySigmaMps", ctypes.c_float),
    ]


class TrackCfg(ctypes.Structure):
    """``l3_track_cfg_t``."""

    _fields_ = [
        ("binWidthM", ctypes.c_float),
        ("gateBins", ctypes.c_float),
        ("maxMisses", ctypes.c_uint32),
        ("minConfidence", ctypes.c_float),
        ("weightRange", ctypes.c_float),
        ("weightVelocity", ctypes.c_float),
        ("weightQuality", ctypes.c_float),
        ("weightStrength", ctypes.c_float),
        ("velocitySpanMps", ctypes.c_float),
        ("cal", RadarCal),
        ("minAcquireDopplerMps", ctypes.c_float),
        ("maxAngleResidualM", ctypes.c_float),
        ("ascendingOnly", ctypes.c_uint32),
        ("maxSameBinPoints", ctypes.c_uint32),
        ("followDopplerTolMps", ctypes.c_float),
        ("followDopplerRiseMps", ctypes.c_float),
        ("approachMaxSameBinPoints", ctypes.c_uint32),
        ("standingFrames", ctypes.c_uint32),
        ("acquireMinStepBins", ctypes.c_float),
        ("acquireMaxStepBins", ctypes.c_float),
        ("acquireDopplerTolMps", ctypes.c_float),
        ("acquireMinConfidence", ctypes.c_float),
        ("acquireExpectedStepBins", ctypes.c_float),
        ("kf", TrackKfCfg),
    ]


class TrackPoint(ctypes.Structure):
    """``l3_track_point_t``: one held observation of the clubhead."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("timestampUs", ctypes.c_uint32),
        ("rangeBin", ctypes.c_float),
        ("rangeM", ctypes.c_float),
        ("radialVelocityMps", ctypes.c_float),
        ("dopplerAliasMps", ctypes.c_float),
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
        ("anglesValid", ctypes.c_uint8),
        ("energy", ctypes.c_float),
        ("coherence", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("position", Vec3),
        ("angleConfidence", ctypes.c_float),
        ("filteredPosition", Vec3),
        ("filterAccepted", ctypes.c_uint8),
        ("filterHypothesis", ctypes.c_uint8),
    ]


class Delivery(ctypes.Structure):
    """``l3_delivery_t``: the club's velocity vector and the metrics read from it."""

    _fields_ = [
        ("points", ctypes.c_uint32),
        ("azimuthPoints", ctypes.c_uint32),
        ("elevationPoints", ctypes.c_uint32),
        ("velocity", Vec3),
        ("position", Vec3),
        ("timestampUs", ctypes.c_uint32),
        ("speedMps", ctypes.c_float),
        ("radialSpeedMps", ctypes.c_float),
        ("pathRad", ctypes.c_float),
        ("attackRad", ctypes.c_float),
        ("residualM", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("speedValid", ctypes.c_uint8),
        ("pathValid", ctypes.c_uint8),
        ("attackValid", ctypes.c_uint8),
    ]


class TrackHeld(ctypes.Structure):
    """``l3_track_held_t``: the track before its tentative point."""

    _fields_ = [
        ("active", ctypes.c_uint8),
        ("following", ctypes.c_uint8),
        ("misses", ctypes.c_uint32),
        ("lastFrame", ctypes.c_uint32),
        ("lastBin", ctypes.c_float),
        ("velocityBinsPerFrame", ctypes.c_float),
        ("followBinsPerS", ctypes.c_float),
        ("sameBin", ctypes.c_int32),
        ("sameBinCount", ctypes.c_uint32),
    ]


class ClubTrack(ctypes.Structure):
    """``l3_club_track_t``."""

    _fields_ = [
        ("cfg", TrackCfg),
        ("active", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("next", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("total", ctypes.c_uint32),
        ("misses", ctypes.c_uint32),
        ("lastFrame", ctypes.c_uint32),
        ("lastBin", ctypes.c_float),
        ("velocityBinsPerFrame", ctypes.c_float),
        ("predictedBin", ctypes.c_float),
        ("lastTargetIndex", ctypes.c_uint32),
        ("points", TrackPoint * TRACK_POINTS),
        ("counters", ctypes.c_uint32 * len(TRACK_WHY_NAMES)),
        ("sameBin", ctypes.c_int32),
        ("sameBinCount", ctypes.c_uint32),
        ("following", ctypes.c_uint8),
        ("tentative", ctypes.c_uint8),
        ("followBinsPerS", ctypes.c_float),
        ("releasedValid", ctypes.c_uint8),
        ("releasedBin", ctypes.c_float),
        ("releasedDopplerMps", ctypes.c_float),
        ("held", TrackHeld),
        ("standHold", ctypes.c_uint8 * 128),
        ("candidateCount", ctypes.c_uint32),
        ("candidates", TargetObs * TRACK_CANDIDATES),
        ("prevFrame", ctypes.c_uint32),
        ("prevCount", ctypes.c_uint32),
        ("prevBins", ctypes.c_float * OBS_MAX_TARGETS),
    ]


class FollowCtx(ctypes.Structure):
    """``l3_follow_ctx_t``: the scene after impact for ``l3_track_follow``."""

    _fields_ = [
        ("bandValid", ctypes.c_uint8),
        ("bandHiBin", ctypes.c_float),
        ("originBin", ctypes.c_float),
        ("impactTimestampUs", ctypes.c_uint32),
        ("approachBinsPerS", ctypes.c_float),
        ("ballBinsPerS", ctypes.c_float),
        ("ballClaimIndex", ctypes.c_uint32),
        ("frameUs", ctypes.c_uint32),
        ("approachKnown", ctypes.c_uint8),
    ]


class ImpactCfg(ctypes.Structure):
    """``l3_impact_cfg_t``."""

    _fields_ = [("horizonS", ctypes.c_float), ("endM", ctypes.c_float)]


# l3_dsp_ipc.h
L3_DSP_MAGIC = 0x4C445331
L3_DSP_CMD_PING = 1
L3_DSP_CMD_PROBE = 2
L3_DSP_CMD_SCORE = 3
L3_DSP_OK = 0
L3_DSP_ERR_MAGIC = 1
L3_DSP_ERR_CMD = 2
L3_DSP_ERR_GEOMETRY = 3
L3_DSP_ERR_RANGE = 4
L3_DSP_ERR_SPANS = 5
L3_DSP_ERR_STALE = 6
L3_DSP_ERR_GATHER = 7
L3_DSP_MAX_SPANS = 4
L3_DSP_MAX_BINS = 64
# 16 loops, 3 TX, 4 RX, 64 bins of IQ16 (l3_dsp_ipc.h)
L3_DSP_GATHER_MAX_BYTES = 16 * 3 * 4 * L3_DSP_MAX_BINS * 4
L3_DSP_BITMAP_WORDS = L3_DSP_MAX_BINS // 32
L3_DSP_RESULT_MAGIC = 0x4C445352
L3_DSP_RESULT_HSRAM_OFFSET = 0x7400
L3_DSP_FIELD_NAMES = ("energy", "peak", "loop0", "r1Re", "r1Im", "set")
HSRAM_BYTES = 32 * 1024
L3_DSP_STATUS_MAGIC = 0x4C445353
L3_DSP_STATUS_HSRAM_OFFSET = 0x7F00
L3_DSP_STAGE_MAIN = 1
L3_DSP_STAGE_SOC = 2
L3_DSP_STAGE_TASK = 3
L3_DSP_STAGE_MAILBOX = 4
L3_DSP_STAGE_LINK = 5
L3_DSP_STAGE_RESET = 0x10
L3_DSP_STAGE_FIRST = 0x11
L3_DSP_STAGE_LAST = 0x12
L3_DSP_STAGE_EXCEPTION = 0x13
L3_DSP_STAGE_FAILED = 0x80
L3_DSP_GPREG_TAG = 0xD5500000


class DspHw(ctypes.Structure):
    """``l3_dsp_hw_t``: the DSS as the MSS reads it, without the DSS's help."""

    _fields_ = [
        ("gpreg", ctypes.c_uint32),
        ("halt", ctypes.c_uint32),
        ("power", ctypes.c_uint32),
        ("stc", ctypes.c_uint32),
        ("esm", ctypes.c_uint32 * 4),
        ("hsramOk", ctypes.c_uint32),
    ]


class DspStatus(ctypes.Structure):
    """``l3_dsp_status_t``: the DSS's boot status in HS-RAM."""

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("stage", ctypes.c_uint32),
        ("errCode", ctypes.c_int32),
        ("heartbeat", ctypes.c_uint32),
        ("served", ctypes.c_uint32),
        ("excPc", ctypes.c_uint32),
        ("excFlags", ctypes.c_uint32),
    ]


class DspRequest(ctypes.Structure):
    """``l3_dsp_request_t``: an MSS -> DSS detect-link request."""

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("cmd", ctypes.c_uint32),
        ("seq", ctypes.c_uint32),
        ("frameOffset", ctypes.c_uint32),
        ("binCount", ctypes.c_uint32),
        ("firstBin", ctypes.c_uint32),
        ("nBins", ctypes.c_uint32),
        ("ntx", ctypes.c_uint32),
        ("loops", ctypes.c_uint32),
        ("epoch", ctypes.c_uint32),
        ("nSpans", ctypes.c_uint32),
        ("spanFirst", ctypes.c_uint32 * L3_DSP_MAX_SPANS),
        ("spanCount", ctypes.c_uint32 * L3_DSP_MAX_SPANS),
    ]


class DspReply(ctypes.Structure):
    """``l3_dsp_reply_t``: the DSS's answer."""

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("cmd", ctypes.c_uint32),
        ("seq", ctypes.c_uint32),
        ("status", ctypes.c_uint32),
        ("nBins", ctypes.c_uint32),
        ("cycles", ctypes.c_uint32),
        ("energySum", ctypes.c_float),
        ("r1ReSum", ctypes.c_float),
        ("r1ImSum", ctypes.c_float),
        ("prepCycles", ctypes.c_uint32),
        ("gathered", ctypes.c_uint32),
    ]


class Span(ctypes.Structure):
    """``l3_span_t``: global first bin and count."""

    _fields_ = [("first", ctypes.c_uint32), ("count", ctypes.c_uint32)]


class DspResult(ctypes.Structure):
    """``l3_dsp_result_t``: SCORE's observations in HS-RAM."""

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("seq", ctypes.c_uint32),
        ("epoch", ctypes.c_uint32),
        ("status", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("invCycles", ctypes.c_uint32),
        ("scoreCycles", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("scored", ctypes.c_uint32 * L3_DSP_BITMAP_WORDS),
        ("obs", BinObs * L3_DSP_MAX_BINS),
    ]


class DspIq16Ctx(ctypes.Structure):
    """``l3_dsp_iq16_ctx_t``: the shared IQ16 scorer's frame."""

    _fields_ = [
        ("frame", ctypes.POINTER(ctypes.c_int16)),
        ("binCount", ctypes.c_uint32),
        ("ntx", ctypes.c_uint32),
        ("loops", ctypes.c_uint32),
        ("binBase", ctypes.c_uint32),
    ]


class DspGather(ctypes.Structure):
    """``l3_dsp_gather_t``: the window a request reads, copied into L2."""

    _fields_ = [
        ("lo", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("rows", ctypes.c_uint32),
        ("srcOffset", ctypes.c_uint32),
        ("srcStride", ctypes.c_uint32),
        ("rowBytes", ctypes.c_uint32),
        ("bytes", ctypes.c_uint32),
    ]


# l3_dsp_ipc.h: int32_t (*)(void *ctx, uint32_t localBin, l3_bin_obs_t *out)
BinScorer = ctypes.CFUNCTYPE(
    ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(BinObs)
)

# l3_detect_core.h
DETECT_CORE_NAMES = ("mss", "dss", "verify")
DETECT_CORE_MSS, DETECT_CORE_DSS, DETECT_CORE_VERIFY = 0, 1, 2
DETECT_OUTCOME_OK, DETECT_OUTCOME_FAILED, DETECT_OUTCOME_MISMATCH = 0, 1, 2
DETECT_CORE_FAIL_LIMIT_DEFAULT = 3


class DetectCore(ctypes.Structure):
    """``l3_detect_core_t``."""

    _fields_ = [
        ("requested", ctypes.c_uint8),
        ("active", ctypes.c_uint8),
        ("latched", ctypes.c_uint8),
        ("failLimit", ctypes.c_uint8),
        ("failStreak", ctypes.c_uint32),
        ("mssFrames", ctypes.c_uint32),
        ("dssFrames", ctypes.c_uint32),
        ("verifyFrames", ctypes.c_uint32),
        ("ineligible", ctypes.c_uint32),
        ("failures", ctypes.c_uint32),
        ("fallbacks", ctypes.c_uint32),
        ("latches", ctypes.c_uint32),
        ("mismatches", ctypes.c_uint32),
        ("haveMismatch", ctypes.c_uint8),
        ("mismatchSlot", ctypes.c_uint32),
        ("mismatchBin", ctypes.c_uint32),
        ("mismatchField", ctypes.c_uint32),
        ("dssInvCyclesLast", ctypes.c_uint32),
        ("dssInvCyclesMax", ctypes.c_uint32),
        ("dssScoreCyclesLast", ctypes.c_uint32),
        ("dssScoreCyclesMax", ctypes.c_uint32),
    ]


# l3_timing.h
TIMING_TIMELINE_DEPTH = 16
TIMING_STAT_NAMES = ("wait", "score", "service", "latency", "arrival")
TIMING_FLAG_POST, TIMING_FLAG_BEHIND, TIMING_FLAG_STALE = 1, 2, 4
TIMING_FLAG_FIRED, TIMING_FLAG_SCORED = 8, 16
TIMING_CORE_FALLBACK = 3


class TimingEvent(ctypes.Structure):
    """``l3_timing_event_t``: one frame's cycle stamps."""

    _fields_ = [
        ("slot", ctypes.c_uint32),
        ("epoch", ctypes.c_uint32),
        ("acquired", ctypes.c_uint32),
        ("dequeued", ctypes.c_uint32),
        ("scoreStart", ctypes.c_uint32),
        ("scoreEnd", ctypes.c_uint32),
        ("decided", ctypes.c_uint32),
        ("core", ctypes.c_uint8),
        ("flags", ctypes.c_uint8),
        ("depth", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
    ]


class TimingStat(ctypes.Structure):
    """``l3_timing_stat_t``."""

    _fields_ = [
        ("count", ctypes.c_uint32),
        ("lastUs", ctypes.c_uint32),
        ("minUs", ctypes.c_uint32),
        ("maxUs", ctypes.c_uint32),
        ("sumUs", ctypes.c_uint32),
        ("sumOverflow", ctypes.c_uint32),
    ]


class Timing(ctypes.Structure):
    """``l3_timing_t``."""

    _fields_ = [
        ("ticksPerUs", ctypes.c_uint32),
        ("budgetUs", ctypes.c_uint32),
        ("ringFrames", ctypes.c_uint32),
        ("frames", ctypes.c_uint32),
        ("overBudget", ctypes.c_uint32),
        ("depthMax", ctypes.c_uint32),
        ("marginCount", ctypes.c_uint32),
        ("marginLastUs", ctypes.c_int32),
        ("marginMinUs", ctypes.c_int32),
        ("marginNegative", ctypes.c_uint32),
        ("havePrevious", ctypes.c_uint8),
        ("previousEpoch", ctypes.c_uint32),
        ("previousAcquired", ctypes.c_uint32),
        ("stat", TimingStat * len(TIMING_STAT_NAMES)),
        ("timelineNext", ctypes.c_uint32),
        ("timelineCount", ctypes.c_uint32),
        ("timeline", TimingEvent * TIMING_TIMELINE_DEPTH),
    ]


class ScanCfg(ctypes.Structure):
    """``l3_scan_cfg_t``: which range bins a frame scores."""

    _fields_ = [
        ("clubBins", ctypes.c_uint32),
        ("leaveBins", ctypes.c_uint32),
        ("postBins", ctypes.c_uint32),
        ("postBehindBins", ctypes.c_uint32),
        ("postClubBins", ctypes.c_uint32),
        ("mapChunkBins", ctypes.c_uint32),
    ]


class LeaveCfg(ctypes.Structure):
    """``l3_leave_cfg_t``."""

    _fields_ = [
        ("startBins", ctypes.c_float),
        ("minSpeedMps", ctypes.c_float),
        ("maxSpeedMps", ctypes.c_float),
        ("binWidthM", ctypes.c_float),
        ("snr", ctypes.c_float),
        ("clubHoldFrames", ctypes.c_uint32),
        ("clubNearBins", ctypes.c_float),
        ("newBins", ctypes.c_float),
    ]


class Leave(ctypes.Structure):
    """``l3_leave_t``: the ball-leave fallback."""

    _fields_ = [
        ("cfg", LeaveCfg),
        ("fired", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("started", ctypes.c_uint8),
        ("clubHold", ctypes.c_uint32),
        ("prevCount", ctypes.c_uint32),
        ("prevBins", ctypes.c_float * OBS_MAX_TARGETS),
        ("startBin", ctypes.c_float),
        ("startUs", ctypes.c_uint32),
        ("speedMps", ctypes.c_float),
        ("impactTimestampUs", ctypes.c_uint32),
        ("startTarget", TargetObs),
        ("stepTarget", TargetObs),
        ("counters", ctypes.c_uint32 * len(LEAVE_WHY_NAMES)),
    ]


class ImpactClub(ctypes.Structure):
    """``l3_impact_club_t``: the club track this frame, for the approach-end rule."""

    _fields_ = [
        ("appended", ctypes.c_uint8),
        ("rangeM", ctypes.c_float),
        ("timeUs", ctypes.c_uint32),
        ("ballRangeM", ctypes.c_float),
    ]


class Impact(ctypes.Structure):
    """``l3_impact_t``: the range-only impact, the self-trigger."""

    _fields_ = [
        ("cfg", ImpactCfg),
        ("fired", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("cause", ctypes.c_uint8),
        ("endArmed", ctypes.c_uint8),
        ("endTimeUs", ctypes.c_uint32),
        ("offsetS", ctypes.c_float),
        ("impactTimestampUs", ctypes.c_uint32),
        ("counters", ctypes.c_uint32 * len(IMPACT_WHY_NAMES)),
    ]


class ImpactFitCfg(ctypes.Structure):
    """``l3_impact_fit_cfg_t``."""

    _fields_ = [
        ("binWidthM", ctypes.c_float),
        ("bandBins", ctypes.c_float),
        ("fitPoints", ctypes.c_uint32),
        ("minPoints", ctypes.c_uint32),
        ("clubMinMps", ctypes.c_float),
        ("clubMaxMps", ctypes.c_float),
        ("clubOutMaxRatio", ctypes.c_float),
        ("ballMinMps", ctypes.c_float),
        ("ballMaxMps", ctypes.c_float),
        ("gateSigmas", ctypes.c_float),
        ("minSigmaUs", ctypes.c_float),
        ("maxSigmaUs", ctypes.c_float),
        ("bandSearchBins", ctypes.c_float),
        ("clutterSigmas", ctypes.c_float),
    ]


class FitEstimate(ctypes.Structure):
    """``l3_fit_estimate_t``: one track's impact estimate."""

    _fields_ = [
        ("why", ctypes.c_uint8),
        ("points", ctypes.c_uint32),
        ("timeUs", ctypes.c_float),
        ("sigmaUs", ctypes.c_float),
        ("speedMps", ctypes.c_float),
    ]


class ImpactFit(ctypes.Structure):
    """``l3_impact_fit_t``: the three estimates, fused."""

    _fields_ = [
        ("track", FitEstimate * len(FIT_TRACK_NAMES)),
        ("verdict", ctypes.c_uint8),
        ("droppedTrack", ctypes.c_uint8),
        ("noLock", ctypes.c_uint8),
        ("impactUs", ctypes.c_float),
        ("spreadUs", ctypes.c_float),
        ("refinedMinusTriggerUs", ctypes.c_float),
    ]


class FitList(ctypes.Structure):
    """``l3_fit_list_t``: an array of points."""

    _fields_ = [("points", ctypes.POINTER(TrackPoint)), ("count", ctypes.c_uint32)]


class FitSpan(ctypes.Structure):
    """``l3_fit_span_t``: a run of a track's held points."""

    _fields_ = [
        ("track", ctypes.POINTER(ClubTrack)),
        ("first", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
    ]


class ShotCfg(ctypes.Structure):
    """``l3_shot_cfg_t``."""

    _fields_ = [("requireBall", ctypes.c_uint8), ("ballTrackFrames", ctypes.c_uint32)]


class ShotInput(ctypes.Structure):
    """``l3_shot_input_t``: what the machine reads each frame."""

    _fields_ = [
        ("ballLocked", ctypes.c_uint8),
        ("ballPosition", Vec3),
        ("clubActive", ctypes.c_uint8),
        ("clubPoints", ctypes.c_uint32),
        ("impactTimestampUs", ctypes.c_uint32),
        ("delivery", ctypes.POINTER(Delivery)),
        ("club", ctypes.POINTER(ClubTrack)),
        ("postFrame", ctypes.c_uint8),
        ("ballTrackDone", ctypes.c_uint8),
        ("solved", ctypes.c_uint8),
        ("rangeFired", ctypes.c_uint8),
    ]


class Shot(ctypes.Structure):
    """``l3_shot_t``: the shot state machine and its frozen impact record."""

    _fields_ = [
        ("cfg", ShotCfg),
        ("state", ctypes.c_uint8),
        ("previous", ctypes.c_uint8),
        ("enteredFrame", ctypes.c_uint32),
        ("transitions", ctypes.c_uint32),
        ("postFrames", ctypes.c_uint32),
        ("impactSource", ctypes.c_uint8),
        ("impactFrame", ctypes.c_uint32),
        ("impactTimestampUs", ctypes.c_uint32),
        ("ballOrigin", Vec3),
        ("delivery", Delivery),
        ("clubPoints", ctypes.c_uint32),
        ("clubTrajectory", TrackPoint * TRACK_POINTS),
        ("entries", ctypes.c_uint32 * len(SHOT_STATE_NAMES)),
    ]


BALL_ANCHOR_SOURCE_NAMES = ("gate", "club")


class BallAnchor(ctypes.Structure):
    """``l3_ball_anchor_t``: where and when the ball was struck."""

    _fields_ = [
        ("anchorBin", ctypes.c_float),
        ("acceptFromBin", ctypes.c_float),
        ("gateUs", ctypes.c_uint32),
        ("anchorUs", ctypes.c_uint32),
        ("anchorTolUs", ctypes.c_uint32),
        ("anchorSigmaUs", ctypes.c_float),
        ("source", ctypes.c_uint8),
    ]


class BallHypPoint(ctypes.Structure):
    """``l3_ball_hyp_point_t``."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("timestampUs", ctypes.c_uint32),
        ("rangeBin", ctypes.c_float),
        ("dopplerAliasMps", ctypes.c_float),
        ("stat", ctypes.c_float),
        ("clubStat", ctypes.c_float),
        ("coherence", ctypes.c_float),
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
        ("anglesValid", ctypes.c_uint8),
        ("angleConfidence", ctypes.c_float),
    ]


class BallHyp(ctypes.Structure):
    """``l3_ball_hyp_t``: one candidate ball trajectory."""

    _fields_ = [
        ("active", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("misses", ctypes.c_uint8),
        ("id", ctypes.c_uint32),
        ("lastTargetIndex", ctypes.c_uint32),
        ("points", BallHypPoint * BALL_HYP_POINTS),
    ]


class BallHistoryTarget(ctypes.Structure):
    """``l3_ball_history_target_t``."""

    _fields_ = [
        ("rangeBin", ctypes.c_float),
        ("dopplerAliasMps", ctypes.c_float),
        ("stat", ctypes.c_float),
        ("coherence", ctypes.c_float),
    ]


class BallHistoryFrame(ctypes.Structure):
    """``l3_ball_history_frame_t``."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("timestampUs", ctypes.c_uint32),
        ("count", ctypes.c_uint8),
        ("clubMask", ctypes.c_uint8),
        ("targets", BallHistoryTarget * BALL_HISTORY_TARGETS),
    ]


class BallHistory(ctypes.Structure):
    """``l3_ball_history_t``: the post-impact target ring."""

    _fields_ = [
        ("next", ctypes.c_uint32),
        ("count", ctypes.c_uint32),
        ("frames", BallHistoryFrame * BALL_HISTORY_FRAMES),
    ]


class BallRecoverCfg(ctypes.Structure):
    """``l3_ball_recover_cfg_t``."""

    _fields_ = [
        ("binWidthM", ctypes.c_float),
        ("velocitySpanMps", ctypes.c_float),
        ("gateM", ctypes.c_float),
        ("tieBins", ctypes.c_float),
        ("maxResidualBins", ctypes.c_float),
        ("dopplerToleranceMps", ctypes.c_float),
    ]


class BallRecoverResult(ctypes.Structure):
    """``l3_ball_recover_result_t``."""

    _fields_ = [
        ("count", ctypes.c_uint32),
        ("recovered", ctypes.c_uint32),
        ("firstFrame", ctypes.c_uint32),
        ("mask", ctypes.c_uint32),
        ("residualBins", ctypes.c_float),
    ]


class BallHypsCfg(ctypes.Structure):
    """``l3_ball_hyps_cfg_t``."""

    _fields_ = [
        ("binWidthM", ctypes.c_float),
        ("velocitySpanMps", ctypes.c_float),
        ("spawnBehindM", ctypes.c_float),
        ("spawnBeyondM", ctypes.c_float),
        ("gateM", ctypes.c_float),
        ("gateMps", ctypes.c_float),
        ("coastUs", ctypes.c_uint32),
        ("impactCoastUs", ctypes.c_uint32),
        ("impactRegionM", ctypes.c_float),
        ("classifyPoints", ctypes.c_uint32),
        ("minDepartureMps", ctypes.c_float),
        ("maxSpeedMps", ctypes.c_float),
        ("maxResidualBins", ctypes.c_float),
        ("dopplerToleranceMps", ctypes.c_float),
        ("fastBallMps", ctypes.c_float),
        ("fastSupportFraction", ctypes.c_float),
        ("farWindowM", ctypes.c_float),
        ("corridorGate", ctypes.c_uint32),
        ("anchorRangeTolM", ctypes.c_float),
        ("maxDecelMps2", ctypes.c_float),
        ("rangeNoiseM", ctypes.c_float),
        ("wBack", ctypes.c_float),
        ("wVel", ctypes.c_float),
        ("wResid", ctypes.c_float),
        ("wDoppler", ctypes.c_float),
        ("wCoherence", ctypes.c_float),
        ("wWeaker", ctypes.c_float),
    ]


class BallHypVerdict(ctypes.Structure):
    """``l3_ball_hyp_verdict_t``."""

    _fields_ = [
        ("index", ctypes.c_int32),
        ("points", ctypes.c_uint32),
        ("rateMps", ctypes.c_float),
        ("originOffsetUs", ctypes.c_float),
        ("residualBins", ctypes.c_float),
        ("dopplerAgreement", ctypes.c_float),
        ("weakerFraction", ctypes.c_float),
        ("score", ctypes.c_float),
        ("waitingForFast", ctypes.c_uint32),
        ("velocityConsistency", ctypes.c_float),
        ("coherence", ctypes.c_float),
        ("anchorSource", ctypes.c_uint8),
        ("recovered", ctypes.c_uint32),
        ("recoveredFirstFrame", ctypes.c_uint32),
        ("recoveredMask", ctypes.c_uint32),
    ]


class BallHyps(ctypes.Structure):
    """``l3_ball_hyps_t``: the bounded set of candidate ball trajectories."""

    _fields_ = [
        ("cfg", BallHypsCfg),
        ("armed", ctypes.c_uint8),
        ("anchor", BallAnchor),
        ("spawnBehindBins", ctypes.c_float),
        ("spawnBeyondBins", ctypes.c_float),
        ("gateBins", ctypes.c_float),
        ("farWindowBins", ctypes.c_float),
        ("impactRegionBins", ctypes.c_float),
        ("nextId", ctypes.c_uint32),
        ("spawned", ctypes.c_uint32),
        ("dropped", ctypes.c_uint32),
        ("hyp", BallHyp * BALL_HYP_MAX),
    ]


class BallFitCfg(ctypes.Structure):
    """``l3_ball_fit_cfg_t``."""

    _fields_ = [
        ("angleSigmaRad", ctypes.c_float),
        ("gateK", ctypes.c_float),
        ("huberK", ctypes.c_float),
        ("minAccepted", ctypes.c_uint32),
        ("maxRmsRad", ctypes.c_float),
        ("imageSepMinRad", ctypes.c_float),
        ("radarHeightM", ctypes.c_float),
        ("teeBallHeightM", ctypes.c_float),
        ("hlaMinRad", ctypes.c_float),
        ("hlaMaxRad", ctypes.c_float),
        ("vlaMinRad", ctypes.c_float),
        ("vlaMaxRad", ctypes.c_float),
        ("gridSteps", ctypes.c_uint32),
        ("gridLevels", ctypes.c_uint32),
        ("maxAngleSigmaRad", ctypes.c_float),
    ]


class BallTrackCfg(ctypes.Structure):
    """``l3_ball_track_cfg_t``."""

    _fields_ = [
        ("core", TrackCfg),
        ("minDepartureMps", ctypes.c_float),
        ("maxSpeedMps", ctypes.c_float),
        ("originGateBins", ctypes.c_float),
        ("minDepartureBins", ctypes.c_float),
        ("displaceConfidence", ctypes.c_float),
        ("launchPoints", ctypes.c_uint32),
        ("snr", ctypes.c_float),
        ("useHypotheses", ctypes.c_uint32),
        ("skipClubClaim", ctypes.c_uint32),
        ("gateTolUs", ctypes.c_uint32),
        ("anchorMaxSigmaUs", ctypes.c_float),
        ("fit", BallFitCfg),
        ("hyps", BallHypsCfg),
        ("recover", ctypes.c_uint32),
        ("historySnr", ctypes.c_float),
        ("rec", BallRecoverCfg),
    ]


class BallTrack(ctypes.Structure):
    """``l3_ball_track_t``: the departing ball over the trajectory core."""

    _fields_ = [
        ("cfg", BallTrackCfg),
        ("core", ClubTrack),
        ("armed", ctypes.c_uint8),
        ("confirmed", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("done", ctypes.c_uint8),
        ("impactTimestampUs", ctypes.c_uint32),
        ("originBin", ctypes.c_float),
        ("origin", Vec3),
        ("anchor", BallAnchor),
        ("lastTargetIndex", ctypes.c_uint32),
        ("counters", ctypes.c_uint32 * len(BALL_TRACK_WHY_NAMES)),
        ("hyps", BallHyps),
        ("verdict", BallHypVerdict),
        ("history", BallHistory),
    ]


class Launch(ctypes.Structure):
    """``l3_launch_t``: ball speed, horizontal and vertical launch at impact."""

    _fields_ = [
        ("points", ctypes.c_uint32),
        ("velocity", Vec3),
        ("launchPosition", Vec3),
        ("speedMps", ctypes.c_float),
        ("radialSpeedMps", ctypes.c_float),
        ("hlaRad", ctypes.c_float),
        ("vlaRad", ctypes.c_float),
        ("residualM", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("angleRmsRad", ctypes.c_float),
        ("speedValid", ctypes.c_uint8),
        ("hlaValid", ctypes.c_uint8),
        ("vlaValid", ctypes.c_uint8),
        ("anglesAccepted", ctypes.c_uint8),
        ("angleWhy", ctypes.c_uint8),
    ]


class Measurement(ctypes.Structure):
    """``l3_measurement_t``: value, confidence, flags."""

    _fields_ = [
        ("value", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("flags", ctypes.c_uint32),
    ]


class ShotResult(ctypes.Structure):
    """``l3_shot_result_t`` (the in-memory form; the packet is serialised byte by byte)."""

    _fields_ = [
        ("version", ctypes.c_uint32),
        ("shotId", ctypes.c_uint32),
        ("metric", Measurement * RESULT_METRICS),
        ("validFlags", ctypes.c_uint32),
        ("qualityFlags", ctypes.c_uint32),
        ("impactTimestampUs", ctypes.c_uint32),
        ("verdict", ctypes.c_uint8),
        ("impactSource", ctypes.c_uint8),
        ("clubPoints", ctypes.c_uint8),
        ("ballPoints", ctypes.c_uint8),
        ("smash", ctypes.c_float),
        ("impactFit", ImpactFit),
    ]


class ProfileStage(ctypes.Structure):
    """``l3_profile_stage_t``."""

    _fields_ = [
        ("count", ctypes.c_uint32),
        ("lastTicks", ctypes.c_uint32),
        ("maxTicks", ctypes.c_uint32),
        ("sumTicks", ctypes.c_uint32),
        ("sumOverflow", ctypes.c_uint32),
    ]


class Profile(ctypes.Structure):
    """``l3_profile_t``."""

    _fields_ = [
        ("ticksPerUs", ctypes.c_uint32),
        ("frames", ctypes.c_uint32),
        ("stage", ProfileStage * len(PROFILE_STAGE_NAMES)),
    ]


class AdaptiveCfg(ctypes.Structure):
    """``l3_adaptive_cfg_t``."""

    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("approachBins", ctypes.c_uint8),
        ("marginBins", ctypes.c_uint8),
    ]


class Iq8Mode(ctypes.Structure):
    """l3_iq8_mode_t"""

    _fields_ = [
        ("path", ctypes.c_uint8),
        ("hwaShift", ctypes.c_uint8),
        ("hwaRounding", ctypes.c_uint8),
        ("sparseStride", ctypes.c_uint8),
    ]


class Iq16ChannelStats(ctypes.Structure):
    """l3_iq16_channel_stats_t"""

    _fields_ = [
        ("loops", ctypes.c_uint32),
        ("sumIm", ctypes.c_int32),
        ("sumRe", ctypes.c_int32),
        ("energy", ctypes.c_int64),
        ("loopPower", ctypes.c_int64 * 16),
        ("r1Re", ctypes.c_int64),
        ("r1Im", ctypes.c_int64),
    ]


class Iq16BinStats(ctypes.Structure):
    """l3_iq16_bin_stats_t"""

    _fields_ = [
        ("loops", ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("energy", ctypes.c_int64),
        ("loopPower", ctypes.c_int64 * 16),
        ("r1Re", ctypes.c_int64),
        ("r1Im", ctypes.c_int64),
    ]


class Roi(ctypes.Structure):
    """l3_roi_t"""

    _fields_ = [
        ("processStart", ctypes.c_uint8),
        ("processBins", ctypes.c_uint8),
        ("retainStart", ctypes.c_uint8),
        ("retainBins", ctypes.c_uint8),
    ]


class RetainCfg(ctypes.Structure):
    """l3_retain_cfg_t"""

    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("approachBins", ctypes.c_uint8),
        ("approachMarginBins", ctypes.c_uint8),
        ("impactBiasBins", ctypes.c_uint8),
        ("ballSearchLeadBins", ctypes.c_uint8),
        ("ballFollowLeadBins", ctypes.c_uint8),
        ("spinFrames", ctypes.c_uint8),
    ]


class RetainState(ctypes.Structure):
    """l3_retain_state_t"""

    _fields_ = [
        ("shotState", ctypes.c_uint8),
        ("ballLocked", ctypes.c_uint8),
        ("ballBin", ctypes.c_float),
        ("clubActive", ctypes.c_uint8),
        ("clubBin", ctypes.c_float),
        ("postFrame", ctypes.c_uint8),
        ("postIndex", ctypes.c_uint32),
        ("ballTrackConfirmed", ctypes.c_uint8),
        ("ballTrackBin", ctypes.c_float),
    ]


class RetainWindow(ctypes.Structure):
    """l3_retain_window_t"""

    _fields_ = [
        ("start", ctypes.c_uint8),
        ("bins", ctypes.c_uint8),
        ("priority", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
    ]


class FrameDesc(ctypes.Structure):
    """l3_frame_desc_t"""

    _fields_ = [
        ("timestampUs", ctypes.c_uint32),
        ("dataOffset", ctypes.c_uint32),
        ("bytes", ctypes.c_uint32),
        ("frame", ctypes.c_uint16),
        ("globalBinStart", ctypes.c_uint8),
        ("binCount", ctypes.c_uint8),
        ("processStart", ctypes.c_uint8),
        ("processBins", ctypes.c_uint8),
        ("shotState", ctypes.c_uint8),
        ("priority", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("isPost", ctypes.c_uint8),
    ]


class RetainRequest(ctypes.Structure):
    """l3_retain_request_t"""

    _fields_ = [
        ("bytesPerBin", ctypes.c_uint32),
        ("capacityBytes", ctypes.c_uint32),
        ("maxFrames", ctypes.c_uint32),
        ("preBins", ctypes.c_uint8),
        ("impactBins", ctypes.c_uint8),
        ("ballBins", ctypes.c_uint8),
        ("preFrames", ctypes.c_uint8),
        ("impactFrames", ctypes.c_uint8),
        ("ballFrames", ctypes.c_uint8),
    ]


class RetainBudget(ctypes.Structure):
    """l3_retain_budget_t"""

    _fields_ = [
        ("preFrames", ctypes.c_uint8),
        ("impactFrames", ctypes.c_uint8),
        ("ballFrames", ctypes.c_uint8),
        ("cutPre", ctypes.c_uint8),
        ("cutBall", ctypes.c_uint8),
        ("usedBytes", ctypes.c_uint32),
        ("freeBytes", ctypes.c_uint32),
    ]


class AdaptiveWindows(ctypes.Structure):
    """``l3_adaptive_windows_t``."""

    _fields_ = [
        ("preStart", ctypes.c_uint8),
        ("impactStart", ctypes.c_uint8),
        ("postStart", ctypes.c_uint8),
        ("lateStart", ctypes.c_uint8),
    ]


# l3_ball_fit.h
BALL_FIT_WHY_NAMES = ("none", "ok", "few_angles", "scatter", "grid_edge", "no_tee", "uncertain")


class BallFit(ctypes.Structure):
    """``l3_ball_fit_t``."""

    _fields_ = [
        ("hlaRad", ctypes.c_float),
        ("vlaRad", ctypes.c_float),
        ("rmsRad", ctypes.c_float),
        ("hlaSigmaRad", ctypes.c_float),
        ("vlaSigmaRad", ctypes.c_float),
        ("tee", Vec3),
        ("used", ctypes.c_uint32),
        ("accepted", ctypes.c_uint32),
        ("evaluations", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
    ]


_U32 = ctypes.c_uint32
_F32 = ctypes.c_float
_P = ctypes.POINTER
_TEXT = (ctypes.c_char_p, _U32)

# name -> (argtypes, restype); None restype is the C void.
_SIGNATURES: dict[str, tuple[list, object]] = {
    # l3_bin_score.h
    "l3_bin_score_iq16": (
        [_P(ctypes.c_int16), _U32, _U32, _U32, _U32, _U32, _P(BinObs), _P(ctypes.c_float)],
        ctypes.c_int32,
    ),
    # l3_dsp_ipc.h
    "l3_dsp_request_size": ([], _U32),
    "l3_dsp_reply_size": ([], _U32),
    "l3_dsp_status_size": ([], _U32),
    "l3_dsp_hw_size": ([], _U32),
    "l3_dsp_hw_format": ([_P(DspHw), *_TEXT], ctypes.c_int32),
    "l3_dsp_status_format": ([_P(DspStatus), *_TEXT], ctypes.c_int32),
    "l3_dsp_frame_bytes": ([_U32, _U32, _U32, _U32], _U32),
    "l3_dsp_request_check": ([_P(DspRequest), _U32], _U32),
    "l3_dsp_probe_run": (
        [_P(DspRequest), _P(ctypes.c_uint8), _U32, _P(DspReply)],
        None,
    ),
    "l3_dsp_result_size": ([], _U32),
    "l3_dsp_iq16_scorer": ([ctypes.c_void_p, _U32, _P(BinObs)], ctypes.c_int32),
    "l3_dsp_spans_localize": ([_U32, _U32, _P(Span), _U32, _P(Span)], _U32),
    "l3_dsp_spans_score": (
        [_P(Span), _U32, _U32, BinScorer, ctypes.c_void_p, _P(BinObs), _P(_U32)],
        _U32,
    ),
    "l3_dsp_gather_size": ([], _U32),
    "l3_dsp_gather_plan": ([_P(DspRequest), _U32, _U32, _P(DspGather)], _U32),
    "l3_dsp_gather_copy": ([_P(ctypes.c_uint8), _P(DspGather), _P(ctypes.c_uint8)], None),
    "l3_dsp_serve_gathered": (
        [_P(DspRequest), _P(DspGather), _P(ctypes.c_uint8), _P(DspReply), _P(DspResult)],
        None,
    ),
    "l3_dsp_serve": (
        [_P(DspRequest), _P(ctypes.c_uint8), _U32, _P(DspReply), _P(DspResult)],
        None,
    ),
    "l3_dsp_result_check": ([_P(DspResult), _U32, _U32], _U32),
    "l3_dsp_result_merge": ([_P(DspResult), _P(BinObs), _P(_U32)], _U32),
    "l3_dsp_result_compare": (
        [_P(DspResult), _P(BinObs), _P(_U32), _P(_U32), _P(_U32)],
        ctypes.c_int32,
    ),
    # l3_detect_core.h
    "l3_detect_core_init": ([_P(DetectCore)], None),
    "l3_detect_core_reset_counts": ([_P(DetectCore)], None),
    "l3_detect_core_set": ([_P(DetectCore), _U32, ctypes.c_uint8], ctypes.c_int32),
    "l3_detect_core_route": ([_P(DetectCore), ctypes.c_uint8], _U32),
    "l3_detect_core_report": ([_P(DetectCore), _U32, _U32, _U32, _U32], ctypes.c_int32),
    "l3_detect_core_note_mismatch": ([_P(DetectCore), _U32, _U32, _U32], None),
    "l3_detect_core_name": ([_U32], ctypes.c_char_p),
    "l3_detect_core_parse": ([ctypes.c_char_p, _P(_U32)], ctypes.c_int32),
    "l3_detect_core_format": ([_P(DetectCore), _U32, *_TEXT], ctypes.c_int32),
    # l3_timing.h
    "l3_timing_init": ([_P(Timing), _U32, _U32, _U32], None),
    "l3_timing_reset": ([_P(Timing)], None),
    "l3_timing_record": ([_P(Timing), _P(TimingEvent)], None),
    "l3_timing_margin_us": ([_P(Timing), _U32], ctypes.c_int32),
    "l3_timing_mean_us": ([_P(Timing), _U32], _U32),
    "l3_timing_stat_name": ([_U32], ctypes.c_char_p),
    "l3_timing_event": ([_P(Timing), _U32, _P(TimingEvent)], ctypes.c_int32),
    "l3_timing_format_summary": ([_P(Timing), *_TEXT], ctypes.c_int32),
    "l3_timing_format_stat": ([_P(Timing), _U32, *_TEXT], ctypes.c_int32),
    "l3_timing_format_event": ([_P(Timing), _P(TimingEvent), *_TEXT], ctypes.c_int32),
    # l3_observation.h
    "l3_obs_stat": ([_U32, _P(BinObs)], _F32),
    "l3_obs_median": ([_U32, _P(BinObs), _U32], _F32),
    "l3_obs_floor_update": ([_P(_F32), _U32, _P(BinObs), _U32, _U32], None),
    "l3_obs_velocity": ([_F32, _F32, _F32], _F32),
    "l3_obs_extract": (
        [_P(ObsParams), _U32, _U32, _U32, _P(BinObs), _U32, _F32, _P(TargetObs), _U32],
        _U32,
    ),
    "l3_obs_format_target": ([_P(TargetObs), *_TEXT], ctypes.c_int32),
    "l3_obs_parabolic_offset": ([_F32, _F32, _F32], _F32),
    # l3_band.h
    "l3_band_noise_reset": ([_P(BandNoise)], None),
    "l3_band_noise_update": ([_P(BandNoise), _U32, _U32, _P(BinObs), _U32], None),
    "l3_band_place": ([_P(BandNoise), _F32, _F32, _F32, _P(Band)], None),
    "l3_band_contains": ([_P(Band), _F32], ctypes.c_int32),
    "l3_band_filter": ([_P(Band), _P(TargetObs), _U32], _U32),
    "l3_band_keep_short": ([_P(Band), _P(TargetObs), _U32], _U32),
    "l3_band_clutter_filter": ([_P(BandNoise), _F32, _P(TargetObs), _U32], _U32),
    # l3_window.h
    "l3_window_hann_q17": ([_P(ctypes.c_int32), _U32], _U32),
    "l3_window_parse": ([ctypes.c_char_p, _P(ctypes.c_uint8)], ctypes.c_int32),
    "l3_window_name": ([ctypes.c_uint8], ctypes.c_char_p),
    # l3_iq16_stats.h
    "l3_iq16_channel_stats": (
        [_P(ctypes.c_int16), _U32, _U32, _P(Iq16ChannelStats)],
        ctypes.c_int32,
    ),
    "l3_iq16_bin_stats_init": ([_P(Iq16BinStats), _U32], None),
    "l3_iq16_bin_stats_add": ([_P(Iq16BinStats), _P(Iq16ChannelStats)], None),
    "l3_iq16_bin_stats_finish": (
        [_P(Iq16BinStats), _P(_F32), _P(_F32), _P(_F32), _P(_F32), _P(_F32), _P(_F32)],
        None,
    ),
    # l3_frames.h
    "l3_cal_identity": ([_P(RadarCal), _U32], None),
    "l3_frames_from_spherical": ([_P(Spherical), _P(Vec3)], None),
    "l3_frames_to_spherical": ([_P(Vec3), _P(Spherical)], None),
    "l3_frames_radar_to_golf": ([_P(RadarCal), _P(Vec3), _P(Vec3)], None),
    "l3_frames_golf_to_radar": ([_P(RadarCal), _P(Vec3), _P(Vec3)], None),
    "l3_frames_observe": ([_P(RadarCal), _F32, _F32, _F32, _P(Vec3)], None),
    "l3_frames_horizontal_rad": ([_P(Vec3)], _F32),
    "l3_frames_vertical_rad": ([_P(Vec3)], _F32),
    "l3_frames_speed": ([_P(Vec3)], _F32),
    # l3_angle.h
    "l3_angle_snapshot_init": ([_P(AngleSnapshot), _U32, _U32], None),
    "l3_angle_motion_phase": ([_F32, _F32], _F32),
    "l3_angle_chirp_phase": ([_F32, _U32, _F32, _F32], _F32),
    # l3_angle_queue.h
    "l3_angle_queue_size": ([], _U32),
    "l3_angle_job_size": ([], _U32),
    "l3_angle_queue_init": ([_P(AngleQueue)], None),
    "l3_angle_queue_push": ([_P(AngleQueue), _U32, _P(AngleSnapshot)], ctypes.c_int32),
    "l3_angle_queue_pop": ([_P(AngleQueue), _P(AngleJob)], ctypes.c_int32),
    "l3_angle_queue_pending": ([_P(AngleQueue)], _U32),
    "l3_angle_queue_peek": ([_P(AngleQueue), _P(AngleJob)], ctypes.c_int32),
    "l3_angle_queue_finish": (
        [_P(AngleQueue), _P(AngleJob), ctypes.c_int32, _P(AngleObs), _P(ClubTrack)],
        ctypes.c_int32,
    ),
    "l3_angle_queue_apply": (
        [_P(AngleQueue), _P(RadarCal), _P(AngleJob), _P(ClubTrack), _P(AngleObs)],
        ctypes.c_int32,
    ),
    "l3_track_find_point": ([_P(ClubTrack), _U32, _P(ctypes.c_uint32)], ctypes.c_int32),
    # l3_channels.h
    "l3_channels_valid": ([_P(ChannelFrame), _U32], ctypes.c_int32),
    "l3_channels_residual": (
        [_P(ChannelFrame), _U32, _P(ctypes.c_float), _P(BinObs)],
        None,
    ),
    "l3_channels_static_power": ([_P(ChannelFrame), _U32, _U32], ctypes.c_float),
    "l3_channels_snapshot": (
        [_P(ChannelFrame), _U32, ctypes.c_float, ctypes.c_float, _P(AngleSnapshot)],
        None,
    ),
    "l3_channels_snapshot_static": ([_P(ChannelFrame), _U32, _P(AngleSnapshot)], None),
    "l3_angle_tables_init": ([], None),
    "l3_angle_steering_rotor": ([_U32, _P(Cpx)], ctypes.c_int32),
    "l3_angle_bartlett": ([_P(Cpx), _U32, _P(_F32)], _F32),
    "l3_angle_estimate": ([_P(RadarCal), _P(AngleSnapshot), _P(AngleObs)], ctypes.c_int32),
    "l3_angle_format": ([_P(AngleObs), *_TEXT], ctypes.c_int32),
    "l3_angle_confidence": ([_F32, _F32, ctypes.c_uint8], _F32),
    "l3_cal_set_element": ([_P(RadarCal), _U32, _F32, _F32], ctypes.c_int32),
    "l3_cal_element": ([_P(RadarCal), _U32, _P(_F32), _P(_F32)], ctypes.c_int32),
    "l3_cal_format": ([_P(RadarCal), *_TEXT], ctypes.c_int32),
    "l3_cal_format_element": ([_P(RadarCal), _U32, *_TEXT], ctypes.c_int32),
    # l3_text.h
    "l3_text_fixed": ([_F32, _U32, *_TEXT], None),
    "l3_text_fixed2": ([_F32, *_TEXT], None),
    "l3_text_degrees2": ([_F32, *_TEXT], None),
    # l3_trigger.h
    "l3_trig_cfg_defaults": ([_P(TrigCfg)], None),
    "l3_trig_cfg_check": ([_P(TrigCfg)], ctypes.c_int32),
    "l3_trig_init": ([_P(Trig), _P(TrigCfg)], None),
    "l3_trig_region": ([_P(TrigCfg), _U32, _U32, _U32, _P(_U32), _P(_U32)], ctypes.c_int32),
    "l3_trig_observe": ([_P(Trig), _U32, _U32, _U32, _P(BinObs), _U32], None),
    "l3_trig_threshold": ([_P(Trig)], _F32),
    "l3_trig_format_summary": ([_P(Trig), *_TEXT], ctypes.c_int32),
    "l3_trig_format_config": ([_P(Trig), *_TEXT], ctypes.c_int32),
    "l3_trig_trace_clear": ([_P(Trig)], None),
    "l3_trig_trace_count": ([_P(Trig)], _U32),
    "l3_trig_trace_get": ([_P(Trig), _U32, _P(TrigTrace)], ctypes.c_int32),
    "l3_trig_format_trace_header": ([_P(Trig), *_TEXT], ctypes.c_int32),
    "l3_trig_format_trace": ([_P(TrigTrace), *_TEXT], ctypes.c_int32),
    "l3_trig_format_maxhold": ([_P(Trig), _U32, _U32, *_TEXT], ctypes.c_int32),
    # l3_club_track.h
    "l3_track_cfg_defaults": ([_P(TrackCfg)], None),
    "l3_track_init": ([_P(ClubTrack), _P(TrackCfg)], None),
    "l3_track_reset": ([_P(ClubTrack)], None),
    "l3_track_update": ([_P(ClubTrack), _P(TargetObs), _U32, _U32, _U32], ctypes.c_int32),
    "l3_track_wrapped_diff": ([ctypes.c_float, ctypes.c_float, ctypes.c_float], ctypes.c_float),
    "l3_track_follow": (
        [_P(ClubTrack), _P(TargetObs), _U32, _U32, _U32, _P(FollowCtx)],
        ctypes.c_int32,
    ),
    "l3_track_recent_rate": ([_P(ClubTrack)], _F32),
    "l3_track_set_angles": ([_P(ClubTrack), _F32, _F32, ctypes.c_uint8, _F32], ctypes.c_int32),
    "l3_track_unfilter_all": ([_P(ClubTrack)], None),
    "l3_track_set_point_angles": (
        [_P(ClubTrack), _U32, _F32, _F32, ctypes.c_uint8, _F32],
        ctypes.c_int32,
    ),
    "l3_track_point": ([_P(ClubTrack), _U32, _P(TrackPoint)], ctypes.c_int32),
    "l3_track_delivery": ([_P(ClubTrack), _U32, _P(Delivery)], _U32),
    "l3_track_delivery_range": ([_P(ClubTrack), _U32, _U32, _U32, _P(Delivery)], _U32),
    "l3_track_format_delivery": ([_P(Delivery), *_TEXT], ctypes.c_int32),
    "l3_track_fit": ([_P(ClubTrack), _U32, _P(_F32), _P(_F32)], _U32),
    "l3_track_speed_mps": ([_P(ClubTrack), _U32], _F32),
    "l3_track_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_track_format_status": ([_P(ClubTrack), _U32, *_TEXT], ctypes.c_int32),
    # l3_impact.h
    "l3_impact_cfg_defaults": ([_P(ImpactCfg)], None),
    "l3_impact_init": ([_P(Impact), _P(ImpactCfg)], None),
    "l3_impact_rearm": ([_P(Impact)], None),
    "l3_impact_update_range": (
        [_P(Impact), _P(FitEstimate), _P(ImpactClub), _U32],
        ctypes.c_int32,
    ),
    "l3_impact_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_impact_cause_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_band_noise_update_span": (
        [_P(BandNoise), _U32, _U32, _U32, _U32, _P(BinObs), _U32],
        None,
    ),
    "l3_scan_cfg_defaults": ([_P(ScanCfg)], None),
    "l3_scan_pre": (
        [_P(ScanCfg), _U32, _U32, _U32, _U32, _P(Band), _P(Span), _P(Span), _P(Span)],
        None,
    ),
    "l3_scan_map_chunk": ([_P(ScanCfg), _U32, _U32, _P(Band), _P(ctypes.c_uint32), _P(Span)], None),
    "l3_scan_post": (
        [_P(ScanCfg), _U32, _U32, _P(Band), ctypes.c_uint8, _F32, ctypes.c_uint8, _F32]
        + [_P(Span), _P(Span)],
        None,
    ),
    "l3_scan_merge": ([Span, Span, _P(Span)], _U32),
    "l3_scan_count": ([_P(Span), _U32], _U32),
    "l3_leave_cfg_defaults": ([_P(LeaveCfg)], None),
    "l3_leave_init": ([_P(Leave), _P(LeaveCfg)], None),
    "l3_leave_rearm": ([_P(Leave)], None),
    "l3_leave_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_leave_club_near": ([_P(LeaveCfg), ctypes.c_uint8, _U32, _F32, _F32], ctypes.c_uint8),
    "l3_leave_targets": (
        [_P(LeaveCfg), _P(ObsParams), _P(BinObs), _U32, _U32, _U32, _U32, _F32]
        + [_P(TargetObs), _U32, _P(ctypes.c_float)],
        _U32,
    ),
    "l3_leave_update": (
        [_P(Leave), _P(TargetObs), _U32, _F32, _F32, ctypes.c_uint8],
        ctypes.c_int32,
    ),
    "l3_leave_format": ([_P(Leave), *_TEXT], ctypes.c_int32),
    "l3_impact_format": ([_P(Impact), *_TEXT], ctypes.c_int32),
    # l3_impact_fit.h
    "l3_impact_fit_cfg_defaults": ([_P(ImpactFitCfg)], None),
    "l3_ball_anchor_make": (
        [_F32, _F32, _U32, _U32, _P(ImpactFitCfg), _P(ClubTrack), _F32, _P(BallAnchor)],
        None,
    ),
    "l3_ball_anchor_source_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_ball_anchor_struct_bytes": ([], _U32),
    "l3_impact_fit_reset": ([_P(ImpactFit)], None),
    "l3_fit_list_point": ([ctypes.c_void_p, _U32, _P(TrackPoint)], ctypes.c_int32),
    "l3_fit_span_point": ([ctypes.c_void_p, _U32, _P(TrackPoint)], ctypes.c_int32),
    "l3_fit_span_after": ([_P(ClubTrack), _U32, _P(FitSpan)], None),
    "l3_impact_fit_track": (
        [
            _P(ImpactFitCfg),
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_void_p,
            _U32,
            _F32,
            _P(FitEstimate),
        ],
        None,
    ),
    "l3_impact_fit_solve": ([_P(ImpactFitCfg), _P(ImpactFit), _U32], None),
    "l3_impact_fit_run": (
        [
            _P(ImpactFitCfg),
            _P(FitList),
            _P(FitSpan),
            _P(FitSpan),
            _F32,
            ctypes.c_uint8,
            _U32,
            _P(ImpactFit),
        ],
        None,
    ),
    "l3_round_us": ([_F32], _U32),
    "l3_impact_fit_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_impact_fit_verdict_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_impact_fit_format": ([_P(ImpactFit), *_TEXT], ctypes.c_int32),
    # l3_launch.h
    "l3_launch_from_delivery": ([_P(Delivery), _U32, _P(Launch)], None),
    "l3_track_append_point": ([_P(ClubTrack), _P(TrackPoint)], None),
    # l3_track_kf.h
    "l3_track_kf_cfg_defaults": ([_P(TrackKfCfg)], None),
    "l3_track_kf_work_bytes": ([], _U32),
    "l3_track_kf_run": ([_P(TrackKfCfg), _P(ClubTrack), ctypes.c_void_p, _P(TrackKfResult)], _U32),
    "l3_track_delivery_filtered": ([_P(ClubTrack), _U32, _P(Delivery)], _U32),
    "l3_track_kf_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    # l3_ball_track.h
    "l3_ball_hyps_cfg_defaults": ([_P(BallHypsCfg)], None),
    "l3_ball_hyps_init": ([_P(BallHyps), _P(BallHypsCfg)], None),
    "l3_ball_hyps_arm": ([_P(BallHyps), _P(BallAnchor)], None),
    "l3_ball_hyps_update": ([_P(BallHyps), _P(TargetObs), _U32, _U32, _U32, _U32], _U32),
    "l3_ball_hyp_fit": (
        [_P(BallHyp), _U32, _P(ctypes.c_float), _P(ctypes.c_float), _P(ctypes.c_float)],
        ctypes.c_int32,
    ),
    "l3_ball_points_fit": (
        [
            _P(BallHypPoint),
            _U32,
            _U32,
            _P(ctypes.c_float),
            _P(ctypes.c_float),
            _P(ctypes.c_float),
        ],
        ctypes.c_int32,
    ),
    "l3_ball_hyps_set_angles": (
        [_P(BallHyps), _U32, ctypes.c_float, ctypes.c_float, ctypes.c_uint8, ctypes.c_float],
        ctypes.c_int32,
    ),
    "l3_ball_hyps_struct_bytes": ([], _U32),
    "l3_ball_hyps_classify": ([_P(BallHyps), _P(BallHypVerdict)], None),
    # l3_ball_recover.h
    "l3_ball_recover_cfg_defaults": ([_P(BallRecoverCfg)], None),
    "l3_ball_history_reset": ([_P(BallHistory)], None),
    "l3_ball_history_push": ([_P(BallHistory), _P(TargetObs), _U32, _U32, _U32, _U32], None),
    "l3_ball_history_mark_club": ([_P(BallHistory), _U32, _U32], None),
    "l3_ball_history_at": ([_P(BallHistory), _U32], _P(BallHistoryFrame)),
    "l3_ball_recover": (
        [
            _P(BallRecoverCfg),
            _P(BallHistory),
            _P(BallHyp),
            ctypes.c_float,
            _P(BallHypPoint),
            _U32,
            _P(BallRecoverResult),
        ],
        _U32,
    ),
    "l3_ball_history_struct_bytes": ([], _U32),
    # l3_ball_fit.h
    "l3_ball_fit_cfg_defaults": ([_P(BallFitCfg)], None),
    "l3_ball_fit_max_evaluations": ([_P(BallFitCfg)], _U32),
    "l3_ball_fit_direction": ([_F32, _F32, _P(Vec3)], None),
    "l3_ball_fit_run": ([_P(BallFitCfg), _P(Vec3), _P(ClubTrack), _P(BallFit)], _U32),
    "l3_ball_fit_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_ball_track_cfg_defaults": ([_P(BallTrackCfg)], None),
    "l3_ball_track_init": ([_P(BallTrack), _P(BallTrackCfg)], None),
    "l3_ball_track_reset": ([_P(BallTrack)], None),
    "l3_ball_track_arm": ([_P(BallTrack), _P(BallAnchor), _P(Vec3)], None),
    "l3_ball_track_anchor": (
        [
            _P(BallTrack),
            _F32,
            _F32,
            _U32,
            _P(ImpactFitCfg),
            _P(ClubTrack),
            _P(BallAnchor),
        ],
        None,
    ),
    "l3_ball_track_seed": ([_P(BallTrack), _P(TargetObs), _P(TargetObs)], ctypes.c_int32),
    "l3_ball_track_update": ([_P(BallTrack), _P(TargetObs), _U32, _U32, _U32], ctypes.c_int32),
    "l3_ball_track_update_joint": (
        [_P(BallTrack), _P(TargetObs), _U32, _U32, _U32, _U32],
        ctypes.c_int32,
    ),
    "l3_ball_track_note_club": ([_P(BallTrack), _U32, _U32], None),
    "l3_ball_track_extract_snr": ([_P(BallTrackCfg), _F32], _F32),
    "l3_ball_track_struct_bytes": ([], _U32),
    "l3_ball_track_set_angles": (
        [_P(BallTrack), _F32, _F32, ctypes.c_uint8, _F32],
        ctypes.c_int32,
    ),
    "l3_ball_track_launch": ([_P(BallTrack), _P(Launch)], _U32),
    "l3_ball_track_reconstruct": ([_P(BallTrack), _P(Launch)], _U32),
    "l3_ball_track_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_ball_track_format_status": ([_P(BallTrack), *_TEXT], ctypes.c_int32),
    "l3_launch_format": ([_P(Launch), *_TEXT], ctypes.c_int32),
    # l3_result.h
    "l3_result_build": (
        [_P(Shot), _P(BallTrack), _P(Launch), _P(ImpactFit), _U32, ctypes.c_uint8, _P(ShotResult)],
        None,
    ),
    "l3_result_serialize": ([_P(ShotResult), ctypes.c_char_p, _U32], _U32),
    "l3_result_metric_name": ([_U32], ctypes.c_char_p),
    "l3_result_verdict_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_result_format": ([_P(ShotResult), *_TEXT], ctypes.c_int32),
    "l3_result_format_metric": ([_P(ShotResult), _U32, *_TEXT], ctypes.c_int32),
    "l3_result_format_hex": ([_P(ShotResult), *_TEXT], ctypes.c_int32),
    # l3_profile.h
    "l3_profile_init": ([_P(Profile), _U32], None),
    "l3_profile_reset": ([_P(Profile)], None),
    "l3_profile_add": ([_P(Profile), _U32, _U32], None),
    "l3_profile_frame": ([_P(Profile)], None),
    "l3_profile_mean_us": ([_P(Profile), _U32], _U32),
    "l3_profile_max_us": ([_P(Profile), _U32], _U32),
    "l3_profile_frame_us": ([_P(Profile)], _U32),
    "l3_profile_stage_name": ([_U32], ctypes.c_char_p),
    "l3_profile_format": ([_P(Profile), _U32, *_TEXT], ctypes.c_int32),
    "l3_profile_format_summary": ([_P(Profile), *_TEXT], ctypes.c_int32),
    # l3_adaptive.h
    "l3_adaptive_cfg_defaults": ([_P(AdaptiveCfg)], None),
    "l3_adaptive_windows": (
        [_P(AdaptiveCfg), _U32, _U32, _U32, _U32, _U32, _P(AdaptiveWindows)],
        ctypes.c_int32,
    ),
    "l3_adaptive_differs": ([_P(AdaptiveWindows), _U32, _U32, _U32, _U32], ctypes.c_int32),
    "l3_adaptive_format": ([_P(AdaptiveCfg), _P(AdaptiveWindows), *_TEXT], ctypes.c_int32),
    # l3_iq8.h
    "l3_iq8_mode_defaults": ([_P(Iq8Mode), ctypes.c_uint8], None),
    "l3_iq8_hwa_scale": ([ctypes.c_int16, ctypes.c_uint8, ctypes.c_uint8], ctypes.c_int16),
    "l3_iq8_pack_shift": ([_P(ctypes.c_int16), _U32, _U32], ctypes.c_uint8),
    "l3_iq8_quantize_shift": ([ctypes.c_int16, ctypes.c_uint8, _P(_U32)], ctypes.c_int8),
    "l3_iq8_quantize_scale": ([ctypes.c_int16, ctypes.c_uint16], ctypes.c_int8),
    "l3_iq8_dump_scale": ([_U32], ctypes.c_uint16),
    "l3_iq8_low_byte": ([ctypes.c_int16], ctypes.c_int8),
    "l3_iq8_max_abs": ([_P(ctypes.c_int16), _U32], _U32),
    "l3_iq8_emulate_frame": (
        [_P(Iq8Mode), _P(ctypes.c_int16), _P(ctypes.c_int8), _U32, _P(ctypes.c_uint16)],
        _U32,
    ),
    # l3_retain.h
    "l3_retain_cfg_defaults": ([_P(RetainCfg)], None),
    "l3_retain_cfg_check": ([_P(RetainCfg)], ctypes.c_int32),
    "l3_retain_predict": ([_F32, _F32], _F32),
    "l3_retain_window": (
        [_P(RetainCfg), _P(RetainState), _U32, _U32, _U32, _P(RetainWindow)],
        None,
    ),
    "l3_retain_roi": ([_U32, _U32, _P(RetainWindow), _P(Roi)], None),
    "l3_retain_budget": ([_P(RetainRequest), _P(RetainBudget)], ctypes.c_int32),
    "l3_retain_priority_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_retain_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_retain_format": ([_P(RetainWindow), *_TEXT], ctypes.c_int32),
    "l3_retain_format_budget": ([_P(RetainBudget), *_TEXT], ctypes.c_int32),
    "l3_frame_desc_format": ([_P(FrameDesc), *_TEXT], ctypes.c_int32),
    # l3_shot.h
    "l3_shot_cfg_defaults": ([_P(ShotCfg)], None),
    "l3_shot_init": ([_P(Shot), _P(ShotCfg)], None),
    "l3_shot_rearm": ([_P(Shot)], None),
    "l3_shot_update": ([_P(Shot), _P(ShotInput), _U32], ctypes.c_uint8),
    "l3_shot_wants_departing": ([_P(Shot)], ctypes.c_int32),
    "l3_shot_state_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_shot_source_name": ([ctypes.c_uint8, ctypes.c_char_p, _U32], ctypes.c_char_p),
    "l3_shot_format": ([_P(Shot), *_TEXT], ctypes.c_int32),
    "l3_track_format_point": ([_P(TrackPoint), _U32, *_TEXT], ctypes.c_int32),
}


def _ziglang_available() -> bool:
    """True when the ``ziglang`` wheel (a bundled ``zig cc``) is installed."""
    return importlib.util.find_spec("ziglang") is not None


def host_compiler_command() -> list[str] | None:
    """The command that runs a host C compiler, or None.

    The first of cc, gcc or clang on PATH; failing those, ``zig cc`` from the
    ``ziglang`` wheel, which is how a Windows checkout with no toolchain
    builds the modules.
    """
    for name in ("cc", "gcc", "clang"):
        found = shutil.which(name)
        if found:
            return [found]
    if _ziglang_available():
        return [sys.executable, "-m", "ziglang", "cc"]
    return None


def host_compiler() -> str | None:
    """The host C compiler command as display text, or None without one."""
    command = host_compiler_command()
    return None if command is None else " ".join(command)


def _source_digest(sources: tuple[Path, ...], defines: tuple[str, ...] = ()) -> str:
    digest = hashlib.sha256()
    for define in defines:
        digest.update(define.encode("ascii") + b"\0")
    for source in sources:
        digest.update(source.read_bytes())
        for header in sorted(source.parent.glob("l3_*.h")):
            digest.update(header.read_bytes())
    return digest.hexdigest()[:16]


def build_firmware_library(
    out_dir: str | Path | None = None,
    *,
    firmware_dir: Path = FIRMWARE_DIR,
    defines: tuple[str, ...] = (),
) -> ctypes.CDLL:
    """Compile the host-testable firmware modules into one shared library and bind it.

    Without ``out_dir`` the build lands in a temp directory named after the
    sources' (and defines') digest, so repeated replays skip the compile.
    ``defines`` are ``NAME=VALUE`` preprocessor switches, e.g. the board
    image's ``L3_FEATURE_DEFS``; the ctypes mirrors below describe the default
    (empty) build, so a build that changes a layout is only safe through
    functions that treat that struct as opaque. Raises RuntimeError without a
    compiler; the C compile's own errors propagate.
    """
    compiler = host_compiler_command()
    if compiler is None:
        raise RuntimeError(
            "no host C compiler (cc, gcc, clang or the ziglang wheel) for the firmware modules"
        )
    sources = tuple(firmware_dir / name for name in HOST_SOURCES)
    if out_dir is None:
        digest = _source_digest(sources, defines)
        out_dir = Path(tempfile.gettempdir()) / f"openflight-l3-host-{digest}"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    library_path = out_dir / "l3_host.so"
    if not library_path.exists():
        build = out_dir / "l3_host.build.so"
        subprocess.run(
            [
                *compiler,
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-shared",
                "-fPIC",
                "-O1",
                *(f"-D{define}" for define in defines),
                "-o",
                str(build),
                *(str(source) for source in sources),
                "-lm",
            ],
            check=True,
            cwd=firmware_dir,
        )
        build.replace(library_path)  # atomic against a parallel build
    library = ctypes.CDLL(str(library_path))
    for name, (argtypes, restype) in _SIGNATURES.items():
        function = getattr(library, name)
        function.argtypes = argtypes
        function.restype = restype
    return library


def fit_reader(lib: ctypes.CDLL, name: str = "l3_fit_list_point") -> ctypes.c_void_p:
    """The address of a C point reader (l3_fit_list_point / l3_fit_span_point),
    to pass where the C API takes an l3_point_at_fn."""
    return ctypes.cast(getattr(lib, name), ctypes.c_void_p)


def c_text(function, *args, cap: int = 200) -> str:
    """Call a firmware ``format`` function into a fresh buffer and return the text."""
    buffer = ctypes.create_string_buffer(cap)
    function(*args, buffer, cap)
    return buffer.value.decode("ascii")


__all__ = [
    "FIRMWARE_DIR",
    "HOST_SOURCES",
    "OBS_FLOOR_MIN",
    "OBS_MAX_BINS",
    "OBS_MAX_TARGETS",
    "OBS_WAVELENGTH_M",
    "STAT_ENERGY",
    "STAT_NAMES",
    "STAT_PEAK",
    "TRACK_POINTS",
    "TRACK_WHY_NAMES",
    "BALL_HYP_MAX",
    "BALL_HYP_NONE",
    "BALL_HISTORY_FRAMES",
    "BALL_HISTORY_TARGETS",
    "BALL_HYP_POINTS",
    "BallHistory",
    "BallHistoryFrame",
    "BallHistoryTarget",
    "BallHyp",
    "BallHypPoint",
    "BallHypVerdict",
    "BallHyps",
    "BallHypsCfg",
    "BallRecoverCfg",
    "BallRecoverResult",
    "TRIG_MAX_BINS",
    "TRIG_TRACE_DEPTH",
    "ANGLE_GRID_STEPS",
    "ANGLE_MAX_CHANNELS",
    "ANGLE_MAX_RX",
    "ANGLE_MAX_TX",
    "CAL_MAX_VIRTUAL",
    "BALL_TRACK_WHY_NAMES",
    "AngleObs",
    "AngleSnapshot",
    "BallTrack",
    "BallTrackCfg",
    "BinObs",
    "BinScorer",
    "DETECT_CORE_DSS",
    "DETECT_CORE_FAIL_LIMIT_DEFAULT",
    "DETECT_CORE_MSS",
    "DETECT_CORE_NAMES",
    "DETECT_CORE_VERIFY",
    "DETECT_OUTCOME_FAILED",
    "DETECT_OUTCOME_MISMATCH",
    "DETECT_OUTCOME_OK",
    "DetectCore",
    "DspIq16Ctx",
    "DspResult",
    "TIMING_CORE_FALLBACK",
    "TIMING_FLAG_BEHIND",
    "TIMING_FLAG_FIRED",
    "TIMING_FLAG_POST",
    "TIMING_FLAG_SCORED",
    "TIMING_FLAG_STALE",
    "TIMING_STAT_NAMES",
    "TIMING_TIMELINE_DEPTH",
    "Timing",
    "TimingEvent",
    "TimingStat",
    "Launch",
    "ANGLE_AZIMUTH",
    "ANGLE_ELEVATION",
    "TRACK_NO_TARGET",
    "TRACK_FOLLOW_UNKNOWN_APPROACH_MPS",
    "ClubTrack",
    "Cpx",
    "Delivery",
    "FollowCtx",
    "IMPACT_CAUSE_NAMES",
    "IMPACT_WHY_NAMES",
    "LEAVE_WHY_NAMES",
    "MEAS_FALLBACK",
    "MEAS_IMPLAUSIBLE",
    "MEAS_MEASURED",
    "MEAS_RADIAL_ONLY",
    "MEAS_VALID",
    "FILTER_HYP_NAMES",
    "FILTER_HYP_UNFILTERED",
    "PROFILE_STAGE_NAMES",
    "QUALITY_FLAGS",
    "AdaptiveCfg",
    "AdaptiveWindows",
    "Profile",
    "ProfileStage",
    "RESULT_METRIC_NAMES",
    "RESULT_METRICS",
    "RESULT_PACKET_BYTES",
    "RESULT_V1_PACKET_BYTES",
    "RESULT_VERDICT_NAMES",
    "RESULT_VERSION",
    "Measurement",
    "ShotResult",
    "SHOT_IMPACT_GATE",
    "SHOT_IMPACT_GEOMETRY",
    "SHOT_IMPACT_RANGE",
    "SHOT_STATE_NAMES",
    "Shot",
    "ShotCfg",
    "ShotInput",
    "Impact",
    "ImpactCfg",
    "ImpactClub",
    "Leave",
    "LeaveCfg",
    "ScanCfg",
    "Span",
    "FIT_MAX_POINTS",
    "FIT_NO_TRACK",
    "FIT_TRACK_NAMES",
    "FIT_CLUB_IN",
    "FIT_CLUB_OUT",
    "FIT_BALL_OUT",
    "FIT_WHY_NAMES",
    "FIT_VERDICT_NAMES",
    "ImpactFitCfg",
    "FitEstimate",
    "ImpactFit",
    "FitList",
    "FitSpan",
    "fit_reader",
    "fit_track_timed",
    "fit_verdict_decided",
    "round_us",
    "RadarCal",
    "Spherical",
    "Vec3",
    "ObsParams",
    "TargetObs",
    "TrackCfg",
    "TrackKfCfg",
    "TrackPoint",
    "Trig",
    "TrigCfg",
    "TrigTrace",
    "build_firmware_library",
    "c_text",
    "host_compiler",
    "host_compiler_command",
]
