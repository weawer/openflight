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
import shutil
import subprocess
import tempfile
from pathlib import Path

FIRMWARE_DIR = Path(__file__).resolve().parents[3] / "firmware" / "iwr6843"
HOST_SOURCES = (
    "l3_text.c",
    "l3_frames.c",
    "l3_angle.c",
    "l3_observation.c",
    "l3_trigger.c",
    "l3_club_track.c",
    "l3_impact.c",
    "l3_shot.c",
    "l3_ball_track.c",
    "l3_result.c",
    "l3_profile.c",
    "l3_adaptive.c",
    "l3_iq8.c",
    "l3_retain.c",
    "l3_iq16_stats.c",
)

# l3_observation.h
OBS_MAX_BINS = 64
OBS_MAX_TARGETS = 8
OBS_WAVELENGTH_M = 0.00484
OBS_FLOOR_MIN = 1.0
STAT_ENERGY, STAT_PEAK = 0, 1
STAT_NAMES = {"energy": STAT_ENERGY, "peak": STAT_PEAK}
SUBBIN_CENTROID, SUBBIN_PARABOLIC = 0, 1
SUBBIN_NAMES = {"centroid": SUBBIN_CENTROID, "parabolic": SUBBIN_PARABOLIC}

# l3_trigger.h
TRIG_MAX_BINS = 64
TRIG_LOG_DEPTH = 64
TRIG_TRACE_DEPTH = 32
TRIG_COUNT_TOTAL = 13
TRIG_NO_BIN = 0xFF
TRIG_STATE_IDLE, TRIG_STATE_TRACKING, TRIG_STATE_FIRED = 0, 1, 2
TRIG_STATE_NAMES = ("idle", "tracking", "fired")

# l3_frames.h / l3_angle.h
CAL_MAX_VIRTUAL = 8
ANGLE_MAX_TX, ANGLE_MAX_RX = 3, 4
ANGLE_MAX_CHANNELS = ANGLE_MAX_TX * ANGLE_MAX_RX
ANGLE_GRID_STEPS = 161

# l3_observation.h anglesValid bits
ANGLE_AZIMUTH, ANGLE_ELEVATION = 1, 2

# l3_impact.h
IMPACT_WHY_NAMES = (
    "none",
    "noball",
    "nodelivery",
    "slow",
    "unsure",
    "far",
    "pending",
    "passed",
    "fired",
)

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
SHOT_IMPACT_GATE, SHOT_IMPACT_GEOMETRY = 1, 2

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
)

# l3_result.h
RESULT_VERSION = 1
RESULT_METRICS = 9
RESULT_PACKET_BYTES = 100
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
)

# l3_club_track.h
TRACK_POINTS = 32
TRACK_NO_TARGET = 0xFFFFFFFF
TRACK_WHY_NAMES = ("none", "acquired", "associated", "coasted", "dropped", "idle")


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
    """``l3_trig_cfg_t``."""

    _fields_ = [
        ("teeBin", ctypes.c_uint32),
        ("snr", ctypes.c_float),
        ("trackFrames", ctypes.c_uint32),
        ("approachBins", ctypes.c_uint32),
        ("gateBins", ctypes.c_uint32),
        ("minCoherence", ctypes.c_float),
        ("minStepBins", ctypes.c_float),
        ("stat", ctypes.c_uint32),
        ("minSpeedMps", ctypes.c_float),
        ("minApproachBins", ctypes.c_uint32),
    ]


class TrigTrace(ctypes.Structure):
    """``l3_trig_trace_t``: the region's strongest bin of one frame."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("gap", ctypes.c_uint16),
        ("bin", ctypes.c_uint8),
        ("state", ctypes.c_uint8),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("floor", ctypes.c_float),
        ("threshold", ctypes.c_float),
        ("coherencePct", ctypes.c_uint8),
        ("dest", ctypes.c_uint8),
    ]


class TrigRecord(ctypes.Structure):
    """``l3_trig_record_t``: one logged detector frame."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("gap", ctypes.c_uint16),
        ("state", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("bin", ctypes.c_uint8),
        ("age", ctypes.c_uint8),
        ("velocityCms", ctypes.c_int16),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("floor", ctypes.c_float),
        ("coherencePct", ctypes.c_uint8),
        ("dest", ctypes.c_uint8),
    ]


class Trig(ctypes.Structure):
    """``l3_trig_t``: the self-trigger detector."""

    _fields_ = [
        ("cfg", TrigCfg),
        ("state", ctypes.c_uint8),
        ("floor", ctypes.c_float),
        ("loopPeriodS", ctypes.c_float),
        ("trackBin", ctypes.c_uint8),
        ("trackStartBin", ctypes.c_uint8),
        ("trackAge", ctypes.c_uint8),
        ("trackMisses", ctypes.c_uint8),
        ("trackStartFrame", ctypes.c_uint32),
        ("counters", ctypes.c_uint32 * TRIG_COUNT_TOTAL),
        ("quietSince", ctypes.c_uint32),
        ("logNext", ctypes.c_uint32),
        ("logCount", ctypes.c_uint32),
        ("log", TrigRecord * TRIG_LOG_DEPTH),
        ("traceQuiet", ctypes.c_uint32),
        ("traceNext", ctypes.c_uint32),
        ("traceCount", ctypes.c_uint32),
        ("trace", TrigTrace * TRIG_TRACE_DEPTH),
        ("maxFirstBin", ctypes.c_uint32),
        ("maxBins", ctypes.c_uint32),
        ("maxStat", ctypes.c_float * TRIG_MAX_BINS),
        ("maxFrame", ctypes.c_uint32 * TRIG_MAX_BINS),
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
        ("velocitySpanMps", ctypes.c_float),
        ("cal", RadarCal),
        ("minAcquireDopplerMps", ctypes.c_float),
        ("maxAngleResidualM", ctypes.c_float),
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
    ]


class ImpactCfg(ctypes.Structure):
    """``l3_impact_cfg_t``."""

    _fields_ = [
        ("toleranceM", ctypes.c_float),
        ("horizonS", ctypes.c_float),
        ("minSpeedMps", ctypes.c_float),
        ("minConfidence", ctypes.c_float),
    ]


class Impact(ctypes.Structure):
    """``l3_impact_t``: the geometric impact detector."""

    _fields_ = [
        ("cfg", ImpactCfg),
        ("fired", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("closestM", ctypes.c_float),
        ("offsetS", ctypes.c_float),
        ("impactTimestampUs", ctypes.c_uint32),
        ("contact", Vec3),
        ("velocity", Vec3),
        ("counters", ctypes.c_uint32 * len(IMPACT_WHY_NAMES)),
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
        ("gateFired", ctypes.c_uint8),
        ("geometricFired", ctypes.c_uint8),
        ("impactTimestampUs", ctypes.c_uint32),
        ("delivery", ctypes.POINTER(Delivery)),
        ("club", ctypes.POINTER(ClubTrack)),
        ("postFrame", ctypes.c_uint8),
        ("ballTrackDone", ctypes.c_uint8),
        ("solved", ctypes.c_uint8),
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


class BallTrackCfg(ctypes.Structure):
    """``l3_ball_track_cfg_t``."""

    _fields_ = [
        ("core", TrackCfg),
        ("minDepartureMps", ctypes.c_float),
        ("maxSpeedMps", ctypes.c_float),
        ("originGateBins", ctypes.c_float),
        ("minDepartureBins", ctypes.c_float),
        ("launchPoints", ctypes.c_uint32),
        ("snr", ctypes.c_float),
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
        ("lastTargetIndex", ctypes.c_uint32),
        ("counters", ctypes.c_uint32 * len(BALL_TRACK_WHY_NAMES)),
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
        ("speedValid", ctypes.c_uint8),
        ("hlaValid", ctypes.c_uint8),
        ("vlaValid", ctypes.c_uint8),
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


_U32 = ctypes.c_uint32
_F32 = ctypes.c_float
_P = ctypes.POINTER
_TEXT = (ctypes.c_char_p, _U32)

# name -> (argtypes, restype); None restype is the C void.
_SIGNATURES: dict[str, tuple[list, object]] = {
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
    "l3_angle_chirp_phase": ([_F32, _U32, _F32, _F32], _F32),
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
    "l3_trig_init": ([_P(Trig), _P(TrigCfg), _F32], None),
    "l3_trig_rearm": ([_P(Trig)], None),
    "l3_trig_region": ([_P(TrigCfg), _U32, _U32, _U32, _P(_U32), _P(_U32)], ctypes.c_int32),
    "l3_trig_update": ([_P(Trig), _U32, _U32, _U32, _P(BinObs), _U32], ctypes.c_int32),
    "l3_trig_log_count": ([_P(Trig)], _U32),
    "l3_trig_log_get": ([_P(Trig), _U32, _P(TrigRecord)], ctypes.c_int32),
    "l3_trig_format_summary": ([_P(Trig), *_TEXT], ctypes.c_int32),
    "l3_trig_format_config": ([_P(Trig), *_TEXT], ctypes.c_int32),
    "l3_trig_format_record": ([_P(TrigRecord), *_TEXT], ctypes.c_int32),
    "l3_trig_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
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
    "l3_track_set_angles": ([_P(ClubTrack), _F32, _F32, ctypes.c_uint8], ctypes.c_int32),
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
    "l3_impact_closest": ([_P(Vec3), _P(Vec3), _P(Vec3), _P(_F32), _P(_F32), _P(Vec3)], None),
    "l3_impact_update": ([_P(Impact), _P(Delivery), _P(Vec3), ctypes.c_uint8], ctypes.c_int32),
    "l3_impact_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_impact_format": ([_P(Impact), *_TEXT], ctypes.c_int32),
    # l3_ball_track.h
    "l3_ball_track_cfg_defaults": ([_P(BallTrackCfg)], None),
    "l3_ball_track_init": ([_P(BallTrack), _P(BallTrackCfg)], None),
    "l3_ball_track_reset": ([_P(BallTrack)], None),
    "l3_ball_track_arm": ([_P(BallTrack), _F32, _P(Vec3), _U32], None),
    "l3_ball_track_update": ([_P(BallTrack), _P(TargetObs), _U32, _U32, _U32], ctypes.c_int32),
    "l3_ball_track_set_angles": ([_P(BallTrack), _F32, _F32, ctypes.c_uint8], ctypes.c_int32),
    "l3_ball_track_launch": ([_P(BallTrack), _P(Launch)], _U32),
    "l3_ball_track_why_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_ball_track_format_status": ([_P(BallTrack), *_TEXT], ctypes.c_int32),
    "l3_launch_format": ([_P(Launch), *_TEXT], ctypes.c_int32),
    # l3_result.h
    "l3_result_build": (
        [_P(Shot), _P(BallTrack), _P(Launch), _U32, ctypes.c_uint8, _P(ShotResult)],
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
    "l3_shot_format": ([_P(Shot), *_TEXT], ctypes.c_int32),
    "l3_track_format_point": ([_P(TrackPoint), _U32, *_TEXT], ctypes.c_int32),
}


def host_compiler() -> str | None:
    """The first host C compiler on PATH, or None."""
    for name in ("cc", "gcc", "clang"):
        found = shutil.which(name)
        if found:
            return found
    return None


def _source_digest(sources: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for source in sources:
        digest.update(source.read_bytes())
        for header in sorted(source.parent.glob("l3_*.h")):
            digest.update(header.read_bytes())
    return digest.hexdigest()[:16]


def build_firmware_library(
    out_dir: str | Path | None = None, *, firmware_dir: Path = FIRMWARE_DIR
) -> ctypes.CDLL:
    """Compile the host-testable firmware modules into one shared library and bind it.

    Without ``out_dir`` the build lands in a temp directory named after the
    sources' digest, so repeated replays skip the compile. Raises RuntimeError
    without a compiler; the C compile's own errors propagate.
    """
    compiler = host_compiler()
    if compiler is None:
        raise RuntimeError("no host C compiler (cc, gcc or clang) for the firmware modules")
    sources = tuple(firmware_dir / name for name in HOST_SOURCES)
    if out_dir is None:
        out_dir = Path(tempfile.gettempdir()) / f"openflight-l3-host-{_source_digest(sources)}"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    library_path = out_dir / "l3_host.so"
    if not library_path.exists():
        build = out_dir / "l3_host.build.so"
        subprocess.run(
            [
                compiler,
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-shared",
                "-fPIC",
                "-O1",
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
    "TRIG_COUNT_TOTAL",
    "TRIG_LOG_DEPTH",
    "TRIG_MAX_BINS",
    "TRIG_NO_BIN",
    "TRIG_STATE_FIRED",
    "TRIG_STATE_IDLE",
    "TRIG_STATE_NAMES",
    "TRIG_STATE_TRACKING",
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
    "Launch",
    "ANGLE_AZIMUTH",
    "ANGLE_ELEVATION",
    "TRACK_NO_TARGET",
    "ClubTrack",
    "Cpx",
    "Delivery",
    "IMPACT_WHY_NAMES",
    "MEAS_FALLBACK",
    "MEAS_IMPLAUSIBLE",
    "MEAS_MEASURED",
    "MEAS_RADIAL_ONLY",
    "MEAS_VALID",
    "PROFILE_STAGE_NAMES",
    "QUALITY_FLAGS",
    "AdaptiveCfg",
    "AdaptiveWindows",
    "Profile",
    "ProfileStage",
    "RESULT_METRIC_NAMES",
    "RESULT_METRICS",
    "RESULT_PACKET_BYTES",
    "RESULT_VERDICT_NAMES",
    "RESULT_VERSION",
    "Measurement",
    "ShotResult",
    "SHOT_IMPACT_GATE",
    "SHOT_IMPACT_GEOMETRY",
    "SHOT_STATE_NAMES",
    "Shot",
    "ShotCfg",
    "ShotInput",
    "Impact",
    "ImpactCfg",
    "RadarCal",
    "Spherical",
    "Vec3",
    "ObsParams",
    "TargetObs",
    "TrackCfg",
    "TrackPoint",
    "Trig",
    "TrigCfg",
    "TrigRecord",
    "TrigTrace",
    "build_firmware_library",
    "c_text",
    "host_compiler",
]
