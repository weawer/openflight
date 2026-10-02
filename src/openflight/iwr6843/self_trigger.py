"""Self-trigger settings the Pi, the replay and the viewer share.

The board's self-trigger fires on the club track's range-only impact
(``firmware/iwr6843/l3_impact.c``); ``firmware_replay`` replays it. What is
left here is what the host needs around it: the ``triggerCfg`` and tracker
defaults the Pi sends, and the empty-lane floor sample the monitor takes when
it arms. The host ball-leave detector that once replayed an older firmware
trigger was removed on 2026-09-30.
"""

from __future__ import annotations

import math

import numpy as np

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M

# The firmware trigger's ``triggerCfg`` defaults as the Pi sends them for the
# stock setup (tee 1.575 m from the enclosure front → array 1.605 m → bin 34,
# watched two bins short). The replay and the viewer start from the same values.
FIRMWARE_TRIGGER_DEFAULT_BIN = 32
FIRMWARE_TRIGGER_DEFAULT_SNR = 1.0
# The ball tracker's own snr on the board (l3_ball_track_cfg_defaults), which
# "trackCfg ballSnr 0" restores.
FIRMWARE_BALL_DEFAULT_SNR = 1.0
# The tee band's width the Pi sends by default ("trackCfg impactFit"):
# l3_impact_fit_cfg_defaults', the ridge on the 2026-09-28 capture. 0: off.
TEE_BAND_DEFAULT_BINS = 6.0
# "trackCfg ballSnr"'s limit (L3_BALL_SNR_MAX in l3_dump.c).
BALL_SNR_MAX = 1.0e6


def check_ball_snr(snr: float) -> float:
    """The ball tracker's snr as ``trackCfg ballSnr`` accepts it: 1..1e6.

    Below 1 every bin would be a target; ``not`` in the test also refuses NaN.
    """
    if not 1.0 <= snr <= BALL_SNR_MAX:
        raise ValueError(f"ball snr must be 1..{BALL_SNR_MAX:g}, got {snr}")
    return float(snr)


# Wide 24-frame profile; replay reads the real period from the dump header.
DEFAULT_FRAME_PERIOD_S = 0.003
FLOOR_PERCENTILE = 95.0
# Empty-lane tee power moves by about this much over a couple of seconds.
FLOOR_MARGIN = 1.5
FLOOR_MIN_SAMPLES = 8
FLOOR_SAMPLE_S = 2.0
FLOOR_PAUSE_S = 0.05


def tee_power_from_stats(text: str) -> float | None:
    """Tee residual from a stats ``trig`` line, or None when that line is absent."""
    latest: float | None = None
    for line in text.splitlines():
        marker = line.find("trig ")
        if marker < 0:
            continue
        for token in line[marker:].split():
            if not token.startswith("tee="):
                continue
            try:
                latest = float(token.split("=", 1)[1])
            except ValueError:
                continue
    return latest


def level_above_floor(samples: list[float]) -> tuple[float, float]:
    """Return ``(p95 floor, armed level)`` from empty-lane tee samples."""
    values = [float(sample) for sample in samples if math.isfinite(sample) and sample > 0.0]
    if len(values) < FLOOR_MIN_SAMPLES:
        raise ValueError(f"need at least {FLOOR_MIN_SAMPLES} background samples, got {len(values)}")
    floor = float(np.percentile(values, FLOOR_PERCENTILE))
    if not math.isfinite(floor) or floor <= 0.0:
        raise ValueError(f"background floor must be > 0, got {floor}")
    return floor, floor * FLOOR_MARGIN


__all__ = [
    "BALL_SNR_MAX",
    "DEFAULT_FRAME_PERIOD_S",
    "DEFAULT_TEE_RANGE_M",
    "FIRMWARE_BALL_DEFAULT_SNR",
    "FIRMWARE_TRIGGER_DEFAULT_BIN",
    "FIRMWARE_TRIGGER_DEFAULT_SNR",
    "FLOOR_MARGIN",
    "FLOOR_MIN_SAMPLES",
    "FLOOR_PAUSE_S",
    "FLOOR_PERCENTILE",
    "FLOOR_SAMPLE_S",
    "TEE_BAND_DEFAULT_BINS",
    "check_ball_snr",
    "level_above_floor",
    "tee_power_from_stats",
]
