"""What the live selector followed during an adaptive capture's flight frames.

Adaptive16 retention centres each flight frame's window on the selector's bin
(``windowStart = selected - windowBins / 2`` in ``live_selector.c``), so the
retained window positions trace the target the firmware tracked. Its range
rate, compared with the OPS ball speed, shows whether that target was the
ball. ``retention.reason == "complete"`` does not: in the 2026-09-27 range
sessions most complete captures followed a near-stationary return just past
the tee (club follow-through, golfer), not the ball.

The window centre can sit a bin or two off the selected bin when retention
widens the window to keep a held or ambiguous candidate, which the rate fit
over several frames absorbs.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from openflight.iwr6843.tracking import RANGE_SPAN_M

RANGE_FFT_SIZE = 128
BIN_M = RANGE_SPAN_M / RANGE_FFT_SIZE
MPS_TO_MPH = 2.2369362920544
MIN_FLIGHT_FRAMES = 3
# Radial range rate / OPS ball speed for a tracked ball. A ball reads a bit
# under 1.0 from the line-of-sight projection (tracking.py: a 66 mph ball
# reads ~27.5 m/s radial, ratio 0.93). The post-impact clubhead moves at
# ~0.7x ball speed and decelerates, so the lower bound sits above it.
FOLLOWS_BALL_RATIO = (0.8, 1.2)


@dataclass(frozen=True)
class FlightTrack:
    """Range rate of the target the firmware selector retained."""

    flight_frames: int
    start_bin: float | None
    end_bin: float | None
    range_rate_mph: float | None


def measure_flight_track(metadata: dict, *, impact_frames: int) -> FlightTrack | None:
    """Fit the retained flight windows' centre bin against frame time.

    Returns None for captures without adaptive retention: their windows are
    fixed by the cfg and do not follow anything. ``range_rate_mph`` is None
    when fewer than ``MIN_FLIGHT_FRAMES`` flight frames were retained.
    """
    retention = metadata.get("retention")
    starts = metadata.get("range_bin_starts")
    counts = metadata.get("range_bin_counts")
    offsets_us = metadata.get("frame_time_offsets_us")
    if retention is None or starts is None or counts is None or offsets_us is None:
        return None
    first_flight = retention["pre_frames"] + impact_frames
    centres = [start + count / 2.0 for start, count in zip(starts, counts)][first_flight:]
    times_s = [offset / 1e6 for offset in offsets_us][first_flight:]
    if not centres:
        return FlightTrack(flight_frames=0, start_bin=None, end_bin=None, range_rate_mph=None)
    rate_mph = None
    if len(centres) >= MIN_FLIGHT_FRAMES:
        bins_per_s = float(np.polyfit(times_s, centres, 1)[0])
        rate_mph = bins_per_s * BIN_M * MPS_TO_MPH
    return FlightTrack(
        flight_frames=len(centres),
        start_bin=centres[0],
        end_bin=centres[-1],
        range_rate_mph=rate_mph,
    )


def score_flight_track(track: FlightTrack | None, ops_ball_mph: float | None) -> dict | None:
    """Session-log record: the tracked range rate against the OPS ball speed.

    Diagnostic only; capture acceptance does not use it.
    """
    if track is None:
        return None
    record = asdict(track)
    record["ops_ball_mph"] = ops_ball_mph
    record["speed_ratio"] = None
    if track.range_rate_mph is None:
        record["status"] = "too_few_flight_frames"
    elif not ops_ball_mph or ops_ball_mph <= 0:
        record["status"] = "no_ops_speed"
    else:
        ratio = track.range_rate_mph / ops_ball_mph
        record["speed_ratio"] = ratio
        low, high = FOLLOWS_BALL_RATIO
        record["status"] = "follows_ball" if low <= ratio <= high else "not_ball"
    return record
