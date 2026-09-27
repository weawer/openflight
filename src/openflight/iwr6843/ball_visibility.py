"""Was the ball visible to the live selector, and did clutter outrank it?

Offline Phase 0 analysis for the flight-selector redesign. From a dump's
rebuilt selector input (``live_selector.gated_selector_inputs``: the
magnitude rise after the coherent gate) it finds the ball
as a straight range-vs-time line leaving the tee, then asks for each frame
whether the ball was a qualifying peak (local maximum at or above the
selector's SNR threshold) and where it ranked. The firmware keeps only the two
strongest qualifying peaks as candidates, so a ball ranked third or lower
could never be selected in that frame.

In the impact phase the firmware analyses exactly the impact window, which
adaptive dumps retain, so input, coherent gate and noise (the gated input's
mean over all 128 bins, as ``l3_live_select`` computes it) match the
firmware there. In flight the firmware analyses all 128 bins but a dump keeps
only its window: peaks outside it are missing (ranks are best cases), the
gate's mean is over the window, and zeros stand in for unretained bins in the
noise mean (which makes the threshold optimistic).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from openflight.iwr6843.live_selector import SelectorParams

SNR_THRESHOLD = SelectorParams().snr_q8 / 256.0
CANDIDATE_SLOTS = 2
MIN_LINE_FRAMES = 3
# Radial ball rates to search without an OPS speed: 0.6 bins/frame is ~30
# mph, 3.5 is ~180 mph at 2 ms frames and 4.69 cm bins.
UNGUIDED_BINS_PER_FRAME = (0.6, 3.5)
RATE_STEP_BINS_PER_FRAME = 0.05


@dataclass(frozen=True)
class FrameBall:
    """The ball line's bin in one frame and how the selector would see it."""

    frame: int
    window: tuple[int, int]
    ball_bin: int
    ball_snr: float
    ball_rank: int | None
    top_peaks: tuple[int, ...]

    @property
    def visible(self) -> bool:
        """The ball was a qualifying peak."""
        return self.ball_rank is not None

    @property
    def gated_out(self) -> bool:
        """Nothing reached the selector at the ball: no rise, or the coherent gate."""
        return self.ball_snr == 0.0

    @property
    def candidate(self) -> bool:
        """The ball was among the peaks the firmware keeps as candidates."""
        return self.ball_rank is not None and self.ball_rank <= CANDIDATE_SLOTS


@dataclass(frozen=True)
class BallLine:
    """Best straight ball track through the rebuilt rise frames."""

    impact_frame: int
    bins_per_frame: float
    score: float
    frames: tuple[FrameBall, ...]


def qualifying_peaks(rise: list[int], window: tuple[int, int], noise: float) -> list[int]:
    """Local maxima at or above the SNR threshold, strongest first.

    Ties go to the lower bin, as in ``l3_live_select``. Window edge bins are
    skipped because their outer neighbour was not retained.
    """
    start, stop = window
    peaks = [
        b
        for b in range(start + 1, stop - 1)
        if rise[b] >= rise[b - 1] and rise[b] >= rise[b + 1] and rise[b] >= SNR_THRESHOLD * noise
    ]
    return sorted(peaks, key=lambda b: (-rise[b], b))


@dataclass(frozen=True)
class _Frame:
    rise: list[int]
    window: tuple[int, int]
    noise: float
    peaks: list[int]

    def inside(self, bin_index: int) -> bool:
        return self.window[0] + 1 <= bin_index <= self.window[1] - 2

    def ball(self, frame: int, predicted: int) -> FrameBall:
        lo, hi = max(self.window[0] + 1, predicted - 1), min(self.window[1] - 2, predicted + 1)
        ball_bin = max(range(lo, hi + 1), key=lambda b: (self.rise[b], -b))
        return FrameBall(
            frame=frame,
            window=self.window,
            ball_bin=ball_bin,
            ball_snr=self.rise[ball_bin] / self.noise,
            ball_rank=self.peaks.index(ball_bin) + 1 if ball_bin in self.peaks else None,
            top_peaks=tuple(self.peaks[:CANDIDATE_SLOTS]),
        )


def _frame(rise: list[int], window: tuple[int, int]) -> _Frame:
    noise = float(max(1, sum(rise) // len(rise)))  # l3_live_select: mean over all bins
    return _Frame(rise, window, noise, qualifying_peaks(rise, window, noise))


def _rate_grid(ops_bins_per_frame: float | None, ratio_band: tuple[float, float]):
    if ops_bins_per_frame:
        low, high = (ops_bins_per_frame * ratio for ratio in ratio_band)
    else:
        low, high = UNGUIDED_BINS_PER_FRAME
    return np.arange(low, high + RATE_STEP_BINS_PER_FRAME / 2, RATE_STEP_BINS_PER_FRAME)


def find_ball_line(
    rises: list[list[int] | None],
    windows: list[tuple[int, int]],
    times_s: list[float],
    *,
    tee_bin: int,
    impact_frames: range,
    frame_period_s: float,
    ops_bins_per_frame: float | None,
    ratio_band: tuple[float, float],
) -> BallLine | None:
    """Search ball launch frame and radial rate for the strongest line.

    Every hypothesis is scored over the same frames (all rebuilt frames after
    the earliest launch considered): the rise SNR at the line's exact bin,
    with the ball on the tee before its launch and nothing counted where the
    line leaves the window. A common frame set keeps slower lines, which stay
    in the window longer, from winning on frame count alone. With an OPS
    speed the rate search is limited to ``ratio_band`` times that speed,
    which keeps the post-impact clubhead (~0.7x) out; without one it spans
    ``UNGUIDED_BINS_PER_FRAME`` and may lock onto the club instead.
    """
    first = min(impact_frames) + 1
    frames = {
        index: _frame(rise, windows[index])
        for index, rise in enumerate(rises)
        if index >= first and rise is not None
    }
    best: BallLine | None = None
    for launch in impact_frames:
        for rate in _rate_grid(ops_bins_per_frame, ratio_band):
            score = 0.0
            on_line = []
            for index, frame in frames.items():
                elapsed = max(0.0, times_s[index] - times_s[launch]) / frame_period_s
                predicted = int(round(tee_bin + rate * elapsed))
                if not frame.inside(predicted):
                    continue
                score += frame.rise[predicted] / frame.noise
                if index > launch:
                    on_line.append(frame.ball(index, predicted))
            if len(on_line) < MIN_LINE_FRAMES:
                continue
            if best is None or score > best.score:
                best = BallLine(
                    impact_frame=launch,
                    bins_per_frame=float(rate),
                    score=score,
                    frames=tuple(on_line),
                )
    return best
