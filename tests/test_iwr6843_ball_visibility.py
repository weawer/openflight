"""Phase 0 ball-visibility analysis (ball_visibility.py)."""

from __future__ import annotations

import pytest

from openflight.iwr6843.ball_visibility import (
    SNR_THRESHOLD,
    find_ball_line,
    qualifying_peaks,
)

WINDOW = (30, 90)
TEE = 41
LAUNCH = 5
FRAMES = 14
PERIOD_S = 0.002


def _rises(ball_power, clutter=(), rate=2.0, floor=100):
    rises = [None]
    for frame in range(1, FRAMES):
        rise = [0] * 128
        for b in range(*WINDOW):
            rise[b] = floor
        for bin_index, power in clutter:
            rise[bin_index] = power
        if frame > LAUNCH:
            rise[int(round(TEE + rate * (frame - LAUNCH)))] = ball_power
        rises.append(rise)
    return rises


def _line(rises, ops_bins=2.0):
    return find_ball_line(
        rises,
        [WINDOW] * FRAMES,
        [PERIOD_S * i for i in range(FRAMES)],
        tee_bin=TEE,
        impact_frames=range(3, 8),
        frame_period_s=PERIOD_S,
        ops_bins_per_frame=ops_bins,
        ratio_band=(0.8, 1.2),
    )


def test_ball_alone_is_found_and_always_a_candidate():
    line = _line(_rises(5000))

    assert line.impact_frame == LAUNCH
    # Rates within one grid step round to the same bins over a short line.
    assert line.bins_per_frame == pytest.approx(2.0, abs=0.06)
    assert all(item.candidate and item.ball_rank == 1 for item in line.frames)


def test_two_stronger_returns_crowd_the_ball_out_of_the_candidates():
    """The recorded failure pattern: golfer and club returns fill both slots."""
    line = _line(_rises(5000, clutter=((33, 20000), (36, 15000))))

    assert all(item.visible for item in line.frames)
    assert not any(item.candidate for item in line.frames)
    assert all(item.ball_rank == 3 for item in line.frames)
    assert all(item.top_peaks == (33, 36) for item in line.frames)


def test_one_stronger_return_leaves_the_ball_a_candidate():
    line = _line(_rises(5000, clutter=((33, 20000),)))

    assert all(item.candidate and item.ball_rank == 2 for item in line.frames)


def test_weak_ball_is_below_the_selector_threshold():
    """Noise is the mean over all 128 bins, as in l3_live_select; a 60-bin
    floor of 100 puts the 3x threshold near 141."""
    line = _line(_rises(130))

    assert line is not None
    assert not any(item.visible or item.gated_out for item in line.frames)
    assert all(0 < item.ball_snr < SNR_THRESHOLD for item in line.frames)


def test_gated_out_ball_is_reported_separately_from_a_weak_one():
    """After the coherent gate most bins are exactly zero; a ball the gate
    removed leaves nothing at its bin."""
    rises = _rises(5000, floor=0)
    for frame in range(LAUNCH + 4, FRAMES):
        rises[frame] = [0] * 128

    line = _line(rises)

    assert [item.visible for item in line.frames[:3]] == [True] * 3
    assert all(item.gated_out and not item.visible for item in line.frames[3:])


def test_search_stays_inside_the_ops_band():
    """A clubhead-rate return (0.7x) must not be chosen when the band is 0.8-1.2x."""
    line = _line(_rises(5000, rate=1.4), ops_bins=2.0)

    assert line.bins_per_frame >= 1.6 - 1e-9


def test_too_few_rebuilt_frames_give_no_line():
    """Adaptive windows move every flight frame, so only unmoved ones rebuild."""
    rises = _rises(5000)
    for frame in range(5, FRAMES):
        rises[frame] = None

    assert _line(rises) is None


def test_peak_ties_rank_the_lower_bin_first():
    rise = [0] * 128
    for b in range(*WINDOW):
        rise[b] = 10
    rise[50] = rise[40] = 900

    assert qualifying_peaks(rise, WINDOW, noise=10.0) == [40, 50]


def test_window_edges_are_not_peaks():
    rise = [0] * 128
    rise[WINDOW[0]] = rise[WINDOW[1] - 1] = 900

    assert qualifying_peaks(rise, WINDOW, noise=10.0) == []
