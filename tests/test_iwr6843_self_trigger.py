"""Replay the ball-leave detector, including every saved IWR dump."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.dump import SAMPLE_RANGE_FFT_IQ16, pack_dump
from openflight.iwr6843.self_trigger import (
    DEFAULT_HITS,
    DEFAULT_LEVEL,
    BallLeaveDetector,
    iter_dump_files,
    replay_dump,
)

DUMP_DIRS = (
    Path("session_logs/iwr"),
    Path(r"C:\Users\corma\Desktop\OF Sessions\iwr6843"),
)
TEE_BIN = 14
LEVEL = DEFAULT_LEVEL


def _row(peaks: dict[int, float], bins: int = 53) -> np.ndarray:
    power = np.zeros(bins, dtype=np.float64)
    for index, value in peaks.items():
        power[index] = value
    return power


def _trace(observations) -> str:
    return "\n".join(
        f"  frame {step.frame}: {step.phase} tee={step.tee:.0f} "
        f"approach={step.approach:.0f} peak={step.peak_bin} have={int(step.have_peak)}"
        for step in observations
    )


def test_approach_reversal_then_a_quiet_tee_does_not_fire():
    detector = BallLeaveDetector(level=LEVEL, hits=DEFAULT_HITS)
    frames = [
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL, 2: LEVEL}),
        _row({TEE_BIN: LEVEL, 6: LEVEL + 1}),
        _row({TEE_BIN: LEVEL, 3: LEVEL + 2}),
        _row({}),
    ]

    steps = [detector.step(index, row, TEE_BIN, len(row)) for index, row in enumerate(frames)]

    assert [step.phase for step in steps] == [
        "occupying",
        "watching",
        "watching",
        "toward",
        "watching",
        "tee-low",
    ]
    assert not any(step.fired for step in steps)
    assert detector.step(6, _row({TEE_BIN: LEVEL}), TEE_BIN, 53).phase == "occupying"


def test_energy_past_the_tee_fires_while_the_tee_stays_loud():
    """A hit does not quiet the tee bin. The ball shows up downrange of it."""
    detector = BallLeaveDetector(level=LEVEL, hits=1)
    detector.step(0, _row({TEE_BIN: LEVEL}), TEE_BIN, 53)
    detector.step(1, _row({TEE_BIN: LEVEL, 4: LEVEL}), TEE_BIN, 53)
    detector.step(2, _row({TEE_BIN: LEVEL, 8: LEVEL}), TEE_BIN, 53)
    fired = detector.step(3, _row({TEE_BIN: LEVEL, 8: LEVEL, TEE_BIN + 3: LEVEL + 1}), TEE_BIN, 53)

    assert fired.phase == "away"
    assert detector.step(4, _row({TEE_BIN + 5: LEVEL + 1}), TEE_BIN, 53).fired


def test_tee_drop_before_the_club_returns_resets():
    detector = BallLeaveDetector(level=LEVEL, hits=2)
    detector.step(0, _row({TEE_BIN: LEVEL}), TEE_BIN, 53)
    detector.step(1, _row({TEE_BIN: LEVEL}), TEE_BIN, 53)
    detector.step(2, _row({TEE_BIN: LEVEL, 4: LEVEL}), TEE_BIN, 53)
    toward = detector.step(3, _row({TEE_BIN: LEVEL, 8: LEVEL}), TEE_BIN, 53)
    for frame in range(4, 7):
        reset = detector.step(frame, _row({}), TEE_BIN, 53)
    again = detector.step(7, _row({TEE_BIN: LEVEL}), TEE_BIN, 53)

    assert toward.phase == "toward"
    assert reset.phase == "no-approach"
    assert not reset.ready and not reset.toward
    assert again.phase == "occupying"


def test_bin_zero_is_a_real_approach_peak():
    """A peak in bin 0 counts. The approach window has to include that bin."""
    tee = 5
    detector = BallLeaveDetector(level=LEVEL, hits=1)
    detector.step(0, _row({tee: LEVEL}), tee, 53)
    at_zero = detector.step(1, _row({tee: LEVEL, 0: LEVEL}), tee, 53)
    moved = detector.step(2, _row({tee: LEVEL, 3: LEVEL}), tee, 53)

    assert at_zero.phase == "watching"
    assert at_zero.have_peak
    assert at_zero.peak_bin == 0
    assert moved.phase == "toward"


def test_missing_tee_bin_clears_motion():
    detector = BallLeaveDetector(level=LEVEL, hits=1)
    detector.step(0, _row({TEE_BIN: LEVEL}), TEE_BIN, 53)
    detector.step(1, _row({TEE_BIN: LEVEL, 2: LEVEL}), TEE_BIN, 53)
    detector.step(2, _row({TEE_BIN: LEVEL, 5: LEVEL}), TEE_BIN, 53)
    outside = detector.step(3, _row({}), None, 0)
    away = detector.step(4, _row({TEE_BIN: LEVEL, 3: LEVEL}), TEE_BIN, 53)

    assert outside.phase == "bin-outside"
    assert away.phase == "watching"
    assert not away.toward


def _moving_cube() -> bytes:
    """Range snapshot whose loop-0 residual walks in, then leaves the tee."""
    n_frames, loops, n_tx, n_rx, bins = 6, 4, 2, 4, 53
    cube = np.zeros((n_frames, loops * n_tx, n_rx, bins), dtype=np.complex128)
    occupied = {0: {}, 1: {}, 2: {2: 1}, 3: {6: 1}, 4: {16: 1}, 5: {18: 1}}
    for frame, peaks in occupied.items():
        if frame < 5:
            peaks = {TEE_BIN: 1, **peaks}
        for local, _flag in peaks.items():
            cube[frame, 0, :, local] = 200.0
            cube[frame, 1, :, local] = 200.0
    return pack_dump(
        cube,
        n_tx=n_tx,
        trigger_frame=0,
        version=3,
        frame_period_us=3000,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16,
        range_bin_start=20,
    )


def test_replay_of_a_leaving_ball_fires_on_outward_progression():
    steps = replay_dump(
        _moving_cube(),
        tee_range_m=DEFAULT_TEE_RANGE_M,
        level=LEVEL,
        hits=DEFAULT_HITS,
    )

    assert [step.phase for step in steps] == [
        "occupying",
        "watching",
        "watching",
        "toward",
        "away",
        "fired",
    ], _trace(steps)


def _saved_dumps() -> list[Path]:
    found: list[Path] = []
    for directory in DUMP_DIRS:
        found.extend(iter_dump_files(directory))
    return found


def test_saved_dumps_fire_when_the_ball_leaves():
    """Every local ILD1 capture must latch while the tee bin is still stored."""
    dumps = _saved_dumps()
    if not dumps:
        pytest.skip("local IWR dumps not present")
    failures = []
    for path in dumps:
        steps = replay_dump(path.read_bytes())
        fired_at = next((step.frame for step in steps if step.fired), None)
        if fired_at is not None:
            continue
        visible = [step for step in steps if step.phase != "bin-outside"]
        last = visible[-1] if visible else steps[-1]
        failures.append(f"{path.name}: last {last.phase} at frame {last.frame}, tee={last.tee:.0f}")
    assert not failures, f"{len(failures)}/{len(dumps)} never fired\n" + "\n".join(failures)


def test_reversal_without_a_departing_ball_does_not_fire():
    detector = BallLeaveDetector()
    peaks = [None, None, 2, 6, 3]
    steps = [
        detector.step(
            i, _row({TEE_BIN: LEVEL, **({p: LEVEL + 1} if p is not None else {})}), TEE_BIN, 53
        )
        for i, p in enumerate(peaks)
    ]
    assert not any(step.fired for step in steps)


def test_motion_gap_discards_the_previous_approach():
    detector = BallLeaveDetector()
    frames = [
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL, 2: LEVEL + 1}),
        _row({TEE_BIN: LEVEL, 6: LEVEL + 1}),
    ]
    frames += [_row({TEE_BIN: LEVEL}) for _ in range(300)]
    frames += [
        _row({TEE_BIN: LEVEL, 3: LEVEL + 1, TEE_BIN + 2: LEVEL + 2}),
        _row({TEE_BIN: LEVEL, 3: LEVEL + 1, TEE_BIN + 4: LEVEL + 2}),
    ]
    assert not any(detector.step(i, row, TEE_BIN, 53).fired for i, row in enumerate(frames))


def test_departing_ball_requires_progression_and_can_leave_a_quiet_tee():
    detector = BallLeaveDetector()
    frames = [
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL}),
        _row({TEE_BIN: LEVEL, 2: LEVEL}),
        _row({TEE_BIN: LEVEL, 6: LEVEL}),
        _row({TEE_BIN + 2: LEVEL}),
        _row({TEE_BIN + 4: LEVEL}),
    ]
    steps = [detector.step(i, row, TEE_BIN, 53) for i, row in enumerate(frames)]
    assert not any(step.fired for step in steps[:-1])
    assert steps[-1].fired
