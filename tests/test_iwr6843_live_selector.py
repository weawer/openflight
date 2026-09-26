"""Parity and behavior tests for the bounded live range selector."""

from __future__ import annotations

import ctypes
import subprocess
from pathlib import Path

import pytest

from openflight.iwr6843.live_selector import (
    SelectorParams,
    SelectorResult,
    SelectorState,
    retention_window,
    select_window,
)

CONFIRMED = SelectorParams().confirm_frames

ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "firmware" / "iwr6843" / "live_selector.c"


class CParams(ctypes.Structure):
    _fields_ = [
        ("window_bins", ctypes.c_uint16),
        ("max_jump_bins", ctypes.c_uint16),
        ("max_misses", ctypes.c_uint16),
        ("snr_q8", ctypes.c_uint16),
        ("confirm_frames", ctypes.c_uint16),
    ]


class CState(ctypes.Structure):
    _fields_ = [
        ("selected_bin", ctypes.c_int16),
        ("velocity_q8", ctypes.c_int16),
        ("misses", ctypes.c_uint16),
        ("active", ctypes.c_uint8),
        ("hits", ctypes.c_uint8),
    ]


class CResult(ctypes.Structure):
    _fields_ = [
        ("candidate_bins", ctypes.c_uint16 * 2),
        ("candidate_power", ctypes.c_uint32 * 2),
        ("noise", ctypes.c_uint32),
        ("selected_bin", ctypes.c_uint16),
        ("window_start", ctypes.c_uint16),
        ("window_bins", ctypes.c_uint16),
        ("confidence_q8", ctypes.c_uint16),
        ("held_bin", ctypes.c_uint16),
        ("candidate_count", ctypes.c_uint8),
        ("accepted", ctypes.c_uint8),
        ("ambiguous", ctypes.c_uint8),
        ("coasting", ctypes.c_uint8),
    ]


@pytest.fixture(scope="module")
def c_select(tmp_path_factory: pytest.TempPathFactory):
    library = tmp_path_factory.mktemp("live-selector") / "live_selector.so"
    subprocess.run(
        ["cc", "-shared", "-fPIC", "-std=c99", "-Wall", "-Wextra", "-Werror",
         "-o", str(library), str(SOURCE)],
        check=True,
    )
    function = ctypes.CDLL(str(library)).l3_live_select
    function.argtypes = [
        ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16,
        ctypes.POINTER(CParams), ctypes.POINTER(CState), ctypes.POINTER(CResult),
    ]
    function.restype = ctypes.c_int32
    function.library_path = str(library)
    return function


def _powers(*peaks: tuple[int, int]) -> list[int]:
    values = [100] * 128
    for bin_index, power in peaks:
        values[bin_index] = power
    return values


def _c_params(params: SelectorParams) -> CParams:
    return CParams(
        params.window_bins, params.max_jump_bins, params.max_misses, params.snr_q8,
        params.confirm_frames,
    )


def _c_state(state: SelectorState) -> CState:
    return CState(state.selected_bin, state.velocity_q8, state.misses, state.active, state.hits)


def _state_tuple(state) -> tuple[int, int, int, int, int]:
    return (state.selected_bin, state.velocity_q8, state.misses, state.active, state.hits)


def _c_run(c_select, powers, params, state):
    c_power = (ctypes.c_uint32 * len(powers))(*powers)
    c_params = _c_params(params)
    c_state = _c_state(state)
    result = CResult()
    assert c_select(c_power, len(powers), ctypes.byref(c_params), ctypes.byref(c_state), ctypes.byref(result)) == 0
    return c_state, result


def test_python_and_c_follow_motion_instead_of_stronger_distractor(c_select):
    params = SelectorParams()
    py_state = SelectorState()
    sequences = [
        _powers((30, 1000)),
        _powers((33, 900), (80, 2000)),
        _powers((36, 850), (78, 2200)),
    ]

    for powers in sequences:
        before = py_state.copy()
        py_result = select_window(powers, params, py_state)
        c_state, c_result = _c_run(c_select, powers, params, before)
        assert c_result.selected_bin == py_result.selected_bin
        assert c_result.window_start == py_result.window_start
        assert c_result.candidate_count == len(py_result.candidate_bins)
        assert c_result.ambiguous == py_result.ambiguous
        assert _state_tuple(c_state) == _state_tuple(py_state)
    assert py_state.selected_bin == 36


def test_missing_return_is_reported_and_prediction_stays_bounded(c_select):
    params = SelectorParams(max_misses=2)
    state = SelectorState(selected_bin=124, velocity_q8=3 * 256, active=1)

    before = state.copy()
    result = select_window(_powers(), params, state)

    assert not result.accepted
    assert result.window_start == 116
    assert result.window_start + result.window_bins == 128
    c_state, c_result = _c_run(c_select, _powers(), params, before)
    assert not c_result.accepted
    assert c_result.window_start == result.window_start
    assert c_state.misses == state.misses


def test_ties_and_reversals_are_deterministic(c_select):
    params = SelectorParams()
    for state, powers in (
        (SelectorState(), _powers((20, 1000), (40, 1000))),
        (SelectorState(selected_bin=40, velocity_q8=2 * 256, active=1), _powers((35, 2000))),
    ):
        before = state.copy()
        expected = select_window(powers, params, state)
        c_state, actual = _c_run(c_select, powers, params, before)
        assert actual.selected_bin == expected.selected_bin
        assert actual.accepted == expected.accepted
        assert c_state.misses == state.misses


def test_rejects_invalid_layout_without_mutating_state(c_select):
    params = SelectorParams(window_bins=0)
    state = SelectorState(selected_bin=12, active=1)
    before = state.copy()
    powers = (ctypes.c_uint32 * 128)(*([100] * 128))
    c_params = _c_params(params)
    c_state = _c_state(state)
    result = CResult()

    assert c_select(powers, 128, ctypes.byref(c_params), ctypes.byref(c_state), ctypes.byref(result)) == -1
    assert (c_state.selected_bin, c_state.active) == (before.selected_bin, before.active)


def test_saturated_candidates_and_bin_edges_match(c_select):
    params = SelectorParams(window_bins=12)
    state = SelectorState()
    powers = _powers((1, 0xFFFFFFFF), (126, 0xFFFFFFFE))
    before = state.copy()

    expected = select_window(powers, params, state)
    c_state, actual = _c_run(c_select, powers, params, before)

    assert actual.selected_bin == expected.selected_bin == 1
    assert actual.window_start == expected.window_start == 0
    assert actual.confidence_q8 == expected.confidence_q8
    assert c_state.selected_bin == state.selected_bin


def test_miss_holds_range_and_clears_stale_velocity(c_select):
    params = SelectorParams()
    state = SelectorState(selected_bin=13, velocity_q8=8 * 256, active=1, hits=CONFIRMED)
    powers = _powers((28, 5000), (22, 4000), (13, 3000))
    before = state.copy()

    expected = select_window(powers, params, state)
    c_state, actual = _c_run(c_select, powers, params, before)

    assert not expected.accepted
    assert expected.selected_bin == 13
    assert expected.window_start == 7
    assert state.velocity_q8 == 0
    assert actual.selected_bin == expected.selected_bin
    assert actual.window_start == expected.window_start
    assert c_state.velocity_q8 == 0


def test_home_motion_reference_stays_inside_proposed_windows(c_select):
    params = SelectorParams()
    python_state = SelectorState(selected_bin=13, velocity_q8=8 * 256, active=1, hits=CONFIRMED)
    candidate_frames = [
        ((24, 4829), (22, 4012)),
        ((28, 5606), (36, 4720)),
        ((13, 7305), (71, 3816)),
        ((13, 11996), (31, 4273)),
        ((31, 3474), (33, 3373)),
        ((95, 3828), (15, 3661)),
    ]
    reference_bins = (13, 13, 13, 13, 14, 15)

    for peaks, reference_bin in zip(candidate_frames, reference_bins, strict=True):
        powers = _powers(*peaks)
        before = python_state.copy()
        expected = select_window(powers, params, python_state)
        c_state, actual = _c_run(c_select, powers, params, before)

        assert expected.window_start <= reference_bin < (
            expected.window_start + expected.window_bins
        )
        assert actual.window_start == expected.window_start
        assert c_state.selected_bin == python_state.selected_bin


@pytest.fixture(scope="module")
def c_retention(c_select):
    function = ctypes.CDLL(c_select.library_path).l3_retention_window
    function.argtypes = [ctypes.POINTER(CResult), ctypes.c_uint16]
    function.restype = ctypes.c_uint16
    return function


@pytest.mark.parametrize(
    "accepted,ambiguous,candidates,selected,start,reason,expected_start,held,coasting",
    [
        (1, 0, (30,), 30, 24, 0, 24, None, 0),
        (0, 0, (30,), 30, 24, 1, 24, None, 0),
        (1, 1, (30, 50), 30, 24, 2, 24, None, 0),
        (1, 1, (30, 37), 30, 24, 0, 28, None, 0),
        (1, 1, (30, 23), 30, 24, 0, 21, None, 0),
        (1, 0, (126,), 126, 116, 3, 116, None, 0),
        (1, 0, (1,), 1, 0, 3, 0, None, 0),
        # Coasting (not accepted) keeps both the held bin and the prediction.
        (0, 0, (30,), 33, 27, 0, 27, 30, 1),
        (0, 0, (30,), 37, 31, 0, 28, 30, 1),
        (0, 0, (30,), 38, 32, 1, 32, 30, 1),
        (0, 1, (30, 90), 33, 27, 0, 27, 30, 1),
        (0, 0, (30,), 126, 116, 3, 116, 124, 1),
    ],
)
def test_retention_requires_candidate_and_uncertainty_margin(
    c_retention, accepted, ambiguous, candidates, selected, start, reason, expected_start,
    held, coasting,
):
    held = selected if held is None else held
    result = CResult()
    result.accepted = accepted
    result.ambiguous = ambiguous
    result.coasting = coasting
    result.held_bin = held
    result.candidate_count = len(candidates)
    result.selected_bin = selected
    result.window_start = start
    result.window_bins = 12
    for index, value in enumerate(candidates):
        result.candidate_bins[index] = value
    python_result = SelectorResult(
        candidate_bins=candidates, candidate_power=(0,) * len(candidates), noise=1,
        selected_bin=selected, window_start=start, window_bins=12, confidence_q8=0,
        accepted=bool(accepted), ambiguous=bool(ambiguous), held_bin=held,
        coasting=bool(coasting),
    )
    assert c_retention(ctypes.byref(result), 128) == reason
    assert result.window_start == expected_start
    assert retention_window(python_result, 128) == (reason, expected_start)
    if reason == 0 and coasting:
        for kept in (held, selected):
            assert result.window_start <= kept - 2
            assert kept + 2 < result.window_start + result.window_bins
    elif reason == 0:
        for candidate in candidates:
            assert result.window_start <= candidate - 2
            assert candidate + 2 < result.window_start + result.window_bins


def test_sixteen_flight_windows_keep_fast_target_with_stronger_distractor(c_select, c_retention):
    params = SelectorParams()
    state = SelectorState(selected_bin=35, velocity_q8=3 * 256, active=1, hits=CONFIRMED)
    for target in range(38, 86, 3):
        powers = _powers((target, 3000), (110, 6000))
        before = state.copy()
        expected = select_window(powers, params, state)
        _, result = _c_run(c_select, powers, params, before)
        assert expected.selected_bin == result.selected_bin == target
        assert c_retention(ctypes.byref(result), 128) == 0
        assert result.window_start <= target - 2
        assert target + 2 < result.window_start + result.window_bins


SESSIONS = ROOT / "openflight_sessions"
STATIC_SMOKE_DIR = SESSIONS / "iwr-adaptive-smoke"
MOTION_REFERENCE = SESSIONS / "iwr-shadow-association" / "shadow-reference-001.l3dump"
SHADOW_LOOP_STRIDE = 3
SHADOW_RX = (0, 2)


def _shadow_powers(path: Path) -> list[list[int] | None]:
    """Rebuild the firmware selector input from a retained IQ16 dump.

    Mirrors ``l3_storeCompletedScratchFrame``: sum |I|+|Q| over every third loop,
    all TX and RX 0/2, then keep only the rise over the previous frame. Frames
    whose predecessor retained a different bin window cannot be rebuilt and are
    returned as None, as is frame 0.
    """
    import numpy as np

    from openflight.iwr6843.dump import parse_dump

    meta, cube = parse_dump(path.read_bytes())[:2]
    frames = meta["n_frames"]
    starts = meta.get("range_bin_starts") or [meta["range_bin_start"]] * frames
    counts = meta.get("range_bin_counts") or [meta["n_samples"]] * frames
    chirps = [loop * 3 + tx for loop in range(0, 12, SHADOW_LOOP_STRIDE) for tx in range(3)]
    sums = []
    for frame in range(frames):
        window = cube[frame][chirps][:, list(SHADOW_RX), : counts[frame]]
        power = [0] * 128
        per_bin = (np.abs(window.real) + np.abs(window.imag)).sum(axis=(0, 1))
        for offset, value in enumerate(per_bin):
            power[starts[frame] + offset] = int(value)
        sums.append(power)
    rises: list[list[int] | None] = [None]
    for frame in range(1, frames):
        same_window = (starts[frame], counts[frame]) == (starts[frame - 1], counts[frame - 1])
        rises.append(
            [max(0, now - before) for now, before in zip(sums[frame], sums[frame - 1])]
            if same_window
            else None
        )
    return rises


def _step(c_select, powers, params, state):
    """Advance the Python reference and the C selector together and require parity."""
    before = state.copy()
    expected = select_window(powers, params, state)
    c_state, actual = _c_run(c_select, powers, params, before)
    assert (
        actual.selected_bin, actual.window_start, bool(actual.accepted),
        actual.held_bin, bool(actual.coasting), actual.confidence_q8,
    ) == (
        expected.selected_bin, expected.window_start, expected.accepted,
        expected.held_bin, expected.coasting, expected.confidence_q8,
    )
    assert _state_tuple(c_state) == _state_tuple(state)
    return expected, actual


def _retain(c_retention, expected, actual) -> int:
    """Run the C retention decision and require the Python reference to agree."""
    reason = c_retention(ctypes.byref(actual), 128)
    assert retention_window(expected, 128) == (reason, actual.window_start)
    return reason


def _logged_candidates(capture_index: int) -> list[tuple[int, int]]:
    import json

    captures = [
        entry
        for entry in map(json.loads, (STATIC_SMOKE_DIR / "run.jsonl").read_text().splitlines())
        if entry["event"] == "capture_received"
    ]
    return [(d["c0"], d["c1"]) for d in captures[capture_index]["shadow_decisions"]]


@pytest.mark.parametrize("capture", [1, 2, 3])
def test_recorded_static_scene_never_confirms_a_retained_track(c_select, c_retention, capture):
    """2026-09-26 smoke test: an empty room was accepted as a track in 34% of frames."""
    dump = STATIC_SMOKE_DIR / f"shadow-reference-00{capture}.l3dump"
    if not dump.exists():
        pytest.skip("recorded smoke capture not present")
    rises = _shadow_powers(dump)
    logged = _logged_candidates(capture - 1)
    params = SelectorParams()
    state = SelectorState()

    # Frames 15-19 are the broad impact frames whose input the dump fully preserves.
    for frame in range(15, 20):
        expected, actual = _step(c_select, rises[frame], params, state)
        assert expected.candidate_bins == logged[frame], "replay must match firmware input"
        assert not expected.accepted, f"frame {frame} accepted noise at bin {expected.selected_bin}"
        assert _retain(c_retention, expected, actual) != 0


def test_recorded_motion_is_confirmed_then_retained_through_misses(c_select, c_retention):
    """Phase 3 reference: an object moving 12 -> 15 -> 17 -> 18, then two missed frames."""
    if not MOTION_REFERENCE.exists():
        pytest.skip("recorded Phase 3 reference capture not present")
    rises = _shadow_powers(MOTION_REFERENCE)
    params = SelectorParams()
    state = SelectorState()
    outcomes = []
    for frame in range(1, 7):
        expected, actual = _step(c_select, rises[frame], params, state)
        reason = _retain(c_retention, expected, actual)
        outcomes.append((frame, expected, actual, reason))

    accepted = [(frame, result.selected_bin) for frame, result, _, _ in outcomes if result.accepted]
    assert accepted == [(3, 17), (4, 18)], "a track is confirmed on its third associated frame"
    for frame, expected, actual, reason in outcomes[2:]:
        assert reason == 0, f"frame {frame} ended retention (reason {reason})"
    for frame, expected, actual, _ in outcomes[2:5]:
        strongest = expected.candidate_bins[0]
        assert actual.window_start <= strongest < actual.window_start + actual.window_bins


def _ball_frame(bin_index: int | None) -> list[int]:
    """One frame of selector input: a lone ball return, or only background."""
    return _powers((bin_index, 3000)) if bin_index is not None else _powers()


@pytest.mark.parametrize("missed_frames", [(8,), (8, 9)])
def test_fast_ball_is_retained_through_missed_frames(c_select, c_retention, missed_frames):
    """75 m/s moves ~3.2 bins per 2 ms frame; one or two faint frames must not end retention."""
    params = SelectorParams()
    state = SelectorState()
    confirmed = False
    for frame, truth in enumerate(range(20, 20 + 3 * 14, 3)):
        visible = frame not in missed_frames
        expected, actual = _step(c_select, _ball_frame(truth if visible else None), params, state)
        confirmed = confirmed or expected.accepted
        if not confirmed:
            continue
        assert _retain(c_retention, expected, actual) == 0, f"frame {frame} ended retention"
        assert actual.window_start <= truth - 2, f"frame {frame} lost the near margin"
        assert truth + 2 < actual.window_start + actual.window_bins, f"frame {frame} lost the far margin"
        if visible:
            assert expected.accepted and expected.selected_bin == truth
    assert confirmed


def test_fast_ball_retention_ends_explicitly_after_too_many_misses(c_select, c_retention):
    params = SelectorParams(max_misses=2)
    state = SelectorState()
    reasons = []
    for frame, truth in enumerate(range(20, 20 + 3 * 10, 3)):
        visible = frame < 5
        expected, actual = _step(c_select, _ball_frame(truth if visible else None), params, state)
        reasons.append(_retain(c_retention, expected, actual))
    assert reasons[5:7] == [0, 0]
    assert reasons[7] == 1  # third consecutive miss exceeds max_misses: track_lost


def test_isolated_jumping_peaks_never_become_a_retained_track(c_select, c_retention):
    params = SelectorParams()
    state = SelectorState()
    for peak in (20, 40, 60, 80, 100, 30, 50, 70):
        expected, actual = _step(c_select, _powers((peak, 5000)), params, state)
        assert not expected.accepted
        assert _retain(c_retention, expected, actual) != 0
