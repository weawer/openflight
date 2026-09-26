"""Parity and behavior tests for the bounded live range selector."""

from __future__ import annotations

import ctypes
import subprocess
from pathlib import Path

import pytest

from openflight.iwr6843.live_selector import SelectorParams, SelectorState, select_window

ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "firmware" / "iwr6843" / "live_selector.c"


class CParams(ctypes.Structure):
    _fields_ = [
        ("window_bins", ctypes.c_uint16),
        ("max_jump_bins", ctypes.c_uint16),
        ("max_misses", ctypes.c_uint16),
        ("snr_q8", ctypes.c_uint16),
    ]


class CState(ctypes.Structure):
    _fields_ = [
        ("selected_bin", ctypes.c_int16),
        ("velocity_q8", ctypes.c_int16),
        ("misses", ctypes.c_uint16),
        ("active", ctypes.c_uint8),
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
        ("candidate_count", ctypes.c_uint8),
        ("accepted", ctypes.c_uint8),
        ("ambiguous", ctypes.c_uint8),
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


def _c_run(c_select, powers, params, state):
    c_power = (ctypes.c_uint32 * len(powers))(*powers)
    c_params = CParams(params.window_bins, params.max_jump_bins, params.max_misses, params.snr_q8)
    c_state = CState(state.selected_bin, state.velocity_q8, state.misses, state.active)
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
        assert (c_state.selected_bin, c_state.velocity_q8, c_state.misses, c_state.active) == (
            py_state.selected_bin, py_state.velocity_q8, py_state.misses, py_state.active
        )
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
    c_params = CParams(0, params.max_jump_bins, params.max_misses, params.snr_q8)
    c_state = CState(state.selected_bin, state.velocity_q8, state.misses, state.active)
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
    state = SelectorState(selected_bin=13, velocity_q8=8 * 256, active=1)
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
    python_state = SelectorState(selected_bin=13, velocity_q8=8 * 256, active=1)
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
    "accepted,ambiguous,candidates,selected,start,reason,expected_start",
    [
        (1, 0, (30,), 30, 24, 0, 24),
        (0, 0, (30,), 30, 24, 1, 24),
        (1, 1, (30, 50), 30, 24, 2, 24),
        (1, 1, (30, 37), 30, 24, 0, 28),
        (1, 1, (30, 23), 30, 24, 0, 21),
        (1, 0, (126,), 126, 116, 3, 116),
        (1, 0, (1,), 1, 0, 3, 0),
    ],
)
def test_retention_requires_candidate_and_uncertainty_margin(
    c_retention, accepted, ambiguous, candidates, selected, start, reason, expected_start
):
    result = CResult()
    result.accepted = accepted
    result.ambiguous = ambiguous
    result.candidate_count = len(candidates)
    result.selected_bin = selected
    result.window_start = start
    result.window_bins = 12
    for index, value in enumerate(candidates):
        result.candidate_bins[index] = value
    assert c_retention(ctypes.byref(result), 128) == reason
    assert result.window_start == expected_start
    if reason == 0:
        for candidate in candidates:
            assert result.window_start <= candidate - 2
            assert candidate + 2 < result.window_start + result.window_bins


def test_sixteen_flight_windows_keep_fast_target_with_stronger_distractor(c_select, c_retention):
    params = SelectorParams()
    state = SelectorState(selected_bin=35, velocity_q8=3 * 256, active=1)
    for target in range(38, 86, 3):
        powers = _powers((target, 3000), (110, 6000))
        before = state.copy()
        expected = select_window(powers, params, state)
        _, result = _c_run(c_select, powers, params, before)
        assert expected.selected_bin == result.selected_bin == target
        assert c_retention(ctypes.byref(result), 128) == 0
        assert result.window_start <= target - 2
        assert target + 2 < result.window_start + result.window_bins
