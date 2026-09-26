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
