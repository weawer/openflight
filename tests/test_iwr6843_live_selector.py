"""Parity and behavior tests for the bounded live range selector."""

from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843.driver import parse_retention_stop
from openflight.iwr6843.dump import RETENTION_REASONS as RETENTION_REASON_NAMES
from openflight.iwr6843.live_selector import (
    SelectorParams,
    SelectorResult,
    SelectorState,
    coherent_gate,
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
        [
            "cc",
            "-shared",
            "-fPIC",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-o",
            str(library),
            str(SOURCE),
        ],
        check=True,
    )
    function = ctypes.CDLL(str(library)).l3_live_select
    function.argtypes = [
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_uint16,
        ctypes.POINTER(CParams),
        ctypes.POINTER(CState),
        ctypes.POINTER(CResult),
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
        params.window_bins,
        params.max_jump_bins,
        params.max_misses,
        params.snr_q8,
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
    assert (
        c_select(
            c_power,
            len(powers),
            ctypes.byref(c_params),
            ctypes.byref(c_state),
            ctypes.byref(result),
        )
        == 0
    )
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

    assert (
        c_select(powers, 128, ctypes.byref(c_params), ctypes.byref(c_state), ctypes.byref(result))
        == -1
    )
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

        assert (
            expected.window_start <= reference_bin < (expected.window_start + expected.window_bins)
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
    c_retention,
    accepted,
    ambiguous,
    candidates,
    selected,
    start,
    reason,
    expected_start,
    held,
    coasting,
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
        candidate_bins=candidates,
        candidate_power=(0,) * len(candidates),
        noise=1,
        selected_bin=selected,
        window_start=start,
        window_bins=12,
        confidence_q8=0,
        accepted=bool(accepted),
        ambiguous=bool(ambiguous),
        held_bin=held,
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


@pytest.mark.parametrize(
    "session,shot,ball_mph,candidates,powers,selected,held,accepted,ambiguous,noise,"
    "proposed_start,proposed_bins,reason",
    [
        # session_20260927_175111_range.jsonl (1.845 m tee, triggerCfg 41 6.0 2 12 3 0.0 1.0 1 1.5)
        ("175111", 1, 91.3, (51, 45), (11112, 1885), 53, 53, 0, 0, 115, 47, 12, "track_lost"),
        ("175111", 5, 39.2, (51, 48), (23245, 16770), 55, 55, 0, 0, 701, 49, 12, "track_lost"),
        ("175111", 6, 40.9, (48, 25), (60186, 19516), 51, 51, 0, 0, 1005, 45, 12, "track_lost"),
        ("175111", 7, 95.2, (47,), (25385,), 57, 57, 0, 0, 198, 51, 12, "track_lost"),
        ("175111", 8, 79.5, (51, 48), (37302, 12753), 53, 53, 0, 0, 586, 47, 12, "track_lost"),
        ("175111", 9, 95.4, (49, 51), (15429, 8524), 51, 51, 0, 0, 225, 45, 12, "track_lost"),
        # session_20260927_180613_range.jsonl (pitching wedge, same tee/triggerCfg)
        ("180613", 1, 65.0, (46, 57), (35282, 29594), 51, 51, 0, 0, 938, 45, 12, "track_lost"),
        ("180613", 3, 33.9, (23, 45), (50389, 35879), 49, 49, 0, 0, 883, 43, 12, "track_lost"),
        ("180613", 4, 53.5, (40, 47), (10535, 6106), 52, 52, 0, 0, 130, 46, 12, "track_lost"),
        ("180613", 5, 65.4, (46,), (16015,), 51, 51, 0, 0, 167, 45, 12, "track_lost"),
        ("180613", 6, 43.2, (46, 27), (21047, 18860), 48, 48, 0, 0, 401, 42, 12, "track_lost"),
        ("180613", 7, 46.2, (27, 49), (12198, 11925), 49, 49, 1, 1, 316, 43, 12, "ambiguous"),
        ("180613", 8, 34.2, (24, 47), (53147, 36675), 50, 50, 0, 0, 1268, 44, 12, "track_lost"),
        ("180613", 9, 35.7, (23, 49), (31333, 30684), 49, 49, 1, 1, 505, 43, 12, "ambiguous"),
        ("180613", 10, 47.1, (), (), 49, 49, 0, 0, 1, 43, 12, "track_lost"),
        # session_20260927_184327_range.jsonl (5 hybrid then 7 iron, same session)
        ("184327", 1, 37.4, (22, 24), (28948, 7832), 40, 40, 0, 0, 465, 34, 12, "track_lost"),
        ("184327", 2, 45.3, (48, 23), (15300, 14681), 48, 48, 1, 1, 327, 42, 12, "ambiguous"),
        ("184327", 3, 106.3, (48,), (39914,), 52, 52, 0, 0, 671, 46, 12, "track_lost"),
        ("184327", 4, 32.3, (48, 22), (9681, 9283), 48, 48, 1, 1, 226, 42, 12, "ambiguous"),
        ("184327", 5, 60.0, (49, 45), (18127, 3677), 53, 53, 0, 0, 222, 47, 12, "track_lost"),
        ("184327", 6, 41.7, (47, 44), (36187, 8285), 51, 51, 0, 0, 530, 45, 12, "track_lost"),
        # Only recorded case where ambiguous is set on a rejected (not accepted) candidate.
        ("184327", 7, 71.7, (45, 70), (12426, 11189), 56, 56, 0, 1, 368, 50, 12, "track_lost"),
        ("184327", 8, 39.5, (22, 47), (31630, 30326), 47, 47, 1, 1, 614, 41, 12, "ambiguous"),
        ("184327", 9, 83.9, (51, 46), (76757, 40033), 53, 53, 0, 0, 1096, 47, 12, "track_lost"),
        ("184327", 10, 49.5, (37,), (7970,), 45, 45, 0, 0, 62, 39, 12, "track_lost"),
        # session_20260927_185647_range.jsonl (driver/longer clubs, 56-115 mph)
        ("185647", 1, 76.8, (), (), 48, 48, 0, 0, 1, 42, 12, "track_lost"),
        ("185647", 2, 63.9, (39, 45), (24996, 19386), 47, 47, 0, 0, 586, 41, 12, "track_lost"),
        ("185647", 3, 68.2, (39, 37), (20505, 17564), 44, 44, 0, 0, 494, 38, 12, "track_lost"),
        ("185647", 4, 115.0, (44, 29), (23693, 10539), 46, 46, 0, 0, 408, 40, 12, "track_lost"),
        ("185647", 6, 103.6, (43, 63), (36996, 14961), 46, 46, 0, 0, 620, 40, 12, "track_lost"),
        ("185647", 7, 115.1, (39, 42), (16989, 10144), 45, 45, 0, 0, 301, 39, 12, "track_lost"),
        ("185647", 8, 115.5, (44, 38), (24619, 23177), 46, 46, 0, 1, 545, 40, 12, "track_lost"),
        ("185647", 9, 56.4, (43, 22), (10406, 9705), 37, 37, 0, 1, 281, 31, 12, "track_lost"),
        ("185647", 10, 61.7, (22, 44), (28162, 11171), 39, 39, 0, 0, 404, 33, 12, "track_lost"),
        ("185647", 11, 108.2, (34, 41), (30941, 2858), 48, 48, 0, 0, 327, 42, 12, "track_lost"),
    ],
)
def test_recorded_range_session_retention_stops_reproduce(
    c_retention,
    session,
    shot,
    ball_mph,
    candidates,
    powers,
    selected,
    held,
    accepted,
    ambiguous,
    noise,
    proposed_start,
    proposed_bins,
    reason,
):
    """Real ``RST`` records from four range sessions (per-shot detail in
    ``plans/iwr-trigger-timing-handoff.md``): every one of these hit either
    ``track_lost`` or ``ambiguous``, with no correlation to ball speed
    (32-115 mph on both sides fail the same way) and, for every
    ``track_lost`` case, ``held_bin == selected_bin`` -- the selector never
    switches to the wrong candidate, it loses continuity on the one it already
    confirmed. This locks in that observed behavior as a fixture so a future
    threshold change can be checked against real captures instead of only
    synthetic ones. Do not "fix" these by loosening a threshold without first
    understanding why the continuity check rejects a candidate this close to
    the held bin (session 180613 shots 7 and 9: two candidates within 1.02x
    power of each other, both correctly flagged ambiguous). ``accepted=false``
    with ``ambiguous=true`` set together (184327 shot 7; 185647 shots 8 and 9)
    still resolves to ``track_lost`` in every recorded instance, confirming
    ``accepted`` takes priority over ``ambiguous`` in the reason -- not a
    one-off, it recurs across sessions.
    """
    result = CResult()
    result.accepted = accepted
    result.ambiguous = ambiguous
    result.coasting = 0
    result.held_bin = held
    result.candidate_count = len(candidates)
    result.selected_bin = selected
    result.window_start = proposed_start
    result.window_bins = proposed_bins
    result.noise = noise
    for index, (bin_value, power) in enumerate(zip(candidates, powers)):
        result.candidate_bins[index] = bin_value
        result.candidate_power[index] = power

    code = c_retention(ctypes.byref(result), 128)

    assert RETENTION_REASON_NAMES[code] == reason, (
        f"session {session} shot {shot} ({ball_mph} mph): expected {reason}, "
        f"got {RETENTION_REASON_NAMES[code]}"
    )
    if reason == "track_lost":
        assert held == selected, "recorded track_lost stops never switch bins"


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
        actual.selected_bin,
        actual.window_start,
        bool(actual.accepted),
        actual.held_bin,
        bool(actual.coasting),
        actual.confidence_q8,
    ) == (
        expected.selected_bin,
        expected.window_start,
        expected.accepted,
        expected.held_bin,
        expected.coasting,
        expected.confidence_q8,
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
        assert truth + 2 < actual.window_start + actual.window_bins, (
            f"frame {frame} lost the far margin"
        )
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


# --- Coherent gate (plans/iwr-coherent-gate.md) -----------------------------
#
# The magnitude-rise detector above flickers on static clutter often enough to
# false-confirm a track: the 2026-09-26 100-cycle static soak kept flight
# frames in 7/100 cycles with nothing moving. A candidate bin is trustworthy
# only if it also shows up in the *coherent* (complex, not rectified)
# frame-to-frame difference well above the in-window mean; static returns
# cancel almost exactly under coherent differencing, but a bin a target is
# departing does not (that departure is what the magnitude rise already sees).

sys.path.insert(0, str(ROOT / "scripts" / "analysis"))
from iwr_selector_replay import (  # noqa: E402
    N_BINS,
    coherent_difference,
    hybrid_gated_rise,
    load_dump,
    magnitude_rise,
    masked_powers,
    run_selector,
    would_retain_flight_frames,
)

ALL_STATIC_CAPTURES = (
    sorted((SESSIONS / "iwr-adaptive-smoke").glob("*.l3dump"))
    + sorted((SESSIONS / "iwr-adaptive-confirm-smoke").glob("*.l3dump"))
    + sorted((SESSIONS / "iwr-adaptive-confirm-soak-no-fail").glob("*.l3dump"))
)


def test_recorded_static_soak_reproduces_the_2026_09_26_false_confirmations():
    """Sanity check on the replay tool itself, not the fix: the magnitude
    detector must still show real false confirmations on this hardware
    evidence, or the tests below would be exercising nothing."""
    if not ALL_STATIC_CAPTURES:
        pytest.skip("recorded static captures not present")
    false_confirms = sum(
        would_retain_flight_frames(magnitude_rise, *load_dump(path)) for path in ALL_STATIC_CAPTURES
    )
    assert false_confirms > 0, "replay tool no longer reproduces the known false confirmations"


def test_coherent_gate_removes_every_recorded_static_false_confirmation():
    if not ALL_STATIC_CAPTURES:
        pytest.skip("recorded static captures not present")
    false_confirms = [
        path
        for path in ALL_STATIC_CAPTURES
        if would_retain_flight_frames(hybrid_gated_rise, *load_dump(path))
    ]
    assert not false_confirms, f"gate still confirms a track on: {false_confirms}"


def test_coherent_gate_still_tracks_recorded_indoor_motion():
    if not MOTION_REFERENCE.exists():
        pytest.skip("recorded Phase 3 reference capture not present")
    meta, cube, starts, counts = load_dump(MOTION_REFERENCE)
    powers, _ = masked_powers(hybrid_gated_rise, meta, cube, starts, counts)
    results = run_selector(powers)
    accepted = [(frame, result.selected_bin) for frame, result in results if result.accepted]
    assert accepted, "the gate must still confirm the recorded moving object"
    # Bin 13 (~0.6 m) is the persistent mover both recorded motion captures show.
    assert all(abs(bin_index - 13) <= 1 for _frame, bin_index in accepted)


@pytest.mark.parametrize(
    "step_bins_per_frame",
    [3.2, 1.5],  # ~75 m/s and ~35 m/s ball speed at the 2 ms/frame, 128-bin profile
)
def test_coherent_gate_detects_an_injected_ball_at_least_as_often_as_current(
    step_bins_per_frame,
):
    """A synthetic point target with random per-chirp phase (Doppler), added to
    real recorded static IQ. Absolute detection rates are pessimistic (no real
    ball data exists yet); the comparison between detectors is the point."""
    if not ALL_STATIC_CAPTURES:
        pytest.skip("recorded static captures not present")
    rng_seed_captures = ALL_STATIC_CAPTURES[:20]
    detected = {"magnitude": 0, "hybrid": 0}
    for seed, path in enumerate(rng_seed_captures):
        meta, cube, starts, counts = load_dump(path)
        pre_frames = meta.get("retention", {}).get("pre_frames")
        if pre_frames is None:
            continue
        injected, truth = inject_synthetic_ball(
            cube,
            range(pre_frames, pre_frames + 6),
            starts[pre_frames],
            first_bin=starts[pre_frames] + 4,
            step=step_bins_per_frame,
            seed=seed,
        )
        for name, detector in (("magnitude", magnitude_rise), ("hybrid", hybrid_gated_rise)):
            if would_retain_flight_frames(detector, meta, injected, starts, counts):
                detected[name] += 1
    assert detected["hybrid"] >= detected["magnitude"]


def inject_synthetic_ball(cube, frames, window_start, first_bin, step, seed):
    """Add a point target moving `step` bins/frame with random per-chirp phase."""
    rng = np.random.default_rng(seed)
    amplitude = float(np.median(np.abs(cube[frames.start])))
    injected = cube.copy()
    truth = {}
    for offset, frame in enumerate(frames):
        target_bin = first_bin + round(offset * step)
        truth[frame] = target_bin
        phase = np.exp(1j * rng.uniform(0, 2 * np.pi, size=cube.shape[1:3]))
        injected[frame][:, :, target_bin - window_start] += amplitude * phase
    return injected, truth


@pytest.fixture(scope="module")
def c_coherent_gate(c_select):
    function = ctypes.CDLL(c_select.library_path).l3_coherent_gate
    function.argtypes = [
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.POINTER(ctypes.c_uint32),
    ]
    function.restype = None
    return function


def _run_gate(c_coherent_gate, rise, coherent, gate_q8=512):
    c_rise = (ctypes.c_uint32 * len(rise))(*rise)
    c_coherent = (ctypes.c_uint32 * len(coherent))(*coherent)
    c_gated = (ctypes.c_uint32 * len(rise))()
    c_coherent_gate(c_rise, c_coherent, len(rise), gate_q8, c_gated)
    expected = coherent_gate(list(rise), list(coherent), gate_q8)
    assert list(c_gated) == expected
    return expected


def test_coherent_gate_keeps_only_bins_well_above_the_mean(c_coherent_gate):
    rise = [10, 20, 30, 40]
    coherent = [100, 100, 250, 100]  # mean 137; only bin 2 clears 2x (274 needed... )
    gated = _run_gate(c_coherent_gate, rise, coherent)
    assert gated == [0, 0, 0, 0]  # none clear 2x the mean of 137 (274)
    gated = _run_gate(c_coherent_gate, [10, 20, 30, 40], [50, 50, 400, 50])
    assert gated == [0, 0, 30, 0]


def test_coherent_gate_matches_recorded_static_clutter_ratio(c_coherent_gate):
    # Static clutter measured at up to 1.51x its in-window mean; must be gated out.
    coherent = [140] * 127 + [211]  # 1.51x mean
    rise = [5] * 128
    gated = _run_gate(c_coherent_gate, rise, coherent)
    assert gated == [0] * 128


def test_coherent_gate_passes_a_target_at_the_recommended_factor(c_coherent_gate):
    coherent = [140] * 63 + [400] + [140] * 64  # ~2.8x mean, above the 2x gate
    rise = [5] * 63 + [90] + [5] * 64
    gated = _run_gate(c_coherent_gate, rise, coherent)
    assert gated[63] == 90
    assert sum(gated) == 90


def test_coherent_gate_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="same bins"):
        coherent_gate([1, 2], [1, 2, 3])


def test_coherent_gate_handles_all_zero_input(c_coherent_gate):
    gated = _run_gate(c_coherent_gate, [0] * 16, [0] * 16)
    assert gated == [0] * 16


def test_coherent_gate_c_matches_python_on_recorded_captures(c_coherent_gate):
    """Parity across real recorded static and motion captures, not just synthetic vectors."""
    if not ALL_STATIC_CAPTURES and not MOTION_REFERENCE.exists():
        pytest.skip("no recorded captures present")
    paths = ALL_STATIC_CAPTURES[:10] + ([MOTION_REFERENCE] if MOTION_REFERENCE.exists() else [])
    checked = 0
    for path in paths:
        meta, cube, starts, counts = load_dump(path)
        for frame in range(1, meta["n_frames"]):
            if (starts[frame], counts[frame]) != (starts[frame - 1], counts[frame - 1]):
                continue
            rise = [int(v) for v in magnitude_rise(cube, frame, starts, counts)]
            coherent = [int(v) for v in coherent_difference(cube, frame, starts, counts)]
            _run_gate(c_coherent_gate, rise, coherent)
            checked += 1
    assert checked > 0


def _to_absolute_bins(values: list[int], start: int) -> list[int]:
    power = [0] * N_BINS
    for offset, value in enumerate(values):
        power[start + offset] = value
    return power


def test_gate_mean_must_exclude_the_masked_out_bins():
    """2026-09-26 hardware regression: l3_dump.c passed the coherent gate the
    full 128-bin array *after* masking everything outside the ~53-bin active
    window to zero, diluting the mean to ~41% of the true in-window value and
    weakening the intended 2x gate to an effective ~0.83x. Reproduced on the
    operator's cycle-18 static-soak capture, which the (buggy) first coherent
    release still false-confirmed at bin 42 despite a 0.92-1.02x true ratio
    there. This test encodes the two call shapes so a future change cannot
    silently regress to the diluted-mean integration.

    The frames checked here are impact frames, whose *stored* window happens
    to equal the active analysis window exactly (by construction: the impact
    phase's retained window is defined as [impactStart, impactStart+
    impactBins)). The firmware's actual gate call, though, always operates on
    the full 128-bin scratch array regardless of what a frame's retained
    window is, so the dilution has to be reproduced at that width, not at the
    frame's own (already window-sized) sample count.
    """
    path = SESSIONS / "iwr-adaptive-coherent-soak" / "shadow-reference-018.l3dump"
    if not path.exists():
        pytest.skip("recorded regression capture not present")
    meta, cube, starts, counts = load_dump(path)
    active_start, active_bins = 32, 53  # this capture's impact window
    # Frames where bin 42's own magnitude rise is nonzero, i.e. actually a
    # candidate the gate has something to decide about (17 and 19 track the
    # coasting prediction to a different bin and have nothing at 42 to gate).
    for frame in (15, 16, 18):
        rise = [int(v) for v in magnitude_rise(cube, frame, starts, counts)]
        coherent = [int(v) for v in coherent_difference(cube, frame, starts, counts)]
        full_rise = _to_absolute_bins(rise, starts[frame])
        full_coherent = _to_absolute_bins(coherent, starts[frame])
        for bin_index in range(N_BINS):
            if bin_index < active_start or bin_index >= active_start + active_bins:
                full_rise[bin_index] = 0
                full_coherent[bin_index] = 0
        # The bug: gate the already-masked full 128-bin array (mean diluted).
        buggy = coherent_gate(full_rise, full_coherent)
        # The fix: gate only the active-window slice, mean taken over just it.
        fixed_slice = coherent_gate(
            full_rise[active_start : active_start + active_bins],
            full_coherent[active_start : active_start + active_bins],
        )
        assert buggy[42] != 0, (
            "the diluted-mean call must still pass this static bin through "
            "(this documents the bug, not the desired behavior)"
        )
        assert fixed_slice[42 - active_start] == 0, (
            f"frame {frame}: the correctly-scoped gate must reject this static bin"
        )


@pytest.fixture(scope="module")
def c_format_stop(c_select):
    function = ctypes.CDLL(c_select.library_path).l3_format_retention_stop
    function.argtypes = [
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.POINTER(CResult),
        ctypes.c_char_p,
        ctypes.c_uint32,
    ]
    function.restype = ctypes.c_int32
    return function


@pytest.mark.parametrize(
    "accepted,ambiguous,coasting,candidates,powers,selected,held,reason",
    [
        # No candidate associated and no coast budget: the track is lost.
        (0, 0, 0, (), (), 41, 0, "track_lost"),
        # Two candidates too far apart for one retained window.
        (1, 1, 0, (30, 50), (9000, 8000), 30, 0, "ambiguous"),
        # Coasting prediction too far from the held bin to fit one window.
        (0, 0, 1, (30,), (700,), 38, 30, "track_lost"),
    ],
)
def test_retention_stop_record_round_trips_to_the_host(
    c_retention,
    c_format_stop,
    accepted,
    ambiguous,
    coasting,
    candidates,
    powers,
    selected,
    held,
    reason,
):
    result = CResult()
    result.accepted = accepted
    result.ambiguous = ambiguous
    result.coasting = coasting
    result.candidate_count = len(candidates)
    for index, (candidate, power) in enumerate(zip(candidates, powers)):
        result.candidate_bins[index] = candidate
        result.candidate_power[index] = power
    result.selected_bin = selected
    result.held_bin = held
    result.window_start = selected - 6
    result.window_bins = 12
    result.noise = 120

    code = c_retention(ctypes.byref(result), 128)
    buffer = ctypes.create_string_buffer(128)
    written = c_format_stop(code, 22, ctypes.byref(result), buffer, len(buffer))

    assert 0 < written < len(buffer)
    stop = parse_retention_stop(buffer.value)
    assert stop == {
        "reason": reason,
        "frame": 22,
        "candidate_bins": list(candidates),
        "candidate_powers": list(powers),
        "selected_bin": selected,
        "held_bin": held,
        "accepted": bool(accepted),
        "ambiguous": bool(ambiguous),
        "coasting": bool(coasting),
        "noise": 120,
        "proposed_start": selected - 6,
        "proposed_bins": 12,
    }


def test_retention_stop_record_fits_the_firmware_buffer_at_field_maxima(c_format_stop):
    result = CResult()
    result.candidate_count = 2
    result.candidate_bins[0] = result.candidate_bins[1] = 0xFFFF
    result.candidate_power[0] = result.candidate_power[1] = 0xFFFFFFFF
    result.noise = 0xFFFFFFFF
    result.selected_bin = result.held_bin = 0xFFFF
    result.window_start = result.window_bins = 0xFFFF
    result.accepted = result.ambiguous = result.coasting = 1
    buffer = ctypes.create_string_buffer(128)

    written = c_format_stop(4, 0xFFFF, ctypes.byref(result), buffer, len(buffer))

    # l3_cli_dump formats into a 128-byte stack buffer.
    assert 0 < written < len(buffer)
