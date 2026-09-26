"""Parity between the firmware tracker (track_select.c) and the host planner.

The firmware ``l3track`` command runs ``firmware/iwr6843/track_select.c`` on
the R4F. That file is plain C99, so these tests build it with the host C
compiler and drive it through ctypes. Given the same random pairs as NumPy,
it must name exactly the cells ``sparse.plan_cells`` names.
"""

from __future__ import annotations

import ctypes
import inspect
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openflight.iwr6843 import sparse, tracking
from openflight.iwr6843.sparse import PowerSummary, parse_power, plan_cells
from openflight.iwr6843.tracking import Geometry, find_ball_from_power

SOURCE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "track_select.c"
MAX_FRAMES = 64
MAX_LOOPS = 16
MAX_BINS = 64
MAX_GATES = 2
MAX_ROWS = MAX_FRAMES * MAX_LOOPS
MAX_DETECTIONS = MAX_ROWS * MAX_GATES
RES_M = tracking.RANGE_SPAN_M / 128


class Gate(ctypes.Structure):
    _fields_ = [("loM", ctypes.c_double), ("hiM", ctypes.c_double)]


class Layout(ctypes.Structure):
    _fields_ = [
        ("nFrames", ctypes.c_uint32),
        ("nLoops", ctypes.c_uint32),
        ("maxBins", ctypes.c_uint32),
        ("binStarts", ctypes.POINTER(ctypes.c_uint8)),
        ("binCounts", ctypes.POINTER(ctypes.c_uint8)),
        ("framePeriodS", ctypes.c_double),
        ("loopPeriodS", ctypes.c_double),
        ("rangeResM", ctypes.c_double),
    ]


class Params(ctypes.Structure):
    _fields_ = [
        ("ballGates", Gate * MAX_GATES),
        ("nBallGates", ctypes.c_uint32),
        ("maxRangeM", ctypes.c_double),
        ("clubGate", Gate),
        ("snrMin", ctypes.c_double),
        ("speedMinMs", ctypes.c_double),
        ("speedMaxMs", ctypes.c_double),
        ("fastTrackMs", ctypes.c_double),
        ("fastSupportFrac", ctypes.c_double),
        ("minPairDtS", ctypes.c_double),
        ("cellPadS", ctypes.c_double),
        ("iterations", ctypes.c_uint32),
        ("minDetections", ctypes.c_uint32),
        ("cellMargin", ctypes.c_uint32),
        ("clubMargin", ctypes.c_uint32),
    ]


class Result(ctypes.Structure):
    _fields_ = [
        ("found", ctypes.c_uint8),
        ("nInliers", ctypes.c_uint32),
        ("slopeBins", ctypes.c_double),
        ("interceptBins", ctypes.c_double),
        ("rmsBins", ctypes.c_double),
        ("tFirstS", ctypes.c_double),
        ("tLastS", ctypes.c_double),
    ]


class Workspace(ctypes.Structure):
    _fields_ = [
        ("detRow", ctypes.c_uint16 * MAX_DETECTIONS),
        ("detBin", ctypes.c_float * MAX_DETECTIONS),
        ("detCount", ctypes.c_uint32 * MAX_GATES),
        ("order", ctypes.c_uint16 * MAX_DETECTIONS),
        ("nOrder", ctypes.c_uint32),
        ("row", ctypes.c_float * MAX_BINS),
        ("scratch", ctypes.c_float * MAX_BINS),
        ("cellMask", ctypes.c_uint64 * MAX_FRAMES),
    ]


class Rng(ctypes.Structure):
    _fields_ = [("state", ctypes.c_uint32)]


ROW_FN = ctypes.CFUNCTYPE(
    None,
    ctypes.c_void_p,
    ctypes.c_uint32,
    ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_uint32,
)
PAIR_FN = ctypes.CFUNCTYPE(
    None,
    ctypes.c_void_p,
    ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
)


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("no host C compiler to build track_select.c")
    out = tmp_path_factory.mktemp("track_select") / "libtrack_select.so"
    subprocess.run(
        [
            compiler,
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-ffp-contract=off",
            "-shared",
            "-fPIC",
            "-o",
            str(out),
            str(SOURCE),
            "-lm",
        ],
        check=True,
    )
    library = ctypes.CDLL(str(out))
    library.l3track_default_params.argtypes = [ctypes.POINTER(Params)]
    library.l3track_default_params.restype = None
    library.l3track_inlier_tol.argtypes = [ctypes.c_uint32]
    library.l3track_inlier_tol.restype = ctypes.c_double
    library.l3track_round.argtypes = [ctypes.c_double]
    library.l3track_round.restype = ctypes.c_double
    library.l3track_select.argtypes = [
        ctypes.POINTER(Layout),
        ctypes.POINTER(Params),
        ROW_FN,
        ctypes.c_void_p,
        PAIR_FN,
        ctypes.c_void_p,
        ctypes.POINTER(Workspace),
        ctypes.POINTER(Result),
    ]
    library.l3track_select.restype = ctypes.c_int32
    library.l3track_rng_seed.argtypes = [ctypes.POINTER(Rng), ctypes.c_uint32]
    library.l3track_rng_seed.restype = None
    return library


def _numpy_pairs(seed: int = 1):
    """The draws find_ball_from_power makes, one per RANSAC iteration."""
    rng = np.random.default_rng(seed)

    def draw(_ctx, n, i, j):
        first, second = rng.choice(n, 2, replace=False)
        i[0] = int(first)
        j[0] = int(second)

    return PAIR_FN(draw)


def _run_c(
    lib,
    summary: PowerSummary,
    *,
    max_range_m: float | None,
    club_gate_m: tuple[float, float] | None,
    firmware_rng: bool = False,
    layout_override: dict | None = None,
):
    geometry = summary.geometry
    frames = geometry.n_frames
    starts = (ctypes.c_uint8 * MAX_FRAMES)(*[geometry.frame_bin_start(f) for f in range(frames)])
    counts = (ctypes.c_uint8 * MAX_FRAMES)(*[geometry.frame_bin_count(f) for f in range(frames)])
    layout = Layout(
        nFrames=frames,
        nLoops=summary.n_loops,
        maxBins=geometry.n_samples,
        binStarts=starts,
        binCounts=counts,
        framePeriodS=geometry.frame_period_s,
        loopPeriodS=geometry.loop_period_s,
        rangeResM=geometry.range_res_m,
    )
    for name, value in (layout_override or {}).items():
        setattr(layout, name, value)
    params = Params()
    lib.l3track_default_params(ctypes.byref(params))
    params.maxRangeM = max_range_m or 0.0
    if club_gate_m is not None:
        params.clubGate.loM, params.clubGate.hiM = club_gate_m
    power = summary.power

    def row(_ctx, frame, loop, out, count):
        values = power[frame * summary.n_loops + loop]
        for index in range(count):
            out[index] = float(values[index])

    row_fn = ROW_FN(row)
    if firmware_rng:
        rng = Rng()
        lib.l3track_rng_seed(ctypes.byref(rng), 1)
        pair_fn = ctypes.cast(lib.l3track_rng_pair, PAIR_FN)
        pair_ctx = ctypes.cast(ctypes.byref(rng), ctypes.c_void_p)
    else:
        pair_fn = _numpy_pairs()
        pair_ctx = None
    workspace = Workspace()
    result = Result()
    count = lib.l3track_select(
        ctypes.byref(layout),
        ctypes.byref(params),
        row_fn,
        None,
        pair_fn,
        pair_ctx,
        ctypes.byref(workspace),
        ctypes.byref(result),
    )
    cells = {
        (frame, local)
        for frame in range(min(frames, MAX_FRAMES))
        for local in range(MAX_BINS)
        if workspace.cellMask[frame] >> local & 1
    }
    return count, cells, result


def _summary(
    *,
    frames: int = 24,
    loops: int = 12,
    starts: list[int] | None = None,
    width: int = 53,
    ball: tuple[float, float] | None = (2.3, 45.0),
    slow: tuple[float, float] | None = None,
    club_m: float | None = None,
    seed: int = 0,
    frame_period_s: float = 0.003,
    flat_noise: bool = False,
) -> PowerSummary:
    """A float32 residual-power map with moving returns, sent through ILP1.

    ``ball``/``slow`` are (start range m, radial m/s). ``club_m`` puts a
    return near the tee in the early frames, as the club approaches.
    Exponential noise alone clears the SNR gate now and then, as on
    hardware; ``flat_noise`` keeps the background below it.
    """
    rng = np.random.default_rng(seed)
    starts = starts or [20] * frames
    shape = (frames * loops, width)
    noise = rng.uniform(0.9, 1.1, size=shape) if flat_noise else rng.exponential(1.0, size=shape)
    power = noise.astype(np.float32)
    bins = np.arange(width)

    def add(frame, loop, absolute_bin, amplitude):
        local = absolute_bin - starts[frame]
        shape = amplitude * np.exp(-0.5 * ((bins - local) / 0.6) ** 2)
        power[frame * loops + loop] += shape.astype(np.float32)

    for frame in range(frames):
        for loop in range(loops):
            t_s = frame * frame_period_s + loop * tracking.LOOP_PRI_S
            if ball is not None:
                add(frame, loop, (ball[0] + ball[1] * t_s) / RES_M, 300.0)
            if slow is not None:
                add(frame, loop, (slow[0] + slow[1] * t_s) / RES_M, 500.0)
            if club_m is not None and frame < 6:
                add(frame, loop, club_m / RES_M, 200.0)
    uniform = all(start == starts[0] for start in starts)
    geometry = Geometry(
        n_frames=frames,
        chirps_per_frame=loops * 2,
        n_tx=2,
        n_rx=4,
        n_samples=width,
        frame_period_s=frame_period_s,
        trigger_frame=0,
        range_bin_start=starts[0],
        range_fft_size=128,
        range_bin_starts=None if uniform else tuple(starts),
        range_bin_counts=None if uniform else tuple([width] * frames),
    )
    summary = PowerSummary(
        power=power,
        n_tx=2,
        n_rx=4,
        n_loops=loops,
        noise_power=0.0,
        geometry=geometry,
    )
    # Round-trip so both sides see exactly what the host decodes on hardware.
    return parse_power(summary.to_bytes())


WINDOWED = [20] * 12 + [32] * 12


def _assert_parity(lib, summary, *, max_range_m=None, club_gate_m=None):
    expected = plan_cells(summary, max_range_m=max_range_m, club_gate_m=club_gate_m)
    count, cells, result = _run_c(lib, summary, max_range_m=max_range_m, club_gate_m=club_gate_m)
    assert cells == set(expected)
    assert count == len(expected)
    track = find_ball_from_power(summary.power, summary.geometry, max_range_m=max_range_m)
    assert bool(result.found) == (track is not None)
    if track is not None:
        assert result.nInliers == track.n_inliers
        assert result.slopeBins == pytest.approx(track.slope_bins, rel=1e-9)
        assert result.interceptBins == pytest.approx(track.intercept_bins, rel=1e-9)
        assert result.rmsBins == pytest.approx(track.rms_bins, rel=1e-6, abs=1e-9)
        assert result.tFirstS == pytest.approx(track.t_first)
        assert result.tLastS == pytest.approx(track.t_last)
    return expected, result


def test_uniform_window_matches_host_planner(lib):
    expected, result = _assert_parity(lib, _summary())

    assert result.found
    assert result.slopeBins * RES_M == pytest.approx(45.0, rel=0.02)
    assert len(expected) > 20


def test_windowed_capture_matches_host_planner(lib):
    """Per-frame windows switch the host to row-major detection order."""
    summary = _summary(starts=WINDOWED)
    assert summary.geometry.range_bin_starts is not None

    _expected, result = _assert_parity(lib, summary)

    assert result.found


def test_fast_ball_beats_slower_tee_as_host_does(lib):
    """Fastest-credible selection: the slow, strong tee streak must lose."""
    summary = _summary(starts=WINDOWED, ball=(2.3, 30.0), slow=(2.3, 18.0))
    # The slow streak has the most inliers, so selection must override it.
    most_inliers = find_ball_from_power(summary.power, summary.geometry, min_ball_ms=1e9)
    assert most_inliers.speed_ms < tracking.FAST_TRACK_MS

    _expected, result = _assert_parity(lib, summary)

    assert result.slopeBins * RES_M == pytest.approx(30.0, rel=0.02)


def test_club_gate_cells_match_host_planner(lib):
    summary = _summary(starts=WINDOWED, club_m=1.3)
    gate = (0.8, 1.45)

    expected, _result = _assert_parity(lib, summary, club_gate_m=gate)

    club_cells = [(frame, local) for frame, local in expected if local + 20 < 2.25 / RES_M]
    assert club_cells


def test_net_clamp_matches_host_planner(lib):
    """The ball gate stops short of the net on both sides."""
    _assert_parity(lib, _summary(ball=(2.3, 60.0)), max_range_m=3.0)


def test_noise_only_capture_keeps_club_cells_without_a_track(lib):
    summary = _summary(ball=None, club_m=1.3, flat_noise=True)

    expected, result = _assert_parity(lib, summary, club_gate_m=(0.8, 1.45))

    assert not result.found
    assert expected


def test_empty_capture_names_no_cells(lib):
    summary = _summary(ball=None, flat_noise=True)

    expected, result = _assert_parity(lib, summary)

    assert expected == []
    assert not result.found


def test_short_streak_below_minimum_detections_has_no_track(lib):
    """Fewer than eight detections: both sides give up on the ball."""
    summary = _summary(frames=1, loops=6, starts=[20])

    _expected, result = _assert_parity(lib, summary)

    assert not result.found


def test_noise_streaks_match_host_planner(lib):
    """Exponential noise alone can clear the SNR gate; both sides agree on it."""
    for seed in range(4):
        _assert_parity(lib, _summary(ball=None, seed=seed), club_gate_m=(0.8, 1.45))


@pytest.mark.parametrize("seed", range(12))
def test_randomised_captures_match_host_planner(lib, seed):
    rng = np.random.default_rng(100 + seed)
    windowed = bool(seed % 2)
    summary = _summary(
        starts=WINDOWED if windowed else None,
        ball=(float(rng.uniform(2.2, 2.6)), float(rng.uniform(28.0, 75.0))),
        slow=(float(rng.uniform(2.3, 2.6)), float(rng.uniform(15.0, 25.0))) if seed % 3 else None,
        club_m=float(rng.uniform(1.0, 1.4)) if seed % 4 else None,
        seed=seed,
    )
    max_range = 4.2 if seed % 5 == 0 else None

    _assert_parity(lib, summary, max_range_m=max_range, club_gate_m=(0.8, 1.45))


def test_firmware_rng_finds_the_same_ball(lib):
    """The firmware's own xorshift draws land on the host's track."""
    summary = _summary(starts=WINDOWED, slow=(2.3, 18.0), club_m=1.3)
    gate = (0.8, 1.45)
    expected = set(plan_cells(summary, max_range_m=None, club_gate_m=gate))
    track = find_ball_from_power(summary.power, summary.geometry)

    _count, cells, result = _run_c(
        lib, summary, max_range_m=None, club_gate_m=gate, firmware_rng=True
    )

    assert result.found
    assert result.slopeBins == pytest.approx(track.slope_bins, rel=0.01)
    assert len(cells & expected) >= 0.95 * len(expected)


def test_firmware_rng_draws_distinct_indices(lib):
    rng = Rng()
    lib.l3track_rng_seed(ctypes.byref(rng), 0)
    pair = ctypes.cast(lib.l3track_rng_pair, PAIR_FN)
    first = ctypes.c_uint32()
    second = ctypes.c_uint32()
    for n in (2, 3, 17, 500):
        for _ in range(200):
            pair(ctypes.cast(ctypes.byref(rng), ctypes.c_void_p), n, first, second)
            assert first.value != second.value
            assert first.value < n and second.value < n


@pytest.mark.parametrize(
    "override",
    [
        {"nLoops": MAX_LOOPS + 1},
        {"maxBins": MAX_BINS + 1},
        {"maxBins": 40},  # narrower than the frames it describes
        {"rangeResM": 0.0},
        {"nFrames": 0},
    ],
)
def test_layout_beyond_firmware_limits_is_rejected(lib, override):
    count, _cells, result = _run_c(
        lib, _summary(), max_range_m=None, club_gate_m=None, layout_override=override
    )

    assert count == -1
    assert not result.found


def test_round_matches_python_half_even(lib):
    for value in (-2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 47.5, 48.5, 3.49999, 3.50001, -3.7, 12.0):
        assert lib.l3track_round(value) == round(value)


def test_inlier_tolerance_follows_stored_width(lib):
    assert lib.l3track_inlier_tol(53) == 0.8
    assert lib.l3track_inlier_tol(127) == 0.8
    assert lib.l3track_inlier_tol(128) == 1.2


def test_default_params_mirror_host_constants(lib):
    """A host constant that moves without the C table fails here."""
    params = Params()
    lib.l3track_default_params(ctypes.byref(params))
    detections = inspect.signature(tracking._detections).parameters  # pylint: disable=protected-access
    finder = inspect.signature(find_ball_from_power).parameters
    cells = inspect.signature(sparse.track_cells).parameters

    gates = tuple(
        (params.ballGates[i].loM, params.ballGates[i].hiM) for i in range(params.nBallGates)
    )
    assert gates == tracking.BALL_GATES_M
    assert params.snrMin == detections["snr_min"].default
    assert (params.speedMinMs, params.speedMaxMs) == tracking.SPEED_BOUNDS_MS
    assert params.fastTrackMs == tracking.FAST_TRACK_MS
    assert params.fastSupportFrac == tracking.FAST_SUPPORT_FRAC
    assert params.iterations == finder["iterations"].default
    assert params.cellMargin == cells["margin"].default
    assert params.clubMargin == sparse.CLUB_CELL_MARGIN
    source = inspect.getsource(find_ball_from_power)
    assert "loops_idx.size < 8" in source and params.minDetections == 8
    assert "abs(d_t) < 3e-3" in source and params.minPairDtS == 3e-3
    assert "2e-3" in inspect.getsource(sparse.track_cells) and params.cellPadS == 2e-3


# --- runtime: one source for the rig limits --------------------------------

from openflight.iwr6843.calibration import Calibration  # noqa: E402
from openflight.iwr6843.club import CLUB_APPROACH_DEPTH_M, CLUB_GATE_TEE_MARGIN_M  # noqa: E402
from openflight.iwr6843.driver import IWR6843Radar  # noqa: E402
from openflight.iwr6843.dump import parse_dump  # noqa: E402
from openflight.iwr6843.runtime import IWR6843Runtime  # noqa: E402
from openflight.iwr6843.sparse import TRACK_MAGIC, OnboardTrack, vertical_loop_power  # noqa: E402


def _runtime(*, tee_m=1.4, net_m=4.064, flight="net", config_path=None):
    calibration = Calibration.identity()
    calibration.tee_range_m = tee_m
    return IWR6843Runtime(
        capture_monitor=(SimpleNamespace(config_path=config_path) if config_path else None),
        calibration=calibration,
        net_range_m=net_m,
        flight_mode=flight,
    )


def _track_cfg_fields(command: str) -> list[float]:
    name, *fields = command.split()
    assert name == "trackCfg"
    assert len(fields) == 5
    return [float(field) for field in fields]


def test_track_config_sends_the_host_planner_limits_exactly():
    runtime = _runtime()
    loop_s, res_m, max_range, club_lo, club_hi = _track_cfg_fields(runtime.track_config_command())

    assert loop_s == tracking.LOOP_PRI_S
    assert (
        res_m
        == RES_M
        == Geometry(1, 2, 2, 4, 53, 0.003, 0, range_fft_size=sparse.RANGE_FFT_SIZE).range_res_m
    )
    assert max_range == runtime.tracking_net_m - 0.25
    assert club_lo == 1.4 - CLUB_APPROACH_DEPTH_M
    assert club_hi == 1.4 + CLUB_GATE_TEE_MARGIN_M


def test_track_config_uses_three_tx_profile_loop_period(tmp_path):
    config = tmp_path / "radar.cfg"
    config.write_text(
        "\n".join(
            (
                "profileCfg 0 60.0 7 3 38 0 0 100 1 128 4000 0 0 30",
                "chirpCfg 0 0 0 0 0 0 0 1",
                "chirpCfg 1 1 0 0 0 0 0 2",
                "chirpCfg 2 2 0 0 0 0 0 4",
                "frameCfg 0 2 12 0 2 1 0",
            )
        )
        + "\n",
        encoding="utf-8",
    )

    loop_s, *_rest = _track_cfg_fields(_runtime(config_path=config).track_config_command())

    assert loop_s == pytest.approx(135e-6)


def test_track_config_open_flight_has_no_net_clamp():
    runtime = _runtime(flight="range")

    assert runtime.tracking_net_m is None
    assert _track_cfg_fields(runtime.track_config_command())[2] == 0.0


def test_track_config_without_tee_disables_club_cells():
    _loop, _res, _max, club_lo, club_hi = _track_cfg_fields(
        _runtime(tee_m=None).track_config_command()
    )

    assert club_hi <= club_lo


def test_club_gate_floor_near_the_radar():
    _loop, _res, _max, club_lo, _hi = _track_cfg_fields(_runtime(tee_m=0.7).track_config_command())

    assert club_lo == 0.35


def test_runtime_planner_and_firmware_use_the_same_limits(lib):
    """plan_sparse_cells and trackCfg must describe the same search."""
    runtime = _runtime(net_m=3.4)
    summary = _summary(starts=WINDOWED, ball=(2.3, 60.0), club_m=1.2)
    _loop, _res, max_range, club_lo, club_hi = _track_cfg_fields(runtime.track_config_command())

    _count, cells, _result = _run_c(
        lib, summary, max_range_m=max_range, club_gate_m=(club_lo, club_hi)
    )

    assert cells == set(runtime.plan_sparse_cells(summary).cells)


# --- end to end: firmware stream -> driver -> the same dump ------------------


class _Stream:
    def __init__(self, stream: bytes):
        self._stream = stream

    @property
    def in_waiting(self) -> int:
        return len(self._stream)

    def read(self, count: int) -> bytes:
        chunk, self._stream = self._stream[:count], self._stream[count:]
        return chunk

    def write(self, _data: bytes) -> None:
        return None

    def reset_input_buffer(self) -> None:
        return None


def test_firmware_cells_rebuild_the_dump_the_host_planner_would(lib):
    """Same capture through l3track and l3sparse gives identical dump bytes."""
    rng = np.random.default_rng(7)
    frames, loops, bins = 24, 12, 53
    cube = (
        rng.normal(size=(frames, loops * 2, 4, bins))
        + 1j * rng.normal(size=(frames, loops * 2, 4, bins))
    ).astype(np.complex64) * 3
    for frame in range(frames):
        for loop in range(loops):
            t_s = frame * 0.003 + loop * tracking.LOOP_PRI_S
            local = int(round((2.3 + 45.0 * t_s) / RES_M)) - 20
            if 0 <= local < bins:
                # A phase that turns loop to loop, so burst MTI keeps it.
                cube[frame, 2 * loop : 2 * loop + 2, :, local] += 120 * np.exp(1j * 2.1 * loop)
    geometry = Geometry(
        n_frames=frames,
        chirps_per_frame=loops * 2,
        n_tx=2,
        n_rx=4,
        n_samples=bins,
        frame_period_s=0.003,
        trigger_frame=0,
        range_bin_start=20,
        range_fft_size=128,
    )
    summary = parse_power(vertical_loop_power(cube, n_tx=2, geometry=geometry).to_bytes())
    planned = plan_cells(summary, max_range_m=None, club_gate_m=None)
    assert planned

    _count, cells, result = _run_c(lib, summary, max_range_m=None, club_gate_m=None)
    track = OnboardTrack(
        found=bool(result.found),
        n_inliers=result.nInliers,
        slope_bins=result.slopeBins,
        intercept_bins=result.interceptBins,
        rms_bins=result.rmsBins,
        t_first=result.tFirstS,
        t_last=result.tLastS,
    )
    # The firmware streams cells frame by frame, bin by bin.
    stream = (
        summary.header_bytes(TRACK_MAGIC)
        + track.to_bytes()
        + summary.pack_slices(cube, sorted(cells))
        + b"Done\n"
    )
    radar = IWR6843Radar.__new__(IWR6843Radar)
    radar.ser = _Stream(stream)

    raw, _noise, onboard = radar.read_tracked()
    host_raw = sparse.assemble_dump(summary, summary.pack_slices(cube, planned))

    assert raw == host_raw
    ball = onboard.ball_track(summary.geometry.range_res_m)
    assert ball.speed_ms == pytest.approx(45.0, rel=0.02)
    _meta, rebuilt = parse_dump(raw)
    assert np.count_nonzero(np.any(rebuilt != 0, axis=(1, 2))) == len(planned)
