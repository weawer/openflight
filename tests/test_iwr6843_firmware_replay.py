"""Tests for the IWR6843 firmware replay harness, openflight.iwr6843.firmware_replay.

The harness must compute the observations ``l3_dump.c`` computes on the
board (checked against a line-by-line port of ``l3_verticalResidual``) and,
fed a synthetic swing, must drive the compiled trigger and club track to one
continuous approach trajectory: one acquisition, no gap, a fitted speed equal
to the club's radial speed. Recorded captures in tests/radar/recordings run
through the same path when present.
"""

from __future__ import annotations

import ctypes
import importlib.util
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from iwr6843_synth import CLUB_SPEED_MS, FRAME_PERIOD_S, synth_club_dump, synth_shot_dump

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.dump import SAMPLE_INT16_IQ, SAMPLE_RANGE_FFT_IQ16, pack_dump, parse_dump
from openflight.iwr6843.firmware_replay import (
    RECORDINGS_DIR,
    BallTuning,
    ReplayConfig,
    RetainReplay,
    bin_observation_table,
    bin_observations,
    channel_snapshot,
    format_report,
    frame_timestamps_us,
    frame_window,
    recording_configs,
    recording_expectations,
    replay_dump,
    static_channel_snapshot,
    vertical_tx_indices,
)

SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "replay_iwr_track.py"
BIN_M = 6.0 / 128
TEE_RANGE_M = 1.372
TEE_BIN = int(TEE_RANGE_M / BIN_M)  # 29


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


@pytest.fixture(scope="module")
def swing() -> bytes:
    return synth_club_dump(0.0, tee_range_m=TEE_RANGE_M)


def _reference_residual(cube, frame, local_bin, n_tx):
    """``l3_verticalResidual`` transcribed loop for loop, float64."""
    chirps, n_rx = cube.shape[1], cube.shape[2]
    loops = chirps // n_tx
    loop_power = [0.0] * loops
    energy = r1re = r1im = 0.0
    for tx in range(n_tx):
        if n_tx == 3 and tx == 1:
            continue
        for rx in range(n_rx):
            samples = [cube[frame, loop * n_tx + tx, rx, local_bin] for loop in range(loops)]
            mean_re = sum(s.real for s in samples) / loops
            mean_im = sum(s.imag for s in samples) / loops
            prev_re = prev_im = 0.0
            for loop, s in enumerate(samples):
                re, im = s.real - mean_re, s.imag - mean_im
                power = im * im + re * re
                energy += power
                loop_power[loop] += power
                if loop > 0:
                    r1re += re * prev_re + im * prev_im
                    r1im += im * prev_re - re * prev_im
                prev_re, prev_im = re, im
    return energy, max(loop_power), loop_power[0], r1re, r1im


def _random_cube(seed: int, n_tx: int, loops: int = 12, n_rx: int = 4, bins: int = 20):
    rng = np.random.default_rng(seed)
    shape = (2, loops * n_tx, n_rx, bins)
    return rng.integers(-2000, 2000, shape) + 1j * rng.integers(-2000, 2000, shape)


@pytest.mark.parametrize("n_tx", [3, 2, 1])
def test_bin_observations_match_a_line_by_line_port_of_the_firmware(n_tx):
    cube = _random_cube(n_tx, n_tx)
    obs = bin_observations(cube, 1, 3, 9, n_tx)
    assert len(obs) == 9
    for i in range(9):
        expected = _reference_residual(cube, 1, 3 + i, n_tx)
        got = (obs[i].energy, obs[i].peak, obs[i].loop0, obs[i].r1Re, obs[i].r1Im)
        for name, e, g in zip(("energy", "peak", "loop0", "r1Re", "r1Im"), expected, got):
            assert g == pytest.approx(e, rel=1e-5, abs=1e-2), (i, name)


def test_the_observation_table_is_the_ctypes_array_as_numpy():
    cube = _random_cube(3, 3)
    table = bin_observation_table(cube, 1, 2, 7, 3)
    obs = bin_observations(cube, 1, 2, 7, 3)
    assert table.shape == (7,)
    assert table.tobytes() == bytes(obs)
    assert set(table.dtype.names) == {"energy", "peak", "loop0", "r1Re", "r1Im"}


def test_three_tx_loops_skip_the_azimuth_element_and_fewer_use_every_transmitter():
    assert vertical_tx_indices(3) == (0, 2)
    assert vertical_tx_indices(2) == (0, 1)
    assert vertical_tx_indices(1) == (0,)
    with pytest.raises(ValueError):
        vertical_tx_indices(0)


def test_a_static_target_leaves_no_residual():
    cube = np.zeros((1, 36, 4, 8), dtype=complex)
    cube[0, :, :, 5] = 1000.0 + 250.0j  # identical in every loop
    obs = bin_observations(cube, 0, 0, 8, 3)
    assert all(obs[i].energy == 0.0 and obs[i].peak == 0.0 for i in range(8))


def test_bin_observations_reject_a_window_outside_the_frame_or_a_partial_loop():
    cube = _random_cube(7, 3)
    with pytest.raises(ValueError):
        bin_observations(cube, 0, 15, 8, 3)
    with pytest.raises(ValueError):
        bin_observations(cube, 0, 0, 4, 5)  # 36 chirps is not a whole number of 5-TX loops


def _board_bin_stats(lib, cube, frame, local_bin, n_tx, window):
    """``l3_verticalResidual``'s IQ16 path on the board's memory layout:
    [loop][tx][rx][bin][Im, Re] int16, stats through the compiled C."""
    chirps, n_rx, bins = cube.shape[1], cube.shape[2], cube.shape[3]
    loops = chirps // n_tx
    words = np.zeros((loops, n_tx, n_rx, bins, 2), dtype=np.int16)
    data = cube[frame].reshape(loops, n_tx, n_rx, bins)
    words[..., 0] = data.imag
    words[..., 1] = data.real
    flat = words.reshape(-1)
    stride = n_tx * n_rx * bins * 2
    bin_stats = fw.Iq16BinStats()
    lib.l3_iq16_bin_stats_init(ctypes.byref(bin_stats), loops)
    for tx in vertical_tx_indices(n_tx):
        for rx in range(n_rx):
            offset = ((tx * n_rx + rx) * bins + local_bin) * 2
            pointer = ctypes.cast(
                flat.ctypes.data + offset * flat.itemsize, ctypes.POINTER(ctypes.c_int16)
            )
            stats = fw.Iq16ChannelStats()
            assert (
                lib.l3_iq16_channel_stats_windowed(
                    pointer, loops, stride, window, ctypes.byref(stats)
                )
                == 0
            )
            lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(stats))
    values = [ctypes.c_float() for _ in range(5)]
    lib.l3_iq16_bin_stats_finish(ctypes.byref(bin_stats), *map(ctypes.byref, values), None)
    return tuple(v.value for v in values)


def test_the_hann_table_matches_the_board_layout_through_the_c_statistics(lib):
    """The replay's window equals the board's: the neighbours the C reads at
    +/- 2 words are the adjacent range bins, and the edge bins stay plain."""
    cube = _random_cube(11, 3)
    table = bin_observation_table(cube, 1, 0, 20, 3, window="hann")
    for local_bin in range(20):
        edge = local_bin in (0, 19)
        window = fw.RANGE_WINDOW_NONE if edge else fw.RANGE_WINDOW_HANN
        expected = _board_bin_stats(lib, cube, 1, local_bin, 3, window)
        got = tuple(
            float(table[name][local_bin]) for name in ("energy", "peak", "loop0", "r1Re", "r1Im")
        )
        assert got == pytest.approx(expected, rel=1e-5, abs=1e-2), local_bin


def test_the_hann_window_changes_interior_bins_and_honours_the_frames_valid_bins():
    cube = _random_cube(12, 3)
    plain = bin_observation_table(cube, 0, 0, 20, 3)
    hann = bin_observation_table(cube, 0, 0, 20, 3, window="hann")
    assert hann["energy"][0] == plain["energy"][0] and hann["energy"][19] == plain["energy"][19]
    assert all(hann["energy"][k] != plain["energy"][k] for k in range(1, 19))
    # A frame holding 12 valid bins: bin 11 is its edge, bin 10 is interior.
    short = bin_observation_table(cube, 0, 0, 20, 3, window="hann", valid_bins=12)
    assert short["energy"][11] == plain["energy"][11]
    assert short["energy"][10] == hann["energy"][10]
    with pytest.raises(ValueError, match="window"):
        bin_observation_table(cube, 0, 0, 20, 3, window="kaiser")


def test_frame_window_and_timestamps_prefer_the_per_frame_metadata():
    fixed = {"n_frames": 3, "n_samples": 128, "range_bin_start": 20, "frame_period_us": 3000}
    assert frame_window(fixed, 2) == (20, 128)
    assert frame_timestamps_us(fixed) == (0, 3000, 6000)
    timed = {
        **fixed,
        "range_bin_starts": (20, 20, 47),
        "range_bin_counts": (53, 53, 40),
        "frame_time_offsets_us": (0, 3000, 15000),
    }
    assert frame_window(timed, 2) == (47, 40)
    assert frame_timestamps_us(timed) == (0, 3000, 15000)
    # A pre-version-3 header has no period: the host's 12 ms fallback applies.
    assert frame_timestamps_us({**fixed, "frame_period_us": 0}) == (0, 12000, 24000)


# The club-only synth has no ball: its club keeps going through the tee, so
# these club-track tests keep scoring it after the gate fires (post_impact
# off) as the first replay did. With post_impact on, post frames go to the
# ball tracker instead, which the whole-shot tests below cover.
CLUB_ONLY = ReplayConfig(tee_bin=TEE_BIN, post_impact=False)


def test_the_synthetic_swing_replays_as_one_continuous_approach_track(lib, swing):
    """The acceptance criterion: an approach trajectory without reacquisition."""
    result = replay_dump(swing, CLUB_ONLY, lib=lib)

    assert result.fired_frame is not None
    assert result.acquisitions == 1
    assert len(result.points) >= 7
    assert result.longest_run == len(result.points), "no frame of the approach was missed"
    assert result.approach_fraction == 1.0, "every step closed on the tee"
    assert result.speed_mps == pytest.approx(CLUB_SPEED_MS, abs=0.5)
    assert result.fit_residual_bins < 0.3
    # Points are 4 ms apart at the club's radial rate, in global bins.
    step = CLUB_SPEED_MS * FRAME_PERIOD_S / BIN_M
    bins = [point.range_bin for point in result.points]
    assert all(b - a == pytest.approx(step, abs=0.5) for a, b in zip(bins, bins[1:]))
    assert bins[-1] > TEE_BIN > bins[0]
    assert all(
        point.range_m == pytest.approx(point.range_bin * BIN_M, rel=1e-4) for point in result.points
    )


def test_the_trigger_fires_inside_the_impact_gate_and_frames_carry_the_detector_state(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    fired = result.frames[result.fired_frame]

    assert fired.fired and fired.trig_state == "fired"
    assert all(f.trig_state == "tracking" for f in result.frames[1 : result.fired_frame])
    assert fired.track_bin is not None and TEE_BIN - fired.track_bin <= 3
    assert fired.first_bin == TEE_BIN - 12 and fired.count == 16
    assert len(fired.targets) == 1 and fired.targets[0].confidence > 0.5
    assert result.trig_counters["fired"] == 1
    assert result.frames[0].track_why == "acquired"
    assert result.frames[result.fired_frame].track_why == "associated"


def test_stop_at_fire_ends_the_replay_where_the_board_stops_scoring(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, stop_at_fire=True), lib=lib)
    assert len(result.frames) == result.fired_frame + 1
    assert result.track_counters["dropped"] == 0


def test_a_destination_outside_the_window_scores_nothing(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=29, dest_bin=200), lib=lib)
    assert all(f.count == 0 for f in result.frames)
    assert result.points == [] and result.fired_frame is None
    assert result.acquisitions == 0 and result.longest_run == 0


def test_energy_statistic_and_an_explicit_loop_period_are_honoured(lib, swing):
    result = replay_dump(
        swing, ReplayConfig(tee_bin=TEE_BIN, stat="energy", loop_period_s=100e-6), lib=lib
    )
    assert result.trig.cfg.stat == fw.STAT_ENERGY
    assert result.trig.loopPeriodS == pytest.approx(100e-6)
    assert result.track.cfg.velocitySpanMps == pytest.approx(2 * fw.OBS_WAVELENGTH_M / 4e-4)
    assert result.acquisitions == 1


def test_replay_rejects_raw_adc_dumps_bad_stats_and_bad_trigger_configs(lib, swing):
    cube = np.zeros((2, 36, 4, 64), dtype=complex)
    raw_adc = pack_dump(cube, n_tx=3, version=3, sample_fmt=SAMPLE_INT16_IQ)
    with pytest.raises(ValueError, match="range-FFT snapshot"):
        replay_dump(raw_adc, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    with pytest.raises(ValueError, match="stat"):
        replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, stat="loop0"), lib=lib)
    with pytest.raises(ValueError, match="rejects"):
        replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, snr=0.5), lib=lib)
    with pytest.raises(ValueError, match="range_window"):
        replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, range_window="kaiser"), lib=lib)


def test_the_synthetic_swing_still_fires_through_the_hann_window(lib, swing):
    """The window must not cost the trigger a clean single-target swing. Scored
    as the board scores (stop at the fire): after it, the wider main lobe of a
    club that has left the watch region still lifts the region's last bin."""
    board = replace(CLUB_ONLY, stop_at_fire=True)
    plain = replay_dump(swing, board, lib=lib)
    hann = replay_dump(swing, replace(board, range_window="hann"), lib=lib)
    assert hann.fired_frame == plain.fired_frame
    assert hann.acquisitions == 1
    assert hann.speed_mps == pytest.approx(CLUB_SPEED_MS, abs=0.5)
    assert [p.range_bin for p in hann.points] == pytest.approx(
        [p.range_bin for p in plain.points], abs=0.1
    )


def test_report_leads_with_the_continuity_numbers(lib, swing):
    result = replay_dump(swing, CLUB_ONLY, lib=lib)
    report = format_report(result, name="swing", points=True)
    head = report.splitlines()[0]
    assert head.startswith("swing: fired frame")
    assert "acquisitions 1" in head and "longest run 8" in head and "approach 100%" in head
    assert "clubtrack active=" in report and "trig state=fired" in report
    assert report.count("\n  p frame=") == len(result.points)
    assert f"dist={TEE_BIN - result.points[0].range_bin:.1f}" in report


def test_recording_configs_merge_the_manifest_default_and_per_file_entries(tmp_path):
    for name in ("b.l3dump", "a.l3dump"):
        (tmp_path / name).write_bytes(b"")
    (tmp_path / "manifest.json").write_text(
        json.dumps({"default": {"tee_bin": 34}, "b.l3dump": {"dest_bin": 46, "notes": "ball"}})
    )
    configs = recording_configs(tmp_path)
    assert [path.name for path, _ in configs] == ["a.l3dump", "b.l3dump"]
    assert configs[0][1] == ReplayConfig(tee_bin=34)
    assert configs[1][1] == ReplayConfig(tee_bin=34, dest_bin=46)
    assert recording_configs(tmp_path, default_tee_bin=40)[0][1].tee_bin == 40


def test_recording_configs_refuse_to_guess_a_tee_bin(tmp_path):
    (tmp_path / "a.l3dump").write_bytes(b"")
    with pytest.raises(ValueError, match="--tee-bin or --tee-range-m"):
        recording_configs(tmp_path)
    assert recording_configs(tmp_path, default_tee_bin=34)[0][1].tee_bin == 34
    assert recording_configs(tmp_path / "missing") == []


def _script():
    spec = importlib.util.spec_from_file_location("replay_iwr_track", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_script_replays_a_file_by_tee_range_and_a_directory_by_manifest(
    lib, swing, tmp_path, capsys
):
    (tmp_path / "one.l3dump").write_bytes(swing)
    (tmp_path / "two.l3dump").write_bytes(synth_club_dump(4.0, tee_range_m=TEE_RANGE_M))
    (tmp_path / "manifest.json").write_text(json.dumps({"default": {"tee_bin": TEE_BIN}}))
    script = _script()

    assert script.main([str(tmp_path / "one.l3dump"), "--tee-range-m", str(TEE_RANGE_M)]) == 0
    single = capsys.readouterr().out
    assert single.startswith("one.l3dump: fired frame") and "  p frame=" not in single

    assert script.main([str(tmp_path), "--points", "--stop-at-fire"]) == 0
    both = capsys.readouterr().out
    assert "one.l3dump: fired frame" in both and "two.l3dump: fired frame" in both
    assert "  p frame=0 " in both
    assert both.rstrip().splitlines()[-1].startswith("two.l3dump")
    assert "coasted 0" in both, "--stop-at-fire reached the replay config"


def test_script_needs_a_tee_for_a_bare_file_and_reports_an_empty_directory(tmp_path, capsys):
    script = _script()
    with pytest.raises(SystemExit):
        script.main([str(tmp_path / "x.l3dump")])
    (tmp_path / "no_tee.l3dump").write_bytes(b"")
    with pytest.raises(SystemExit, match="--tee-bin or --tee-range-m"):
        script.main([str(tmp_path)])
    (tmp_path / "no_tee.l3dump").unlink()
    assert script.main([str(tmp_path)]) == 1
    assert "no .l3dump" in capsys.readouterr().err


def test_synth_dump_is_a_range_snapshot_the_replay_can_read(swing):
    meta, _ = parse_dump(swing)
    assert meta["sample_fmt"] == SAMPLE_RANGE_FFT_IQ16 and meta["n_tx"] == 3


_RECORDINGS = recording_configs(RECORDINGS_DIR) if RECORDINGS_DIR.exists() else []
_EXPECTATIONS = recording_expectations(RECORDINGS_DIR) if RECORDINGS_DIR.exists() else {}


@pytest.mark.parametrize("path,config", _RECORDINGS, ids=[p.name for p, _ in _RECORDINGS])
def test_recorded_swings_meet_their_manifest_expectations(lib, path, config):
    """The regression corpus: each recording's manifest entry states the ranges
    its replay must land in (see tests/radar/recordings/README.md). A file
    without an entry gets the weak, capture-independent check instead."""
    result = replay_dump(path.read_bytes(), config, lib=lib)
    report = format_report(result, name=path.name)
    expectation = _EXPECTATIONS.get(path.name)
    if expectation is not None:
        assert expectation.check(result) == [], report
    else:
        assert result.points, report
        assert result.acquisitions <= 2, report
        assert result.longest_run >= 3, report
    assert math.isfinite(result.speed_mps)


# --- angles, delivery and geometric impact -------------------------------------


def test_channel_snapshot_matches_a_loop_by_loop_port_of_the_firmware():
    rng = np.random.default_rng(11)
    shape = (1, 36, 4, 8)
    cube = rng.integers(-2000, 2000, shape) + 1j * rng.integers(-2000, 2000, shape)
    phase = 0.7
    snap = channel_snapshot(
        cube, 0, 5, 3, lag1_phase_rad=phase, radial_velocity_mps=20.0, chirp_period_s=45e-6
    )
    assert (snap.ntx, snap.nrx) == (3, 4)
    for tx in range(3):
        for rx in range(4):
            samples = [cube[0, loop * 3 + tx, rx, 5] for loop in range(12)]
            mean = sum(samples) / 12
            expected = sum(
                (s - mean) * np.exp(-1j * phase * loop) for loop, s in enumerate(samples)
            )
            got = snap.channel[tx * 4 + rx]
            assert complex(got.re, got.im) == pytest.approx(expected, rel=1e-5, abs=1e-2)
    assert snap.lag1PhaseRad == pytest.approx(phase)
    assert snap.radialVelocityMps == 20.0 and snap.chirpPeriodS == pytest.approx(45e-6)


@pytest.mark.parametrize("path_deg", [0.0, 6.0, -4.0])
def test_replay_reads_club_path_and_speed_from_the_three_tx_synth(lib, path_deg):
    """The synthetic club carries the azimuth on TX1 and the TDM phases, so the
    replay must give back the path it was built with, a level attack and the
    club speed from the 3D fit, not just the radial projection."""
    result = replay_dump(
        synth_club_dump(path_deg, tee_range_m=TEE_RANGE_M),
        ReplayConfig(tee_bin=TEE_BIN, post_impact=False),
        lib=lib,
    )
    assert result.delivery is not None
    assert result.delivery.path_deg == pytest.approx(path_deg, abs=0.5)
    assert result.delivery.attack_deg == pytest.approx(0.0, abs=0.3)
    assert result.delivery.speed_mps == pytest.approx(CLUB_SPEED_MS, abs=0.5)
    assert result.delivery.points == len(result.points) - 1, "the first point is range-only"
    assert result.delivery.confidence > 0.6
    angled = [f for f in result.frames if f.angle is not None]
    assert len(angled) == len(result.points) - 1
    assert all(f.angle.elevation_deg == pytest.approx(0.0, abs=0.3) for f in angled)
    assert all(f.angle.azimuth_coherence > 0.9 for f in angled)


def test_the_first_point_of_a_track_carries_no_angles(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    first = result.frames[result.points[0].frame]
    assert first.track_why == "acquired" and first.angle is None
    second = result.frames[result.points[1].frame]
    assert second.angle is not None and second.angle.azimuth_deg is not None


def test_geometric_impact_fires_at_the_tee_with_a_sub_frame_time(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.geometric_frame is not None
    assert result.impact_timestamp_us is not None
    # The club crosses the tee bin between frames 5 and 6 (20 and 24 ms).
    assert 20_000 < result.impact_timestamp_us < 24_000
    assert result.frames[result.geometric_frame].impact_why == "fired"
    assert any(f.impact_why == "pending" for f in result.frames[: result.geometric_frame])
    assert "impact fired=1 why=fired" in result.impact_status
    assert "geometric impact frame" in format_report(result)


def test_impact_armed_ends_a_stop_at_fire_replay_on_the_geometric_fire(lib, swing):
    gated = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, stop_at_fire=True), lib=lib)
    armed = replay_dump(
        swing, ReplayConfig(tee_bin=TEE_BIN, stop_at_fire=True, impact_armed=True), lib=lib
    )
    assert len(armed.frames) == armed.geometric_frame + 1
    assert len(armed.frames) <= len(gated.frames)


def test_replay_calibration_attitude_rotates_the_delivery_into_the_golf_frame(lib):
    """A radar yawed 3 degrees right reads a straight swing as 3 degrees left
    unless the calibration says so; with it the path comes back straight."""
    raw = synth_club_dump(0.0, tee_range_m=TEE_RANGE_M)
    unaware = replay_dump(raw, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    aware = replay_dump(raw, ReplayConfig(tee_bin=TEE_BIN, yaw_deg=3.0), lib=lib)
    assert unaware.delivery.path_deg == pytest.approx(0.0, abs=0.5)
    assert aware.delivery.path_deg == pytest.approx(3.0, abs=0.5)


def test_report_points_carry_angles_and_the_manifest_takes_calibration_keys(lib, swing, tmp_path):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    report = format_report(result, name="swing", points=True)
    assert " az=+0.0 el=+0.0" in report or " az=-0.0 el=+0.0" in report
    (tmp_path / "a.l3dump").write_bytes(b"")
    (tmp_path / "manifest.json").write_text(
        json.dumps({"default": {"tee_bin": 34, "pitch_deg": 10.4, "range_bias_m": 0.066}})
    )
    config = recording_configs(tmp_path)[0][1]
    assert config.pitch_deg == 10.4 and config.range_bias_m == 0.066


# --- the whole shot: ball tracker, launch and the shot machine ------------------


@pytest.fixture(scope="module")
def whole_shot() -> bytes:
    return synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )


def test_post_impact_frames_go_to_the_ball_tracker_and_the_launch_is_recovered(lib, whole_shot):
    """The acceptance for items 11-15: from the frames after the trigger the
    replay finds the departing ball and reads its speed, HLA and VLA back."""
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.fired_frame is not None
    assert result.launch is not None
    # The synth's one-spike-per-loop ball hops bins within a burst, which
    # quantises the sub-bin centroid; a few percent of speed bias is that.
    assert result.launch.speed_mps == pytest.approx(60.0, abs=3.0)
    assert result.launch.hla_deg == pytest.approx(2.0, abs=0.7)
    assert result.launch.vla_deg == pytest.approx(12.0, abs=0.7)
    assert result.launch.points >= 5 and result.launch.confidence > 0.5
    assert len(result.ball_points) >= 6
    bins = [p.range_bin for p in result.ball_points]
    assert all(b > a for a, b in zip(bins, bins[1:])), "the ball only ever departs"
    assert "balltrack armed=1 confirmed=1" in result.ball_status


def test_joint_search_arms_from_the_club_tracks_own_last_point(lib, whole_shot):
    """Regression: arming once read delivery.rangeBin, a field l3_delivery_t
    does not have, raising AttributeError as soon as a real (non-empty) club
    track reached impact with joint_search enabled. The seed must come from
    the club track's own last point and its range-rate fitted speed."""
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, joint_search=True), lib=lib)
    assert result.fired_frame is not None
    assert len(result.joint_ball_points) > 0
    assert len(result.joint_club_points) > 0


def test_the_shot_machine_walks_the_whole_sequence_on_the_replay(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    states = [f.shot_state for f in result.frames]
    assert states[0] == "ready"
    assert "club_track" in states and "impact" in states and "ball_track" in states
    assert states[-1] == "result"
    order = [states.index(s) for s in ("ready", "club_track", "impact", "ball_track", "result")]
    assert order == sorted(order)
    assert result.frames[result.fired_frame].shot_state == "impact"
    assert "shot state=result" in result.shot_status and "source=gate" in result.shot_status
    verdicts = [f.ball_why for f in result.frames if f.shot_state in ("ball_track", "result")]
    # The first post frame sees the ball still at the origin (excluded by the
    # departure band); the flight is then acquired, confirmed and tracked.
    assert "acquired" in verdicts and "confirmed" in verdicts and "tracked" in verdicts
    first = verdicts.index("acquired")
    assert verdicts[first : first + 3] == ["acquired", "confirmed", "tracked"]
    assert all(v == "nocandidate" for v in verdicts[:first])


def test_the_club_delivery_is_read_from_the_pre_impact_frames_alone(lib, whole_shot):
    """The delivery is read at impact and rests on the five angled approach
    points; the club track carries on past impact beside the ball's (it only
    follows then), but those points never reach the delivery. The synth's
    one-spike-per-loop club quantises the sub-bin centroid, which with fewer
    points leaves about a metre per second of bias."""
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.delivery is not None
    assert result.delivery.points == 5
    approach = [p for p in result.points if p.frame <= result.fired_frame]
    assert len(approach) == result.fired_frame + 1
    assert result.delivery.path_deg == pytest.approx(3.0, abs=1.0)
    assert result.delivery.speed_mps == pytest.approx(CLUB_SPEED_MS, abs=1.5)
    assert result.delivery.attack_deg == pytest.approx(0.0, abs=0.3)


def test_points_carry_their_golf_frame_position(lib, whole_shot):
    """PointSummary.position is l3_track_point_t.position: with an identity
    calibration its length is the point's range."""
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    points = result.points + result.ball_points
    assert points and any(point.angles_valid for point in points)
    for point in points:
        assert point.position is not None
        assert math.dist((0.0, 0.0, 0.0), point.position) == pytest.approx(point.range_m, rel=1e-3)
        assert point.position[0] > 0  # downrange of the radar


def test_post_impact_can_be_disabled_to_keep_scoring_the_trigger(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, post_impact=False), lib=lib)
    assert result.launch is None and result.ball_points == []
    assert all(f.ball_why == "none" for f in result.frames)


def test_report_carries_launch_shot_and_ball_lines(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    report = format_report(result, name="shot", points=True)
    assert "launch: " in report and "ball speed 6" in report
    assert "shot state=result" in report and "balltrack armed=1" in report
    assert report.count("\n  b frame=") == len(result.ball_points)
    without = replay_dump(
        synth_club_dump(0.0, tee_range_m=TEE_RANGE_M),
        ReplayConfig(tee_bin=TEE_BIN, post_impact=False),
        lib=lib,
    )
    assert "launch: none" in format_report(without)


# --- the locked ball's own direction --------------------------------------------


def test_static_channel_snapshot_sums_the_raw_loops_without_mean_removal():
    rng = np.random.default_rng(5)
    shape = (1, 36, 4, 8)
    cube = rng.integers(-500, 500, shape) + 1j * rng.integers(-500, 500, shape)
    snap = static_channel_snapshot(cube, 0, 3, 3, chirp_period_s=45e-6)
    for tx in range(3):
        for rx in range(4):
            expected = sum(cube[0, loop * 3 + tx, rx, 3] for loop in range(12))
            got = snap.channel[tx * 4 + rx]
            assert complex(got.re, got.im) == pytest.approx(expected, abs=1e-2)
    assert snap.lag1PhaseRad == 0.0 and snap.radialVelocityMps == 0.0


def test_a_static_ball_on_the_tee_gives_the_destination_a_direction(lib):
    """A stationary reflector at the destination bin with a known direction:
    the replay reads it back and aims the impact test at it."""
    n_tx, n_rx, bins, loops, frames = 3, 4, 128, 12, 12
    cube = np.zeros((frames, loops * n_tx, n_rx, bins), dtype=complex)
    el, az = math.radians(-6.0), math.radians(4.0)
    physical = 3000.0 * np.exp(1j * math.pi * math.sin(el) * np.arange(2 * n_rx))
    logical = physical[::-1]
    az_phase = -math.pi * math.sin(az)
    for frame in range(frames):
        for loop in range(loops):
            cube[frame, loop * n_tx + 0, :, 40] = logical[:n_rx]
            cube[frame, loop * n_tx + 2, :, 40] = logical[n_rx:]
            cube[frame, loop * n_tx + 1, :, 40] = (
                0.5 * (logical[:n_rx] + logical[n_rx:]) * np.exp(1j * az_phase)
            )
    raw = pack_dump(
        cube, n_tx=n_tx, version=3, frame_period_us=3000, sample_fmt=SAMPLE_RANGE_FFT_IQ16
    )
    result = replay_dump(raw, ReplayConfig(tee_bin=34, dest_bin=40), lib=lib)
    assert result.ball_angle is not None
    assert result.ball_angle.elevation_deg == pytest.approx(-6.0, abs=0.3)
    assert result.ball_angle.azimuth_deg == pytest.approx(4.0, abs=0.3)
    assert "ball direction: az +4.0 deg, el -6.0 deg" in format_report(result)
    # Without a locked ball, or with the direction switched off, boresight.
    assert replay_dump(raw, ReplayConfig(tee_bin=40), lib=lib).ball_angle is None
    assert (
        replay_dump(
            raw, ReplayConfig(tee_bin=34, dest_bin=40, ball_angles=False), lib=lib
        ).ball_angle
        is None
    )
    assert "boresight" in format_report(replay_dump(raw, ReplayConfig(tee_bin=40), lib=lib))


def test_an_empty_destination_bin_keeps_boresight(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN + 20), lib=lib)
    assert result.ball_angle is None


def test_adaptive_retention_keeps_every_club_and_ball_point_of_the_synthetic_shot(lib):
    """The mirror's windows must hold the points the trackers appended, while
    storing far fewer bins than the processing window."""
    raw = synth_shot_dump(path_deg=0.0, hla_deg=0.0, vla_deg=12.0)
    config = ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN, retain=RetainReplay())
    result = replay_dump(raw, config, lib=lib)

    covered, judged = result.retain_coverage
    assert judged >= len(result.points) + len(result.ball_points) - 2
    missed = [w for w in result.retain_windows if w.covered is False]
    # The club is acquired from the wide processing window; its first point can
    # land outside the ball-centred slot of a frame that had no club to follow
    # yet. Every point after acquisition, and every ball point, is kept.
    assert len(missed) <= 1 and all(w.why in ("ball", "tee") for w in missed), missed
    assert covered >= judged - 1
    kept, processed = result.retain_bins_saved
    assert 0 < kept < 0.6 * processed
    reasons = [w.why for w in result.retain_windows]
    assert reasons[0] in ("ball", "tee"), "before the club shows, the window sits on the ball"
    assert "club" in reasons or "approach" in reasons
    assert any(r in ("impact", "ballsearch") for r in reasons)
    assert "ballfollow" in reasons, "a confirmed flight is followed"
    assert all(w.bins in (16, 24) for w in result.retain_windows)
    report = format_report(result)
    assert "retention:" in report and f"{covered}/{judged} points inside" in report


def test_centred_retention_is_the_compact16_baseline_and_can_miss_the_club(lib):
    raw = synth_shot_dump(path_deg=0.0)
    adaptive = replay_dump(
        raw,
        ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN, retain=RetainReplay(pre_bins=8)),
        lib=lib,
    )
    centred = replay_dump(
        raw,
        ReplayConfig(
            tee_bin=TEE_BIN, dest_bin=TEE_BIN, retain=RetainReplay(pre_bins=8, enabled=False)
        ),
        lib=lib,
    )
    assert all(w.why == "centred" for w in centred.retain_windows)
    assert centred.retain_coverage[0] < adaptive.retain_coverage[0]
    assert centred.retain_coverage[1] == adaptive.retain_coverage[1], (
        "same points appended: the mirror never alters the replay"
    )
    assert centred.points == adaptive.points


def test_retention_config_is_checked_by_the_firmware(lib, swing):
    with pytest.raises(ValueError, match="retention configuration"):
        replay_dump(
            swing, ReplayConfig(tee_bin=TEE_BIN, retain=RetainReplay(approach_bins=0)), lib=lib
        )
    plain = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert plain.retain_windows == [] and plain.retain_coverage == (0, 0)
    assert "retention:" not in format_report(plain)


def test_the_hypothesis_search_recovers_the_synthetic_launch(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, ball_hypotheses=True), lib=lib)
    assert result.launch is not None
    assert result.launch.speed_mps == pytest.approx(60.0, rel=0.1)
    seen = [frame.ball_hypotheses for frame in result.frames if frame.ball_hypotheses]
    assert seen and all(len(snapshot) <= fw.BALL_HYP_MAX for snapshot in seen)


def test_the_search_switch_leaves_the_default_replay_alone(lib, whole_shot):
    default = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    off = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, ball_hypotheses=False), lib=lib)
    assert [p.range_bin for p in default.ball_points] == [p.range_bin for p in off.ball_points]
    assert all(frame.ball_hypotheses == () for frame in off.frames)


def test_ball_tuning_writes_only_what_it_sets(lib):
    default = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(default))
    untouched = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(untouched))
    BallTuning().apply(untouched)
    assert bytes(untouched) == bytes(default)

    tuned = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(tuned))
    BallTuning(
        fast_ball_mps=34.0,
        fast_support_fraction=0.5,
        min_departure_mps=18.0,
        far_window_bins=3.0,
    ).apply(tuned)
    assert (tuned.hyps.fastBallMps, tuned.hyps.fastSupportFraction) == (34.0, 0.5)
    # One floor, both searches: the legacy acquisition and the hypotheses.
    assert (tuned.minDepartureMps, tuned.hyps.minDepartureMps) == (18.0, 18.0)
    assert tuned.hyps.farWindowBins == 3.0
    assert tuned.useHypotheses == default.useHypotheses


def test_empty_ball_tuning_leaves_the_replay_alone(lib, whole_shot):
    plain = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    tuned = replay_dump(
        whole_shot, ReplayConfig(tee_bin=TEE_BIN, ball_tuning=BallTuning()), lib=lib
    )
    assert [p.range_bin for p in plain.ball_points] == [p.range_bin for p in tuned.ball_points]
    assert plain.launch == tuned.launch


def test_ball_tuning_reaches_the_replayed_launch(lib, whole_shot):
    """The synthetic ball flies clean: every switch on still finds it."""
    tuning = BallTuning(fast_ball_mps=30.0, far_window_bins=2.0)
    result = replay_dump(
        whole_shot,
        ReplayConfig(tee_bin=TEE_BIN, ball_hypotheses=True, ball_tuning=tuning),
        lib=lib,
    )
    assert result.launch is not None
    assert result.launch.speed_mps == pytest.approx(60.0, rel=0.1)
    too_fast = replay_dump(
        whole_shot,
        ReplayConfig(tee_bin=TEE_BIN, ball_tuning=BallTuning(min_departure_mps=80.0)),
        lib=lib,
    )
    assert too_fast.launch is None, "the hard floor reaches the legacy acquisition"
