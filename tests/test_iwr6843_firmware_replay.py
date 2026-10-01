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

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr
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
from openflight.iwr6843.self_trigger import FIRMWARE_TRIGGER_DEFAULT_SNR, TEE_BAND_DEFAULT_BINS

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
# these club-track tests keep scoring it after the trigger fires (post_impact
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


def test_the_trigger_fires_on_the_club_track_range_impact_and_frames_say_when(lib, swing):
    """The range gate is gone (2026-09-30): the club track's range-only impact
    fires the self-trigger, as the club's line crosses the tee's range."""
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    fired = result.frames[result.fired_frame]

    assert result.fired_frame == result.range_frame
    assert fired.fired
    assert not any(f.fired for f in result.frames[: result.fired_frame])
    assert fired.track_bin is not None and TEE_BIN - fired.track_bin <= 3
    assert fired.first_bin == TEE_BIN - 12 and fired.count == 16
    assert len(fired.targets) == 1 and fired.targets[0].confidence > 0.5
    assert result.shot.impactSource == fw.SHOT_IMPACT_RANGE
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
    assert result.track.cfg.velocitySpanMps == pytest.approx(
        2.0 * fw.OBS_WAVELENGTH_M / (4.0 * 100e-6)
    )
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


def test_report_leads_with_the_continuity_numbers(lib, swing):
    result = replay_dump(swing, CLUB_ONLY, lib=lib)
    report = format_report(result, name="swing", points=True)
    head = report.splitlines()[0]
    assert head.startswith("swing: fired frame")
    assert "acquisitions 1" in head and "longest run 8" in head and "approach 100%" in head
    assert "clubtrack active=" in report and "trig frames=" in report
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


# --- angles, delivery and the range impact ------------------------------------


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


def test_the_range_impact_fires_at_the_tee_with_a_sub_frame_time(lib, swing):
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.fired_frame is not None
    assert result.impact_timestamp_us is not None
    # The club crosses the tee bin between frames 5 and 6 (20 and 24 ms).
    assert 20_000 < result.impact_timestamp_us < 24_000
    assert result.frames[result.fired_frame].impact_why == "fired"
    assert any(f.impact_why == "pending" for f in result.frames[: result.fired_frame])
    assert "impact fired=1 why=fired" in result.impact_status


def test_the_geometric_detector_is_gone_from_the_replay():
    """Removed with the range gate (2026-09-30): nothing armed it on the kiosk
    and it never fired on the recorded swings."""
    fields = set(ReplayConfig.__dataclass_fields__) | set(fr.ReplayResult.__dataclass_fields__)
    for gone in ("geometry_armed", "impact_armed", "geometric_frame"):
        assert gone not in fields


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
    # The synthetic scene has no floor and its tee is at antenna height, so the
    # direction fit's tee anchor is put there too.
    config = ReplayConfig(
        tee_bin=TEE_BIN,
        overrides={"ball.fit.teeBallHeightM": 0.152, "ball.fit.radarHeightM": 0.152},
    )
    result = replay_dump(whole_shot, config, lib=lib)
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


def test_the_shot_machine_walks_the_whole_sequence_on_the_replay(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    states = [f.shot_state for f in result.frames]
    assert states[0] == "ready"
    assert "club_track" in states and "impact" in states and "ball_track" in states
    assert states[-1] == "result"
    order = [states.index(s) for s in ("ready", "club_track", "impact", "ball_track", "result")]
    assert order == sorted(order)
    assert result.frames[result.fired_frame].shot_state == "impact"
    assert "shot state=result" in result.shot_status and "source=range" in result.shot_status
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


def test_with_the_band_on_no_pre_impact_club_point_is_in_or_beyond_it(lib):
    """Band on, the club track only sees targets short of the band before
    impact: none of its pre-impact points lies in the band or beyond it.

    Impact is each capture's recorded freeze (post_from_frame), so every
    recording is judged up to its own impact whether or not the replay's
    self-trigger fires on it: a band that thaws after a missed swing moves,
    and the last band placed says nothing about the frames before."""
    judged = 0
    for path, config in fr.recording_configs():
        freeze = config.post_from_frame or fr.freeze_frame(parse_dump(path.read_bytes())[0])
        result = fr.replay_file(
            path, replace(config, band_bins=6.0, post_from_frame=freeze), lib=lib
        )
        assert result.band is not None
        lo, _hi = result.band
        assert fw.SHOT_STATE_NAMES[result.shot.state] not in fr.PRE_IMPACT_SHOT_STATES
        not_short = [
            p for p in result.points if p.frame <= result.shot.impactFrame and p.range_bin >= lo
        ]
        assert not not_short, f"{path.name}: club points in or beyond the band {not_short}"
        judged += 1
    assert judged >= 40


def test_the_band_is_off_by_default_and_with_zero(lib):
    path, config = next(iter(fr.recording_configs()))
    assert config.band_bins is None
    assert fr.replay_file(path, config, lib=lib).band is None
    assert fr.replay_file(path, replace(config, band_bins=0.0), lib=lib).band is None
    assert fr.replay_file(path, replace(config, band_bins=6.0), lib=lib).band is not None


def test_impact_fit_is_reported_exactly_when_the_shot_reaches_result(lib):
    """The board fits at RESULT (l3_impactFitRun beside l3_result_build); a
    capture that declared impact but stopped short of RESULT has no fit and
    keeps its frozen impact time, as on the board."""
    reached_any = False
    for path, config in fr.recording_configs():
        result = fr.replay_file(path, config, lib=lib)
        state = fw.SHOT_STATE_NAMES[result.shot.state]
        reached = state == "result"
        reached_any |= reached
        assert (result.impact_fit is not None) == reached, path.name
        assert result.impact_fit_status.startswith("impactfit verdict=")
        assert "impactfit verdict=" in fr.format_report(result)
        declared = state not in fr.PRE_IMPACT_SHOT_STATES
        assert (result.frozen_impact_timestamp_us is not None) == declared, path.name
        if reached:
            assert result.impact_fit.verdict in fw.FIT_VERDICT_NAMES
            assert set(result.impact_fit.tracks) == set(fw.FIT_TRACK_NAMES)
        elif declared:
            assert result.shot.impactTimestampUs == result.frozen_impact_timestamp_us, path.name
            assert result.impact_fit_status.startswith("impactfit verdict=none"), path.name
    assert reached_any, "no recording reached RESULT: the test proves nothing"


def test_a_shot_that_stops_before_result_is_not_fitted(lib):
    """Post-impact frames cut short: IMPACT is declared, RESULT never comes."""
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    config = fr.ReplayConfig(tee_bin=29, dest_bin=29, post_impact=False)
    result = fr.replay_dump(raw, config, lib=lib)

    assert fw.SHOT_STATE_NAMES[result.shot.state] not in fr.PRE_IMPACT_SHOT_STATES
    assert fw.SHOT_STATE_NAMES[result.shot.state] != "result"
    assert result.impact_fit is None
    assert result.shot.impactTimestampUs == result.frozen_impact_timestamp_us


@pytest.mark.parametrize("band_bins", [None, 6.0])
def test_the_fit_uses_the_tracks_as_they_stood_when_result_was_reached(lib, band_bins):
    """Board parity: the fit runs on the first RESULT frame, so later ball and
    club points never reach it."""
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    config = fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=band_bins)
    result = fr.replay_dump(raw, config, lib=lib)

    assert result.impact_fit is not None
    result_frame = next(f.frame for f in result.frames if f.shot_state == "result")
    k = 4  # l3_impact_fit_cfg_defaults fitPoints
    ball_then = sum(1 for p in result.ball_points if p.frame <= result_frame)
    club_out_then = sum(
        1 for p in result.points if result.shot.impactFrame < p.frame <= result_frame
    )
    assert result.impact_fit.tracks["ball_out"].points == min(k, ball_then)
    assert result.impact_fit.tracks["club_out"].points == min(k, club_out_then)


@pytest.mark.parametrize("band_bins", [None, 6.0])
def test_synthetic_shot_impact_lands_on_the_synthesized_time(lib, band_bins):
    """tests/iwr6843_synth.py: club at 22 m/s to a known impact, then the ball
    leaving at 60 m/s. Point timestamps are frame starts, so allow half a
    4 ms frame of integration offset plus the 0.5 ms gate floor."""
    from iwr6843_synth import IMPACT_S  # pylint: disable=import-outside-toplevel

    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    config = fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=band_bins)
    result = fr.replay_dump(raw, config, lib=lib)

    assert (result.band is not None) == (band_bins is not None)
    assert result.impact_fit is not None
    assert result.impact_fit.verdict in ("consistent", "single_track")
    assert result.impact_fit.tracks["ball_out"].why == "ok"
    assert result.impact_fit.impact_us == pytest.approx(IMPACT_S * 1e6, abs=2_500)


@pytest.mark.parametrize("band_bins", [None, 6.0])
def test_a_fit_verdict_replaces_the_shot_impact_time_as_the_board_does(lib, band_bins):
    """l3_impactFitRun: a verdict other than none puts the refined time on the
    shot; the refinement is measured against the ORIGINAL frozen time."""
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    config = fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=band_bins)
    result = fr.replay_dump(raw, config, lib=lib)

    fit = result.impact_fit
    assert fit is not None and fit.verdict != "none"
    frozen = result.frozen_impact_timestamp_us
    assert frozen is not None and frozen > 0
    assert result.shot.impactTimestampUs == fw.round_us(fit.impact_us)
    assert result.shot.impactTimestampUs != frozen, "the synthetic refinement moves the time"
    assert fit.refined_minus_trigger_us == pytest.approx(fit.impact_us - frozen, abs=1.0)
    assert f"impact={result.shot.impactTimestampUs} " in result.shot_status


@pytest.mark.parametrize(
    ("verdict", "impact_us", "expected"),
    [
        ("none", 0.0, 21_000),
        ("none", 30_000.0, 21_000),
        ("single_track", 0.0, 21_000),
        ("single_track", 30_000.4, 30_000),
        ("consistent", 30_000.5, 30_001),
        ("inconsistent", 19_999.6, 20_000),
    ],
)
def test_the_fit_replaces_the_frozen_time_only_with_a_verdict(verdict, impact_us, expected):
    """The rule of l3_impactFitRun: verdict none (or no time) keeps the frozen
    time; any other verdict rounds the fit's time onto the shot."""
    shot = fw.Shot()
    shot.impactTimestampUs = 21_000
    fit = fw.ImpactFit()
    fit.verdict = fw.FIT_VERDICT_NAMES.index(verdict)
    fit.impactUs = impact_us

    fr.apply_impact_fit(shot, fit)

    assert shot.impactTimestampUs == expected


def test_an_uncertain_track_keeps_its_time_and_sigma_in_the_summary():
    """An estimate over the sigma cap is shown as uncertain with its numbers."""
    fit = fw.ImpactFit()
    ball = fw.FIT_TRACK_NAMES.index("ball_out")
    fit.track[ball].why = fw.FIT_WHY_NAMES.index("uncertain")
    fit.track[ball].points = 3
    fit.track[ball].timeUs = 5370.0
    fit.track[ball].sigmaUs = 9514.0
    fit.droppedTrack = len(fw.FIT_TRACK_NAMES)
    summary = fr._impact_fit_summary(fit)  # pylint: disable=protected-access
    track = summary.tracks["ball_out"]
    assert track.why == "uncertain"
    assert (track.time_us, track.sigma_us) == (5370.0, 9514.0)


@pytest.mark.parametrize("band_bins", [None, 6.0])
def test_a_return_standing_at_the_tee_does_not_hold_the_trigger(lib, band_bins):
    """20260927: a return standing at bin 44 once held the range gate's track,
    so the club was taken only at frame 11 and the fit came out inconsistent
    (band off) or single-track with a +-9.5 ms ball (band on). The club track
    that fires now follows the club in and fires from its own approach."""
    path, config = next(p for p in fr.recording_configs() if "20260927" in p[0].name)
    result = fr.replay_file(path, replace(config, band_bins=band_bins), lib=lib)
    approach = [p.range_bin for p in result.points if p.frame <= result.fired_frame]
    # Band off, the first point may be the return itself (43.1); from the
    # next on the track is the club closing on the tee, well short of 44.
    club = approach[1:]
    assert len(club) >= 6
    assert club == sorted(club), "the club, approaching, not the return at 44"
    assert max(club) < 41.0
    assert result.fired_frame == 12
    assert result.impact_fit.tracks["ball_out"].why == "ok"
    # Band on, the scan plan's 16 post bins (l3_scan.h) lose the club after
    # impact in this capture's clutter beyond the band (hand and body returns
    # at 45-47): club_out goes missing and the fit is inconsistent, where the
    # whole post window found it consistent. Re-baselined 2026-09-30.
    expected = "consistent" if band_bins is None else "inconsistent"
    assert result.impact_fit.verdict == expected
    assert result.launch.speed_mps < 60.0, "a two-angle ball track is radial only, never 339 m/s"


# --- where the ball tracker is armed ---------------------------------------------


# bandBins is a total width: 6 bins with no noise history (the synthetic club
# is in view from the first frame, so the band freezes centred) is 27..32 --
# lo = 29 - (6 - 1) // 2 -- and its far edge is 32 (was 29 + 6 = 35 as a half width).
@pytest.mark.parametrize("band_bins, arm_bin", [(None, 29.0), (6.0, 32.0)])
def test_impact_arms_the_ball_tracker_at_the_band_edge_or_the_ball(lib, band_bins, arm_bin):
    """The IMPACT arm: band on, at the band's far edge as placed; off, at the ball."""
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    config = ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN, band_bins=band_bins)
    result = replay_dump(raw, config, lib=lib)
    assert result.fired_frame is not None and result.ball_track.armed == 1
    if band_bins is not None:
        assert result.band[1] == arm_bin
    assert result.ball_track.originBin == arm_bin
    assert f"origin={arm_bin:.2f}" in result.ball_status


@pytest.mark.parametrize("band_bins, arm_bin", [(None, 29.0), (6.0, 32.0)])
def test_a_forced_post_frame_arms_the_ball_tracker_at_the_band_edge_or_the_ball(
    lib, band_bins, arm_bin
):
    """The post_from_frame arm, before the gate could fire: the same rule."""
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    config = ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN, band_bins=band_bins, post_from_frame=2)
    result = replay_dump(raw, config, lib=lib)
    assert result.fired_frame is None, "only the forced arm ran"
    assert result.ball_track.armed == 1
    assert result.ball_track.originBin == arm_bin


def test_post_frames_come_from_the_retention_report():
    meta = {
        "n_frames": 47,
        "retention": {"reason": "complete", "pre_frames": 24, "planned_frames": 47},
    }
    assert fr.post_frame_count(meta) == 23


def test_post_frames_come_from_the_first_window_change():
    meta = {"n_frames": 24, "range_bin_starts": (20,) * 9 + (32,) * 7 + (47,) * 8}
    assert fr.post_frame_count(meta) == 15


def test_post_frames_are_unknown_without_windows_or_a_report():
    assert fr.post_frame_count({"n_frames": 18}) is None
    assert fr.post_frame_count({"n_frames": 24, "range_bin_starts": (20,) * 24}) is None


def test_replay_uses_the_post_frame_count_and_says_when_it_could_not(lib):
    result = fr.replay_dump(
        synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372),
        fr.ReplayConfig(tee_bin=29, dest_bin=29),
        lib=lib,
    )
    assert result.ball_track_frames_known is False
    assert result.ball_track_frames == result.shot.cfg.ballTrackFrames == 18


def phased_dump(pre: int, post: int) -> bytes:
    """A timed variable-width IQ16 dump: `pre` frames at window 20, `post` at 32."""
    from openflight.iwr6843.dump import SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED, pack_dump as pack

    n_tx, loops, n_rx, width = 2, 4, 4, 16
    frames = pre + post
    cube = np.zeros((frames, loops * n_tx, n_rx, width), dtype=complex)
    cube[..., 9] += 3000.0
    return pack(
        cube,
        n_tx=n_tx,
        version=6,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=(20,) * pre + (32,) * post,
        range_bin_counts=(width,) * frames,
        frame_time_offsets_us=[3000 * f for f in range(frames)],
    )


def test_replay_of_a_phased_dump_counts_only_its_post_frames(lib):
    raw = phased_dump(pre=3, post=2)
    result = fr.replay_dump(raw, fr.ReplayConfig(tee_bin=29), lib=lib)
    assert result.ball_track_frames_known is True
    assert result.ball_track_frames == 2


def _club_after_impact(result):
    impact = result.shot.impactFrame
    return [p for p in result.points if p.frame > impact]


# bandBins is the band's total width, placed on the noisiest idle bins (Task 6).
@pytest.mark.parametrize("band_bins", [5.0, 10.0])
def test_with_the_band_on_every_recording_has_the_club_after_impact(lib, band_bins):
    """The club crosses the band coasting and is re-acquired beyond it: every
    recording that declared impact keeps club points after it, none of them
    the ball's."""
    for path, config in fr.recording_configs():
        result = fr.replay_file(path, replace(config, band_bins=band_bins), lib=lib)
        if fw.SHOT_STATE_NAMES[result.shot.state] in fr.PRE_IMPACT_SHOT_STATES:
            continue
        club = _club_after_impact(result)
        assert len(club) >= 3, f"{path.name}: {len(club)} club points after impact"
        ball_keys = {(p.frame, round(p.range_bin, 3)) for p in result.ball_points}
        assert not ball_keys & {(p.frame, round(p.range_bin, 3)) for p in club}, path.name


def test_synthetic_club_after_impact_is_slower_than_the_ball_and_gives_club_out(lib):
    raw = synth_shot_dump(ball_speed_ms=60.0, club_out_speed_ms=20.0, tee_range_m=1.372)
    result = fr.replay_dump(
        raw,
        fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=5.0),
        lib=lib,
    )
    club = _club_after_impact(result)
    assert len(club) >= 3
    assert result.impact_fit is not None
    assert result.impact_fit.tracks["club_out"].why == "ok"
    assert (
        result.impact_fit.tracks["club_out"].speed_mps
        < result.impact_fit.tracks["ball_out"].speed_mps
    )


@pytest.mark.parametrize("valid", [True, False])
def test_follow_ctx_approach_is_the_delivery_or_the_club_ceiling(lib, valid):
    """No valid delivery at impact: the approach is the fastest club, as the board."""
    bin_width_m = 0.046875
    shot = fw.Shot()
    shot.impactTimestampUs = 21_000
    shot.delivery.speedValid = 1 if valid else 0
    shot.delivery.radialSpeedMps = 30.0
    ball_track = fw.BallTrack()
    band = fw.Band(1, 34.0, 44.0)
    follow = fr._follow_ctx(  # pylint: disable=protected-access
        lib, shot, ball_track, band, 39, bin_width_m, 3000, fw.TRACK_NO_TARGET
    )
    expected = 30.0 if valid else fw.TRACK_FOLLOW_UNKNOWN_APPROACH_MPS
    assert follow.approachBinsPerS == pytest.approx(expected / bin_width_m)
    assert (follow.bandValid, follow.bandHiBin, follow.originBin) == (1, 44.0, 39.0)
    assert follow.impactTimestampUs == 21_000 and follow.frameUs == 3000
    assert follow.ballBinsPerS == 0.0 and follow.ballClaimIndex == fw.TRACK_NO_TARGET


def test_a_club_seen_only_after_impact_is_reacquired_beyond_the_band(lib):
    """The capture starts with the club already inside the band: no approach is
    measured, and the club leaving slower than the ball is still found."""
    raw = synth_shot_dump(
        ball_speed_ms=60.0, club_out_speed_ms=20.0, tee_range_m=1.372, t_impact_s=0.004
    )
    # Width 11 with no history is 24..34, the band this test was written
    # around (+-5 before bandBins became a total width). Known limitation, to
    # be checked on real full captures: with a narrow band (5) the ball
    # tracker, armed at the band's far edge, can take the slower club as the
    # ball, and the fast ball can already be beyond its 8-bin origin gate.
    # With no approach there is nothing for the club track to fire on (the
    # removed range gate fired at frame 2): impact is given, as a sound
    # trigger gives it, and the test is what follows it.
    result = fr.replay_dump(
        raw,
        fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=11.0, post_from_frame=2),
        lib=lib,
    )
    assert result.band == (24.0, 34.0)
    assert fw.SHOT_STATE_NAMES[result.shot.state] not in fr.PRE_IMPACT_SHOT_STATES
    assert result.shot.delivery.speedValid == 0
    club = _club_after_impact(result)
    assert len(club) >= 3
    assert all(p.range_bin > result.band[1] for p in club)
    ball_keys = {(p.frame, round(p.range_bin, 3)) for p in result.ball_points}
    assert not ball_keys & {(p.frame, round(p.range_bin, 3)) for p in club}


# --- the automatic band: placed on the noise, frozen on the swing ----------------


def test_the_band_lands_on_the_ridge_not_centred_on_the_tee(lib):
    from iwr6843_synth import synth_shot_dump  # pylint: disable=import-outside-toplevel

    # Impact at 150 ms: the club (22 m/s) is out of range before ~88 ms, so
    # the first ~29 frames are idle. The scan plan (l3_scan.h) refreshes the
    # band's interior 2 bins an idle frame, so every bin the band might cover
    # needs ~16 idle frames for its 8 updates (a whole-window map took 8).
    ridge = (33, 34, 35, 36, 37)  # beyond the tee at 29
    raw = synth_shot_dump(
        ball_speed_ms=60.0, tee_range_m=1.372, ridge_bins=ridge, n_frames=60, t_impact_s=0.15
    )
    result = fr.replay_dump(
        raw,
        fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=5.0),
        lib=lib,
    )
    # The band must hold the tee (29): the ridge 33..37 lies entirely beyond
    # it, so the band slides to the ridge-side edge (final-review ruling).
    assert result.band == (29.0, 33.0)


def test_band_freezes_on_the_frame_the_club_is_acquired(lib):
    """The first freeze is the acquisition frame. That the band then HOLDS is
    not observable in the replay: the noise map is only fed on idle frames
    and the destination is fixed, so re-placing while frozen would give the
    same band. On the board the destination follows the ball lock, and the
    hold is pinned by the wiring test (gBandFrozen guards l3_band_place)."""
    from iwr6843_synth import synth_shot_dump  # pylint: disable=import-outside-toplevel

    raw = synth_shot_dump(
        ball_speed_ms=60.0,
        tee_range_m=1.372,
        ridge_bins=(33, 34, 35, 36, 37),
        n_frames=36,
        t_impact_s=0.1,
    )
    result = fr.replay_dump(
        raw,
        fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=5.0),
        lib=lib,
    )
    acquired = next(f.frame for f in result.frames if f.track_why == "acquired")
    assert result.band_frozen_frame == acquired


def test_band_off_places_nothing_and_keeps_the_trigger_view(lib):
    path, config = next(iter(fr.recording_configs()))
    assert fr.replay_file(path, config, lib=lib).band is None


def test_band_thaws_when_the_club_drops_and_moves_to_a_ridge_learned_since(lib):
    """Club acquired at frame 10 over a quiet scene: the band freezes centred
    (27..31, no ridge in the map yet). A ridge appears at frame 13 while the
    radar loses the club for frames 13..17: the track drops, the band thaws,
    the idle frames feed the ridge to the map and the band moves toward it,
    where it freezes again when the club is re-acquired (holding the tee,
    29..33: the ridge lies entirely beyond it)."""
    from iwr6843_synth import synth_shot_dump  # pylint: disable=import-outside-toplevel

    ridge = (33, 34, 35, 36, 37)
    raw = synth_shot_dump(
        ball_speed_ms=60.0,
        tee_range_m=1.372,
        ridge_bins=ridge,
        ridge_start_frame=13,
        club_hidden_frames=range(13, 18),
        n_frames=36,
        t_impact_s=0.1,
    )
    result = fr.replay_dump(
        raw,
        fr.ReplayConfig(tee_bin=29, dest_bin=29, band_bins=5.0),
        lib=lib,
    )
    whys = [f.track_why for f in result.frames]
    acquired = [f.frame for f in result.frames if f.track_why == "acquired"]
    assert len(acquired) >= 2 and "dropped" in whys, whys
    assert result.band_frozen_frame == acquired[0], "the first freeze"
    # The band must hold the tee (29): the ridge 33..37 lies entirely beyond
    # it, so the band slides to the ridge-side edge (final-review ruling).
    assert result.band == (29.0, 33.0), "thawed, re-placed toward the ridge, frozen again"
    quiet = [result.band_noise[b] for b in (27, 28, 29, 30, 31)]
    loud = [result.band_noise[b] for b in ridge]
    assert min(loud) > max(quiet), "the map kept accumulating on the idle frames"


def test_replay_applies_element_calibration_through_the_firmware_setter(lib):
    phases = (0.28, 0.38, 0.43, 0.31, -0.43, -0.32, -0.24, -0.36)
    gains = (0.95, 0.86, 0.99, 1.13, 1.01, 0.92, 1.04, 1.09)
    config = ReplayConfig(tee_bin=TEE_BIN, elem_phase_rad=phases, elem_gain=gains)
    cal = fr._radar_cal(lib, config)  # pylint: disable=protected-access
    for i, (phase, gain) in enumerate(zip(phases, gains)):
        assert cal.correctionRe[i] == pytest.approx(math.cos(-phase) / gain, rel=1e-5)
        assert cal.correctionIm[i] == pytest.approx(math.sin(-phase) / gain, rel=1e-5)


def test_replay_without_element_calibration_is_identity(lib):
    cal = fr._radar_cal(lib, ReplayConfig(tee_bin=TEE_BIN))  # pylint: disable=protected-access
    assert [cal.correctionRe[i] for i in range(8)] == [1.0] * 8


@pytest.mark.parametrize("phases, gains", [((0.1,) * 7, (1.0,) * 8), ((0.1,) * 8, (1.0,) * 8 + (1.0,))])
def test_element_calibration_needs_eight_of_each(phases, gains):
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    with pytest.raises(ValueError, match="8 element"):
        replay_dump(raw, ReplayConfig(tee_bin=TEE_BIN, elem_phase_rad=phases, elem_gain=gains))


def test_replay_ball_angles_use_the_track_rate(lib, monkeypatch):
    seen = []
    original = fr._estimate_angles  # pylint: disable=protected-access

    def spy(*args, track_rate_mps=None, **kwargs):
        seen.append(track_rate_mps)
        return original(*args, track_rate_mps=track_rate_mps, **kwargs)

    monkeypatch.setattr(fr, "_estimate_angles", spy)
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    replay_dump(raw, ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN), lib=lib)
    assert any(rate is not None and rate > 0.0 for rate in seen)


def test_replay_track_rate_picks_the_branch_and_the_measured_phase_stays_the_rotor(lib):
    raw = synth_shot_dump(ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=TEE_RANGE_M, n_frames=24)
    meta, cube = parse_dump(raw)
    n_tx = int(meta["n_tx"])
    chirp_period_s = 45e-6
    hit = fw.TargetObs()
    hit.peakBin = TEE_BIN
    hit.dopplerPhaseRad = 2.5
    cal = fr._radar_cal(lib, ReplayConfig(tee_bin=TEE_BIN))  # pylint: disable=protected-access
    rate = 58.0
    obs, _ = fr._estimate_angles(  # pylint: disable=protected-access
        lib, cal, cube, 8, 0, n_tx, hit, 1.0, chirp_period_s, track_rate_mps=rate
    )
    assert obs is not None
    assert obs.chirpPhaseRad == pytest.approx(
        lib.l3_angle_chirp_phase(2.5, n_tx, rate, chirp_period_s), rel=1e-6
    )


def test_an_early_fire_does_not_make_the_frames_before_post_from_frame_post_impact(lib):
    """A sound-triggered recording's freeze is the impact (post_from_frame). A
    self-trigger fire before it (tee 33 fired the old range gate at frame 4 of
    a 2026-08-24 swing whose impact was frame 9) must not turn the approach
    into post-impact frames, or the club track is in follow mode early."""
    path = next(p for p in fr.RECORDINGS_DIR.glob("*20260824_120934_201_011.l3dump"))
    config = fr.ReplayConfig(tee_bin=33, post_from_frame=9)
    result = fr.replay_file(path, config, lib=lib)
    assert result.fired_frame is None or result.fired_frame >= 9
    assert not any(f.fired for f in result.frames if f.frame < 9)
    assert all(f.ball_why == "none" for f in result.frames if f.frame < 9)
    approach = [p for p in result.points if p.frame < 9]
    assert len(approach) >= 6
    assert [round(p.range_bin, 1) for p in approach if 2 <= p.frame <= 6] == [
        25.7,
        28.0,
        31.4,
        33.2,
        34.7,
    ]


# The kiosk's self-trigger as it runs on the board: the tee bin two short of
# the stock ball, the default snr and tee band. The club track's range-only
# impact fires it.
_KIOSK_TRIGGER = dict(
    tee_bin=38,
    snr=FIRMWARE_TRIGGER_DEFAULT_SNR,
    band_bins=TEE_BAND_DEFAULT_BINS,
    stop_at_fire=True,
    post_impact=False,
)


def _kiosk_recordings() -> list[Path]:
    # The 24-frame, 3 ms profile: the other captures of the day (36 frames of
    # 2 ms) have the ball elsewhere, so a fixed tee bin does not fit them.
    recordings = sorted(fr.RECORDINGS_DIR.glob("*20260824*.l3dump"))
    return [p for p in recordings if parse_dump(p.read_bytes())[0]["n_frames"] == 24]


def test_the_self_trigger_fires_around_impact_on_the_2026_08_24_recordings(lib):
    """2026-09-30: the kiosk recognised no swings. The range gate's own tracker
    held the tee's standing clutter (107 of its gate verdicts on the misses
    were 'slow') and fired on 10 of these 20 swings at the kiosk's settings,
    so it was removed. The club track, which predicts the club through
    standing bins and reads past the tee band, fires on 18, each at most
    three frames before the frame the capture froze on and at most one after
    it."""
    paths = _kiosk_recordings()
    assert len(paths) >= 20
    missed, early, late = [], [], []
    for path in paths:
        raw = path.read_bytes()
        result = replay_dump(raw, ReplayConfig(**_KIOSK_TRIGGER), lib=lib)
        if result.fired_frame is None:
            missed.append(path.name)
            continue
        lead = fr.freeze_frame(parse_dump(raw)[0]) - result.fired_frame
        if lead > 3:
            early.append((path.name, lead))
        elif lead < -1:
            late.append((path.name, lead))
    assert len(missed) <= 1, missed
    assert len(early) <= 1, early
    assert not late, late


def test_the_self_trigger_is_the_range_only_impact_unless_the_geometry_is_armed(lib):
    """The board freezes on the range-only impact, and the shot machine
    records that source."""
    fired = 0
    for path in _kiosk_recordings():
        result = replay_dump(path.read_bytes(), ReplayConfig(**_KIOSK_TRIGGER), lib=lib)
        assert result.fired_frame == result.range_frame, path.name
        if result.fired_frame is not None:
            fired += 1
            assert result.shot.impactSource == fw.SHOT_IMPACT_RANGE, path.name
            assert len(result.frames) == result.fired_frame + 1, "stop_at_fire"
    assert fired >= 18


def test_the_replay_only_sets_fields_the_shot_input_has():
    """ctypes takes any attribute silently: a field removed from
    l3_shot_input_t (gateFired went with the range gate) would leave the
    replay setting a Python attribute the C never reads."""
    import re

    source = Path(fr.__file__).read_text(encoding="utf-8")
    fields = {name for name, _type in fw.ShotInput._fields_}
    assigned = set(re.findall(r"\b(?:shot_in|forced_in)\.(\w+) =", source))
    assert assigned, "the replay feeds the shot machine"
    assert assigned <= fields, sorted(assigned - fields)


def _synthetic_shot_config() -> ReplayConfig:
    return ReplayConfig(
        tee_bin=TEE_BIN,
        dest_bin=TEE_BIN,
        overrides={"ball.fit.teeBallHeightM": 0.152, "ball.fit.radarHeightM": 0.152},
    )


def test_a_replayed_shot_carries_both_reconstructions(lib):
    raw = synth_shot_dump(
        ball_speed_ms=60.0, vla_deg=12.0, hla_deg=2.0, tee_range_m=TEE_RANGE_M, n_frames=24
    )
    result = replay_dump(raw, _synthetic_shot_config(), lib=lib)
    assert result.launch is not None
    assert result.launch.angle_why == "ok" and result.launch.angles_accepted >= 4
    assert result.launch.angle_rms_deg is not None
    fitted_ball = [p for p in result.ball_points if p.filtered_position is not None]
    assert len(fitted_ball) >= 4
    assert all(p.filter_hypothesis in fw.FILTER_HYP_NAMES for p in result.ball_points)
    fitted_club = [p for p in result.points if p.filtered_position is not None]
    assert len(fitted_club) >= 3, "the club is reconstructed over its held points"
    assert any(p.angle_confidence > 0.0 for p in result.points)


def test_the_club_is_reconstructed_once_for_the_viewer_only(lib, swing, monkeypatch):
    """The board never reconstructs the club and its frozen delivery is the
    unfiltered one; the replay reconstructs once at the end, for the viewer."""
    calls = []
    original_run = lib.l3_track_kf_run
    original_filtered = lib.l3_track_delivery_filtered

    def spy_run(*args):
        calls.append("kf")
        return original_run(*args)

    def spy_filtered(*args):
        calls.append("filtered")
        return original_filtered(*args)

    monkeypatch.setattr(lib, "l3_track_kf_run", spy_run)
    monkeypatch.setattr(lib, "l3_track_delivery_filtered", spy_filtered)
    result = replay_dump(swing, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    assert result.fired_frame is not None
    assert calls == ["kf"]


def test_a_point_summary_of_an_unfiltered_point_has_no_filtered_position():
    point = fw.TrackPoint()
    point.filterHypothesis = fw.FILTER_HYP_UNFILTERED
    summary = fr._point_summary(point)  # pylint: disable=protected-access
    assert summary.filtered_position is None and summary.filter_hypothesis == "unfiltered"


def test_the_ball_is_reconstructed_once_per_replay(lib, monkeypatch):
    """The board reconstructs the ball once, at RESULT; the replay once, at the end."""
    calls = []
    original = lib.l3_ball_track_reconstruct

    def spy(*args):
        calls.append("ball")
        return original(*args)

    monkeypatch.setattr(lib, "l3_ball_track_reconstruct", spy)
    raw = synth_shot_dump(
        ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=TEE_RANGE_M, n_frames=24
    )
    replay_dump(raw, _synthetic_shot_config(), lib=lib)
    assert calls == ["ball"]


def test_synthetic_shot_vla_is_read_back_by_the_direction_fit(lib):
    """End to end: the synthesized 12 deg launch is read back by the tee-anchored
    fit. The synthetic scene has no floor and its tee is at antenna height, so
    the anchor is put there too."""
    raw = synth_shot_dump(
        ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=TEE_RANGE_M, n_frames=24
    )
    result = replay_dump(raw, _synthetic_shot_config(), lib=lib)
    assert result.launch is not None and result.launch.vla_deg is not None
    assert result.launch.vla_deg == pytest.approx(12.0, abs=2.0)
    assert result.launch.angle_why == "ok"


def test_the_launch_line_says_why_the_angles_are_missing():
    from types import SimpleNamespace

    launch = fr.LaunchSummary(
        points=6,
        speed_mps=60.0,
        radial_speed_mps=59.0,
        hla_deg=None,
        vla_deg=None,
        residual_m=0.001,
        confidence=0.9,
        velocity=(0.0, 0.0, 0.0),
        angles_accepted=5,
        angle_why="uncertain",
    )
    line = fr._launch_line(SimpleNamespace(launch=launch))
    assert "why=uncertain angles=5" in line
