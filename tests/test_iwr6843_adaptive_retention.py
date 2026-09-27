"""Adaptive IQ16 retains channel identity and reports discarded flight evidence."""

import argparse
import struct
from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843.dump import (
    HEADER,
    pack_dump,
    parse_dump,
    project_tx_pair,
    select_tdm_loops,
)
from openflight.iwr6843.shot import prepare_shot_dump

ROOT = Path(__file__).parents[1]


def _capture(reason="complete", frames=36):
    # Matches config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg's calibrated
    # phaseCaptureCfg (preStart=22, impactStart=34); see that file's comment
    # for the range-bias calibration this is derived from.
    counts = ([32] * 14 + [53] * 6 + [12] * 16)[:frames]
    starts = ([22] * 14 + [34] * 6 + list(range(47, 95, 3)))[:frames]
    values = np.arange(frames * 36 * 4 * 53).reshape(frames, 36, 4, 53)
    cube = ((values % 65536) - 32768) + 1j * ((values % 32768) - 16384)
    raw = pack_dump(
        cube,
        n_tx=3,
        version=8,
        frame_period_us=2000,
        sample_fmt=4,
        range_bin_starts=starts,
        range_bin_counts=counts,
        frame_time_offsets_us=list(range(0, frames * 2000, 2000)),
        retention=dict(reason=reason, pre_frames=14, planned_frames=36),
    )
    return raw, cube, starts, counts


def test_complete_adaptive_capture_retains_every_channel_and_loop_exactly():
    raw, original, starts, counts = _capture()
    meta, decoded = parse_dump(raw)
    assert len(raw) == 551_808 + 20 + 8 + 36 * 4
    assert meta["retention"] == dict(reason="complete", pre_frames=14, planned_frames=36)
    assert meta["frame_time_offsets_us"] == tuple(range(0, 72_000, 2000))
    assert meta["range_bin_starts"] == tuple(starts)
    assert decoded.shape == (36, 36, 4, 53)
    for frame, count in enumerate(counts):
        np.testing.assert_array_equal(decoded[frame, ..., :count], original[frame, ..., :count])
        assert not np.any(decoded[frame, ..., count:])
    projected_meta, projected = parse_dump(project_tx_pair(raw, (0, 2)))
    assert projected_meta["retention"] == meta["retention"]
    expected = decoded.reshape(36, 12, 3, 4, 53)[:, :, [0, 2]].reshape(36, 24, 4, 53)
    np.testing.assert_array_equal(projected, expected)
    reduced_meta, reduced = parse_dump(select_tdm_loops(raw, start=1, count=10))
    assert reduced_meta["retention"] == meta["retention"]
    np.testing.assert_array_equal(reduced, decoded[:, 3:33])


@pytest.mark.parametrize("reason", ["track_lost", "ambiguous", "range_edge", "short_history"])
def test_early_stop_retains_diagnostics_but_cannot_be_measured_as_complete(reason):
    raw, _, _, _ = _capture(reason, frames=20)
    meta, decoded = parse_dump(raw)
    assert meta["retention"]["reason"] == reason
    assert decoded.shape[0] == 20
    with pytest.raises(ValueError, match="adaptive retention stopped"):
        prepare_shot_dump(raw)


def test_adaptive_metadata_cannot_silently_claim_missing_frames_are_complete():
    raw, _, _, _ = _capture("track_lost", frames=20)
    corrupt = bytearray(raw)
    struct.pack_into("<H", corrupt, HEADER.size, 0)
    with pytest.raises(ValueError, match="missing frames"):
        parse_dump(corrupt)
    for end in (20, 24, len(raw) - 1):
        with pytest.raises(ValueError, match="short"):
            parse_dump(raw[:end])


def test_experimental_profile_fits_actual_max_loop_scratch_reservation():
    commands = {
        fields[0]: fields[1:]
        for line in (ROOT / "config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg")
        .read_text()
        .splitlines()
        if (fields := line.split()) and not line.startswith("%")
    }
    assert commands["captureFormat"] == ["adaptive16"]
    assert commands["frameCfg"] == "0 2 12 0 2 1 0".split()
    phase = list(map(int, commands["phaseCaptureCfg"]))
    bins = phase[1] * phase[2] + phase[4] * phase[5] + phase[7] * phase[9]
    assert phase[2] + phase[5] + phase[9] == 36
    assert phase[10] == 1
    assert bins * 576 == 551808
    assert 786432 - bins * 576 - 2 * 16 * 3 * 4 * 128 * 4 == 38016


def test_firmware_v9_layout_with_temperature_and_retention():
    from openflight.iwr6843.dump import TEMP_REPORT

    raw, expected, _, counts = _capture()
    fields = list(HEADER.unpack_from(raw))
    fields[1] = 9
    wire = HEADER.pack(*fields) + TEMP_REPORT.pack(1234, *range(40, 50)) + raw[20:]
    meta, actual = parse_dump(wire)
    assert meta["temperature_report"]["device_time_ms"] == 1234
    assert meta["retention"]["reason"] == "complete"
    for frame, count in enumerate(counts):
        np.testing.assert_array_equal(actual[frame, ..., :count], expected[frame, ..., :count])
    projected_meta, _ = parse_dump(project_tx_pair(wire))
    assert projected_meta["retention"] == meta["retention"]
    assert projected_meta["temperature_report"] == meta["temperature_report"]


@pytest.fixture
def hardware_check():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "adaptive_hardware_check", ROOT / "scripts/hardware-test/check_iwr_2ms_iq16.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_combined_frame_work_must_fit_the_period(hardware_check):
    hardware_check._check_timing({"frame_work_max_us": 1999}, 2000, True)
    with pytest.raises(RuntimeError, match="combined frame work"):
        hardware_check._check_timing({"frame_work_max_us": 2000}, 2000, True)


def test_combined_frame_work_target_is_stricter_than_the_hard_deadline(hardware_check):
    hardware_check._check_timing(
        {"frame_work_max_us": 1500}, 2000, True, target_work_us=1500
    )
    with pytest.raises(RuntimeError, match="engineering target"):
        hardware_check._check_timing(
            {"frame_work_max_us": 1501}, 2000, True, target_work_us=1500
        )


@pytest.mark.parametrize("latched", [0, 1])
def test_static_soak_failure_preserves_stats_and_trigger_log(hardware_check, latched):
    import io
    import json

    class Radar:
        def cmd(self, command):
            assert command == "triggerLog"
            return "frame=2 why=fired bin=40\nDone"

    output = io.StringIO()
    stats = {"latched": latched, "frame_work_max_us": 3224}
    with pytest.raises(RuntimeError):
        hardware_check._check_static_health(Radar(), output, stats, {}, 2000, True, True)
    event = json.loads(output.getvalue())
    assert event["event"] == "failure"
    assert event["stats"] == stats
    assert "why=fired" in event["trigger_log"]


def test_static_trigger_failure_saves_the_frozen_capture(hardware_check, tmp_path):
    import io
    import json

    class Radar:
        def cmd(self, command):
            assert command == "triggerLog"
            return "frame=2 why=fired bin=40\nDone"

        def read_dump(self):
            return b"frozen IQ16 dump"

    output = io.StringIO()
    stats = {"latched": 1, "frame_work_max_us": 1200}
    with pytest.raises(RuntimeError):
        hardware_check._check_static_health(
            Radar(), output, stats, {}, 2000, True, True, capture_dir=tmp_path
        )
    event = json.loads(output.getvalue())
    capture_path = Path(event["capture_path"])
    assert capture_path.parent == tmp_path
    assert capture_path.read_bytes() == b"frozen IQ16 dump"


def test_waits_for_full_pretrigger_history_after_capture_rearm(hardware_check, monkeypatch):
    readings = iter(
        [
            {"pre_seen": 8, "frame_work_max_us": 1200},
            {"pre_seen": 14, "frame_work_max_us": 1300},
        ]
    )
    monkeypatch.setattr(hardware_check, "_health", lambda _radar: next(readings))
    monkeypatch.setattr(hardware_check.time, "sleep", lambda _seconds: None)

    result = hardware_check._wait_for_pretrigger_history(
        radar=object(),
        output=None,
        stats={"pre_seen": 0, "frame_work_max_us": 1000},
        period_us=2000,
        compact_mode=True,
        armed=False,
        target_work_us=1500,
        capture_dir=None,
        required_frames=14,
    )

    assert result["pre_seen"] == 14


def test_flight_frames_kept_counts_only_selector_controlled_frames(hardware_check):
    config = str(ROOT / "config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg")
    static_raw, *_ = _capture(reason="track_lost", frames=20)
    static_metadata, _ = parse_dump(static_raw)
    assert hardware_check.flight_frames_kept(static_metadata, config) == 0

    complete_raw, *_ = _capture(reason="complete", frames=36)
    complete_metadata, _ = parse_dump(complete_raw)
    assert hardware_check.flight_frames_kept(complete_metadata, config) == 16

    partial_raw, *_ = _capture(reason="range_edge", frames=24)
    partial_metadata, _ = parse_dump(partial_raw)
    assert hardware_check.flight_frames_kept(partial_metadata, config) == 4


def test_expect_no_flight_frames_fails_a_static_scene_that_kept_any(hardware_check, monkeypatch):
    """The 2026-09-26 confirmation fix must be caught by this flag if it regresses:
    a static scene that ends up retaining even one selector-controlled frame means a
    track was accepted or coasted from something other than trigger noise."""
    config = str(ROOT / "config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg")
    raw, _, starts, counts = _capture(
        reason="range_edge", frames=22
    )  # 14 pre + 6 impact + 2 flight
    metadata, _ = parse_dump(raw)
    decisions = [
        dict(accepted=1, coasting=0, selected=start, proposed_start=start, proposed_bins=count)
        for start, count in zip(starts, counts, strict=False)
    ]

    class FakeRadar:
        def __init__(self, _port):
            pass

        def send_config(self, _config):
            pass

        def stats(self):
            return "Done"

        def read_shadow_dump(self):
            return raw, decisions

        def stop_sensor(self):
            pass

        def close(self):
            pass

    monkeypatch.setattr(hardware_check, "IWR6843Radar", FakeRadar)
    monkeypatch.setattr(
        hardware_check,
        "parse_capture_stats",
        lambda _response: {"frames": 0, "format": "adaptive16", "pre_seen": 14},
    )
    args = argparse.Namespace(
        port=None,
        config=config,
        soak_frames=0,
        cycles=1,
        poll_s=1.0,
        output=None,
        allow_early_stop=True,
        capture_dir=None,
        shadow=True,
        expect_no_flight_frames=True,
        self_trigger_tee_m=None,
    )
    with pytest.raises(RuntimeError, match="expected a static scene"):
        hardware_check.run(args)

    args.expect_no_flight_frames = False
    hardware_check.run(args)  # same capture passes once the flag is off


def test_operator_check_verifies_actual_stored_windows(hardware_check):
    raw, _, starts, _ = _capture()
    metadata, _ = parse_dump(raw)
    config = ROOT / "config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg"
    decisions = [dict(accepted=1, proposed_start=start, proposed_bins=12) for start in starts]
    assert hardware_check._expected_geometry(str(config)) == (36, 2000)
    hardware_check._check_retention_layout(metadata, decisions, str(config))
    decisions[20]["proposed_start"] += 1
    with pytest.raises(RuntimeError, match="differs from selector"):
        hardware_check._check_retention_layout(metadata, decisions, str(config))
    metadata["n_tx"] = 2
    with pytest.raises(RuntimeError, match="antenna channels"):
        hardware_check._check_retention_layout(metadata, [], str(config))


def test_operator_check_accepts_coasting_flight_frames_but_not_untracked_ones(hardware_check):
    raw, _, starts, _ = _capture()
    metadata, _ = parse_dump(raw)
    config = ROOT / "config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg"
    decisions = [
        dict(accepted=1, coasting=0, proposed_start=start, proposed_bins=12) for start in starts
    ]
    decisions[25].update(accepted=0, coasting=1)
    hardware_check._check_retention_layout(metadata, decisions, str(config))
    decisions[25].update(coasting=0)
    with pytest.raises(RuntimeError, match="differs from selector"):
        hardware_check._check_retention_layout(metadata, decisions, str(config))


def test_operator_check_uses_combined_work_not_unrelated_stage_maxima(hardware_check):
    hardware_check._check_timing(
        dict(frame_work_max_us=1499, shadow_max_us=1500, compact16_max_us=537),
        2000,
        True,
    )


def test_stats_keeps_acquisition_frames_separate_from_detector_frames(hardware_check):
    stats = hardware_check.parse_capture_stats(
        "frames=1500 active=1\n"
        "workmax_frame=847 workmax_rearm_us=39 workmax_shadow_us=412 "
        "workmax_compact_us=311 workmax_trigger_us=228 workmax_total_us=1041\n"
        "trig state=idle frames=0 cand=0\nlatched=0 enabled=0\n"
    )
    assert stats["frames"] == 1500
    assert stats["workmax_frame"] == 847
    assert stats["workmax_shadow_us"] == 412
    assert stats["workmax_total_us"] == 1041
    assert stats["trigger_frames"] == 0
