"""Adaptive IQ16 retains channel identity and reports discarded flight evidence."""

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
    counts = ([32] * 14 + [53] * 6 + [12] * 16)[:frames]
    starts = ([20] * 14 + [32] * 6 + list(range(47, 95, 3)))[:frames]
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


def test_operator_check_rejects_processing_overrun_in_adaptive_capture(hardware_check):
    hardware_check._check_timing(dict(shadow_max_us=773, compact16_max_us=537), 2000, True)
    with pytest.raises(RuntimeError, match="selector plus compaction"):
        hardware_check._check_timing(dict(shadow_max_us=1500, compact16_max_us=537), 2000, True)
