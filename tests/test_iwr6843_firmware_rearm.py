"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from __future__ import annotations

from pathlib import Path

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"
FIRMWARE_MAKEFILE = Path(__file__).parents[1] / "firmware" / "Makefile"
CONFIG_DIR = Path(__file__).parents[1] / "config"
WIDE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
DENSE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_36f2ms_53bin_iq8.cfg"
DENSE_IQ16_DIAGNOSTIC_CONFIG = CONFIG_DIR / "iwr6843_l3dump_diagnostic_24f2ms_53bin_iq16.cfg"
DENSE_WIDE_LATE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_36f2ms_53bin_iq8_wide_late.cfg"
COMPACT_IQ16_CONFIG = CONFIG_DIR / "iwr6843_l3dump_compact_16f2ms_32bin_iq16.cfg"
SHADOW_REFERENCE_CONFIG = (
    CONFIG_DIR / "iwr6843_l3dump_shadow_reference_7f2ms_128bin_iq16.cfg"
)


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_hwa_dump_stops_at_boundary_before_streaming():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    stop = dump.index("l3_stopCaptureAtBoundary")
    stream = dump.index("UART_writePolling")

    assert stop < stream


def test_hwa_chain_processes_and_rearms_one_frame_at_a_time():
    source = FIRMWARE.read_text(encoding="utf-8")
    common = _function_source(source, "static int32_t l3_configHwaCommon", "static void l3_drain")
    output = _function_source(
        source,
        "static int32_t l3_configHwaOutputEdma",
        "static int32_t l3_configHwaSignatureEdma",
    )

    assert "gCapturePlan.chirpsPerFrame / 2U" in common
    assert "gCapturePlan.chirpsPerFrame / 2U" in output


def test_hwa_rearm_task_preempts_cli_diagnostics():
    source = FIRMWARE.read_text(encoding="utf-8")

    assert "#define L3_HWA_REARM_TASK_PRIORITY (L3_CLI_TASK_PRIORITY + 1U)" in source


def test_completed_frame_advances_circular_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    callback = _function_source(
        source,
        "static void l3_hwaOutputDoneCB",
        "static int32_t l3_hwaStartRing",
    )

    assert "gRingFrame++" in callback
    assert "gRingFrame % RING_FRAMES" in callback


def test_freeze_request_keeps_rearming_until_post_trigger_target():
    source = FIRMWARE.read_text(encoding="utf-8")
    queue = _function_source(
        source, "static void l3_hwaMaybeQueueRearm", "static void l3_hwaChainDoneCB"
    )
    freeze = _function_source(
        source,
        "static int32_t l3_freezeHwaAfterPostFrames",
        "static int32_t l3_armHwaChain",
    )

    assert "gHwaFreezeRequested" in queue
    assert "gRingFrame >= gHwaFreezeTargetFrame" in queue
    assert "Semaphore_post(gHwaFreezeSemaphore)" in queue
    assert "gHwaFreezeTargetFrame = gRingFrame + HWA_POST_TRIGGER_FRAMES" in freeze


def test_sensor_stop_cancels_post_capture_at_next_completed_frame():
    source = FIRMWARE.read_text(encoding="utf-8")
    shutdown_freeze = _function_source(
        source,
        "static int32_t l3_freezeHwaForShutdown",
        "static int32_t l3_finishCaptureStop",
    )
    shutdown_stop = _function_source(
        source,
        "static int32_t l3_stopCaptureForShutdown",
        "static int32_t l3_armHwaChain",
    )
    sensor_stop = _function_source(
        source,
        "static int32_t l3_cli_sensorStop",
        "static void l3_initTask",
    )

    assert "gHwaShutdownRequested = 1U" in shutdown_freeze
    assert "gHwaFreezeRequested = 0U" in shutdown_freeze
    assert "gPostFramesCaptured = 0U" not in shutdown_freeze
    assert "gCapturePlan.postFrames" not in shutdown_freeze
    assert "l3_hwaMaybeQueueRearm()" in shutdown_freeze
    assert "l3_freezeHwaForShutdown" in shutdown_stop
    assert "l3_finishCaptureStop" in shutdown_stop
    assert "if (gCaptureActive)" in sensor_stop
    assert "l3_stopCaptureForShutdown()" in sensor_stop


def test_sensor_stop_closes_the_front_end_so_the_next_start_can_reconfigure():
    """MMWave_config is refused (-3110 subsys 83) while the BSS holds the old
    profile. sensorStop must close it; sensorStart reopens when unopened."""
    source = FIRMWARE.read_text(encoding="utf-8")
    sensor_stop = _function_source(
        source,
        "static int32_t l3_cli_sensorStop",
        "static void l3_initTask",
    )
    sensor_start = _function_source(
        source,
        "static int32_t l3_cli_sensorStart",
        "static int32_t l3_cli_sensorStop",
    )

    assert "MMWave_close(gMMWaveHandle" in sensor_stop
    assert "gSensorOpened = 0U" in sensor_stop
    assert sensor_stop.index("l3_stopCaptureForShutdown()") < sensor_stop.index("MMWave_close")
    assert "if (!gSensorOpened)" in sensor_start
    assert sensor_start.index("MMWave_open") < sensor_start.index("MMWave_config(")


def test_shutdown_waits_for_iq8_pack_without_rearming():
    source = FIRMWARE.read_text(encoding="utf-8")
    rearm = _function_source(
        source,
        "static void l3_hwaRearmTask",
        "/* Fill the 20-byte fixed dump header",
    )

    shutdown = rearm.index("gHwaShutdownRequested")
    pack = rearm.index("l3_waitForAllIq8Edma", shutdown)
    completion = rearm.index("Semaphore_post(gHwaFreezeSemaphore)", shutdown)
    restart = rearm.index("l3_restartCompletedHwaFrame", shutdown)

    assert shutdown < pack < completion < restart


def test_iq8_edma_pack_compacts_int16_scratch_without_cpu_loop():
    source = FIRMWARE.read_text(encoding="utf-8")
    pack = _function_source(
        source,
        "static int32_t l3_startIq8EdmaPack",
        "static void l3_waitForIq8EdmaScratch",
    )
    rearm = _function_source(
        source,
        "static void l3_hwaRearmTask",
        "/* Fill the 20-byte fixed dump header",
    )

    assert "param->aCount = 1U" in pack
    assert "param->bCount = (uint16_t)components" in pack
    assert "param->sourceBindex = (int16_t)sizeof(int16_t)" in pack
    assert "param->destinationBindex = 1" in pack
    assert "EDMA_startDmaTransfer" in pack
    assert "l3_restartCompletedHwaFrame" in rearm
    assert rearm.count("l3_storeCompletedScratchFrame") == 2


def test_iq8_edma_pack_waits_before_reusing_ping_pong_scratch():
    source = FIRMWARE.read_text(encoding="utf-8")
    rearm = _function_source(
        source,
        "static void l3_hwaRearmTask",
        "/* Fill the 20-byte fixed dump header",
    )

    assert "l3_waitForIq8EdmaScratch(nextScratch)" in rearm
    assert "l3_waitForAllIq8Edma()" in rearm


def test_dump_header_rotates_from_oldest_completed_frame():
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "gRingFrame % RING_FRAMES" in dump


def test_production_build_uses_configurable_compression_and_single_release():
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")

    assert "--define=N_TX=3" in target
    assert "--define=CONFIGURABLE_CAPTURE=1" in target
    assert "--define=HYBRID_CADENCE_CAPTURE=1" in target
    assert "--define=L3_RING_IQ8=1" in target
    assert "--define=L3_IQ8_EDMA_PACK=1" in target
    assert "--define=L3_IQ8_SPARSE_SCALE=1" not in target
    assert "--define=LOOPS=" not in target
    assert "--define=RING_FRAMES=" not in target
    assert "RELEASE_NAME ?= l3_dump_configurable_capture_20260818.bin" in source
    assert '"$(RELEASE_DIR)/$(RELEASE_NAME)"' in target
    assert source.count("\nbuild-native:") == 1


def _config_lines(path: Path) -> set[str]:
    return {
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("%")
    }


def test_wide_profile_uses_24_frames_at_3ms_with_53_bin_iq16_windows():
    lines = _config_lines(WIDE_CONFIG)

    assert "frameCfg 0 2 12 0 3 1 0" in lines
    assert "captureFormat iq16" in lines
    assert "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1" in lines


def test_dense_profile_uses_36_frames_at_2ms_with_53_bin_iq8_windows():
    lines = _config_lines(DENSE_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat iq8" in lines
    assert "phaseCaptureCfg 20 53 14 32 53 10 47 53 64 12 1" in lines


def test_diagnostic_profile_uses_memory_safe_2ms_iq16_capture():
    lines = _config_lines(DENSE_IQ16_DIAGNOSTIC_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat iq16" in lines
    assert "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1" in lines

    payload_bytes = 3 * 12 * 4 * 24 * 53 * 4
    assert payload_bytes == 732_672
    assert payload_bytes < 786_432


def test_compact_iq16_profile_keeps_2ms_cadence_with_fixed_window():
    lines = _config_lines(COMPACT_IQ16_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat compact16" in lines
    assert "phaseCaptureCfg 20 32 8 20 32 4 20 32 20 4 1" in lines

    retained_bytes = 3 * 12 * 4 * 16 * 32 * 4
    scratch_bytes = 2 * 3 * 16 * 4 * 128 * 4
    assert retained_bytes + scratch_bytes == 491_520
    assert retained_bytes + scratch_bytes < 786_432


def test_shadow_reference_profile_retains_seven_complete_frames():
    lines = _config_lines(SHADOW_REFERENCE_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat compact16" in lines
    assert "phaseCaptureCfg 0 128 3 0 128 2 0 128 0 2 1" in lines

    retained_bytes = 3 * 12 * 4 * 7 * 128 * 4
    scratch_bytes = 2 * 3 * 16 * 4 * 128 * 4
    assert retained_bytes + scratch_bytes == 712_704
    assert retained_bytes + scratch_bytes < 786_432


def test_compact_iq16_rearms_alternate_full_frame_before_copying():
    source = FIRMWARE.read_text(encoding="utf-8")
    output = _function_source(
        source, "static int32_t l3_configHwaFrameOutput", "static void l3_drain"
    )
    rearm = _function_source(
        source, "static void l3_hwaRearmTask", "/* Fill the 20-byte fixed dump header"
    )

    assert "binCount = N_SAMPLES" in output
    assert "l3_captureUsesScratch()" in output
    assert rearm.index("l3_restartCompletedHwaFrame()") < rearm.rindex(
        "l3_storeCompletedScratchFrame"
    )
    assert "gCaptureIncomplete = 1U" in source
    assert "compact16_max_us=%u" in source

    freeze = _function_source(
        source, "static int32_t l3_freezeCapture", "static void l3_sparseWindow"
    )
    assert "if (gCaptureIncomplete)" in freeze
    assert "capture incomplete" in freeze


def test_dense_wide_late_profile_keeps_dense_timing_and_near_late_window():
    lines = _config_lines(DENSE_WIDE_LATE_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat iq8" in lines
    assert "iq8Scale 128" in lines
    assert "phaseCaptureCfg 20 53 14 32 53 10 47 53 47 12 1" in lines


def test_supported_profiles_keep_the_same_72ms_movie():
    for path, expected_frames, expected_period_ms in (
        (WIDE_CONFIG, 24, 3.0),
        (DENSE_CONFIG, 36, 2.0),
        (DENSE_WIDE_LATE_CONFIG, 36, 2.0),
    ):
        commands = {line.split()[0]: line.split() for line in _config_lines(path)}
        frame = commands["frameCfg"]
        phase = commands["phaseCaptureCfg"]
        assert sum(int(value) for value in (phase[3], phase[6], phase[10])) == expected_frames
        assert float(frame[5]) == expected_period_ms
        assert expected_frames * expected_period_ms == 72.0


def test_supported_profiles_fit_the_l3_capture_budget():
    tx, loops, rx = 3, 12, 4
    wide_bytes = tx * loops * rx * 24 * 53 * 4
    dense_bytes = tx * loops * rx * 36 * 53 * 2

    assert wide_bytes == 732_672
    assert dense_bytes == 549_504
    assert wide_bytes < 786_432
    assert dense_bytes < 688_128


def test_dynamic_window_start_is_recorded_per_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    output = _function_source(
        source,
        "static int32_t l3_configHwaFrameOutput",
        "static void l3_drainHwaRearmSemaphore",
    )
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    assert "gFrameBinStart[ringSlot % RING_FRAMES]" in output
    assert "UART_writePolling(gDataUart, gFrameBinStart" in dump


# --- l3track: on-chip ball track and cell selection -------------------------

import re  # noqa: E402

from openflight.iwr6843 import sparse  # noqa: E402

TRACK_HEADER = Path(__file__).parents[1] / "firmware" / "iwr6843" / "track_select.h"
APP_MAKEFILE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "makefile"


def _define(source: str, name: str) -> int:
    match = re.search(rf"#define\s+{name}\s+(\d+)U", source)
    assert match, name
    return int(match.group(1))


def test_track_commands_are_registered_on_the_cli():
    source = FIRMWARE.read_text(encoding="utf-8")

    assert 'tableEntry[13].cmd           = "l3track"' in source
    assert "tableEntry[13].cmdHandlerFxn = l3_cli_track;" in source
    assert 'tableEntry[14].cmd           = "trackCfg"' in source
    assert "tableEntry[14].cmdHandlerFxn = l3_cli_trackCfg;" in source


def test_track_select_is_built_into_the_image():
    makefile = APP_MAKEFILE.read_text(encoding="utf-8")

    assert re.search(r"^SOURCES\s*=.*\btrack_select\.c\b", makefile, re.MULTILINE)


def test_live_selector_runs_on_full_scratch_before_reference_compaction():
    source = FIRMWARE.read_text(encoding="utf-8")
    makefile = APP_MAKEFILE.read_text(encoding="utf-8")
    store = _function_source(
        source,
        "static void l3_storeCompletedScratchFrame",
        "static uint32_t l3_snapshotBinStart",
    )

    assert re.search(r"^SOURCES\s*=.*\blive_selector\.c\b", makefile, re.MULTILINE)
    assert store.index("l3_live_select(") < store.index("l3_compact_iq16(")
    assert "gShadowPower[bin]" in store
    assert "current - gShadowPreviousPower[bin]" in store
    assert "shadow_max_us=%u" in source


def test_live_selector_scan_is_bounded_below_full_cube_work():
    source = FIRMWARE.read_text(encoding="utf-8")
    store = _function_source(
        source,
        "static void l3_storeCompletedScratchFrame",
        "static uint32_t l3_snapshotBinStart",
    )

    assert "L3_SHADOW_LOOP_STRIDE 3U" in source
    assert "L3_SHADOW_RX_STRIDE 2U" in source
    assert "loop += L3_SHADOW_LOOP_STRIDE" in store
    assert "rx += L3_SHADOW_RX_STRIDE" in store
    assert "chirp < gCapturePlan.chirpsPerFrame" not in store


def test_track_limits_match_the_capture_limits():
    """The tracker's fixed buffers must hold any capture the ring can freeze."""
    firmware = FIRMWARE.read_text(encoding="utf-8")
    header = TRACK_HEADER.read_text(encoding="utf-8")

    assert _define(header, "L3T_MAX_FRAMES") == _define(firmware, "L3_MAX_CAPTURE_FRAMES")
    assert _define(header, "L3T_MAX_LOOPS") == _define(firmware, "L3_MAX_LOOPS")
    assert _define(header, "L3T_MAX_BINS") == _define(firmware, "L3_RING_MAX_BINS")


def test_track_command_freezes_selects_streams_then_rearms():
    source = FIRMWARE.read_text(encoding="utf-8")
    track = _function_source(source, "int32_t l3_cli_track(", "static int32_t l3_cli_trackCfg")

    order = [
        track.index("gTrackConfigured"),
        track.index("l3_sparseFreeze()"),
        track.index("l3track_select("),
        track.index('l3_sparseWriteHeader("ILT1"'),
        track.index('"ILS1"'),
        track.index("l3_sparseWriteCell("),
        track.rindex("return l3_sparseRearm();"),
    ]
    assert order == sorted(order)
    # A rejected layout still restarts the ring before reporting the error.
    rejected = track[track.index("if (cellCount < 0)") : track.index('l3_sparseWriteHeader("ILT1"')]
    assert "l3_sparseRearm()" in rejected


def test_track_record_matches_the_host_parser():
    """ILT1 track record: the firmware writes the fields sparse.parse_track reads."""
    source = FIRMWARE.read_text(encoding="utf-8")
    track = _function_source(source, "int32_t l3_cli_track(", "static int32_t l3_cli_trackCfg")
    record = track[track.index('l3_sparseWriteHeader("ILT1"') : track.index('"ILS1"')]
    writes = re.findall(r"l3_write(U16|F32)\(", record)

    codes = "".join("H" if kind == "U16" else "f" for kind in writes)
    assert "<" + codes == sparse._TRACK_RECORD.format  # pylint: disable=protected-access


def test_sparse_and_track_share_one_header_writer():
    """ILP1 and ILT1 must stay byte-compatible with sparse._parse_layout."""
    source = FIRMWARE.read_text(encoding="utf-8")
    header = _function_source(
        source, "static void l3_sparseWriteHeader", "static void l3_sparseWriteCell"
    )
    writes = re.findall(r"l3_write(U16|F32)\(", header)

    codes = "".join("H" if kind == "U16" else "f" for kind in writes)
    assert "<4s" + codes == sparse._POWER_HEADER.format  # pylint: disable=protected-access
    sparse_cmd = _function_source(
        source, "int32_t l3_cli_sparse(int32_t argc", "static L3TrackWorkspace"
    )
    assert 'l3_sparseWriteHeader("ILP1"' in sparse_cmd


def test_track_config_takes_the_fields_the_runtime_sends():
    source = FIRMWARE.read_text(encoding="utf-8")
    config = _function_source(source, "static int32_t l3_cli_trackCfg", '/* CLI "triggerCfg')

    assert "argc != 6" in config
    assert "strtod(" in config
    assert "l3track_default_params(&gTrackParams)" in config


def test_stats_reports_trigger_state_and_debug_prints_on_phase_change_only():
    """A missed Triggered line must still be visible, without a per-frame UART write."""
    source = FIRMWARE.read_text(encoding="utf-8")
    stats = _function_source(
        source, "static int32_t l3_cli_stats", "static int32_t l3_cli_hwaStats"
    )
    debug_write = _function_source(
        source,
        "static void l3_writeTriggerDebug",
        "static void l3_noteTrigger",
    )
    debug_cfg = _function_source(
        source,
        "static int32_t l3_cli_debugCfg",
        "static int32_t l3_cli_stats",
    )

    assert 'CLI_write("trig phase=%s tee=%u latched=%u enabled=%u\\n"' in stats
    assert "l3_triggerPhaseName(gTriggerPhase)" in stats
    assert stats.index("trig phase=") < stats.index("return 0")
    assert "if (phase == gTriggerDebugPhase)" in debug_write
    assert debug_write.index("gTriggerDebugPhase = phase") < debug_write.index("CLI_write(")
    assert "gTriggerDebugPhase = 0xFFU" in debug_cfg


def test_stats_reports_rearm_latency_maximum_for_2ms_deadline_checks():
    source = FIRMWARE.read_text(encoding="utf-8")
    rearm = _function_source(
        source,
        "static void l3_hwaRearmTask",
        "/* Fill the 20-byte fixed dump header",
    )
    stats = _function_source(
        source, "static int32_t l3_cli_stats", "static int32_t l3_cli_hwaStats"
    )

    assert "Cycleprofiler_getTimeStamp()" in rearm
    assert "gHwaRearmMaxUs" in rearm
    assert 'CLI_write("rearm_last_us=%u rearm_max_us=%u\\n"' in stats


def test_sensor_start_discards_the_previous_self_trigger_latch():
    source = FIRMWARE.read_text(encoding="utf-8")
    start = _function_source(
        source, "static int32_t l3_cli_sensorStart", "static int32_t l3_cli_sensorStop"
    )
    assert start.index("gSelfTriggerLatched = 0U") < start.index("l3_armCapture()")
    assert start.index("gTriggerEnabled = 0U") < start.index("l3_armCapture()")
