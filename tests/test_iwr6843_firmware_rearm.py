"""Regression checks for the HWA snapshot ring freeze/rearm sequence."""

from __future__ import annotations

from pathlib import Path

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"
CAPTURE_PLAN = Path(__file__).parents[1] / "firmware" / "iwr6843" / "capture_plan.c"
FIRMWARE_MAKEFILE = Path(__file__).parents[1] / "firmware" / "Makefile"
CONFIG_DIR = Path(__file__).parents[1] / "config"
WIDE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
DENSE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_45f2ms_53bin_iq8.cfg"
DENSE_WIDE_LATE_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_36f2ms_53bin_iq8_wide_late.cfg"
DENSE_51_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_51f2ms_53bin_iq8.cfg"
DENSE_54_CONFIG = CONFIG_DIR / "iwr6843_l3dump_dense_54f2ms_32prebin_iq8.cfg"


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_hwa_dump_stops_at_boundary_before_streaming():
    """Stopping (or, after a self-trigger, awaiting the already-latched
    freeze -- see test_dump_waits_on_an_already_latched_freeze_instead_of_
    requesting_a_new_one in test_iwr6843_firmware_sparse.py) must happen
    before any ring bytes go out."""
    source = FIRMWARE.read_text(encoding="utf-8")
    dump = _function_source(source, "int32_t l3_cli_dump", "static int32_t l3_cli_stats")

    stop = dump.index("l3_awaitFrozenRing()")
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


def test_completed_frame_advances_circular_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    callback = _function_source(
        source,
        "static void l3_hwaOutputDoneCB",
        "static int32_t l3_hwaStartRing",
    )

    assert "gRingFrame++" in callback
    assert "(gPreFramesCaptured % gCapturePlan.preFrames) == 0U" in callback


def test_freeze_request_keeps_rearming_until_post_trigger_target():
    source = FIRMWARE.read_text(encoding="utf-8")
    queue = _function_source(
        source, "static void l3_hwaMaybeQueueRearm", "static void l3_hwaChainDoneCB"
    )

    assert "gHwaFreezeRequested" in queue
    assert "l3_shouldFreezeNow()" in queue
    assert "Semaphore_post(gHwaFreezeSemaphore)" in queue


def test_freeze_condition_is_written_once():
    source = FIRMWARE.read_text(encoding="utf-8")
    assert source.count("gPostFramesCaptured >= gCapturePlan.postFrames") == 1
    assert "l3_shouldFreezeNow" in source
    assert source.count("l3_shouldFreezeNow()") >= 3


def test_shouldFreezeNow_predicate_holds_the_freeze_condition():
    source = FIRMWARE.read_text(encoding="utf-8")
    predicate = _function_source(
        source,
        "CALLER MUST HOLD THE CRITICAL SECTION",
        "static void l3_hwaMaybeQueueRearm",
    )

    assert "CALLER MUST HOLD THE CRITICAL SECTION" in predicate
    assert "gHwaFreezeRequested" in predicate
    assert "gActiveFrameIsPost" in predicate
    assert "gActiveFrameShouldKeep" in predicate
    assert "gPostFramesCaptured >= gCapturePlan.postFrames" in predicate


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
    # The rearm task hands a completed scratch frame to l3_storeCompletedFrame
    # (once before a freeze, once after the HWA restart); that dispatcher is
    # where the EDMA pack, the CPU pack and the compact copy part ways.
    assert rearm.count("l3_storeCompletedFrame(pendingSlot, pendingScratch);") == 2
    store = _function_source(
        source,
        "static void l3_storeCompletedFrame",
        "static uint32_t l3_snapshotBinStartForNextFrame",
    )
    assert "l3_startIq8EdmaPack(slot, scratch)" in store
    assert store.count("#else\n    l3_packIq8CompletedFrame(slot, scratch);") == 1
    assert store.index("l3_compactCompletedFrame(slot, scratch);") < store.index(
        "l3_startIq8EdmaPack"
    )


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

    assert (
        "oldestPre = (gPreFramesCaptured >= gCapturePlan.preFrames)\n"
        "                    ? (gPreFramesCaptured % gCapturePlan.preFrames) : 0U;" in dump
    )
    assert "uint32_t slot = (oldestPre + i) % gCapturePlan.preFrames;" in dump


def test_production_build_uses_configurable_compression_and_single_release():
    source = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    target = _function_source(source, "build-native:", "clean:")

    assert "--define=N_TX=3" in target
    assert "--define=CONFIGURABLE_CAPTURE" not in target
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


def test_dense_profile_keeps_2ms_frames_and_extends_the_ball_phase():
    lines = _config_lines(DENSE_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat iq8" in lines
    assert "phaseCaptureCfg 20 53 8 32 53 10 47 53 64 27 1" in lines


def test_dense_wide_late_profile_keeps_dense_timing_and_near_late_window():
    lines = _config_lines(DENSE_WIDE_LATE_CONFIG)

    assert "frameCfg 0 2 12 0 2 1 0" in lines
    assert "captureFormat iq8" in lines
    assert "iq8Scale 128" in lines
    assert "phaseCaptureCfg 20 53 14 32 53 10 47 53 47 12 1" in lines


def test_dense_51_profile_extends_the_ball_phase_at_2ms():
    commands = {line.split()[0]: line.split() for line in _config_lines(DENSE_51_CONFIG)}
    frame = commands["frameCfg"]
    phase = commands["phaseCaptureCfg"]
    assert float(frame[5]) == 2.0
    assert int(frame[3]) == 12
    assert commands["captureFormat"][1] == "iq8"
    # preFrames, impactFrames, ballFrames
    assert [int(phase[3]), int(phase[6]), int(phase[10])] == [8, 10, 33]
    assert sum((int(phase[3]), int(phase[6]), int(phase[10]))) == 51
    # every window is 53 bins
    assert [int(phase[2]), int(phase[5]), int(phase[8])] == [53, 53, 53]


def test_narrow_pre_profile_is_opt_in_and_fits():
    commands = {line.split()[0]: line.split() for line in _config_lines(DENSE_54_CONFIG)}
    phase = commands["phaseCaptureCfg"]
    assert int(phase[2]) == 32, "pre window should be narrowed"
    assert [int(phase[5]), int(phase[8])] == [53, 53], "impact and ball stay wide"
    assert sum((int(phase[3]), int(phase[6]), int(phase[10]))) == 54
    tx, loops, rx = 3, 12, 4
    pre = tx * loops * rx * 8 * 32 * 2
    rest = tx * loops * rx * (10 + 36) * 53 * 2
    assert pre + rest <= 786_432


def test_supported_profiles_keep_their_capture_duration():
    for path, expected_frames, expected_period_ms, expected_duration_ms in (
        (WIDE_CONFIG, 24, 3.0, 72.0),
        (DENSE_CONFIG, 45, 2.0, 90.0),
        (DENSE_WIDE_LATE_CONFIG, 36, 2.0, 72.0),
    ):
        commands = {line.split()[0]: line.split() for line in _config_lines(path)}
        frame = commands["frameCfg"]
        phase = commands["phaseCaptureCfg"]
        assert sum(int(value) for value in (phase[3], phase[6], phase[10])) == expected_frames
        assert float(frame[5]) == expected_period_ms
        assert expected_frames * expected_period_ms == expected_duration_ms


def _parse_define(source: str, name: str) -> str:
    match = re.search(rf"^#define\s+{name}\s+(.+)$", source, re.MULTILINE)
    assert match, f"{name} not found in the firmware source"
    return match.group(1).strip()


def test_supported_profiles_fit_the_l3_capture_budget():
    tx, loops, rx = 3, 12, 4
    wide_bytes = tx * loops * rx * 24 * 53 * 4
    dense_bytes = tx * loops * rx * 51 * 53 * 2
    wide_late_bytes = tx * loops * rx * 36 * 53 * 2
    iq8_capacity = 786_432
    dense_frame_bytes = tx * loops * rx * 53 * 2
    dense_post_bytes = (10 + 33) * dense_frame_bytes

    assert wide_bytes == 732_672
    assert dense_bytes == 778_464
    assert wide_late_bytes == 549_504
    assert wide_bytes <= iq8_capacity
    assert dense_bytes <= iq8_capacity
    assert wide_late_bytes <= iq8_capacity
    assert 8 * dense_frame_bytes <= iq8_capacity - dense_post_bytes

    # The arena-capacity function must still describe the whole L3 region:
    # L3_IQ8_CAPTURE_BYTES was removed once l3_captureCapacityBytes() started
    # returning L3_TOTAL_BYTES unconditionally (IQ8 and IQ16 both use the
    # entire arena now that the IQ16 scratch lives in DATA_RAM). The
    # surviving cross-check is that L3_TOTAL_BYTES is derived from the SDK
    # bank defines, and that l3_captureCapacityBytes() returns it
    # unconditionally rather than gating on capture format.
    source = FIRMWARE.read_text(encoding="utf-8")
    assert _parse_define(source, "L3_TOTAL_BYTES") == (
        "(MMWAVE_L3RAM_NUM_BANK * MMWAVE_SHMEM_BANK_SIZE)"
    ), "L3_TOTAL_BYTES is no longer derived from the SDK bank defines"

    capacity_fn = _function_source(
        source,
        "static uint32_t l3_captureCapacityBytes",
        "static uint32_t l3_captureBytesPerComplex",
    )
    assert "return L3_TOTAL_BYTES;" in capacity_fn, (
        "l3_captureCapacityBytes() must unconditionally return the whole L3 arena"
    )


def test_dynamic_window_start_is_recorded_per_ring_slot():
    source = FIRMWARE.read_text(encoding="utf-8")
    descriptor = _function_source(
        source,
        "static void l3_writeFrameDescriptor",
        "static uint16_t l3_iq8FrameScale",
    )
    # The per-frame bin-start arithmetic itself was extracted into
    # capture_plan.c (l3plan_build) so the host compiler/ctypes can test it;
    # l3_dump.c only owns wiring gCapturePlan/gFrameBinStart into the call
    # and consuming the result (l3_writeFrameDescriptor below).
    build = CAPTURE_PLAN.read_text(encoding="utf-8")

    assert "tables->binStart[frame] = plan->preStart;" in build
    assert "tables->binStart[slot] =" in build
    assert "descriptor[0] = gFrameBinStart[slot];" in descriptor


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
        "static int32_t l3_formatTriggerDebug",
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
    # The line is formatted, not written, here: the detect task queues it for
    # the notice task and the CLI task writes it itself.
    assert debug_write.index("gTriggerDebugPhase = phase") < debug_write.index("snprintf(")
    assert "CLI_write(" not in debug_write
    assert "gTriggerDebugPhase = 0xFFU" in debug_cfg


def test_dead_build_variants_are_gone():
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "LIVE_SNAPSHOT_RING" not in source
    assert "CONFIGURABLE_CAPTURE" not in source
    # Live variants must survive the cleanup.
    assert "L3_RING_IQ8" in source
    assert "L3_IQ8_EDMA_PACK" in source
    assert "HWA_CHAINED_SNAPSHOT_RING" in source


def test_l3_total_bytes_is_derived_from_the_sdk_bank_defines():
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "6U * 128U * 1024U" not in source
    assert "MMWAVE_L3RAM_NUM_BANK * MMWAVE_SHMEM_BANK_SIZE" in source


def test_derived_arena_matches_the_linker_region(tmp_path):
    """Pin the 6-bank arena geometry the derived L3_TOTAL_BYTES relies on.

    This does not read l3_dump.c or the bank-count defines themselves (see
    test_l3_total_bytes_is_derived_from_the_sdk_bank_defines for that); it
    only pins that the linker's L3_RAM region is still 6 * 128 KiB, which is
    the assumption MMWAVE_L3RAM_NUM_BANK * MMWAVE_SHMEM_BANK_SIZE must keep
    matching. Skips without a local build, like the other map-based checks.
    """
    from tests.test_iwr6843_memory_layout import _memory_rows

    used, unused = _memory_rows()["L3_RAM"]
    assert used + unused == 6 * 128 * 1024


def test_wide_iq16_profile_still_fits_the_arena():
    tx, loops, rx = 3, 12, 4
    wide_bytes = tx * loops * rx * 24 * 53 * 4
    assert wide_bytes == 732_672
    assert wide_bytes <= 786_432


MSS_CFG = FIRMWARE.parent / "mss.cfg"


def _rearm_task(source: str) -> str:
    return _function_source(
        source, "static void l3_hwaRearmTask(UArg arg0, UArg arg1)\n{", "static void l3_fill_header"
    )


def test_rearm_latency_is_timed_from_the_queue_to_the_next_hwa_arm():
    """rearm_*_us must cover EDMA waits and the restart, the part that has to beat the frame gap."""
    source = FIRMWARE.read_text(encoding="utf-8")
    queue = _function_source(
        source, "static void l3_hwaMaybeQueueRearm(void)\n{", "static void l3_hwaChainDoneCB"
    )
    task = _rearm_task(source)

    stamp = queue.index("gHwaRearmQueuedCycles = Cycleprofiler_getTimeStamp();")
    assert queue.index("key = Hwi_disable();") < stamp < queue.index("Hwi_restore(key);")
    assert "gHwaRearmQueuedValid = 1U;" in queue

    restart = task.index("errCode = l3_restartCompletedHwaFrame();")
    consume = task.index("gHwaRearmQueuedValid = 0U;")
    assert consume < restart
    assert task.rindex("Hwi_disable()", 0, consume) > task.rindex("Hwi_restore(key)", 0, consume)
    assert task.index("gHwaRearmLastUs = ", restart) > restart
    assert "if (timed)" in task


def test_rearm_latency_never_reuses_or_invents_a_start_time():
    source = FIRMWARE.read_text(encoding="utf-8")
    task = _rearm_task(source)

    assert "== 0U) {\n                rearmStartCycles = Cycleprofiler_getTimeStamp()" not in task
    assert task.count("Cycleprofiler_getTimeStamp()") == 1


def test_stats_report_rearm_latency_on_their_own_line():
    source = FIRMWARE.read_text(encoding="utf-8")
    stats = _function_source(
        source, "static int32_t l3_cli_stats", "static int32_t l3_cli_hwaStats"
    )

    line = stats.index('CLI_write("rearm_last_us=%u rearm_max_us=%u rearm_timed=%u\\n"')
    assert stats.index("trig phase=") < line < stats.index("return 0")


def test_sensor_start_resets_rearm_latency_and_the_counter_runs():
    source = FIRMWARE.read_text(encoding="utf-8")
    start = _function_source(
        source,
        "static int32_t l3_cli_sensorStart(int32_t argc, char *argv[])\n{",
        "static int32_t l3_cli_sensorStop(int32_t argc, char *argv[])\n{",
    )
    init = _function_source(
        source, "static void l3_initTask(UArg arg0, UArg arg1)\n{", "int32_t main(void)"
    )

    for name in ("gHwaRearmLastUs", "gHwaRearmMaxUs", "gHwaRearmTimed", "gHwaRearmQueuedValid"):
        assert re.search(rf"\b{name}\s*=\s*0U;", start), name
    assert "#include <ti/utils/cycleprofiler/cycle_profiler.h>" in source
    assert init.index("Cycleprofiler_init();") < init.index("UART_init();")
    assert "xdc.useModule('ti.sysbios.family.arm.v7a.Pmu')" in MSS_CFG.read_text(encoding="utf-8")


def test_capture_and_detection_preempt_polled_cli_output():
    """Health polling must not block scratch consumption or HWA rearming.

    With detection below the CLI, every ~825 B ``stats`` reply (~7.9 ms of
    polled UART at 1,041,667 baud, ~3 frames at 3 ms) left the detect task
    unable to run and cost ~3 stale frames (swing_trigger.py, 2026-09-29).
    Do not move detection back below the CLI to fix an l3dump readback
    failure: the 18-byte readback once blamed on this ordering was
    l3_cli_dump requesting a new freeze instead of awaiting the latched one
    (see test_dump_waits_on_an_already_latched_freeze_instead_of_requesting_a_new_one).
    """
    source = FIRMWARE.read_text()
    assert "#define L3_DETECT_TASK_PRIORITY (L3_CLI_TASK_PRIORITY + 1U)" in source
    assert "#define L3_HWA_REARM_TASK_PRIORITY (L3_DETECT_TASK_PRIORITY + 1U)" in source
    assert "#define L3_CTRL_TASK_PRIORITY (L3_HWA_REARM_TASK_PRIORITY + 1U)" in source
    assert "#define L3_NOTICE_TASK_PRIORITY L3_CLI_TASK_PRIORITY" in source


def test_sensor_start_resets_performance_counters_before_capture():
    source = FIRMWARE.read_text()
    start = source[source.rindex("static int32_t l3_cli_sensorStart(") :]
    assert start.index("l3_profile_reset(&gProfile)") < start.index("gHwaFrameDone      = 0U")
