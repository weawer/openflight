"""Source checks for the l3sparse and self-trigger firmware paths.

l3_dump.c cannot build here (it needs the mmWave SDK); these pin the
properties the host relies on. The detector itself is built and exercised
in test_iwr6843_firmware_trigger.py. scripts/hardware-test/test_iwr_firmware.py
runs the readback section against a board.
"""

from __future__ import annotations

import re
from pathlib import Path

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"


def _source() -> str:
    return FIRMWARE.read_text(encoding="utf-8")


def _function(name: str) -> str:
    source = _source()
    start = source.rindex(name)  # the definition follows any forward declaration
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated {name}")


def test_sparse_request_buffer_uses_the_shared_limit():
    sparse = _function("int32_t l3_cli_sparse(")

    assert "char request[L3_SPARSE_REQUEST_MAX];" in sparse
    assert "request[768]" not in sparse


def test_oversized_request_is_an_error_before_any_slice_bytes():
    sparse = _function("int32_t l3_cli_sparse(")

    overflow = sparse.index("L3_READLINE_OVERFLOW")
    error = sparse.index('CLI_write("Error: sparse cell request longer', overflow)
    assert error < sparse.index('"ILS1"')


def test_read_line_drains_an_overlong_line_instead_of_stopping_mid_line():
    read_line = _function("static int32_t l3_readLine(")

    assert "while (used + 1U < cap" not in read_line
    assert "while (drained < L3_READLINE_DRAIN_MAX)" in read_line
    assert "status = L3_READLINE_OVERFLOW;" in read_line
    assert "L3_SPARSE_REQUEST_TIMEOUT_MS" in read_line


def test_latched_self_trigger_stops_the_front_end_before_rearm():
    """HWA freeze leaves the BSS chirping. MMWave_start then returns
    'RF restart failed' unless this wait stops the front end first."""
    wait = _function("static int32_t l3_awaitFrozenRing(")
    latched = wait.index("if (gSelfTriggerLatched)")
    timeout_return = wait.index("return -1;", wait.index("self-trigger freeze timed out", latched))
    stop = wait.index("return l3_finishCaptureStop();", timeout_return)
    boundary = wait.index("l3_stopCaptureAtBoundary()", stop)

    assert timeout_return < stop < boundary


def test_release_rearms_without_reading_a_cell_line():
    """A second CLI line cannot sit in the one-byte SCI receiver during the power dump."""
    release = _function("static int32_t l3_cli_release(")

    assert "l3_readLine" not in release
    assert "l3_awaitFrozenRing()" in release
    assert release.index("l3_awaitFrozenRing()") < release.index("l3_sparseRearm()")
    assert 'tableEntry[16].cmd           = "l3release"' in _source()


def test_blank_line_before_the_cell_request_is_not_a_missing_request():
    """A stray CR/LF left in the FIFO must not reject the real cells line."""
    sparse = _function("int32_t l3_cli_sparse(")
    read_line = _function("static int32_t l3_readLine(")

    assert "L3_READLINE_EMPTY" in read_line
    retry = sparse.index("lineStatus == L3_READLINE_EMPTY")
    missing = sparse.index('CLI_write("Error: sparse cell request missing')
    assert retry < missing
    assert sparse.count("l3_readLine(request") == 2


def test_slice_count_is_the_number_of_cells_actually_parsed():
    sparse = _function("int32_t l3_cli_sparse(")

    parsed = sparse.index("cellCount++")
    header = sparse.index('"ILS1"')
    count = sparse.index("l3_writeU16((uint16_t)cellCount);")
    assert parsed < header < count


def test_self_trigger_reads_a_finished_slot_beside_capture():
    """Detection runs after the slot is stored, not inside the HWA rearm task."""
    rearm = _function("static void l3_hwaRearmTask")
    detect = _function("static void l3_detectTask")
    done = _function("static void l3_hwaOutputDoneCB")
    packed = _function("static void l3_iq8EdmaDoneCB")
    stats = _function("static int32_t l3_cli_stats")
    consider = _function("static void l3_considerSelfTrigger(")

    assert "l3_considerSelfTrigger" not in rearm
    assert "l3_considerSelfTrigger(queuedSlot)" in detect
    assert "l3detect_slot_live" in detect
    assert "l3_publishDetectFrame" in done
    assert "l3_publishDetectFrame" in packed
    assert (
        'CLI_write("detect dropped=%u stale=%u notice_dropped=%u shed=%u stale_read=%u\\n"' in stats
    )
    assert "gPreFramesCaptured < gCapturePlan.preFrames" in consider


def test_trigger_scores_every_loop_not_just_loop_zero():
    """The detector's observation integrates the MTI residual over all loops."""
    source = _source()
    consider = _function("static void l3_considerSelfTrigger(")

    assert "l3_verticalPowerAt" not in source
    assert "perLoop[0]" not in source
    # Every bin is scored by l3_scoreSpans (all loops), once a frame, on the
    # core the frame is routed to; the MSS's scorer is the full residual. The
    # trigger observes its region of those scores.
    assert "l3_verticalResidual((const l3_detect_frame_t *)ctx, localBin, NULL, out);" in (
        _function("static int32_t l3_mssScorer(")
    )
    assert "plan[0] = region;" in consider
    assert "l3_scoreSpans(&frame, plan, 4U, obs);" in consider
    assert "l3_trig_observe(&gTrig, gPreFramesCaptured, teeBin, region.first," in consider


def test_trigger_no_longer_gates_on_the_tee_bin_or_a_toward_away_sequence():
    consider = _function("static void l3_considerSelfTrigger(")

    for retired in ("gTriggerPower", "gTriggerToward", "gTriggerAway", "gTriggerReady"):
        assert retired not in _source(), retired
    assert "l3_trig_region(&gTrigCfg" in consider


def test_trigger_freeze_request_is_unchanged_by_the_new_detector():
    consider = _function("static void l3_considerSelfTrigger(")
    freeze = consider[consider.index("key = Hwi_disable();") : consider.index("Hwi_restore(key);")]

    for line in (
        "gHwaFreezeRequested = 1U;",
        "gPostCaptureStarted = 0U;",
        "gPostFramesCaptured = 0U;",
        "gPostFramesObserved = 0U;",
        "gActiveFrameShouldKeep = 1U;",
        "gSelfTriggerLatched = 1U;",
        "gHwaFreezeRequests++;",
    ):
        assert line in freeze, line
    assert 'l3_queueNotice("Triggered\\n");' in consider


def test_trigger_config_waits_for_a_frame_in_progress_before_resetting():
    cfg = _function("static int32_t l3_cli_triggerCfg(")

    assert (
        cfg.index("gTriggerEnabled = 0U;")
        < cfg.index("while (gTrigBusy && waited")
        < cfg.index("l3_trig_init(&gTrig")
    )
    consider = _function("static void l3_considerSelfTrigger(")
    # The stale-frame bail-out drops the busy flag early; the normal path
    # drops it after the update.
    assert (
        consider.index("gTrigBusy = 1U;")
        < consider.index("l3_trig_observe(")
        < consider.rindex("gTrigBusy = 0U;")
    )


def test_trigger_config_disables_on_zero_and_checks_the_rest():
    cfg = _function("static int32_t l3_cli_triggerCfg(")

    assert "if (on != 0UL && l3_trig_cfg_check(&cfg) != 0)" in cfg
    assert "gTriggerEnabled = (on != 0UL) ? 1U : 0U;" in cfg


def test_trigger_config_refuses_the_removed_gate_thresholds():
    """[approach past stat] only: a line carrying the range gate's minCoh,
    minStep, minSpeed or minApproach is refused, not half applied."""
    cfg = _function("static int32_t l3_cli_triggerCfg(")

    assert "argc < 4 || argc > 7" in cfg
    assert "cfg.pastBins = (uint32_t)value;" in cfg
    assert "cfg.stat = (uint32_t)value;" in cfg
    for gone in ("minCoherence", "minStepBins", "minSpeedMps", "minApproachBins", "trackFrames"):
        assert gone not in cfg


def test_every_ring_rearm_resets_the_club_track_and_keeps_the_floor():
    source = _source()

    assert source.count("    gPreFramesCaptured = 0U;\n    l3_trigRearm();\n") == 3
    rearm = _function("static void l3_trigRearm(")
    assert "l3_track_reset(&gClubTrack);" in rearm
    assert "&gTrig" not in rearm, "the floor and the trace outlive a shot"


def test_the_range_gate_is_gone_from_the_board():
    source = _source()

    for gone in (
        "l3_trig_update(",
        "l3_trig_rearm(",
        "l3_trig_log_",
        "l3_trig_format_record(",
        "L3_TRIG_STATE_",
        "gFireMode",
        "gImpactArmed",
        "L3_SHOT_FIRE_",
        "l3_cli_trackCfgFire",
        "gateFired",
    ):
        assert gone not in source, gone


def test_trigger_log_command_is_registered_and_ends_with_done():
    source = _source()
    log = _function("static int32_t l3_cli_triggerLog(")

    assert 'cliCfg.tableEntry[17].cmd           = "triggerLog";' in source
    assert "l3_trig_format_summary(&gTrig" in log
    assert "l3_trig_format_config(&gTrig" in log
    assert log.rindex('CLI_write("Done\\n");') > log.rindex("l3_trig_format_config(")


def test_detect_task_never_writes_the_cli_uart_itself():
    """A host command mid-line would let the CLI task splice its reply into the notice."""
    source = _source()
    consider = _function("static void l3_considerSelfTrigger(")
    note = _function("static void l3_noteTrigger(")

    assert "CLI_write" not in consider
    assert "CLI_write" not in note
    assert 'l3_queueNotice("Triggered\\n");' in consider
    assert consider.index('l3_queueNotice("Triggered') < consider.index(
        "l3_noteTrigger(9U, gTrig.floor)"
    )
    assert "l3_queueNotice(line);" in note
    assert "#define L3_NOTICE_TASK_PRIORITY L3_CLI_TASK_PRIORITY" in source
    assert "Task_create(l3_noticeTask, &taskParams, NULL);" in source


def test_debug_cfg_answers_with_the_current_line_before_done():
    handler = _function("static int32_t l3_cli_debugCfg(")

    assert handler.index("l3_formatTriggerDebug(gTriggerPhase") < handler.index('CLI_write("Done')
    assert "l3_queueNotice" not in handler


def test_notice_queue_writes_whole_lines_from_one_task():
    task = _function("static void l3_noticeTask(")
    queue = _function("static void l3_queueNotice(")

    assert 'CLI_write("%s", gNoticeLines[gNoticeHead]);' in task
    assert "gNoticeDropped++;" in queue
    assert "Hwi_disable()" in queue and "Semaphore_post(gNoticeSemaphore);" in queue


def test_overlong_request_drain_outlasts_a_four_times_oversized_request():
    source = _source()
    read_line = _function("static int32_t l3_readLine(")

    assert "#define L3_READLINE_DRAIN_MAX (64U * L3_SPARSE_REQUEST_MAX)" in source
    assert "while (drained < L3_READLINE_DRAIN_MAX)" in read_line
    assert "4U * cap" not in read_line


def _channels(signature: str) -> str:
    """A function of l3_channels.c, the per-channel loops the board calls
    (host tested in test_iwr6843_firmware_channels.py)."""
    source = (Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_channels.c").read_text(
        encoding="utf-8"
    )
    body = source[source.index(signature) :]
    return body[: body.index("\n}\n")]


def test_trigger_log_serves_the_raw_input_trace_and_its_clear():
    """A missed swing must be readable: trace and clear ride the existing command."""
    source = _source()
    log = _function("static int32_t l3_cli_triggerLog(")
    trace = _function("static void l3_writeTriggerTrace(")

    assert 'strcmp(argv[1], "trace") == 0' in log and "l3_writeTriggerTrace(line" in log
    assert 'strcmp(argv[1], "clear") == 0' in log and "l3_trig_trace_clear(&gTrig);" in log
    assert "l3_trig_format_trace_header(&gTrig" in trace
    assert "l3_trig_format_maxhold(&gTrig, index, 8U" in trace
    assert "l3_trig_format_trace(&entry" in trace
    assert "tableEntry[19]" not in source, "the CLI table is at the SDK's command limit"
    assert "l3_channels_residual(&frame, localBin, perLoop, obs);" in _function(
        "static void l3_verticalResidual("
    )
    assert "obs->loop0 = loopPower[0];" in _channels("void l3_channels_residual(")


def test_sensor_stop_takes_a_self_trigger_freeze_instead_of_closing_over_it():
    """A fire nobody read back left the BSS chirping; closing then wedged the CLI."""
    stop = _function("static int32_t l3_cli_sensorStop(")

    assert stop.index("if (gSelfTriggerLatched) {") < stop.index("else if (gCaptureActive)")
    assert "status = l3_awaitFrozenRing();" in stop
    assert stop.index("l3_awaitFrozenRing") < stop.index("MMWave_close")


def test_tee_scan_reports_static_power_the_trigger_never_sees():
    """A stationary ball is exactly what MTI removes; ball scan reads it back raw."""
    source = _source()
    assert "l3_channels_static_power(&frame, localBin, loopStep);" in _function(
        "static float l3_verticalStaticPower("
    )
    static = _channels("float l3_channels_static_power(")
    scan = _function("static int32_t l3_ballScan(")

    assert "meanIm" not in static and "meanRe" not in static, "no mean subtraction: static power"
    assert "total += im * im + re * re;" in static
    assert "return (samples > 0U) ? (total / (float)samples) : 0.0F;" in static
    assert (
        scan.index("l3_sparseFreeze()")
        < scan.index("l3_verticalStaticPower(")
        < scan.index("return l3_sparseRearm();")
    )
    assert "if (window.slots[frame] >= gCapturePlan.preFrames)" in scan, "pre frames only"
    assert 'CLI_write("teescan frames=%u loops=%u first=%u count=%u start=%u\\n"' in scan
    assert 'CLI_write("bin=%u power=%u\\n"' in scan
    assert 'cliCfg.tableEntry[18].cmd           = "ball";' in source
    assert "tableEntry[19]" not in source


def test_detector_source_is_built_into_the_firmware():
    makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    sources = re.search(r"^SOURCES\s*=(.*)$", makefile, re.MULTILINE).group(1).split()

    for unit in ("l3_text.c", "l3_observation.c", "l3_trigger.c", "l3_ball.c", "l3_club_track.c"):
        assert unit in sources, unit
    assert "live_selector.c" in sources
    assert "track_select.c" in sources
    assert '#include "l3_trigger.h"' in _source()
    assert '#include "l3_club_track.h"' in _source()


def test_club_track_rides_the_trigger_pass_and_prints_from_trigger_log():
    """The observations the trigger scores are extracted once as ranked
    targets and fed to the persistent club track; nothing is recomputed while
    the tee band is off (with it, l3_preImpactClubTargets reads the window:
    test_iwr6843_firmware_board_wiring)."""
    source = _source()
    consider = _function("static void l3_considerSelfTrigger(")

    update = consider.index("l3_trig_observe(&gTrig,")
    extract = consider.index("l3_preImpactClubTargets(obs, frame.binStart, &club, &leave,")
    track = consider.index("l3_track_update(&gClubTrack, targets, found, gPreFramesCaptured,")
    assert update < extract < track
    assert consider.count("l3_verticalResidual(") == 0, "bins are scored once, by l3_scoreSpans"
    assert "gTrig.floor," in consider[extract:track], "targets use the trigger's floor"
    assert "gClubTrackDest = teeBin;" in consider
    assert consider.rindex("gTrigBusy = 0U;") > track, "the track update is inside the busy window"

    assert "l3_track_reset(&gClubTrack);" in _function("static void l3_trigRearm(")
    configure = _function("static void l3_clubTrackConfigure(")
    assert "cfg.binWidthM = (float)gTrackRangeResM;" in configure
    assert "cfg.velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * gTrigLoopPeriodS);" in (
        configure
    )
    assert "l3_clubTrackConfigure();" in _function("static int32_t l3_cli_triggerCfg(")

    log = _function("static int32_t l3_cli_triggerLog(")
    assert 'strcmp(argv[1], "track") == 0' in log
    assert "l3_track_format_status(&gClubTrack, gClubTrackDest, line, sizeof(line));" in log
    assert "l3_track_format_point(&point, gClubTrackDest, line, sizeof(line));" in log
    assert (
        'CLI_write("Error: triggerLog [trace|track|shot|result|perf|timing|frames|cal|clear]\\n");'
        in log
    )
    assert (
        "triggerLog [trace|track|shot|result|perf|timing|frames|cal|clear]: floor, trace, club, "
        "shot, result, perf, detect timing, stored frames, calibration" in source
    )


def test_loop_period_for_doppler_comes_from_the_accepted_profile():
    source = _source()

    assert "gTrigLoopPeriodS = (float)(profCfg.idleTimeConst + profCfg.rampEndTime)" in source
    assert "params.loopPeriodS = gTrigLoopPeriodS;" in _function(
        "static void l3_considerSelfTrigger("
    )


def test_loop_means_are_computed_once_per_bin():
    residual = _channels("void l3_channels_residual(")

    # One pass accumulates the mean, a second applies it: two loop-index
    # loops per (tx, rx) over the channel's copy, never a mean loop nested
    # inside the output loop. The other two are the per-loop init and the
    # final peak/copy pass.
    assert "meanLoop" not in residual
    assert len(re.findall(r"for \(loop = 0U; loop < frame->loops; loop\+\+\)", residual)) == 4
    # IQ8 rings hold int8 pairs times a per-frame scale; both read alike,
    # through the frame that says where the samples are.
    assert "float im = (xIm[loop] - meanIm) * frame->scale;" in residual
    # The sparse rows and the trigger share that one pass.
    assert "l3_verticalResidual(&frame, localBin, out, NULL);" in _function(
        "static void l3_verticalPowerLoops("
    )


def test_residual_walks_loops_by_stride_instead_of_recomputing_indices():
    residual = _channels("void l3_channels_residual(")
    read = _channels("static void l3_channels_read(")

    assert "l3_iq16Sample" not in residual
    assert "return frame->ntx * frame->nrx * frame->binCount * 2U * frame->cb;" in _channels(
        "static uint32_t l3_channels_loop_stride("
    )
    # One strided walk a channel (the frame is uncached L3 on the R4F): the
    # mean and the residual both come from the copy it makes.
    assert "l3_channels_read(" in residual and "sample += loopStride;" not in residual
    assert read.count("sample += loopStride;") == 1
    # Energy, strongest loop and the Doppler autocorrelation come from the
    # same pass; no second walk over the samples.
    for field in ("obs->energy = energy;", "obs->peak = peak;", "obs->r1Re = r1Re;"):
        assert field in residual, field


def test_power_rows_go_out_in_one_write_per_loop():
    sparse = _function("int32_t l3_cli_sparse(")
    power = sparse[sparse.index("powerRow[loop * maxBins + bin]") : sparse.index("lineStatus =")]

    assert "l3_writeF32" not in power
    assert "UART_writePolling(gDataUart, (uint8_t *)&powerRow[loop * maxBins]" in power


def test_read_line_uses_the_buffered_uart_receive_not_register_polling():
    """The SCI receiver holds one byte. Polling SCIRD from the CLI task loses a
    byte whenever the HWA rearm task (now above the CLI) preempts the poll, and
    a 700-byte cell line spans several frames. On the Pi every l3sparse cell
    request came back "missing" or truncated. UART_read moves the byte capture
    into the driver's RX interrupt; echo must be off so that ISR does not spin
    on TX between bytes."""
    read_line = _function("static int32_t l3_readLine(")
    source = _source()

    assert "UART_read(gCliUart" in read_line
    assert "SCIRD" not in read_line
    assert "SCIFLR" not in read_line
    assert "Task_sleep" not in read_line
    assert "readTimeout = L3_SPARSE_REQUEST_TIMEOUT_MS" in read_line
    init = " ".join(source.split())  # the open block aligns its '=' with spaces
    assert "uartParams.readEcho = UART_ECHO_OFF;" in init
    assert init.index("uartParams.readEcho = UART_ECHO_OFF;") < init.index("gCliUart = UART_open(0")


def test_geometry_sources_are_built_and_included():
    makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    sources = re.search(r"^SOURCES\s*=(.*)$", makefile, re.MULTILINE).group(1).split()

    for unit in ("l3_frames.c", "l3_angle.c", "l3_impact.c"):
        assert unit in sources, unit
    source = _source()
    assert '#include "l3_angle.h"' in source
    assert '#include "l3_impact.h"' in source
    assert "#include <math.h>" in source


def test_angles_are_queued_for_the_associated_target_only():
    """One channel snapshot per frame, for the target the club track appended,
    with the track's range-rate resolving the TDM alias, queued by the point's
    timestamp for the angle task (l3_angle_queue.h): no estimate on the
    decision path, and never shed when behind."""
    consider = _function("static void l3_considerSelfTrigger(")

    assert "gClubTrack.lastTargetIndex < found &&" in consider
    assert "gClubTrack.count > 1U &&" in consider
    assert "gClubTrack.count > 1U && !gDetectBehind" not in consider
    assert "const l3_target_obs_t *hit = &targets[gClubTrack.lastTargetIndex];" in consider
    assert (
        "l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart,\n"
        "                               hit->dopplerPhaseRad, newest.radialVelocityMps, &snapshot);"
    ) in consider
    assert "l3_angleQueuePush(newest.timestampUs, &snapshot);" in consider
    assert "l3_angle_estimate(" not in consider
    assert consider.index("l3_track_update(&gClubTrack") < consider.index("l3_channelSnapshot(")


def test_channel_snapshot_sums_loops_coherently_with_the_lag1_phase_unwound():
    assert "l3_channels_snapshot(&frame, localBin, lag1PhaseRad, radialVelocityMps, out);" in (
        _function("static void l3_channelSnapshot(")
    )
    snapshot = _channels("void l3_channels_snapshot(")
    begin = _channels("static void l3_channels_snapshot_begin(")

    assert "stepIm = -sinf(lag1PhaseRad);" in snapshot
    assert "sumRe += re * rotRe - im * rotIm;" in snapshot
    assert "sumIm += re * rotIm + im * rotRe;" in snapshot
    assert "l3_angle_snapshot_init(out, frame->ntx, frame->nrx);" in begin
    assert "out->chirpPeriodS = frame->loopPeriodS / (float)frame->ntx;" in begin
    assert "frame.loopPeriodS = gTrigLoopPeriodS;" in _function(
        "static l3_channel_frame_t l3_channelFrame("
    )
    # Every TX, unlike the vertical residual: TX1 carries the azimuth.
    assert "l3_channels_vertical(" not in snapshot
    # Each channel read once (l3_channels_read), then mean and unwind.
    assert "l3_channels_read(" in snapshot and "sample += loopStride;" not in snapshot


def test_the_ball_position_and_delivery_are_kept_every_frame_for_the_shot():
    consider = _function("static void l3_considerSelfTrigger(")

    assert "(void)l3_track_delivery(&gClubTrack, 8U, &gDelivery);" in consider
    assert "if (gTrigDestBall && gBallAngleValid) {" in consider
    assert "gBallAngle.azimuthValid ? gBallAngle.azimuthRad : 0.0F," in consider
    assert (
        "l3_frames_observe(&gRadarCal, (float)teeBin * gClubTrack.cfg.binWidthM, 0.0F, 0.0F,\n"
        "                              &gBallPosition);"
    ) in consider, "boresight when the ball has no measured direction"
    configure = _function("static void l3_clubTrackConfigure(")
    assert "cfg.cal = gRadarCal;" in configure


def test_the_geometric_detector_is_gone_from_the_board():
    """Removed on 2026-09-30: the kiosk never armed it and it never fired on
    the recorded swings. The range-only impact is the self-trigger."""
    source = _source()
    for gone in (
        "l3_impact_update(",
        "gImpact,",
        "gImpact)",
        "gImpact.",
        "gGeometryArmed",
        "geometricFired",
        "l3_shot_fire_sources",
        "gTrigFireSource",
    ):
        assert gone not in source, gone


def test_calibration_and_impact_are_configured_through_track_cfg_sub_modes():
    source = _source()
    track_cfg = _function("static int32_t l3_cli_trackCfg(")

    for mode, handler in (("cal", "Cal"), ("elem", "Elem"), ("impact", "Impact")):
        assert f'strcmp(argv[1], "{mode}") == 0' in track_cfg
        assert f"return l3_cli_trackCfg{handler}(argc, argv);" in track_cfg
    cal = _function("static int32_t l3_cli_trackCfgCal(")
    assert "gRadarCal.radarPitchRad = values[0] * (L3_FRAMES_PI / 180.0F);" in cal
    assert "gRadarCal.rangeBiasM = values[5];" in cal
    elem = _function("static int32_t l3_cli_trackCfgElem(")
    assert "l3_cal_set_element(&gRadarCal, index, values[2], values[1])" in elem
    assert "values[0] >= (float)L3_CAL_MAX_VIRTUAL" in elem
    impact = _function("static int32_t l3_cli_trackCfgImpact(")
    # The horizon, then an optional approach-end distance and end speed: the
    # old five-value line of the geometric detector is refused.
    assert "uint32_t count = (argc >= 4) ? (uint32_t)(argc - 2) : 1U;" in impact
    assert "count > 3U || l3_parseFloats(argc, argv, 2, count, values) != 0" in impact
    assert "values[1] > L3_IMPACT_END_MAX_M" in impact
    assert "values[2] > L3_IMPACT_END_MAX_MPS" in impact
    assert "gImpactCfg.horizonS = values[0];" in impact
    assert "gImpactCfg.endM = values[1];" in impact
    assert "gImpactCfg.endMinMps = values[2];" in impact
    assert "l3_impact_init(&gRangeImpact, &gImpactCfg);" in impact
    assert "tableEntry[19]" not in source, "sub-modes, not new commands"
    assert "or cal/elem/impact/impactFit ..." in source


def test_trigger_log_track_prints_delivery_angle_and_impact_lines():
    log = _function("static int32_t l3_cli_triggerLog(")

    assert "l3_track_format_delivery(&gDelivery, line, sizeof(line));" in log
    assert "l3_angle_format(&gLastAngle, line, sizeof(line));" in log
    assert "l3_impact_format(&gRangeImpact, line, sizeof(line));" in log
    assert 'CLI_write("range %s\\n", line);' in log
    track = log[log.index('strcmp(argv[1], "track") == 0') :]
    assert (
        track.index("l3_track_format_status(")
        < track.index("l3_track_format_delivery(")
        < track.index("l3_track_format_point(")
    )


def test_kept_post_frames_reach_the_detect_task_for_the_ball_tracker():
    """Post-impact slots were never published; now every kept IQ16 post frame
    is, under an epoch that marks it as never stale, and routed to the ball
    tracker instead of the trigger."""
    source = _source()
    done = _function("static void l3_hwaOutputDoneCB(")
    task = _function("static void l3_detectTask(")

    assert "#define L3_DETECT_POST_EPOCH 0xFFFFFFFFU" in source
    assert "uint32_t completedPostSlot = gCapturePlan.preFrames + gPostFramesCaptured;" in done
    assert "l3_publishDetectFrame(completedPostSlot, L3_DETECT_POST_EPOCH);" in done
    assert done.index("gPostFramesCaptured++;") < done.index(
        "l3_publishDetectFrame(completedPostSlot"
    )
    assert "if (!l3_captureUsesIq8())  /* IQ8 frames publish after packing */" in done
    # IQ8: a kept post frame is packed first, then published with the post epoch.
    assert "gIq8PendingDetect = 1U;\n            gIq8PendingEpoch = L3_DETECT_POST_EPOCH;" in done
    rearm = _function("static void l3_hwaRearmTask(")
    assert "pendingSlot < gCapturePlan.preFrames" not in rearm, "post slots publish too"
    assert "if (epoch == L3_DETECT_POST_EPOCH && queuedSlot >= gCapturePlan.preFrames) {" in task
    assert "l3_considerBallTrack(queuedSlot);" in task
    assert task.index("l3_considerBallTrack(") < task.index("l3detect_slot_live(")
    for unit in ("l3_shot.c", "l3_ball_track.c"):
        makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
        assert unit in makefile, unit
    assert '#include "l3_shot.h"' in source and '#include "l3_ball_track.h"' in source


def test_ball_tracker_runs_the_whole_post_window_against_the_trigger_floor():
    consider = _function("static void l3_considerBallTrack(")

    assert "if (!gBallTrack.armed || gCapturePlan.loops == 0U)" in consider
    assert "gPostTimestampUs += gFrameDeltaUs[slot];" in consider
    assert "frameIndex = gPreFramesCaptured + gPostFramesScored;" in consider
    assert "l3_scoreSpans(&frame, &whole, 1U, obs);" in consider, "band off: the whole window"
    assert "gBallFloor, targets, L3_OBS_MAX_TARGETS);" in consider
    assert (
        "l3_obs_floor_update(&gBallFloor, gTrigCfg.stat, obs, count, L3_TRIG_FLOOR_SHIFT);"
        in consider
    )
    assert "params.snr = (gBallSnr > 0.0F) ? gBallSnr : gBallTrackCfg.snr;" in consider, (
        "the ball is a weaker return, with its own snr"
    )
    assert "gBallFloor = 0.0F;" in _function("static void l3_trigRearm(")
    assert (
        "l3_ball_track_update_joint(&gBallTrack, targets, found, frameIndex, gPostTimestampUs,"
        in consider
    )
    assert "gBallTrack.core.count > 1U" in consider, "angles once the flight has a range rate"
    assert "const l3_target_obs_t *hit = &targets[gBallTrack.lastTargetIndex];" in consider
    assert "l3_ball_track_set_angles(&gBallTrack, angle.azimuthRad," in consider
    assert "angle.elevationRad, flags, angle.confidence);" in " ".join(consider.split())
    assert "(void)l3_ball_track_launch(&gBallTrack, &gLaunch);" in consider
    assert "in.postFrame = 1U;" in consider
    assert "in.ballTrackDone = gBallTrack.done;" in consider
    assert "if (l3_shot_update(&gShot, &in, frameIndex) == L3_SHOT_SOLVE) {" in consider
    assert "in.solved = 1U;" in consider


def test_shot_machine_sees_every_pre_frame_and_arms_the_ball_tracker_at_impact():
    consider = _function("static void l3_considerSelfTrigger(")
    observe = _function("static void l3_shotObserve(")

    assert "l3_shotObserve(teeBin, fired, impactUs);" in consider
    assert consider.index("l3_shotObserve(") < consider.index("if (!fired) {")
    assert "in.ballLocked = gTrigDestBall;" in observe
    assert "in.clubActive = gClubTrack.active;" in observe
    assert "in.impactTimestampUs = fired ? impactUs : frameUs;" in observe
    assert "gShot.impactFrame == gPreFramesCaptured" in observe
    assert "l3_ball_track_anchor(&gBallTrack, (float)teeBin, l3_ballArmBin(teeBin)," in observe
    assert "l3_ball_track_arm(&gBallTrack, &anchor, &gBallPosition);" in observe
    rearm = _function("static void l3_trigRearm(")
    assert "l3_shot_rearm(&gShot);" in rearm and "l3_ball_track_reset(&gBallTrack);" in rearm
    assert "gPostTimestampUs = 0U;" in rearm and "gPostFramesScored = 0U;" in rearm
    configure = _function("static void l3_clubTrackConfigure(")
    assert "gBallTrackCfg.core.cal = gRadarCal;" in configure
    assert "gShotCfg.requireBall = gBallCfg.follow;" in configure
    assert "gShotCfg.ballTrackFrames = gCapturePlan.postFrames;" in configure


def test_trigger_log_shot_prints_the_machine_the_ball_track_and_the_launch():
    log = _function("static int32_t l3_cli_triggerLog(")
    source = _source()

    assert 'strcmp(argv[1], "shot") == 0' in log
    assert "l3_shot_format(&gShot, line, sizeof(line));" in log
    assert "l3_ball_track_format_status(&gBallTrack, line, sizeof(line));" in log
    assert "l3_launch_format(&gLaunch, line, sizeof(line));" in log
    assert "l3_track_point(&gBallTrack.core, index, &point)" in log
    assert (
        'CLI_write("Error: triggerLog [trace|track|shot|result|perf|timing|frames|cal|clear]\\n");'
        in log
    )
    assert (
        "triggerLog [trace|track|shot|result|perf|timing|frames|cal|clear]: floor, trace, club, "
        "shot, result, perf, detect timing, stored frames, calibration" in source
    )


def test_the_result_is_built_once_the_shot_reaches_result_and_printed_with_its_packet():
    source = _source()
    consider = _function("static void l3_considerBallTrack(")
    log = _function("static int32_t l3_cli_triggerLog(")

    assert '#include "l3_result.h"' in source
    assert "l3_result.c" in (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    assert "if (gShot.state == L3_SHOT_RESULT && !gShotResultReady) {" in consider
    assert (
        "l3_result_build(&gShot, &gBallTrack, &gLaunch, &gImpactFit, ++gShotId, gTrigDestBall,\n"
        "                        &gShotResult);" in consider
    )
    assert "gShotResultReady = 1U;" in consider
    assert 'strcmp(argv[1], "result") == 0' in log
    assert "l3_result_format(&gShotResult, line, sizeof(line));" in log
    assert "l3_result_format_metric(&gShotResult, index, line, sizeof(line));" in log
    assert "l3_result_format_hex(&gShotResult, hex, sizeof(hex));" in log
    assert "static char hex[L3_RESULT_PACKET_BYTES * 2U + 1U];" in log
    assert 'CLI_write("packet %s\\n", line);' in log
    assert 'CLI_write("packet+ %s\\n", &hex[L3_RESULT_PACKET_BYTES]);' in log
    assert "gShotResultReady = 0U;" in _function("static void l3_trigRearm(")


def test_every_stage_is_profiled_with_the_cpu_clock_and_printed_by_perf():
    source = _source()
    consider = _function("static void l3_considerSelfTrigger(")
    ball_track = _function("static void l3_considerBallTrack(")
    ball = _function("static void l3_considerBall(")
    stage = _function("static void l3_profileStage(")
    log = _function("static int32_t l3_cli_triggerLog(")

    assert (
        '#include "l3_profile.h"' in source
        and "l3_profile.c" in (FIRMWARE.parent / "makefile").read_text()
    )
    assert "l3_profile_init(&gProfile, gCpuClock / 1000000U);" in stage
    assert "l3_profile_add(&gProfile, stage, Cycleprofiler_getTimeStamp() - startTicks);" in stage
    for name in ("RESIDUAL", "TRIGGER", "EXTRACT", "CLUB_TRACK", "ANGLE", "IMPACT"):
        assert f"l3_profileStage(L3_PROF_{name}, ticks);" in consider, name
    assert consider.count("ticks = Cycleprofiler_getTimeStamp();") == 6
    assert "l3_profile_frame(&gProfile);" in consider
    assert "l3_profileStage(L3_PROF_BALL_DETECT, ticks);" in ball
    assert "l3_profileStage(L3_PROF_BALL_TRACK, ticks);" in ball_track
    assert 'strcmp(argv[1], "perf") == 0' in log
    assert "l3_profile_format_summary(&gProfile, line, sizeof(line));" in log
    assert "l3_profile_format(&gProfile, index, line, sizeof(line));" in log
    assert (
        'CLI_write("Error: triggerLog [trace|track|shot|result|perf|timing|frames|cal|clear]\\n");'
        in log
    )


def test_adaptive_windows_apply_between_shots_from_the_locked_ball():
    source = _source()
    apply = _function("static void l3_applyAdaptiveWindows(")
    rearm = _function("static void l3_trigRearm(")
    capture_cfg = _function("static int32_t l3_cli_captureCfg(")
    adaptive = _function("static int32_t l3_cli_captureCfgAdaptive(")
    log = _function("static int32_t l3_cli_triggerLog(")

    assert (
        '#include "l3_adaptive.h"' in source
        and "l3_adaptive.c" in (FIRMWARE.parent / "makefile").read_text()
    )
    assert (
        "if (!gAdaptiveCfg.enabled || gCaptureActive || !l3_ball_locked(&gBall, &ballBin))" in apply
    )
    assert "l3_adaptive_windows(&gAdaptiveCfg, ballBin, N_SAMPLES, gCapturePlan.preBins," in apply
    assert "gCapturePlan.preStart = windows.preStart;" in apply
    assert "gCapturePlan.lateStart = windows.lateStart;" in apply
    assert "if (l3_finalizeCapturePlan(gCapturePlan.loops) == 0) {" in apply
    assert rearm.index("l3_applyAdaptiveWindows();") < rearm.index("l3_track_reset(&gClubTrack);")
    sparse_rearm = _function("static int32_t l3_sparseRearm(")
    assert sparse_rearm.index("l3_trigRearm();") < sparse_rearm.index(
        "l3_restartCompletedHwaFrame()"
    )
    assert 'strcmp(argv[1], "adaptive") == 0' in capture_cfg
    assert "gAdaptiveCfg.enabled = (values[0] != 0U) ? 1U : 0U;" in adaptive
    assert "l3_adaptive_format(&gAdaptiveCfg, &gAdaptiveWindows, line, sizeof(line));" in log


def test_the_locked_ball_gets_its_own_direction_from_the_static_return():
    ball = _function("static void l3_considerBall(")
    assert "l3_channels_snapshot_static(&frame, localBin, out);" in _function(
        "static void l3_channelSnapshotStatic("
    )
    snapshot = _channels("void l3_channels_snapshot_static(")
    status = _function("static int32_t l3_cli_ball(")

    # No Doppler to unwind: the output starts zeroed (lag-1 phase and
    # velocity 0) and the raw samples are summed, no mean removed.
    assert "memset(out, 0, sizeof(*out));" in snapshot
    assert "lag1PhaseRad" not in snapshot and "radialVelocityMps" not in snapshot
    assert "sumIm += l3_channels_component(sample, frame->cb) * frame->scale;" in snapshot
    assert "meanIm" not in snapshot
    assert "l3_channelSnapshotStatic(&frame, ballBin - frame.binStart, &snapshot);" in ball
    assert "gBallAngle.elevationPeakRatio >=" in ball
    assert "L3_BALL_ANGLE_MIN_PEAK_RATIO" in ball
    assert "l3_angle_format(&gBallAngle, line, sizeof(line));" in status
    assert "(unsigned)gBallAngleValid" in status


def test_every_ring_reader_handles_iq8_samples_with_the_frame_scale():
    source = _source()
    for name in (
        "static void l3_verticalResidual(",
        "static float l3_verticalStaticPower(",
        "static void l3_channelSnapshot(",
        "static void l3_channelSnapshotStatic(",
    ):
        body = _function(name)
        assert "l3_channel_frame_t frame = l3_channelFrame(source);" in body, name
        assert "(const int16_t *)&g_ring" not in body, name
    frame = _function("static l3_channel_frame_t l3_channelFrame(")
    assert "frame.base = source->base;" in frame
    assert "frame.cb = source->cb;" in frame and "frame.scale = source->scale;" in frame
    ring = _function("static l3_detect_frame_t l3_ringFrameOf(")
    assert (
        "frame.cb = l3_ringComponentBytes();" in ring
        and "frame.scale = l3_ringScale(slot);" in ring
    )
    scale = _function("static float l3_ringScale(")
    assert "return (float)gFrameIq8Scale[slot];" in scale
    component = _channels("static float l3_channels_component(")
    assert "return (float)*(const int8_t *)component;" in component
    assert source.count("l3_ringComponentBytes()") >= 2


def test_iq8_quantisation_in_the_firmware_is_the_shared_module():
    """The host emulator compiles l3_iq8.c; l3_dump.c must not keep a copy of the arithmetic."""
    source = _source()
    assert '#include "l3_iq8.h"' in source
    for gone in (
        "static int8_t l3_quantizeIq8(",
        "static uint8_t l3_iq8PackShift(",
        "static uint8_t l3_iq8SampledPackShift(",
        "static int8_t l3_quantizeIq8Shift(",
    ):
        assert gone not in source, gone
    pack = _function("static void l3_packIq8CompletedFrame(")
    assert "l3_iq8_pack_shift(source, components, L3_IQ8_SCALE_COMPLEX_STRIDE)" in pack
    assert "l3_iq8_pack_shift(source, components, 1U)" in pack
    assert "l3_iq8_quantize_shift(source[component], packShift, &clippedComponents)" in pack
    assert "gIq8ClippedComponents += clippedComponents;" in pack
    scale = _function("static uint16_t l3_iq8FrameScale(")
    assert "l3_iq8_dump_scale(l3_iq8_max_abs(src, words))" in scale
    write = _function("static void l3_writeCompressedIq8Frame(")
    assert "l3_iq8_quantize_scale(src[word], scale)" in write
    makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    assert "l3_iq8.c" in makefile


def test_sensor_start_forgets_every_trigger_from_the_previous_session():
    """A latch or enable left over from a crashed host would freeze or self-trigger the new one."""
    start = _function("static int32_t l3_cli_sensorStart(")

    assert "gSelfTriggerLatched = 0U;" in start
    assert "gTriggerEnabled = 0U;" in start
    assert "gTriggerPhase = 0U;" in start
    assert "l3_trigRearm();" in start


def test_trigger_cfg_resets_the_front_end_after_the_in_flight_frame():
    trigger_cfg = _function("static int32_t l3_cli_triggerCfg(")

    assert "l3_trig_init(&gTrig" in trigger_cfg
    assert "while (gTrigBusy && waited" in trigger_cfg
