"""Source checks for the compact IQ16 capture formats in firmware/iwr6843/l3_dump.c.

l3_dump.c cannot build here; these pin the integration the host relies on:
compact16 and adaptive16 route the HWA output to the IQ16 scratch, the
detect task reads the wide processing frame from that scratch at full
precision and drops a frame whose scratch was reused, the rearm task
compacts the retained window into L3 through l3_compact_iq16 with the
window l3_retain.c chose, and the plan, stats, descriptors and CLI say so.
The policy itself is tested in test_iwr6843_firmware_retain.py, the copy in
test_iwr6843_compact_iq16.py and the plan arithmetic in
test_iwr6843_capture_plan.py.
"""

from __future__ import annotations

from pathlib import Path

import pytest

FIRMWARE = Path(__file__).parents[1] / "firmware" / "iwr6843" / "l3_dump.c"


def _source() -> str:
    return FIRMWARE.read_text(encoding="utf-8")


def _function(name: str) -> str:
    source = _source()
    start = source.index(name + "\n{") if (name + "\n{") in source else source.rindex(name)
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


def test_the_formats_exist_and_the_cli_accepts_them():
    source = _source()
    assert "#define L3_CAPTURE_FORMAT_COMPACT16  2U" in source
    assert "#define L3_CAPTURE_FORMAT_ADAPTIVE16 3U" in source
    cli = _function("static int32_t l3_cli_captureFormat(")
    assert (
        'strcmp(argv[1], "compact16") == 0' in cli and 'strcmp(argv[1], "adaptive16") == 0' in cli
    )
    assert "gRetainCfg.enabled = (uint8_t)(gCaptureFormat == L3_CAPTURE_FORMAT_ADAPTIVE16);" in cli
    assert "gCapturePlan.retainPreBins = 16U;" in cli, "default retain widths when none were given"
    assert "l3_captureFormatName()" in cli
    assert '"captureFormat iq16|iq8|compact16|adaptive16"' in source
    assert '#include "l3_retain.h"' in source and '#include "compact_iq16.h"' in source
    makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    assert "l3_retain.c" in makefile and "compact_iq16.c" in makefile


def test_scratch_and_compact_helpers_cover_the_right_formats():
    scratch = _function("static uint8_t l3_captureUsesScratch(")
    assert "gCaptureFormat != L3_CAPTURE_FORMAT_IQ16" in scratch
    compact = _function("static uint8_t l3_captureCompactsIq16(")
    assert "L3_CAPTURE_FORMAT_COMPACT16" in compact and "L3_CAPTURE_FORMAT_ADAPTIVE16" in compact
    bytes_per = _function("static uint32_t l3_captureBytesPerComplex(")
    assert "l3_captureUsesIq8()" in bytes_per, "compact formats store IQ16: four bytes per complex"


def test_the_hwa_writes_scratch_and_the_output_done_publishes_the_scratch_frame():
    output = _function("static int32_t l3_configHwaFrameOutput(")
    assert "destination = l3_captureUsesScratch()" in output
    assert "gScratchBusy[gIq8ActiveScratch] = 1U;" in output
    assert "gActiveProcessStart = (uint8_t)binStart;" in output
    assert "gActiveProcessBins = (uint8_t)binCount;" in output
    done = _function("static void l3_hwaOutputDoneCB(")
    assert "if (l3_captureUsesScratch() && gActiveFrameShouldKeep)" in done
    compact_block = done[done.index("if (l3_captureCompactsIq16())") :]
    for line in (
        "gFrameScratch[completedSlot] = gIq8ActiveScratch;",
        "gFrameProcessStart[completedSlot] = gActiveProcessStart;",
        "gFrameProcessBins[completedSlot] = gActiveProcessBins;",
        "gScratchFrame[gIq8ActiveScratch] = gHwaOutputDone;",
        "gScratchBusy[gIq8ActiveScratch] = 0U;",
    ):
        assert line in compact_block, line
    # Only IQ8 defers the detect publish to after the pack; compact frames
    # are published at once, from the scratch.
    assert "if (!l3_captureUsesIq8())  /* IQ8 frames publish after packing */" in done
    assert done.index("if (l3_captureCompactsIq16())") < done.index("if (!l3_captureUsesIq8())")


def test_the_rearm_task_compacts_after_the_hwa_restart_from_the_scratch_it_is_not_writing():
    rearm = _function("static void l3_hwaRearmTask(")
    assert (
        "if (l3_captureUsesScratch()) {\n                key = Hwi_disable();\n                if (gIq8Pending) {"
        in rearm
    )
    assert "} else if (l3_captureCompactsIq16()) {" in rearm
    toggle = rearm.index("} else if (l3_captureCompactsIq16()) {")
    restart = rearm.index("errCode = l3_restartCompletedHwaFrame();")
    store = rearm.index(
        "if (l3_captureUsesScratch() && hadPending) {\n                l3_storeCompletedFrame(pendingSlot, pendingScratch);"
    )
    assert toggle < restart < store, (
        "toggle the active scratch, restart the HWA, then compact the other"
    )
    freeze = rearm.index("if (freezeAfterPack) {")
    assert "l3_storeCompletedFrame(pendingSlot, pendingScratch);" in rearm[freeze:restart]
    queue = _function("static void l3_hwaMaybeQueueRearm(")
    assert queue.count("l3_captureUsesScratch()") == 2 and "l3_captureUsesIq8()" not in queue
    store_fn = _function("static void l3_storeCompletedFrame(")
    assert "l3_compactCompletedFrame(slot, scratch);" in store_fn
    assert "l3_startIq8EdmaPack(slot, scratch)" in store_fn


def test_compaction_uses_the_policy_window_and_records_the_descriptor():
    compact = _function("static void l3_compactCompletedFrame(")
    assert "l3_retainState(slot, &state);" in compact
    assert (
        "l3_retain_window(&gRetainCfg, &state, processStart, processBins, gFrameBinCount[slot],"
        in compact
    )
    assert "l3_compact_iq16(&g_iq16FrameScratch[scratch][0]," in compact
    assert "(uint16_t)(window.start - processStart), window.bins)" in compact
    assert "gFrameBinStart[slot] = window.start;" in compact
    assert "gLastRetain = window;" in compact
    for field in (
        "globalBinStart",
        "binCount",
        "processStart",
        "processBins",
        "shotState",
        "priority",
        "why",
        "isPost",
    ):
        assert f"desc->{field} = " in compact, field
    assert "gCompactMaxUs" in compact and "gCompactErrors++" in compact
    state = _function("static void l3_retainState(")
    assert "state->shotState = gShot.state;" in state
    assert "l3_ball_locked(&gBall, &ballBin)" in state
    assert "l3_retain_predict(gClubTrack.lastBin, gClubTrack.velocityBinsPerFrame)" in state
    assert (
        "l3_retain_predict(gBallTrack.core.lastBin, gBallTrack.core.velocityBinsPerFrame)" in state
    )
    assert "state->postIndex = slot - gCapturePlan.preFrames;" in state


def test_the_detect_task_reads_the_scratch_frame_and_drops_a_stale_one():
    of = _function("static l3_detect_frame_t l3_detectFrameOf(")
    assert "if (l3_captureCompactsIq16() && slot < L3_MAX_CAPTURE_FRAMES &&" in of
    assert "frame.base = (const uint8_t *)&g_iq16FrameScratch[frame.scratch][0];" in of
    assert "frame.binStart = gFrameProcessStart[slot];" in of
    assert "frame.cb = 2U;" in of and "frame.scale = 1.0F;" in of
    assert "return l3_ringFrameOf(slot);" in of
    stale = _function("static int32_t l3_detectFrameStale(")
    assert "gScratchBusy[frame->scratch] || gScratchFrame[frame->scratch] != frame->epoch" in stale
    # The check counts what it finds itself: the scratch reused, or the ring
    # slot reached by the writer during the read.
    assert "gDetectScratchStale++;" in stale and "gDetectStaleAfterRead++;" in stale
    for name, guard in (
        (
            "static void l3_considerSelfTrigger(uint32_t slot)",
            "gTrigBusy = 0U;\n        l3_noteTrigger(1U, 0.0F);\n        return;",
        ),
        # Both of its paths (the band's post spans, the whole window) check.
        ("static void l3_considerBallTrack(uint32_t slot)", "gTrigBusy = 0U;\n            return;"),
        ("static void l3_considerBall(uint32_t slot)", "gBallBusy = 0U;\n        return;"),
    ):
        body = _function(name)
        assert "l3_detect_frame_t frame = l3_detectFrameOf(slot);" in body, name
        assert "gFrameBinStart[slot]" not in body and "gFrameBinCount[slot]" not in body, name
        stale_at = body.index("if (l3_detectFrameStale(&frame)) {")
        assert guard in body[stale_at:], name
        # The observations are computed first (the band paths score the scan
        # plan's spans with l3_scoreSpans), the staleness judged before any decision.
        scored_at = min(
            body.index(call) for call in ("l3_vertical", "l3_scoreSpans(") if call in body
        )
        assert scored_at < stale_at, name
    for reader in (
        "static void l3_verticalResidual(",
        "static float l3_verticalStaticPower(",
        "static void l3_channelSnapshot(",
        "static void l3_channelSnapshotStatic(",
    ):
        body = _function(reader)
        assert "const l3_detect_frame_t *source" in body, reader
        assert "g_ring[" not in body and "gFrameBinCount[" not in body, reader


def test_frozen_ring_readers_stay_on_the_ring():
    loops = _function("static void l3_verticalPowerLoops(")
    assert "l3_ringFrameOf(slot)" in loops and "l3_detectFrameOf" not in loops
    scan = _function("static int32_t l3_ballScan(")
    assert "l3_ringFrameOf(window.slots[frame])" in scan


def test_plan_finalisation_budgets_compact_frames_and_checks_scratch_widths():
    finalize = _function("static int32_t l3_finalizeCapturePlan(")
    assert "gCapturePlan.compact = l3_captureCompactsIq16();" in finalize
    assert "scratch capture windows cannot exceed" in finalize
    assert "l3_retain_budget(&request, &budget)" in finalize
    assert "request.preBins = gCapturePlan.retainPreBins;" in finalize
    assert "gCapturePlan.requestedPreFrames = budget.preFrames;" in finalize
    assert (
        "gCapturePlan.postFrames = (uint8_t)(budget.impactFrames + budget.ballFrames);" in finalize
    )
    assert finalize.index("l3_retain_budget(") < finalize.index("l3plan_build(")
    retain = _function("static int32_t l3_cli_captureCfgRetain(")
    assert (
        'strcmp(argv[1], "retain") == 0' in retain
        and "gCapturePlan.retainImpactBins = values[1];" in retain
    )
    assert "cfg.approachBins = values[0];" in retain and "l3_retain_cfg_check(&cfg)" in retain
    header = _function("static void l3_fill_header(")
    assert (
        "if (gCapturePlan.compact) {" in header
        and "h->n_samples = gCapturePlan.retainPreBins;" in header
    )


def test_stats_and_trigger_log_report_the_compact_state():
    stats = _function("static int32_t l3_cli_stats(")
    assert "l3_captureFormatName()," in stats
    assert '"compact frames=%u errors=%u max_us=%u scratch_stale=%u retain=%u/%u/%u %s\\n"' in stats
    log = _function("static int32_t l3_cli_triggerLog(")
    assert 'strcmp(argv[1], "frames") == 0' in log
    assert "l3_frame_desc_format(&gFrameDesc[slot], line, sizeof(line));" in log
    start = _function("static int32_t l3_cli_sensorStart(")
    for reset in (
        "gScratchBusy[0] = 0U;",
        "gDetectScratchStale = 0U;",
        "gCompactFrames = 0U;",
        "memset(gFrameScratch, L3_SCRATCH_NONE, sizeof(gFrameScratch));",
    ):
        assert reset in start, reset


CONFIG_DIR = FIRMWARE.parents[2] / "config"
# The L3 ring arena (.l3ring in mss_linker.cmd), the plan's capacity.
L3_RING_BYTES = 768 * 1024
# L3_MAX_CAPTURE_FRAMES in l3_dump.c; the trackers' arrays are sized to it too.
MAX_CAPTURE_FRAMES = 64
# One stored IQ16 bin of one frame: 3 TX x 12 loops x 4 RX x (I, Q) int16.
BYTES_PER_RETAINED_BIN = 3 * 12 * 4 * 4
DEFAULT_PROFILE = "iwr6843_l3dump_wide_24f2ms_53bin_iq16_window_hann.cfg"


def _profile(name: str) -> tuple[list[list[str]], dict[str, list[list[str]]]]:
    """A cfg's command lines in order, and every line of each command by name."""
    text = (CONFIG_DIR / name).read_text(encoding="utf-8")
    lines = [line.split() for line in text.splitlines() if line and not line.startswith("%")]
    by_name: dict[str, list[list[str]]] = {}
    for line in lines:
        by_name.setdefault(line[0], []).append(line[1:])
    return lines, by_name


def _phases(by_name) -> dict[str, int]:
    names = (
        "pre_start",
        "pre_bins",
        "pre_frames",
        "impact_start",
        "impact_bins",
        "impact_frames",
        "post_start",
        "post_bins",
        "late_start",
        "ball_frames",
        "stride",
    )
    (values,) = by_name["phaseCaptureCfg"]
    return dict(zip(names, (int(v) for v in values), strict=True))


def _retain_widths(by_name) -> tuple[int, int, int]:
    (widths,) = [args[1:] for args in by_name["captureCfg"] if args[0] == "retain"]
    pre, impact, post = (int(v) for v in widths)
    return pre, impact, post


def _frame_period_ms(by_name) -> int:
    (frame_cfg,) = by_name["frameCfg"]
    return int(frame_cfg[4])


@pytest.mark.parametrize(
    ("name", "frames", "period_ms"),
    [
        ("iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg", 47, 3),
        ("iwr6843_l3dump_adaptive_64f2ms_53bin_a16_window_hann.cfg", 64, 2),
    ],
)
def test_the_adaptive_profiles_ask_for_the_compact_format_and_fit_l3(name, frames, period_ms):
    lines, by_name = _profile(name)
    assert by_name["captureFormat"] == [["adaptive16"]]
    assert _retain_widths(by_name) == (16, 24, 16)
    phases = _phases(by_name)
    assert (phases["pre_bins"], phases["impact_bins"], phases["post_bins"]) == (53, 53, 53), (
        "processing windows stay wide"
    )
    assert phases["pre_frames"] + phases["impact_frames"] + phases["ball_frames"] == frames
    assert frames <= MAX_CAPTURE_FRAMES
    assert _frame_period_ms(by_name) == period_ms
    pre_bins, impact_bins, post_bins = _retain_widths(by_name)
    retained_bins = (
        phases["pre_frames"] * pre_bins
        + phases["impact_frames"] * impact_bins
        + phases["ball_frames"] * post_bins
    )
    assert retained_bins * BYTES_PER_RETAINED_BIN <= L3_RING_BYTES
    order = [line[0] if line[0] != "captureCfg" else f"captureCfg {line[1]}" for line in lines]
    assert (
        order.index("captureFormat")
        < order.index("captureCfg retain")
        < order.index("phaseCaptureCfg")
    )
    assert order[-1] == "sensorStart"


def test_the_64_frame_profile_is_the_default_rf_with_the_adaptive_format():
    """Only the stored format and the movie length differ from the 2 ms Hann default."""
    _lines, adaptive = _profile("iwr6843_l3dump_adaptive_64f2ms_53bin_a16_window_hann.cfg")
    _lines, default = _profile(DEFAULT_PROFILE)
    for command in ("channelCfg", "adcCfg", "profileCfg", "chirpCfg", "frameCfg"):
        assert adaptive[command] == default[command], command
    assert ["window", "hann"] in adaptive["captureCfg"]
    phases, wide = _phases(adaptive), _phases(default)
    for key in ("pre_start", "impact_start", "post_start", "late_start", "stride"):
        assert phases[key] == wide[key], key
    assert phases["impact_frames"] == wide["impact_frames"], "every impact frame kept"
    assert phases["pre_frames"] > wide["pre_frames"]
    assert phases["ball_frames"] > wide["ball_frames"]


def test_iq16_frames_take_the_exact_integer_statistics_path():
    """The board's residual is l3_channels_residual (l3_channels.c, host
    tested in test_iwr6843_firmware_channels.py): IQ16 goes to the exact
    integer scoring the DSS runs too, the float path stays for IQ8."""
    residual = _function("static void l3_verticalResidual(")
    assert "l3_channels_residual(&frame, localBin, perLoop, obs);" in residual
    channels = (FIRMWARE.parent / "l3_channels.c").read_text(encoding="utf-8")
    body = channels[channels.index("void l3_channels_residual(") :]
    body = body[: body.index("\n}\n")]
    assert "if (frame->cb == 2U) {" in body
    assert "l3_bin_score_iq16(" in body
    assert "perLoop);" in body[body.index("l3_bin_score_iq16(") :]
    assert body.index("if (frame->cb == 2U)") < body.index(
        "float im = (xIm[loop] - meanIm) * frame->scale;"
    ), "the float path stays for IQ8"
    score = (FIRMWARE.parent / "l3_bin_score.c").read_text(encoding="utf-8")
    assert "l3_iq16_bin_stats_init(&bin, loops);" in score
    assert "l3_iq16_channel_stats(words, loops, loopStrideWords, &channel)" in score
    makefile = (FIRMWARE.parent / "makefile").read_text(encoding="utf-8")
    assert "l3_iq16_stats.c" in makefile and "l3_channels.c" in makefile


def test_the_sub_bin_estimator_is_configurable_and_reaches_both_extractions():
    source = _source()
    assert "static uint32_t          gObsSubBin = L3_OBS_SUBBIN_PARABOLIC;" in source
    assert source.count("params.subBin = gObsSubBin;") == 2, "the club and the ball extraction"
    track_cfg = _function("static int32_t l3_cli_trackCfg(")
    assert 'strcmp(argv[1], "subbin") == 0' in track_cfg
    assert "gObsSubBin = L3_OBS_SUBBIN_CENTROID;" in track_cfg
    assert "gObsSubBin = L3_OBS_SUBBIN_PARABOLIC;" in track_cfg


def test_element_calibration_goes_through_the_shared_helper_and_prints_back():
    elem = _function("static int32_t l3_cli_trackCfgElem(")
    assert "l3_cal_set_element(&gRadarCal, index, values[2], values[1])" in elem
    assert "cosf(-values[1])" not in elem
    log = _function("static int32_t l3_cli_triggerLog(")
    assert 'strcmp(argv[1], "cal") == 0' in log
    assert "l3_cal_format(&gRadarCal, line, sizeof(line));" in log
    assert "l3_cal_format_element(&gRadarCal, index, line, sizeof(line));" in log
