"""The DSS solve needs the C6000 toolchain the SDK install used to strip."""

from __future__ import annotations

import re
from pathlib import Path

FIRMWARE_MAKEFILE = Path(__file__).parents[1] / "firmware" / "Makefile"
FIRMWARE_DIR = Path(__file__).parents[1] / "firmware" / "iwr6843"


def test_sdk_install_keeps_the_c6000_toolchain():
    text = FIRMWARE_MAKEFILE.read_text(encoding="utf-8")
    disabled = [line for line in text.splitlines() if "--disable-components" in line]
    assert disabled, "expected an SDK install line with --disable-components"
    for line in disabled:
        assert "TI_CGT_C6000" not in line
        assert "DSPLIB_C674x" not in line
        assert "MATHLIB_C674x" not in line


def test_meta_image_includes_a_dss_image():
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")
    assert "$(DSS_OUT)" in makefile
    assert (FIRMWARE_DIR / "dss" / "dss_main.c").exists()
    assert (FIRMWARE_DIR / "dss" / "dss_linker.cmd").exists()


def test_the_dss_keeps_the_platform_l2_all_sram_like_ti_mmw_demo():
    """On the board (2026-09-30) the DSS reached Startup.firstFxns and never
    Startup.lastFxns: it died in the xdc/BIOS module startups, with no
    exception and no DSS ESM flag. The one startup difference from TI's
    working mmw demo DSS was this image's L2 cache: it alone overrode the
    platform's ti_sysbios_family_c64p_Cache_l2Size = 0 (all SRAM) with 32 KB
    of cache, which the Cache module applies in exactly that window (and
    which linker warning #10190 flagged on every build). The DSS now keeps
    the platform's caches: L1P and L1D 16 KB each, L2 all SRAM. L3 frames
    are still read through L1D, with the same Cache_inv before scoring.
    """
    cmd = (FIRMWARE_DIR / "dss" / "dss_linker.cmd").read_text(encoding="utf-8")
    code = re.sub(r"/\*.*?\*/", "", cmd, flags=re.DOTALL)  # the code, not its comments
    assert "ti_sysbios_family_c64p_Cache_l2Size" not in code, "no L2 cache override"
    assert ".cacheReserve" not in code, "no L2 carved off for a cache that is not there"
    cfg = (FIRMWARE_DIR / "dss" / "dss.cfg").read_text(encoding="utf-8")
    assert "L2Size_32K" not in cfg


# --- the MSS <-> DSS detect link (Phase 0: ping and probe) --------------------
#
# The detect task is moving to the DSS. Phase 0 proves the link on the board:
# dspPing, and dspProbe, which scores the same ring frame on both cores with
# the same code (l3_bin_score.c) and compares. The C is unit tested on the
# host (test_iwr6843_firmware_dsp.py); these pin the board glue around it.

DSS_MAIN = FIRMWARE_DIR / "dss" / "dss_main.c"
MSS_MAIN = FIRMWARE_DIR / "l3_dump.c"


def _sources(makefile: Path) -> list[str]:
    for line in makefile.read_text(encoding="utf-8").splitlines():
        if line.startswith("SOURCES"):
            return line.split("=", 1)[1].split()
    raise AssertionError(f"no SOURCES in {makefile}")


def test_both_cores_build_the_shared_scoring_and_the_link():
    for makefile in (FIRMWARE_DIR / "makefile", FIRMWARE_DIR / "dss" / "makefile"):
        sources = _sources(makefile)
        for name in ("l3_bin_score.c", "l3_dsp_ipc.c", "l3_iq16_stats.c"):
            assert name in sources, f"{name} missing from {makefile}"


def test_the_dss_links_the_mailbox_driver():
    makefile = (FIRMWARE_DIR / "dss" / "makefile").read_text(encoding="utf-8")
    libs = [line for line in makefile.splitlines() if line.startswith("DSS_STD_LIBS")]
    assert libs and "-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE)" in libs[0]


def test_the_dss_answers_the_link_over_the_mailbox():
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert "Mailbox_init(MAILBOX_TYPE_DSS)" in text
    assert "Mailbox_open(MAILBOX_TYPE_MSS" in text
    assert "l3_dsp_serve(" in text, "PING, PROBE and SCORE, through the host-tested dispatcher"
    assert "Mailbox_readFlush(" in text, "a read message must be released or the next never lands"
    assert "SOC_XWR68XX_DSS_L3RAM_BASE_ADDRESS" in text, "the DSS reads L3 at its own address"


def test_the_dss_invalidates_its_cache_over_the_frame_before_scoring():
    """L2 caches L3 on the DSS and the EDMA rewrites ring slots behind it:
    without an invalidate the DSS scores a stale copy of an old frame."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    invalidate = text.find("Cache_inv(")
    assert invalidate >= 0
    assert invalidate < text.find("l3_dsp_serve(")


def test_the_mss_opens_the_link_with_a_bounded_wait():
    """A DSS that never answers must fail the command, not hang the CLI."""
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "Mailbox_open(MAILBOX_TYPE_DSS" in text
    assert "L3_DSP_REPLY_TIMEOUT_TICKS" in text
    assert "readTimeout = L3_DSP_REPLY_TIMEOUT_TICKS" in text


def test_the_link_commands_are_a_trackcfg_sub_mode_not_new_table_entries():
    """The CLI table is at the SDK's CLI_MAX_CMD (32) with the mmWave
    extension's commands: a new entry would overwrite one (it did: ball)."""
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert 'strcmp(argv[1], "dsp") == 0' in text
    assert "return l3_cli_trackCfgDsp(argc, argv);" in text
    assert '"dspPing"' not in text and '"dspProbe"' not in text
    entries = [int(n) for n in re.findall(r"tableEntry\[(\d+)\]\.cmd\s*=", text)]
    assert sorted(entries) == list(range(19)), "one table entry per command, 0..18"


def test_the_mss_sends_frames_as_l3_offsets_and_scores_them_itself_too():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "SOC_XWR68XX_MSS_L3RAM_BASE_ADDRESS" in text
    assert "l3_dsp_probe_run(" in text, "the MSS runs the same probe to compare"


def test_the_dss_leaves_the_system_clock_to_the_mss():
    """SOC_SysClock_INIT on the DSS re-ungates and unhalts the BSS and spins
    on the APLL calibration flag, the MSS's job; TI's own DSS (the mmw demo)
    uses BYPASS_INIT. Suspect for the DSS not answering on the board."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert "socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;" in text
    assert "SOC_SysClock_INIT;" not in text


def test_the_dss_records_every_boot_stage_and_writes_it_back():
    """The status must leave the DSS's cache or the MSS never sees it."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    for stage in (
        "L3_DSP_STAGE_MAIN",
        "L3_DSP_STAGE_SOC",
        "L3_DSP_STAGE_TASK",
        "L3_DSP_STAGE_MAILBOX",
        "L3_DSP_STAGE_LINK",
    ):
        assert f"dss_status({stage}" in text, stage
    assert "SOC_XWR68XX_DSS_HSRAM_BASE_ADDRESS + L3_DSP_STATUS_HSRAM_OFFSET" in text
    status = text[text.index("static void dss_status(") :]
    assert "Cache_wb(" in status[: status.index("\n}\n")]
    assert status.index("dss_status(L3_DSP_STAGE_MAIN") > 0
    main = text[text.index("int main(void)") :]
    assert main.index("dss_status(L3_DSP_STAGE_MAIN") < main.index("SOC_init(")


def test_the_dss_beats_while_it_waits_for_the_mss():
    """A bounded read, counted on each timeout: beats rising says BIOS runs."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert "cfg.readTimeout = BIOS_WAIT_FOREVER;" not in text
    assert "dss_statusCount(&gDssStatus->heartbeat)" in text


def test_the_mss_prints_the_dss_status_on_request_and_when_it_does_not_answer():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "SOC_XWR68XX_MSS_HSRAM_BASE_ADDRESS + L3_DSP_STATUS_HSRAM_OFFSET" in text
    assert 'strcmp(argv[2], "status") == 0' in text
    assert text.count("l3_dspPrintStatus();") >= 3, "status, and after each unanswered command"


# --- SCORE: the live detector's bins on the DSS ------------------------------
#
# The protocol is host tested (test_iwr6843_firmware_dsp_score.py), the
# per-frame choice (test_iwr6843_firmware_detect_core.py) and the timing
# (test_iwr6843_firmware_timing.py) too. These pin the board glue: cache
# coherence, the shared link, where the result lives, and that every
# scan-plan read goes through the one routed path.


def _function(text: str, name: str) -> str:
    start = text.rindex(name)  # the definition follows any forward declaration
    brace = text.index("{", start)
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated {name}")


def test_the_mss_builds_the_detect_core_and_the_timing():
    sources = _sources(FIRMWARE_DIR / "makefile")
    for name in ("l3_detect_core.c", "l3_timing.c"):
        assert name in sources, name


def test_the_dss_answer_times_the_invalidate_apart_from_the_scoring():
    answer = _function(DSS_MAIN.read_text(encoding="utf-8"), "static void dss_answer(")
    assert "L3_DSP_CMD_SCORE" in answer, "SCORE's frame is invalidated too"
    # Preparing the frame (the gather, or the invalidate) is timed apart from
    # the scoring, and reported in the result's invCycles.
    assert answer.index("Cache_inv(") < answer.index("prepCycles = TSCL - start;")
    assert answer.index("prepCycles = TSCL - start;") < answer.index("l3_dsp_serve(")
    assert "gDssResult->invCycles = reply->prepCycles;" in answer
    assert "gDssResult->scoreCycles = reply->cycles;" in answer


def test_the_dss_writes_the_result_back_before_it_replies():
    """The reply is the MSS's signal to read HS-RAM: the result must have
    left the DSS's cache by then, cycles included."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    answer = _function(text, "static void dss_answer(")
    assert "Cache_wb((Ptr)gDssResult, sizeof(*gDssResult)" in answer
    assert answer.index("gDssResult->scoreCycles") < answer.index("Cache_wb((Ptr)gDssResult")
    task = _function(text, "static void dss_solveTask(")
    assert task.index("dss_answer(") < task.index("Mailbox_write(")


def test_both_cores_address_the_result_block_at_their_own_hs_ram_base():
    assert "SOC_XWR68XX_DSS_HSRAM_BASE_ADDRESS + L3_DSP_RESULT_HSRAM_OFFSET" in DSS_MAIN.read_text(
        encoding="utf-8"
    )
    assert (
        "SOC_XWR68XX_MSS_HSRAM_BASE_ADDRESS +\n                                              L3_DSP_RESULT_HSRAM_OFFSET"
        in (MSS_MAIN.read_text(encoding="utf-8"))
    )


def test_the_detect_task_waits_two_frames_for_the_dss_not_a_tenth_of_a_second():
    text = MSS_MAIN.read_text(encoding="utf-8")
    match = re.search(r"#define L3_DSP_REPLY_TIMEOUT_TICKS (\d+)U", text)
    assert match and int(match.group(1)) <= 6


def test_the_link_is_one_request_at_a_time_and_the_detect_task_never_waits_for_it():
    text = MSS_MAIN.read_text(encoding="utf-8")
    exchange = _function(text, "static int32_t l3_dspExchange(")
    assert exchange.index("l3_dspLinkLock()") < exchange.index("l3_dspSend(")
    assert "l3_dspLinkUnlock();" in exchange
    try_lock = _function(text, "static int32_t l3_dspLinkTryLock(")
    assert "BIOS_NO_WAIT" in try_lock
    spans = _function(text, "static void l3_scoreSpans(")
    assert "l3_dspLinkTryLock()" in spans and "l3_dspLinkLock()" not in spans
    assert spans.count("l3_dspLinkUnlock();") == 2, "released on both routes"


def test_the_mss_accepts_only_the_answer_to_its_request_from_a_snapshot():
    score = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_dspScoreSpans(")
    assert "memcpy(&result, (const void *)l3_dspResultBlock(), sizeof(result));" in score
    assert score.index("memcpy(&result") < score.index("l3_dsp_result_check(")
    assert score.index("l3_dsp_result_check(") < score.index("l3_dsp_result_merge(")


def test_verify_sends_first_so_the_cores_score_at_once():
    score = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_dspScoreSpans(")
    send = score.index("sent = l3_dspSend(&request);")
    mss = score.index("l3_mssScoreSpans(frame, local, n, obs, scored);")
    wait = score.index("l3_dspAwait(&request, &reply)")
    assert send < mss < wait
    assert "l3_dsp_result_compare(" in score and "l3_detect_core_note_mismatch(" in score


def test_a_failed_dss_frame_falls_back_to_the_mss_and_is_reported():
    score = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_dspScoreSpans(")
    assert "gDetectEvent.core = (uint8_t)L3_TIMING_CORE_FALLBACK;" in score
    assert "l3_detect_core_report(" in score
    assert "L3_PROF_DSP_WAIT" in score


def test_the_latch_notice_is_not_an_error_line():
    """The host fails any command whose reply carries "Error"; a notice can
    land inside one."""
    score = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_dspScoreSpans(")
    notice = re.search(r'l3_queueNotice\("([^"]*)"\)', score)
    assert notice and "Error" not in notice.group(1)


def test_every_scan_plan_read_goes_through_the_routed_scoring():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "static void l3_scoreSpan(" not in text, "the per-span MSS-only path is gone"
    trigger = _function(text, "static void l3_considerSelfTrigger(")
    assert trigger.count("l3_scoreSpans(") == 1, "one call, one DSS round trip a frame"
    assert "l3_verticalResidual(" not in trigger
    ball = _function(text, "static void l3_considerBallTrack(")
    assert ball.count("l3_scoreSpans(") == 2  # the scan plan, and the whole window without a band
    assert "l3_verticalResidual(&frame, bin" not in ball


def test_the_routed_scoring_uses_the_shared_span_scorer():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "l3_dsp_spans_localize(" in _function(text, "static void l3_scoreSpans(")
    assert "l3_dsp_spans_score(" in _function(text, "static void l3_mssScoreSpans(")


def test_only_an_iq16_ring_frame_in_l3_goes_to_the_dss():
    eligible = _function(
        MSS_MAIN.read_text(encoding="utf-8"), "static uint8_t l3_dspFrameEligible("
    )
    for condition in (
        "frame->scratch == L3_SCRATCH_NONE",
        "frame->cb == 2U",
        "gCapturePlan.loops <= L3_IQ16_MAX_LOOPS",
        "base >= SOC_XWR68XX_MSS_L3RAM_BASE_ADDRESS",
    ):
        assert condition in eligible, condition


def test_a_read_slot_is_checked_again_after_it_was_read():
    text = MSS_MAIN.read_text(encoding="utf-8")
    stale = _function(text, "static int32_t l3_detectFrameStale(")
    assert "l3detect_slot_live(gDetectRingEpoch, gPreFramesCaptured" in stale
    assert "gDetectStaleAfterRead++;" in stale
    task = _function(text, "static void l3_detectTask(")
    assert task.index("gDetectRingEpoch = epoch;") < task.index("l3_considerSelfTrigger(")
    finish = _function(text, "static void l3_detectFinish(")
    assert "gDetectRingEpoch = L3_DETECT_POST_EPOCH;" in finish


def test_the_writer_stamps_each_frame_it_publishes():
    publish = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_publishDetectFrame(")
    assert "Cycleprofiler_getTimeStamp()" in publish


def test_the_detect_task_records_every_frame_it_finishes():
    task = _function(MSS_MAIN.read_text(encoding="utf-8"), "static void l3_detectTask(")
    assert task.count("l3_detectFinish();") == 2, "post frames and pre frames"
    assert "l3detect_depth(&gDetectQueue)" in task


def test_detect_core_is_a_trackcfg_sub_mode():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert 'strcmp(argv[1], "detectCore") == 0' in text
    assert "return l3_cli_trackCfgDetectCore(argc, argv);" in text
    handler = _function(text, "static int32_t l3_cli_trackCfgDetectCore(")
    assert "l3_detect_core_reset_counts(&gDetectCore);" in handler
    assert "l3_timingRestart();" in handler
    assert "detectCore dss|verify" in handler
    assert "mss|" not in handler, "mss is not a choice"


def test_timing_restarts_with_each_session():
    text = MSS_MAIN.read_text(encoding="utf-8")
    session = text[text.index("/* A new session starts untriggered") - 400 :]
    assert "l3_timingRestart();" in session[:600]


def test_trigger_log_prints_the_detect_timing():
    log = _function(MSS_MAIN.read_text(encoding="utf-8"), "static int32_t l3_cli_triggerLog(")
    assert 'strcmp(argv[1], "timing") == 0' in log
    assert log.count("l3_writeDetectTiming(") == 2, "perf and timing"


def test_the_dss_marks_reset_before_c_init_and_bios():
    """xdc Reset functions run before cinit: a DSS stuck at 'reset' died in
    its C or BIOS startup, before main."""
    cfg = (FIRMWARE_DIR / "dss" / "dss.cfg").read_text(encoding="utf-8")
    assert "xdc.useModule('xdc.runtime.Reset')" in cfg
    assert "'&dss_resetHook'" in cfg
    text = DSS_MAIN.read_text(encoding="utf-8")
    hook = text[text.index("void dss_resetHook(void)") :]
    hook = hook[: hook.index("\n}\n")]
    assert "L3_DSP_STAGE_RESET" in hook
    assert "Cache_" not in hook, "BIOS is not up yet: no Cache calls in the reset hook"


def test_the_dss_mirrors_its_stage_into_dssgpreg0():
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert "SOC_XWR68XX_DSS_DSSREG_BASE_ADDRESS" in text
    assert "L3_DSP_GPREG_TAG | stage" in text


def test_the_mss_reads_the_dss_hardware_state_without_the_dss():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert "SOC_XWR68XX_MSS_DSSREG_BASE_ADDRESS" in text
    assert "GEMPWRSMCFG4" in text and "GEMPWRSMCFG3" in text
    assert "SOC_XWR68XX_MSS_ESM_BASE_ADDRESS" in text
    assert 'strcmp(argv[2], "hw") == 0' in text
    assert text.count("l3_dspPrintHw();") >= 3, "hw, and after each unanswered command"


def test_the_dss_brackets_its_module_startup_and_hooks_exceptions():
    cfg = (FIRMWARE_DIR / "dss" / "dss.cfg").read_text(encoding="utf-8")
    assert "Startup.firstFxns.$add('&dss_startupFirst')" in cfg
    assert "Startup.lastFxns.$add('&dss_startupLast')" in cfg
    assert "xdc.useModule('ti.sysbios.family.c64p.Exception')" in cfg
    assert "Exception.exceptionHook = '&dss_exceptionHook'" in cfg
    text = DSS_MAIN.read_text(encoding="utf-8")
    for fxn, stage in (
        ("dss_startupFirst", "L3_DSP_STAGE_FIRST"),
        ("dss_startupLast", "L3_DSP_STAGE_LAST"),
        ("dss_exceptionHook", "L3_DSP_STAGE_EXCEPTION"),
    ):
        body = text[text.index(f"void {fxn}(void)\n{{") :]
        body = body[: body.index("\n}\n")]
        assert stage in body, fxn
    hook = text[text.index("void dss_exceptionHook(void)\n{") :]
    assert "Exception_getLastStatus(" in hook[: hook.index("\n}\n")]


def test_the_hs_ram_probe_word_is_clear_of_the_result_block():
    """ "trackCfg dsp hw" writes and reads a word of HS-RAM from the MSS; it
    must not land inside SCORE's result block (or the status)."""
    import ctypes  # pylint: disable=import-outside-toplevel

    from openflight.iwr6843 import firmware_host as fw  # pylint: disable=import-outside-toplevel

    match = re.search(r"#define L3_HSRAM_PROBE_OFFSET\s+(0x[0-9A-Fa-f]+)U", MSS_MAIN.read_text())
    assert match
    probe = int(match.group(1), 16)
    result = range(
        fw.L3_DSP_RESULT_HSRAM_OFFSET, fw.L3_DSP_RESULT_HSRAM_OFFSET + ctypes.sizeof(fw.DspResult)
    )
    status = range(
        fw.L3_DSP_STATUS_HSRAM_OFFSET, fw.L3_DSP_STATUS_HSRAM_OFFSET + ctypes.sizeof(fw.DspStatus)
    )
    for word_byte in range(probe, probe + 4):
        assert word_byte not in result and word_byte not in status


# --- MSS diagnostics in HS-RAM, clear of the DSS's words ---------------------
#
# 2026-09-30: detect timing (l3_timing) and the detect-core CLI buffers
# overflowed DATA_RAM by 1,568 B (".myFiqStack" would not fit). HS-RAM's
# lower 29 KB is unused: the MSS-only diagnostics move there, and the DSS's
# words (SCORE result 0x7400, the probe word 0x7E00, the status 0x7F00) are
# reserved in the MSS link so a growing MSS section fails the link instead
# of overwriting them at run time.

MSS_LINKER = FIRMWARE_DIR / "mss_linker.cmd"


def test_the_mss_places_its_hs_ram_diagnostics_below_a_reserved_dss_area():
    cmd = MSS_LINKER.read_text(encoding="utf-8")
    assert ".hsramMss" in cmd and "> HS_RAM" in cmd
    assert ".hsramDss" in cmd
    assert "0x52087400" in cmd, "the reservation starts at the SCORE result block"
    assert "0x00000C00" in cmd, "and runs to the top of HS-RAM"


def test_the_reserved_area_covers_every_dss_word():
    import ctypes  # pylint: disable=import-outside-toplevel

    from openflight.iwr6843 import firmware_host as fw  # pylint: disable=import-outside-toplevel

    reserved = range(0x7400, 0x8000)
    for start, size in (
        (fw.L3_DSP_RESULT_HSRAM_OFFSET, ctypes.sizeof(fw.DspResult)),
        (fw.L3_DSP_STATUS_HSRAM_OFFSET, ctypes.sizeof(fw.DspStatus)),
        (0x7E00, 4),  # L3_HSRAM_PROBE_OFFSET
    ):
        assert start in reserved and start + size - 1 in reserved


def test_the_mss_only_diagnostics_live_in_hs_ram():
    text = MSS_MAIN.read_text(encoding="utf-8")
    assert '#define L3_HSRAM_DIAG __attribute__((section(".hsramMss")))' in text
    for declaration in (
        "static l3_profile_t        gProfile L3_HSRAM_DIAG;",
        "static l3_timing_t         gTiming L3_HSRAM_DIAG;",
        "static char line[L3_DETECT_LINE_BYTES] L3_HSRAM_DIAG;",
        "static char detect[L3_DETECT_LINE_BYTES] L3_HSRAM_DIAG;",
        # CLI-only scratch: l3sparse's power rows, triggerLog, ball and stats lines
        "static float powerRow[L3_MAX_LOOPS * L3_RING_MAX_BINS] L3_HSRAM_DIAG;",
        "static char ballLine[192] L3_HSRAM_DIAG;",
    ):
        assert declaration in text, declaration
    assert text.count("static char line[192] L3_HSRAM_DIAG;") == 2, "triggerLog and ball"


def test_the_hs_ram_diagnostics_start_zeroed_like_bss():
    """HS-RAM is not the zero-initialized .bss: 'triggerLog perf' may format
    them before their lazy init, so the init task clears them first."""
    text = MSS_MAIN.read_text(encoding="utf-8")
    init = text[text.index("static void l3_initTask(UArg arg0, UArg arg1)") :]
    init = init[: init.index("Mailbox_init(MAILBOX_TYPE_MSS);")]
    assert "memset(&gProfile, 0, sizeof(gProfile));" in init
    assert "memset(&gTiming, 0, sizeof(gTiming));" in init


# --- the gather: frames copied into L2 by the EDMA before scoring -------------
#
# The shared C (l3_dsp_gather_*) is unit tested in
# test_iwr6843_firmware_dsp_gather.py; these pin the DSS glue around it.


def _dss_answer() -> str:
    text = DSS_MAIN.read_text(encoding="utf-8")
    body = text[text.index("static void dss_answer(") :]
    return body[: body.index("\n}\n")]


def test_the_dss_links_the_edma_driver():
    makefile = (FIRMWARE_DIR / "dss" / "makefile").read_text(encoding="utf-8")
    libs = [line for line in makefile.splitlines() if line.startswith("DSS_STD_LIBS")]
    assert "-llibedma_$(MMWAVE_SDK_DEVICE_TYPE)" in libs[0]


def test_the_dss_uses_its_own_edma_instance_not_the_captures():
    """The MSS's capture EDMA is instance 0 (l3_dump.c EDMA_open(0, ...))."""
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert "#define DSS_GATHER_EDMA_INSTANCE 1U" in text
    assert "EDMA_open(DSS_GATHER_EDMA_INSTANCE" in text
    assert "EDMA_open(0" not in text
    mss = MSS_MAIN.read_text(encoding="utf-8")
    assert "EDMA_open(0, " in mss


def test_the_gather_buffer_has_its_own_l2_section():
    text = DSS_MAIN.read_text(encoding="utf-8")
    assert '#pragma DATA_SECTION(gDssGather, ".dssGather")' in text
    assert "static uint8_t gDssGather[L3_DSP_GATHER_MAX_BYTES];" in text
    cmd = (FIRMWARE_DIR / "dss" / "dss_linker.cmd").read_text(encoding="utf-8")
    assert ".dssGather" in cmd and "L2SRAM_UMAP1" in cmd


def _dss_gather() -> str:
    text = DSS_MAIN.read_text(encoding="utf-8")
    body = text[text.index("static uint8_t dss_gather(") :]
    return body[: body.index("\n}\n")]


def test_the_gather_starts_the_edma_and_polls_it_with_a_bound():
    """An EDMA that never completes cannot hang the link."""
    gather = _dss_gather()
    assert gather.index("EDMA_startDmaTransfer(") < gather.index("EDMA_isTransferComplete(")
    assert "DSS_GATHER_TIMEOUT_CYCLES" in gather
    assert "set->aCount = (uint16_t)gather->rowBytes;" in gather
    assert "set->bCount = (uint16_t)gather->rows;" in gather
    assert "set->sourceBindex = (int16_t)gather->srcStride;" in gather
    assert "set->destinationBindex = (int16_t)gather->rowBytes;" in gather
    assert "set->transferType = (uint8_t)EDMA3_SYNC_AB;" in gather


def test_the_dss_scores_from_the_copy_only_after_the_gather_and_the_l1d_invalidate():
    answer = _dss_answer()
    gather = answer.index("gathered = dss_gather(&gather);")
    invalidate = answer.index("Cache_inv((Ptr)gDssGather")
    serve = answer.index("l3_dsp_serve_gathered(")
    assert gather < invalidate < serve


def test_a_gather_that_fails_falls_back_to_scoring_in_place():
    answer = _dss_answer()
    assert "if (!gathered && " in answer
    assert "l3_dsp_serve(request, l3, DSS_L3_BYTES, reply, gDssResult);" in answer


def test_the_dss_reports_how_it_prepared_the_frame():
    answer = _dss_answer()
    assert "reply->prepCycles = " in answer
    assert "gDssResult->invCycles = reply->prepCycles;" in answer


def test_an_address_the_soc_cannot_translate_is_never_handed_to_the_edma():
    """SOC_translateAddress returns SOC_TRANSLATEADDR_INVALID on a miss: the
    gather then gives up (the frame is scored in place)."""
    gather = _dss_gather()
    assert len(re.findall(r"SOC_TranslateAddr_Dir_TO_EDMA,\s*&srcErr\)", gather)) == 1
    assert len(re.findall(r"SOC_TranslateAddr_Dir_TO_EDMA,\s*&dstErr\)", gather)) == 1
    check = gather.index("if (srcErr != 0 || dstErr != 0)")
    assert check < gather.index("EDMA_configParamSet(")
