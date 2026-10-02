"""l3_dump.c wires the tee band, the range-only impact and the impact fit the
way firmware_replay does. The board file needs TI headers, so this reads its
source; the behaviour is tested through the replay (test_iwr6843_firmware_replay*).

Band off (bandBins 0, the default) must be exactly the board's old behaviour:
the club track reads the trigger's region, the ball is armed at the tee and
the post-impact targets are unfiltered. Band on: before impact the club track
reads the whole window, keeping only targets short of the band
(firmware_replay._pre_impact_club_targets); after impact the band is dropped
from the targets (l3_band_filter). The band is placed on the noisiest idle
bins near the destination (l3_band_place over gBandNoise) and frozen while a
club track is active."""

from __future__ import annotations

import re

from openflight.iwr6843.firmware_host import FIRMWARE_DIR

SOURCE = (FIRMWARE_DIR / "l3_dump.c").read_text(encoding="utf-8")


def body(name: str) -> str:
    """The body of the static function ``name`` (its definition, not a
    forward declaration: only the definition is followed by a brace)."""
    match = re.search(rf"static \w+ {name}\([^)]*\)\s*\{{(.*?)\n\}}", SOURCE, re.S)
    assert match, name
    return match.group(1)


def test_globals_for_the_band_the_range_impact_and_the_fit():
    for declaration in (
        "static l3_impact_fit_cfg_t gImpactFitCfg;",
        "static l3_band_t           gBand;",
        "static l3_impact_t         gRangeImpact;",
        "static l3_impact_fit_t     gImpactFit;",
        "static l3_band_noise_t     gBandNoise;",
        "static uint8_t             gBandFrozen;",
    ):
        assert declaration in SOURCE, declaration
    assert '#include "l3_band.h"' in SOURCE
    assert '#include "l3_impact_fit.h"' in SOURCE


def test_band_off_is_no_band_on_every_pre_impact_frame():
    self_trigger = body("l3_considerSelfTrigger")
    enabled = self_trigger.index("if (gImpactFitCfg.bandBins > 0.0F) {")
    off = self_trigger.index("gBand.valid = 0U;")
    assert enabled < off < self_trigger.index("l3_preImpactClubTargets(")


def test_noise_map_is_updated_only_from_idle_frames_span_by_span():
    """After the club track update: an active track freezes the band; an idle
    frame thaws it and feeds each span it scored to the noise map (l3_scan.h:
    the club's, the fallback's and the band-interior chunk)."""
    self_trigger = body("l3_considerSelfTrigger")
    track = self_trigger.index("appended = l3_track_update(&gClubTrack,")
    active = self_trigger.index("if (gClubTrack.active) {", track)
    frozen = self_trigger.index("gBandFrozen = 1U;", active)
    thawed = self_trigger.index("gBandFrozen = 0U;", frozen)
    update = self_trigger.index(
        "l3_band_noise_update_span(&gBandNoise, gTrigCfg.stat, frame.binStart,", thawed
    )
    assert "windowCount, fed[k]->first," in self_trigger[update : update + 200]
    assert track < active < frozen < thawed < update
    assert "l3_band_noise_update(&gBandNoise" not in SOURCE, "never the whole window"
    for index, span in enumerate(("club", "leave", "chunk")):
        assert f"fed[{index}] = &{span};" in self_trigger[thawed:update]
    assert "if (fed[k]->count > 0U) {" in self_trigger[thawed:update]


def test_noise_map_is_reset_once_with_the_fit_defaults():
    """The map persists across shots: reset only where the fit's defaults are
    set once, never at rearm."""
    ensure = body("l3_ensureRadarCal")
    defaults = ensure.index("l3_impact_fit_cfg_defaults(&gImpactFitCfg);")
    assert ensure.index("l3_band_noise_reset(&gBandNoise);") > defaults
    assert SOURCE.count("l3_band_noise_reset(") == 1
    assert "l3_band_noise_reset" not in body("l3_trigRearm")


def test_pre_impact_club_targets_come_from_the_club_span_short_of_a_valid_band():
    """The helper scores nothing: obs is indexed by the frame's local bin and the
    scan plan's spans were scored into it."""
    helper = body("l3_preImpactClubTargets")
    assert "l3_verticalResidual(" not in helper
    extract = helper.index("found = l3_obs_extract(params, frameIndex, frameUs, club->first,")
    assert "&obs[club->first - windowFirst], club->count," in helper[extract:]
    keep = helper.index("found = l3_band_keep_short(&gBand, targets, found);")
    clutter = helper.index(
        "return l3_band_clutter_filter(&gBandNoise, gImpactFitCfg.clutterSigmas, targets, found);"
    )
    assert extract < keep < clutter
    assert "l3_band_filter" not in helper, "before impact: short of the band, not merely outside"


def test_the_dump_carries_the_clutter_map_after_the_temperature_report():
    """Version 10 (dump_format.h, dump.py): the map follows the temperature
    report, header then means, spreads and update counts, only with one."""
    match = re.search(r"int32_t l3_cli_dump\([^)]*\)\s*\{(.*?)\n\}", SOURCE, re.S)
    assert match, "l3_cli_dump (a CLI handler, not static)"
    dump = match.group(1)
    version = dump.index("h.version = L3_DUMP_VERSION_CLUTTER;")
    temperature = dump.index(
        "UART_writePolling(gDataUart, (uint8_t *)&tempReport, sizeof(tempReport));"
    )
    header = dump.index("UART_writePolling(gDataUart, (uint8_t *)&clutter, sizeof(clutter));")
    avg = dump.index("(uint8_t *)gBandNoise.avg, mapBins * sizeof(float));")
    dev = dump.index("(uint8_t *)gBandNoise.dev, mapBins * sizeof(float));")
    seen = dump.index("(uint8_t *)gBandNoise.seen, mapBins);")
    assert version < temperature < header < avg < dev < seen
    assert "L3_DUMP_VERSION_CAPTURE_TEMPERATURE" not in dump


def test_club_track_reads_the_helper_and_the_trigger_keeps_its_region():
    self_trigger = body("l3_considerSelfTrigger")
    trig = self_trigger.index("l3_trig_observe(&gTrig, gPreFramesCaptured, teeBin,")
    helper = self_trigger.index(
        "found = l3_preImpactClubTargets(obs, frame.binStart, &club, &leave,"
    )
    track = self_trigger.index("appended = l3_track_update(&gClubTrack, targets, found,")
    assert trig < helper < track
    assert "l3_obs_extract(" not in self_trigger, "the club's targets come from the helper only"
    assert "l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart," in self_trigger


def test_range_impact_runs_every_pre_impact_frame_and_feeds_the_shot():
    self_trigger = body("l3_considerSelfTrigger")
    delivery = self_trigger.index("(void)l3_track_delivery(&gClubTrack, 8U, &gDelivery);")
    club_in = self_trigger.index(
        "l3_impact_fit_track(&gImpactFitCfg, L3_FIT_CLUB_IN, l3_fit_span_point,"
    )
    ranged = self_trigger.index("ranged = l3_impact_update_range(&gRangeImpact, &clubIn, &clubNow,")
    assert delivery < club_in < ranged
    # The approach-end rule sees this frame's newest point only when one was taken.
    club_now = self_trigger.index("clubNow.ballRangeM = (float)teeBin * gClubTrack.cfg.binWidthM;")
    took = self_trigger.index("if (appended && gClubTrack.count > 0U &&", club_now)
    assert club_in < club_now < took < self_trigger.index("clubNow.appended = 1U;") < ranged
    # The range-only impact or the ball-leave fallback is the self-trigger:
    # it feeds the shot, then freezes.
    fired = self_trigger.index("fired = (ranged || left) ? 1 : 0;")
    observe_call = self_trigger.index("l3_shotObserve(teeBin, fired, impactUs);")
    assert ranged < fired < observe_call < self_trigger.index("if (!fired) {")
    observe = body("l3_shotObserve")
    assert "in.rangeFired = (uint8_t)(fired ? 1U : 0U);" in observe
    assert "in.impactTimestampUs = fired ? impactUs : frameUs;" in observe


def test_the_ball_leave_fallback_reads_its_span_before_the_club_targets_reuse_the_buffer():
    """Band on, l3_leave_targets reads the scan plan's span beyond the band into
    the club's target buffer (no new RAM), before the club's own extraction
    overwrites it, and reports the median there as the post window's floor. No
    band, no fallback."""
    targets = body("l3_preImpactClubTargets")
    guard = targets.index("if (gBand.valid && leave->count > 0U) {")
    read = targets.index(
        "uint32_t leaving = l3_leave_targets(&gLeave.cfg, params, &obs[leave->first - windowFirst],"
    )
    assert "&gLeaveFloor);" in targets[read : read + 400]
    update = targets.index("*left = l3_leave_update(&gLeave, targets, leaving, gBand.hiBin,")
    club = targets.index("found = l3_obs_extract(params, frameIndex, frameUs, club->first,")
    assert guard < read < update < club
    # Armed by the club track as it stood after the last frame, near the band,
    # as the replay's _leave_club_near asks it.
    near = targets.index("? l3_leave_club_near(&gLeave.cfg, gClubTrack.active,")
    assert read < near < update
    assert "gClubTrack.count, newest.rangeBin," in targets[near:update]
    assert "gBand.loBin)" in targets[near:update]
    assert "0.5F * (gBand.loBin + gBand.hiBin), near);" in targets[update:club]
    self_trigger = body("l3_considerSelfTrigger")
    assert "int32_t left = 0;" in self_trigger
    assert "&left);" in self_trigger[self_trigger.index("l3_preImpactClubTargets(") :]


def test_the_fire_is_dated_by_the_rule_that_fired():
    self_trigger = body("l3_considerSelfTrigger")
    assert (
        "impactUs = ranged ? gRangeImpact.impactTimestampUs : gLeave.impactTimestampUs;"
        in self_trigger
    )


def test_the_ball_leave_fallback_is_set_up_rearmed_and_logged():
    assert "static l3_leave_t          gLeave;" in SOURCE
    assert "l3_leave_rearm(&gLeave);" in body("l3_trigRearm")
    assert "l3_leave_cfg_defaults(&leaveCfg);" in SOURCE
    assert "leaveCfg.binWidthM = cfg.binWidthM;" in SOURCE
    assert "l3_leave_init(&gLeave, &leaveCfg);" in SOURCE
    log = body("l3_cli_triggerLog")
    track = log[log.index('strcmp(argv[1], "track") == 0') :]
    assert "(void)l3_leave_format(&gLeave, line, sizeof(line));" in track
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")
    assert " l3_leave.c " in makefile


def test_post_impact_targets_are_band_filtered():
    ball_track = body("l3_considerBallTrack")
    extract = ball_track.index("found = l3_obs_extract(&params, frameIndex, gPostTimestampUs,")
    band = ball_track.index("found = l3_band_filter(&gBand, targets, found);")
    follow = ball_track.index("l3_track_follow(&gClubTrack, targets, found,")
    assert extract < band < follow


def test_ball_tracker_is_armed_at_the_band_edge():
    assert "return gBand.valid ? gBand.hiBin : (float)teeBin;" in body("l3_ballArmBin")
    assert "l3_ball_track_anchor(&gBallTrack, (float)teeBin, l3_ballArmBin(teeBin)," in body(
        "l3_shotObserve"
    )


def test_impact_fit_runs_before_the_result_is_built():
    ball_track = body("l3_considerBallTrack")
    assert ball_track.index("l3_impactFitRun();") < ball_track.index("l3_result_build(")
    run = body("l3_impactFitRun")
    assert "l3_fit_span_after(&gClubTrack, gShot.impactFrame, &clubOut);" in run
    assert "l3_impact_fit_run(&gImpactFitCfg, &clubIn, &clubOut, &ballOut," in run
    assert "gShot.impactTimestampUs = l3_round_us(gImpactFit.impactUs);" in run


def test_rearm_forgets_the_range_impact_and_the_fit():
    rearm = body("l3_trigRearm")
    assert "l3_impact_rearm(&gRangeImpact);" in rearm
    assert "l3_impact_fit_reset(&gImpactFit);" in rearm


def test_fit_cfg_defaults_once_so_a_configured_band_persists():
    """trackCfg impactFit may come before or after triggerCfg/sensorStart:
    only the first l3_ensureRadarCal sets the defaults."""
    ensure = body("l3_ensureRadarCal")
    assert "if (!gImpactFitCfgSet) {" in ensure
    assert ensure.index("if (!gImpactFitCfgSet) {") < ensure.index(
        "l3_impact_fit_cfg_defaults(&gImpactFitCfg);"
    )
    assert SOURCE.count("l3_impact_fit_cfg_defaults(") == 1
    configure = body("l3_clubTrackConfigure")
    assert "gImpactFitCfg.binWidthM = cfg.binWidthM;" in configure
    assert "l3_impact_init(&gRangeImpact, &gImpactCfg);" in configure
    assert "l3_impact_init(&gRangeImpact, &gImpactCfg);" in body("l3_cli_trackCfgImpact")


def test_band_command_is_a_track_cfg_sub_mode():
    """The CLI table is at the SDK's CLI_MAX_CMD: a sub-mode, not a new command."""
    track_cfg = body("l3_cli_trackCfg")
    assert 'strcmp(argv[1], "impactFit") == 0' in track_cfg
    assert "return l3_cli_trackCfgImpactFit(argc, argv);" in track_cfg
    handler = body("l3_cli_trackCfgImpactFit")
    assert "l3_parseFloats(argc, argv, 2, 1U, values) != 0" in handler
    assert "!(values[0] >= 0.0F)" in handler, "negative and NaN refused"
    assert "values[0] > L3_IMPACT_FIT_MAX_BAND_BINS" in handler
    assert handler.index("l3_ensureRadarCal();") < handler.index(
        "gImpactFitCfg.bandBins = values[0];"
    )
    assert 'CLI_write("Done\\n");' in handler
    assert "#define L3_IMPACT_FIT_MAX_BAND_BINS 64.0F" in SOURCE
    assert "tableEntry[19]" not in SOURCE


def test_track_log_prints_the_impact_fit_after_the_impact():
    log = body("l3_cli_triggerLog")
    assert log.index("l3_impact_format(&gRangeImpact, line, sizeof(line));") < log.index(
        "l3_impact_fit_format(&gImpactFit, line, sizeof(line));"
    )


def test_new_modules_are_in_the_board_image():
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")
    assert "l3_band.c" in makefile and "l3_impact_fit.c" in makefile


def test_post_impact_ball_runs_before_the_club_which_gets_the_scene():
    ball_track = body("l3_considerBallTrack")
    ball = ball_track.index("l3_ball_track_update_joint(&gBallTrack")
    club = ball_track.index("l3_track_follow(&gClubTrack")
    assert ball < club
    assert "L3_TRACK_NO_TARGET" in ball_track[ball:club]
    assert "&follow)" in ball_track[club : club + 200]
    assert "l3_track_recent_rate(&gBallTrack.core)" in ball_track
    assert "gBallTrack.lastTargetIndex" in ball_track


def test_post_impact_unknown_approach_falls_back_to_the_club_ceiling():
    ball_track = body("l3_considerBallTrack")
    assert "L3_TRACK_FOLLOW_UNKNOWN_APPROACH_MPS" in ball_track
    assert "gShot.delivery.speedValid" in ball_track


def test_board_places_the_band_from_the_noise_map_until_frozen():
    self_trigger = body("l3_considerSelfTrigger")
    place = self_trigger.index("l3_band_place(&gBandNoise,")
    targets = self_trigger.index("l3_preImpactClubTargets(")
    assert place < targets
    assert "gBandFrozen" in self_trigger[: place + 200]
    assert "l3_band_noise_update_span(&gBandNoise," in self_trigger
    assert "gBandFrozen = 0U" in body("l3_trigRearm")
    assert "l3_band_around" not in SOURCE


def test_ball_snr_is_a_track_cfg_sub_mode():
    """The ball tracker's extraction snr is set apart from the trigger's."""
    track_cfg = body("l3_cli_trackCfg")
    assert 'strcmp(argv[1], "ballSnr") == 0' in track_cfg
    assert "return l3_cli_trackCfgBallSnr(argc, argv);" in track_cfg
    handler = body("l3_cli_trackCfgBallSnr")
    assert "l3_parseFloats(argc, argv, 2, 1U, values) != 0" in handler
    # 0 restores the firmware default; otherwise at least the floor. NaN refused.
    assert "!(values[0] == 0.0F || values[0] >= 1.0F)" in handler
    assert "gBallSnr = values[0];" in handler
    assert "static float               gBallSnr;" in SOURCE


def test_ball_extraction_uses_the_configured_ball_snr_else_the_default():
    ball_track = body("l3_considerBallTrack")
    assert "params.snr = (gBallSnr > 0.0F) ? gBallSnr : gBallTrackCfg.snr;" in ball_track
    # ... lowered to the history's snr with recovery on (the replay's same call).
    lowered = "params.snr = l3_ball_track_extract_snr(&gBallTrack.cfg, params.snr);"
    assert ball_track.index(lowered) > ball_track.index("params.snr = (gBallSnr")


def test_ball_angles_take_the_ball_tracks_rate_for_the_tdm_branch():
    ball = body("l3_considerBallTrack")
    assert (
        "float rateMps = l3_track_recent_rate(&gBallTrack.core) * gBallTrack.core.cfg.binWidthM;"
        in ball
    )
    # the measured lag-1 phase stays the rotor; the fitted rate is the radial velocity
    flat = " ".join(ball.split())
    assert "hit->dopplerPhaseRad, (rateMps != 0.0F) ? rateMps : newest.radialVelocityMps" in flat
    assert "continuousTdm" not in SOURCE


def test_hypothesis_angles_take_their_fitted_rate_for_the_tdm_branch():
    ball = body("l3_considerBallTrack")
    hyps = ball[ball.index("l3_ball_hyp_fit(") :]
    assert "hit->dopplerPhaseRad, radial, &snapshot);" in hyps


def test_no_launch_reset_names_the_removed_late_window():
    assert "lateFrom" not in SOURCE and "L3_LAUNCH_NO_LATE" not in SOURCE


def test_track_cfg_cal_and_elem_survive_trigger_cfg_and_sensor_start():
    """Only l3_ensureRadarCal initialises gRadarCal, and only
    when it has never been set; nothing else overwrites it."""
    assert SOURCE.count("l3_cal_identity(&gRadarCal") == 1
    ensure = body("l3_ensureRadarCal")
    assert "if (gRadarCal.virtualElements == 0U) {" in ensure
    for handler in ("l3_cli_triggerCfg", "l3_cli_sensorStart"):
        assert "gRadarCal =" not in body(handler)
        assert "memset(&gRadarCal" not in body(handler)


def test_a_fallback_fire_seeds_the_ball_tracker_with_the_balls_two_points():
    """After the fallback's late fire the ball is too smeared for the tracker to
    acquire; its own two points start the flight, right after the arm. The
    club's approach-end rule may fire on the same frame (20260809_114519): the
    fallback's points are the ball whichever rule dates impact."""
    self_trigger = body("l3_considerSelfTrigger")
    observe = self_trigger.index("l3_shotObserve(teeBin, fired, impactUs);")
    seed = self_trigger.index(
        "(void)l3_ball_track_seed(&gBallTrack, &gLeave.startTarget, &gLeave.stepTarget);"
    )
    guard = self_trigger.rindex("if (left && gBallTrack.armed) {", 0, seed)
    assert observe < guard < seed < self_trigger.index("if (!fired) {")


# --- the scan plan (l3_scan.h) -------------------------------------------------
#
# The board scores a bin in ~73 us and has 3 ms a frame; the detect task
# outranks the CLI and the trigger notices. Scoring the trigger region and then
# the whole window (~5.1 ms) starved them the moment the trigger was armed.


def test_globals_for_the_scan_plan():
    for declaration in (
        "static l3_scan_cfg_t       gScanCfg;",
        "static uint32_t            gMapCursor;",
        "static float               gLeaveFloor;",
        "static uint8_t             gDetectBehind;",
        "static uint32_t            gDetectShed;",
    ):
        assert declaration in SOURCE, declaration
    assert '#include "l3_scan.h"' in SOURCE
    assert "l3_scan_cfg_defaults(&gScanCfg);" in body("l3_clubTrackConfigure")
    assert "gLeaveFloor = 0.0F;" in body("l3_trigRearm")
    makefile = (FIRMWARE_DIR / "makefile").read_text(encoding="utf-8")
    assert " l3_scan.c " in makefile


def test_a_span_is_scored_once_per_frame_into_the_frames_local_bins():
    """The plan's global spans become the frame's local bins, and each bin is
    scored once through the span scorer both cores share (bit-map dedupe:
    host tested in test_iwr6843_firmware_dsp_score)."""
    score = body("l3_scoreSpans")
    assert "l3_dsp_spans_localize(frame->binStart, frame->binCount, spans, n, local);" in score
    assert "uint32_t scored[L3_DSP_BITMAP_WORDS] = { 0U, 0U };" in score
    mss = body("l3_mssScoreSpans")
    assert "l3_dsp_spans_score(local, n, frame->binCount, l3_mssScorer," in mss
    assert "l3_verticalResidual((const l3_detect_frame_t *)ctx, localBin, NULL, out);" in body(
        "l3_mssScorer"
    )


def test_the_band_is_placed_before_the_plan_and_the_trigger_reads_its_clipped_region():
    self_trigger = body("l3_considerSelfTrigger")
    place = self_trigger.index("l3_band_place(&gBandNoise,")
    plan = self_trigger.index(
        "l3_scan_pre(&gScanCfg, frame.binStart, windowCount, frame.binStart + first, count,"
    )
    stale = self_trigger.index("if (l3_detectFrameStale(&frame)) {")
    observe = self_trigger.index(
        "l3_trig_observe(&gTrig, gPreFramesCaptured, teeBin, region.first,"
    )
    helper = self_trigger.index("l3_preImpactClubTargets(obs, frame.binStart, &club, &leave,")
    assert place < plan < stale < observe < helper
    # Every scored bin is a span's; nothing scores the window.
    assert self_trigger.count("l3_verticalResidual(") == 0
    for k, span in enumerate(("region", "club", "leave", "chunk")):
        assert f"plan[{k}] = {span};" in self_trigger
    assert self_trigger.count("l3_scoreSpans(&frame, plan, 4U, obs);") == 1


def test_an_idle_frame_that_is_not_behind_refreshes_a_band_interior_chunk():
    self_trigger = body("l3_considerSelfTrigger")
    chunk = self_trigger.index("l3_scan_map_chunk(&gScanCfg, frame.binStart, windowCount, &gBand,")
    assert "if (!gClubTrack.active && !gDetectBehind) {" in self_trigger[chunk - 120 : chunk]


def test_band_off_scores_only_the_trigger_region():
    self_trigger = body("l3_considerSelfTrigger")
    off = self_trigger.index("gBand.valid = 0U;")
    region = self_trigger.index("region.first = frame.binStart + first;", off)
    assert "club = region;" in self_trigger[region : region + 200]


def test_behind_the_detect_task_sheds_the_ball_detector_and_the_map_chunk():
    """A newer frame landed before this one was taken: skip what the trigger's
    fire does not need, so the backlog drains and the CLI gets its time. The
    club's angle is no longer shed: it left the decision path (a queued
    snapshot, l3_angle_queue.h), and every point keeps it."""
    task = body("l3_detectTask")
    behind = task.index("gDetectBehind = ((uint32_t)(gPreFramesCaptured - epoch) >= 1U) ? 1U : 0U;")
    shed = task.index("if (!gDetectBehind) {", behind)
    ball = task.index("l3_considerBall(queuedSlot);", shed)
    trigger = task.index("l3_considerSelfTrigger(queuedSlot);", ball)
    assert behind < shed < ball < trigger
    assert "gDetectShed++;" in task
    self_trigger = body("l3_considerSelfTrigger")
    assert "if (!gClubTrack.active && !gDetectBehind) {" in self_trigger, "the map chunk"
    assert "gClubTrack.count > 1U && !gDetectBehind" not in self_trigger, "angles never shed"


def test_impact_freezes_the_post_floor_from_the_fallbacks_median():
    observe = body("l3_shotObserve")
    arm = observe.index("l3_ball_track_arm(&gBallTrack, &anchor,")
    freeze = observe.index("gBallFloor = (gLeaveFloor > 0.0F) ? gLeaveFloor : gTrig.floor;", arm)
    assert "if (gBand.valid) {" in observe[arm:freeze]


def test_after_impact_the_ball_tracker_scores_the_post_spans_against_the_frozen_floor():
    """The ball's span and the club's (l3_scan_post), merged so no bin is
    extracted twice, each scored once and extracted after the last one's
    targets; the whole window only without a band."""
    ball_track = body("l3_considerBallTrack")
    band = ball_track.index("if (gImpactFitCfg.bandBins > 0.0F && gBand.valid) {")
    post = ball_track.index("l3_scan_post(&gScanCfg, frame.binStart, count, &gBand,")
    assert "&ballSpan, &clubSpan);" in ball_track[post : post + 400]
    merge = ball_track.index("spans = l3_scan_merge(ballSpan, clubSpan, merged);", post)
    score = ball_track.index("l3_scoreSpans(&frame, merged, spans, obs);", merge)
    extract = ball_track.index(
        "found += l3_obs_extract(&params, frameIndex, gPostTimestampUs, merged[k].first,", score
    )
    assert "&targets[found], L3_OBS_MAX_TARGETS - found);" in ball_track[extract : extract + 300]
    whole = ball_track.index("l3_obs_floor_update(&gBallFloor,", extract)
    assert band < post < merge < score < extract < whole
    # The club track's prediction, as the ball's: from its last point and rate.
    assert "gClubTrack.active && gClubTrack.count > 0U" in ball_track[band:post]


def test_result_reconstructs_the_ball_once_before_building_the_result():
    flat = " ".join(SOURCE.split())
    gate = flat.index("if (gShot.state == L3_SHOT_RESULT && !gShotResultReady) {")
    fit = flat.index("l3_ball_track_reconstruct(&gBallTrack, &gLaunch);", gate)
    build = flat.index("l3_result_build(&gShot, &gBallTrack, &gLaunch,", gate)
    assert gate < fit < build


def test_the_ball_reconstruction_is_profiled():
    assert SOURCE.count("l3_profileStage(L3_PROF_RECONSTRUCT,") == 1


def test_the_per_frame_paths_do_not_reconstruct():
    """The ball's reconstruction has one call site (RESULT); the board never
    reconstructs the club and its frozen delivery is the unfiltered one."""
    assert SOURCE.count("l3_ball_track_reconstruct(") == 1
    assert "l3_track_kf_run(" not in SOURCE
    assert "l3_track_delivery_filtered(" not in SOURCE


def test_per_frame_launch_stops_once_the_result_is_ready():
    # l3_ball_track_launch resets gLaunch, wiping the angles RESULT reconstructed.
    ball_track = body("l3_considerBallTrack")
    assert re.search(
        r"if \(!gShotResultReady\) \{\s*\(void\)l3_ball_track_launch\(&gBallTrack, &gLaunch\);\s*\}",
        ball_track,
    )
    assert ball_track.index("l3_ball_track_launch(") < ball_track.index(
        "l3_ball_track_reconstruct("

    )


# --- the range window (l3_window.h, "captureCfg window") ------------------------


def test_the_window_ram_is_loaded_after_the_hwa_reset_and_before_the_paramsets():
    reset = SOURCE.index("errCode = HWA_reset(gHwaHandle);")
    load = SOURCE.index("errCode = HWA_configRam(gHwaHandle, HWA_RAM_TYPE_WINDOW_RAM,")
    ping = SOURCE.index("errCode = l3_configHwaProcessParam(L3_HWA_PARAM_FFT_PING,")
    pong = SOURCE.index("errCode = l3_configHwaProcessParam(L3_HWA_PARAM_FFT_PONG,")
    assert reset < load < ping < pong
    between = SOURCE[reset:load]
    assert "if (gRangeWindow == L3_RANGE_WINDOW_HANN) {" in between
    assert "l3_window_hann_q17(gRangeWindowCoeffs, N_SAMPLES);" in between
    assert "coeffs * (uint32_t)sizeof(int32_t), 0U);" in SOURCE[load : load + 200]


def test_the_live_paramsets_window_only_when_asked():
    process = body("l3_configHwaProcessParam")
    assert "fftMode.windowEn = (gRangeWindow != L3_RANGE_WINDOW_NONE)" in process
    assert "fftMode.winSymm = (gRangeWindow != L3_RANGE_WINDOW_NONE)" in process
    assert "? HWA_FFT_WINDOW_SYMMETRIC" in process
    assert "fftMode.windowStart = 0U;" in process
    assert "static uint8_t             gRangeWindow = L3_RANGE_WINDOW_NONE;" in SOURCE


def test_captureCfg_window_sets_it_while_stopped():
    window = body("l3_cli_captureCfgWindow")
    assert window.index("if (gCaptureActive) {") < window.index("gRangeWindow = window;")
    assert "l3_window_parse(argv[2], &window) != 0" in window
    dispatch = body("l3_cli_captureCfg")
    assert 'strcmp(argv[1], "window") == 0' in dispatch


def test_the_dump_says_which_window_it_was_recorded_with():
    match = re.search(r"int32_t l3_cli_dump\([^)]*\)\s*\{(.*?)\n\}", SOURCE, re.S)
    dump = match.group(1)
    assert "clutter.rangeWindow = gRangeWindow;" in dump
    assert dump.index("clutter.rangeWindow = gRangeWindow;") < dump.index(
        "UART_writePolling(gDataUart, (uint8_t *)&clutter, sizeof(clutter));"
    )


def test_the_clubs_claim_reaches_the_ball_history_after_the_club_follow():
    """The ball is updated before the club is followed, so its own call can
    only pass L3_TRACK_NO_TARGET: the club's claimed target is noted on the
    ball's history right after the follow (l3_ball_track_note_club), with the
    same frame number and the club's lastTargetIndex, as the replay does."""
    ball_track = body("l3_considerBallTrack")
    ball = ball_track.index("l3_ball_track_update_joint(&gBallTrack")
    club = ball_track.index("l3_track_follow(&gClubTrack")
    note = ball_track.index("l3_ball_track_note_club(&gBallTrack, frameIndex,")
    assert ball < club < note
    assert "gClubTrack.lastTargetIndex" in ball_track[note : note + 120]
    between = ball_track[club:note]
    assert "l3_ball_track_update_joint" not in between and between.count(";") == 1
