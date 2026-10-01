"""Tests for the IWR6843 ball hypotheses, firmware/iwr6843/l3_ball_hyp.c.

After impact the tracker keeps up to four candidate ball trajectories that
start near the origin, assigns each frame's targets to them jointly with the
club track's claim (never the claimed target), and decides later which one is
the ball. Scenes come from tests/iwr6843_twotrack.py.
"""

from __future__ import annotations

import ctypes

import pytest
from iwr6843_twotrack import BIN_M, NO_CLAIM, TwoTracks, obs

from openflight.iwr6843 import firmware_host as fw


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def make_hyps(lib, **overrides):
    cfg = fw.BallHypsCfg()
    lib.l3_ball_hyps_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    hyps = fw.BallHyps()
    lib.l3_ball_hyps_init(ctypes.byref(hyps), ctypes.byref(cfg))
    return hyps


def arm(lib, hyps, origin_bin=46.0, gate_us=0):
    lib.l3_ball_hyps_arm(ctypes.byref(hyps), origin_bin, gate_us)


def feed(lib, hyps, frame, timestamp_us, targets, club_index=NO_CLAIM):
    arr = (fw.TargetObs * max(1, len(targets)))(*targets)
    return lib.l3_ball_hyps_update(
        ctypes.byref(hyps), arr, len(targets), frame, timestamp_us, club_index
    )


def run(lib, scene, **overrides):
    hyps = make_hyps(lib, **overrides)
    arm(lib, hyps, scene.origin_bin, scene.gate_us)
    frames = scene.build()
    for f in frames:
        feed(lib, hyps, f.frame, f.timestamp_us, f.targets, f.club_index)
    return hyps, frames


def active(hyps):
    return [hyps.hyp[i] for i in range(fw.BALL_HYP_MAX) if hyps.hyp[i].active]


def bins(hyp):
    return [round(hyp.points[i].rangeBin, 3) for i in range(hyp.count)]


def truth(frames):
    return [round(f.ball_bin, 3) for f in frames if f.ball_bin is not None]


def test_defaults(lib):
    cfg = fw.BallHypsCfg()
    lib.l3_ball_hyps_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.spawnBehindBins, cfg.spawnBeyondBins, cfg.gateBins, cfg.gateMps) == (
        1.0,
        10.0,
        1.5,
        8.0,
    )
    assert (cfg.maxMisses, cfg.classifyPoints, cfg.impactToleranceUs) == (2, 4, 15000)
    assert (cfg.minDepartureMps, cfg.maxSpeedMps) == (10.0, 100.0)
    assert (cfg.maxResidualBins, cfg.dopplerToleranceMps) == (1.0, 2.5)
    assert cfg.binWidthM == pytest.approx(BIN_M)


def test_the_struct_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallHyps) == lib.l3_ball_hyps_struct_bytes()


def test_unarmed_hypotheses_ignore_targets(lib):
    hyps = make_hyps(lib)
    assert feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.0, 1000.0, 40.0)]) == 0
    assert not active(hyps)


def test_the_ball_is_one_hypothesis_and_the_club_claim_none(lib):
    hyps, frames = run(lib, TwoTracks())
    got = active(hyps)
    assert len(got) == 1
    assert bins(got[0]) == truth(frames)
    assert hyps.spawned == 1


def test_only_targets_near_the_origin_start_a_hypothesis(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    targets = [obs(1, 0, b, 900.0, 5.0) for b in (44.5, 45.2, 55.8, 56.4)]
    feed(lib, hyps, 1, 0, targets)  # at the gate time: origin - 1 .. origin + 10
    assert sorted(h.points[0].rangeBin for h in active(hyps)) == pytest.approx([45.2, 55.8])


def test_the_start_band_moves_out_with_the_time_since_the_gate(lib):
    """2 ms after the gate a 100 m/s ball can be 4.27 bins further out."""
    hyps = make_hyps(lib)
    arm(lib, hyps)
    targets = [obs(1, 2000, b, 900.0, 5.0) for b in (56.4, 60.1, 60.5)]
    feed(lib, hyps, 1, 2000, targets)
    assert sorted(h.points[0].rangeBin for h in active(hyps)) == pytest.approx([56.4, 60.1])


def test_a_merged_first_return_is_a_missed_frame_not_a_point(lib):
    hyps, frames = run(lib, TwoTracks(merged=(1,)))
    (hyp,) = active(hyps)
    assert hyp.points[0].frame == 2
    assert bins(hyp) == truth(frames)


def test_the_ball_coasts_over_two_missing_frames_and_is_picked_up(lib):
    hyps, frames = run(lib, TwoTracks(missing_ball=(3, 4)))
    (hyp,) = active(hyps)
    assert [hyp.points[i].frame for i in range(hyp.count)] == [1, 2, 5, 6, 7, 8]
    assert bins(hyp) == truth(frames)


def test_three_missing_frames_drop_it_and_the_ball_starts_again(lib):
    hyps, frames = run(lib, TwoTracks(missing_ball=(3, 4, 5)))
    assert hyps.dropped == 1
    (hyp,) = active(hyps)  # a new hypothesis from frame 6, inside the widened band
    assert hyp.points[0].frame == 6
    assert bins(hyp) == truth([f for f in frames if f.frame >= 6])


def test_a_ball_return_the_club_claims_is_never_a_ball_point(lib):
    """Filling a gap with the club's return is the failure this module exists to stop."""
    hyps = make_hyps(lib)
    arm(lib, hyps)
    feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.8, 1500.0, 42.0)])
    feed(lib, hyps, 2, 4000, [obs(2, 4000, 49.6, 1500.0, 42.0)])
    feed(lib, hyps, 3, 6000, [obs(3, 6000, 51.4, 9000.0, 42.0)], club_index=0)
    (hyp,) = active(hyps)
    assert (hyp.count, hyp.misses) == (2, 1)


def test_a_full_set_evicts_only_a_single_point_hypothesis(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    first = [obs(1, 2000, b, 1000.0, 30.0) for b in (46.2, 50.0, 53.0, 55.8, 47.5)]
    feed(lib, hyps, 1, 2000, first)  # four slots: the fifth target starts nothing
    assert sorted(round(h.points[0].rangeBin, 1) for h in active(hyps)) == [46.2, 50.0, 53.0, 55.8]
    # 0.5 ms later: 46.2 and 50.0 continue; a new return at 45.3 evicts the
    # oldest single-point hypothesis that missed (53.0), never a two-point one.
    second = [obs(2, 2500, b, 1000.0, 30.0) for b in (46.9, 50.7, 45.3)]
    feed(lib, hyps, 2, 2500, second)
    got = {round(h.points[0].rangeBin, 1): h.count for h in active(hyps)}
    assert got == {46.2: 2, 50.0: 2, 55.8: 1, 45.3: 1}
    assert hyps.dropped == 1


def test_uneven_frame_spacing_is_followed_by_time(lib):
    hyps, frames = run(lib, TwoTracks(timestamps_us=[1000, 2500, 6500, 8000, 12000], frames=5))
    (hyp,) = active(hyps)
    assert bins(hyp) == truth(frames)


def test_set_angles_marks_the_newest_point_this_frame_only(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.8, 1500.0, 42.0)])
    index = next(i for i in range(fw.BALL_HYP_MAX) if hyps.hyp[i].active)
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), index, 0.1, 0.2, both, 1.0) == 1
    point = hyps.hyp[index].points[0]
    assert (point.azimuthRad, point.elevationRad) == pytest.approx((0.1, 0.2))
    assert point.anglesValid == both
    feed(lib, hyps, 2, 4000, [])  # nothing appended this frame
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), index, 0.3, 0.3, both, 1.0) == 0
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), fw.BALL_HYP_MAX, 0.3, 0.3, both, 1.0) == 0


def test_the_fit_reads_the_rate_and_the_range_at_a_reference_time(lib):
    hyps, _ = run(lib, TwoTracks(frames=4))
    (hyp,) = active(hyps)
    rate, at, residual = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
    assert lib.l3_ball_hyp_fit(
        ctypes.byref(hyp), 0, ctypes.byref(rate), ctypes.byref(at), ctypes.byref(residual)
    )
    assert rate.value * BIN_M == pytest.approx(42.0, rel=1e-3)
    assert at.value == pytest.approx(46.0, abs=0.01)  # the origin at the gate time
    assert residual.value == pytest.approx(0.0, abs=1e-3)


def verdict(lib, hyps):
    out = fw.BallHypVerdict()
    lib.l3_ball_hyps_classify(ctypes.byref(hyps), ctypes.byref(out))
    return out


def test_no_verdict_before_four_points(lib):
    hyps, _ = run(lib, TwoTracks(frames=3))
    assert verdict(lib, hyps).index == -1


def test_the_ball_hypothesis_is_classified_with_its_speed(lib):
    hyps, _ = run(lib, TwoTracks(frames=6))
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert bins(hyps.hyp[v.index])[0] == pytest.approx(46.0 + 42.0 * 0.002 / BIN_M, abs=0.01)
    assert v.points == 6
    assert v.rateMps == pytest.approx(42.0, rel=0.02)
    assert (v.weakerFraction, v.dopplerAgreement) == (1.0, 1.0)
    assert abs(v.originOffsetUs) < 200.0


def test_a_stationary_return_near_the_origin_is_never_the_ball(lib):
    """The strong stall beside the ball (2026-08-24: bins 38.2-38.4, SNR up to 970)."""
    scene = TwoTracks(missing_ball=tuple(range(1, 9)), extras=[(48.0, 20000.0, 0.8)])
    hyps, _ = run(lib, scene)
    assert active(hyps)  # it is followed as a hypothesis ...
    assert verdict(lib, hyps).index == -1  # ... but it never leaves: not the ball


@pytest.mark.parametrize("offset_us", [-6000, 6000])
def test_the_gate_need_not_be_the_exact_impact(lib, offset_us):
    hyps, _ = run(lib, TwoTracks(frames=10, impact_offset_us=offset_us))
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert v.originOffsetUs == pytest.approx(offset_us, abs=300.0)


def test_a_ball_leaving_far_from_the_gate_time_is_not_the_ball(lib):
    hyps, _ = run(lib, TwoTracks(frames=24, impact_offset_us=30000))
    assert active(hyps)
    assert verdict(lib, hyps).index == -1


def test_the_tighter_of_two_ball_like_hypotheses_wins(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    step = 42.0 * 0.002 / BIN_M
    jitter = [0.0, 0.6, -0.6, 0.6, -0.6, 0.6]
    for k in range(1, 7):
        ts = 2000 * k
        clean = obs(k, ts, 46.0 + step * k, 1500.0, 42.0)
        noisy = obs(k, ts, 50.0 + step * k + jitter[k - 1], 1500.0, 42.0)
        feed(lib, hyps, k, ts, [clean, noisy])
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert bins(hyps.hyp[v.index])[0] == pytest.approx(46.0 + step, abs=0.01)


def test_unarmed_hypotheses_give_no_verdict(lib):
    assert verdict(lib, make_hyps(lib)).index == -1


@pytest.mark.parametrize(("frame_us", "late_frames"), [(3000, 3), (6000, 2)])
def test_a_late_gate_still_finds_a_fast_ball(lib, frame_us, late_frames):
    """Review finding #2: the gate fired after impact, so by the first post
    frame a 70 m/s ball is already past a fixed start band; the band must
    widen with the time since the gate."""
    scene = TwoTracks(
        frames=10, frame_us=frame_us, ball_mps=70.0, impact_offset_us=-late_frames * frame_us
    )
    hyps, _ = run(lib, scene)
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert v.rateMps == pytest.approx(70.0, rel=0.03)


# --- the Pi detector's rules: fastest credible and the far window -------------


def test_the_pi_detector_rules_are_off_by_default(lib):
    cfg = fw.BallHypsCfg()
    lib.l3_ball_hyps_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.fastBallMps, cfg.farWindowBins) == (0.0, 0.0)
    assert cfg.fastSupportFraction == pytest.approx(0.55)


SLOW_MPS = 25.0  # the club's follow-through or the flying tee: clean and slow
FAST_MPS = 42.0  # the ball: a little ragged
FAST_JITTER = [0.0, 0.4, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4]


def slow_and_fast(lib, *, slow_frames=range(1, 7), fast_frames=range(1, 7), **overrides):
    """A clean slow mover from the origin beside a ragged fast one 4 bins out.

    Neither is the club's claim, so neither is weaker than it: on score alone
    the slow, tighter line wins -- the Pi detector's 2026-07-14 failure."""
    hyps = make_hyps(lib, **overrides)
    arm(lib, hyps)
    slow_step = SLOW_MPS * 0.002 / BIN_M
    fast_step = FAST_MPS * 0.002 / BIN_M
    for k in range(1, max(max(slow_frames), max(fast_frames)) + 1):
        ts = 2000 * k
        targets = []
        if k in slow_frames:
            targets.append(obs(k, ts, 46.0 + slow_step * k, 1500.0, SLOW_MPS))
        if k in fast_frames:
            targets.append(obs(k, ts, 50.0 + fast_step * k + FAST_JITTER[k - 1], 1500.0, FAST_MPS))
        feed(lib, hyps, k, ts, targets)
    return hyps


def test_on_score_alone_the_slow_clean_line_wins(lib):
    v = verdict(lib, slow_and_fast(lib))
    assert v.index >= 0 and v.rateMps == pytest.approx(SLOW_MPS, rel=0.05)
    assert v.waitingForFast == 0


def test_fastest_credible_takes_the_fast_line_with_enough_support(lib):
    v = verdict(lib, slow_and_fast(lib, fastBallMps=30.0))
    assert v.index >= 0 and v.rateMps == pytest.approx(FAST_MPS, rel=0.05)
    assert v.points == 6


def test_a_winner_already_fast_enough_is_kept(lib):
    """The rule only overrides a winner slower than the floor."""
    v = verdict(lib, slow_and_fast(lib, fastBallMps=20.0))
    assert v.rateMps == pytest.approx(SLOW_MPS, rel=0.05)


def test_a_fast_line_without_enough_support_does_not_win(lib):
    hyps = slow_and_fast(lib, slow_frames=range(1, 9), fast_frames=range(5, 9), fastBallMps=30.0)
    v = verdict(lib, hyps)  # 4 fast points < 0.55 x 8 slow points
    assert v.rateMps == pytest.approx(SLOW_MPS, rel=0.05)
    hyps = slow_and_fast(
        lib,
        slow_frames=range(1, 9),
        fast_frames=range(5, 9),
        fastBallMps=30.0,
        fastSupportFraction=0.5,
    )
    assert verdict(lib, hyps).rateMps > 35.0  # the fast line (4 ragged points: rate ~46)


def test_a_slow_winner_waits_for_a_fast_line_still_gathering_points(lib):
    hyps = slow_and_fast(lib, slow_frames=range(1, 5), fast_frames=range(2, 5), fastBallMps=30.0)
    v = verdict(lib, hyps)  # slow: 4 points, classifiable; fast: 3, not yet
    assert (v.index, v.waitingForFast) == (-1, 1)
    off = slow_and_fast(lib, slow_frames=range(1, 5), fast_frames=range(2, 5))
    assert verdict(lib, off).rateMps == pytest.approx(SLOW_MPS, rel=0.05)


def test_the_wait_ends_when_the_fast_line_coasts_out(lib):
    hyps = slow_and_fast(lib, slow_frames=range(1, 9), fast_frames=range(2, 5), fastBallMps=30.0)
    v = verdict(lib, hyps)  # the fast line missed frames 5-7 and was dropped
    assert v.index >= 0 and v.waitingForFast == 0
    assert v.rateMps == pytest.approx(SLOW_MPS, rel=0.05)


def test_the_far_window_keeps_near_returns_out_of_the_search(lib):
    """A stall beside the ball (the hand or the resting club) starts no
    hypothesis inside the far window; the ball is picked up once clear."""
    scene = TwoTracks(frames=8, club_visible=False, extras=[(47.5, 20000.0, 0.8)])
    hyps, frames = run(lib, scene)
    assert any(h.points[0].rangeBin == pytest.approx(47.5) for h in active(hyps))
    hyps, frames = run(lib, scene, farWindowBins=3.0)
    for hyp in active(hyps):
        assert min(bins(hyp)) >= 46.0 + 3.0
    v = verdict(lib, hyps)
    assert v.index >= 0 and v.rateMps == pytest.approx(42.0, rel=0.03)
    assert bins(hyps.hyp[v.index]) == [b for b in truth(frames) if b >= 49.0][-8:]
