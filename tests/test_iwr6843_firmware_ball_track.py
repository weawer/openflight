"""Tests for the IWR6843 ball track, firmware/iwr6843/l3_ball_track.c.

A ball leaving the origin at a known velocity in the golf frame is observed
as (range, azimuth, elevation) per post-impact frame; the launch fit over the
earliest points must return that velocity: ball speed, horizontal launch
(positive right) and vertical launch (positive up), extrapolated to the
impact time. Decoys (the resting club at the origin, a slow mover, a target
short of the origin, an impossibly fast return) must be refused.
"""

from __future__ import annotations

import ctypes
import math

import pytest
from iwr6843_twotrack import TwoTracks, obs

from openflight.iwr6843 import firmware_host as fw

DEG = math.pi / 180.0
BIN_M = 6.0 / 128
FRAME_US = 3000
WHY = {name: index for index, name in enumerate(fw.BALL_TRACK_WHY_NAMES)}
ORIGIN_BIN = 47.0
ORIGIN = (ORIGIN_BIN * BIN_M, 0.0, 0.0)
IMPACT_US = 21_700


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


class Ball:
    def __init__(self, lib, **overrides):
        self.lib = lib
        cfg = fw.BallTrackCfg()
        lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
        for name, value in overrides.items():
            setattr(cfg, name, value)
        self.track = fw.BallTrack()
        # These tests' ORIGIN sits at antenna height (z = 0): anchor the tee there.
        cfg.fit.teeBallHeightM = cfg.fit.radarHeightM
        lib.l3_ball_track_init(ctypes.byref(self.track), ctypes.byref(cfg))

    def arm(self, origin_bin=ORIGIN_BIN, origin=ORIGIN, impact_us=IMPACT_US):
        self.lib.l3_ball_track_arm(
            ctypes.byref(self.track), origin_bin, ctypes.byref(fw.Vec3(*origin)), impact_us
        )

    def update(self, frame, targets, timestamp_us=None) -> bool:
        arr = (fw.TargetObs * max(1, len(targets)))(*targets)
        stamp = frame * FRAME_US if timestamp_us is None else timestamp_us
        return bool(
            self.lib.l3_ball_track_update(ctypes.byref(self.track), arr, len(targets), frame, stamp)
        )

    def set_angles(self, az, el, flags=fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION, confidence=1.0) -> int:
        return self.lib.l3_ball_track_set_angles(ctypes.byref(self.track), az, el, flags, confidence)

    def launch(self):
        """The per-frame launch, then the once-per-shot direction fit, as the board does."""
        out = fw.Launch()
        used = self.lib.l3_ball_track_launch(ctypes.byref(self.track), ctypes.byref(out))
        self.lib.l3_ball_track_reconstruct(ctypes.byref(self.track), ctypes.byref(out))
        return used, out

    @property
    def why(self) -> str:
        return fw.BALL_TRACK_WHY_NAMES[self.track.why]


def target(frame, range_bin, *, confidence=0.9, doppler=0.0) -> fw.TargetObs:
    t = fw.TargetObs()
    t.frame = frame
    t.timestampUs = frame * FRAME_US
    t.peakBin = int(round(range_bin))
    t.rangeBin = range_bin
    t.energy = 5000.0
    t.peak = 1250.0
    t.stat = t.peak
    t.snr = 20.0
    t.coherence = 0.9
    t.dopplerAliasMps = doppler
    t.confidence = confidence
    return t


def fly(
    lib, *, speed=60.0, hla_deg=0.0, vla_deg=12.0, frames=6, first_frame=8, angles=True, ball=None
):
    """A ball leaving ORIGIN at impact, seen once per frame from first_frame on.

    Impact is at IMPACT_US; frame f is at f * FRAME_US, so the first
    post-impact frame (8, 24 ms) sees the ball 2.3 ms into flight.
    """
    ball = ball or Ball(lib)
    ball.arm()
    vx = speed * math.cos(vla_deg * DEG) * math.cos(hla_deg * DEG)
    vy = speed * math.cos(vla_deg * DEG) * math.sin(hla_deg * DEG)
    vz = speed * math.sin(vla_deg * DEG)
    cal = ball.track.cfg.core.cal
    for frame in range(first_frame, first_frame + frames):
        s = (frame * FRAME_US - IMPACT_US) * 1e-6
        golf = fw.Vec3(ORIGIN[0] + vx * s, ORIGIN[1] + vy * s, ORIGIN[2] + vz * s)
        radar = fw.Vec3()
        lib.l3_frames_golf_to_radar(ctypes.byref(cal), ctypes.byref(golf), ctypes.byref(radar))
        sph = fw.Spherical()
        lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        appended = ball.update(frame, [target(frame, sph.rangeM / BIN_M)])
        assert appended, f"frame {frame} not appended ({ball.why})"
        if angles:
            assert ball.set_angles(sph.azimuthRad, sph.elevationRad) == 1
    return ball, (vx, vy, vz)


def test_defaults_are_a_wide_gate_a_fast_departure_and_six_launch_points(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.core.gateBins == pytest.approx(6.0) and cfg.core.maxMisses == 1
    assert cfg.minDepartureMps == pytest.approx(10.0) and cfg.maxSpeedMps == pytest.approx(100.0)
    assert cfg.originGateBins == pytest.approx(8.0) and cfg.launchPoints == 6
    assert cfg.minDepartureBins == pytest.approx(1.0) and cfg.snr == pytest.approx(1.0)


def test_unarmed_track_ignores_everything(lib):
    ball = Ball(lib)
    assert ball.update(1, [target(1, 50.0)]) is False
    assert ball.why == "unarmed" and ball.track.core.count == 0


def test_a_departing_ball_is_acquired_confirmed_and_tracked(lib):
    ball, _ = fly(lib, frames=5)
    assert ball.track.confirmed == 1 and ball.track.done == 0
    assert ball.track.core.count == 5
    assert ball.track.counters[WHY["acquired"]] == 1
    assert ball.track.counters[WHY["confirmed"]] == 1
    assert ball.track.counters[WHY["tracked"]] == 3
    assert ball.why == "tracked"


@pytest.mark.parametrize("hla_deg,vla_deg", [(0.0, 12.0), (3.0, 10.0), (-4.5, 15.0), (1.2, 25.0)])
def test_launch_recovers_speed_hla_and_vla_with_the_documented_signs(lib, hla_deg, vla_deg):
    ball, (vx, vy, vz) = fly(lib, speed=60.0, hla_deg=hla_deg, vla_deg=vla_deg)
    used, launch = ball.launch()
    assert used == 6 and launch.points == 6
    assert (launch.velocity.x, launch.velocity.y, launch.velocity.z) == pytest.approx(
        (vx, vy, vz), abs=0.4
    )
    assert launch.speedMps == pytest.approx(60.0, abs=0.4)
    assert launch.hlaRad / DEG == pytest.approx(hla_deg, abs=0.3)
    assert launch.vlaRad / DEG == pytest.approx(vla_deg, abs=0.3)
    assert launch.speedValid and launch.hlaValid and launch.vlaValid
    assert launch.confidence > 0.8, "six points are a full launch fit"


def test_launch_position_is_extrapolated_back_to_the_impact_time(lib):
    ball, _ = fly(lib)
    _, launch = ball.launch()
    assert (
        launch.launchPosition.x,
        launch.launchPosition.y,
        launch.launchPosition.z,
    ) == pytest.approx(ORIGIN, abs=0.01)


def test_launch_fits_the_earliest_points_not_the_newest(lib):
    """Drag takes speed off from the first metre; the launch reads the start."""
    ball = Ball(lib, launchPoints=4)
    ball.arm()
    speed_bins = 4.0  # bins per frame at first
    rng = ORIGIN_BIN + 1.0
    for frame in range(8, 18):
        rng += speed_bins
        speed_bins *= 0.93  # slowing down the flight
        assert ball.update(frame, [target(frame, rng)])
    used, launch = ball.launch()
    assert used == 4
    early = 4.0 * 0.93 * BIN_M / (FRAME_US * 1e-6)
    late = 4.0 * 0.93**9 * BIN_M / (FRAME_US * 1e-6)
    assert late < launch.radialSpeedMps < early * 1.02
    assert launch.radialSpeedMps == pytest.approx(early * 0.9, rel=0.08)


def test_range_only_flight_gives_speed_but_no_angles(lib):
    ball, _ = fly(lib, angles=False)
    used, launch = ball.launch()
    assert used == 6 and launch.speedValid and not launch.hlaValid and not launch.vlaValid
    # Without angles the radial speed is the whole measurement.
    assert launch.speedMps == pytest.approx(launch.radialSpeedMps, abs=1e-3)


def test_launch_needs_a_confirmed_flight_with_three_points(lib):
    ball = Ball(lib)
    assert ball.launch()[0] == 0
    ball.arm()
    ball.update(8, [target(8, 49.0)])
    assert ball.launch()[0] == 0, "acquired only"
    ball.update(9, [target(9, 53.0)])
    assert ball.track.confirmed == 1 and ball.launch()[0] == 0, "two points"
    ball.update(10, [target(10, 57.0)])
    assert ball.launch()[0] == 3


def test_targets_at_or_short_of_the_origin_are_never_the_ball(lib):
    """Recorded shots: the impact echo sits at the origin with the strongest
    return of the whole capture; the ball is already a bin or more out."""
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, 44.0), target(8, 40.0)]) is False
    assert ball.why == "nocandidate"
    assert ball.update(8, [target(8, ORIGIN_BIN + 0.4, confidence=0.99)]) is False
    assert ball.why == "nocandidate"
    assert ball.update(9, [target(9, ORIGIN_BIN + 1.2)]) is True


def test_targets_far_beyond_the_origin_cannot_start_a_track(lib):
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, ORIGIN_BIN + 9.0)]) is False
    assert ball.why == "nocandidate"
    assert ball.update(8, [target(8, ORIGIN_BIN + 7.0)]) is True


def test_the_resting_club_at_the_origin_is_excluded_and_a_slow_mover_dropped(lib):
    """The club at rest sits within a bin of the origin: never offered. A
    return a bin out that then crawls (a bin over two frames is 8 m/s) is
    acquired, coasts, and is dropped as too slow when it finally moves."""
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, 47.5)]) is False
    assert ball.why == "nocandidate" and ball.track.core.count == 0
    assert ball.update(9, [target(9, 48.2)]) is True and ball.why == "acquired"
    assert ball.update(10, [target(10, 48.3)]) is False and ball.why == "coasted"
    assert ball.update(11, [target(11, 49.3)]) is False
    assert ball.why == "tooslow" and ball.track.core.count == 0 and ball.track.confirmed == 0
    # The search restarts and a real departure is still taken.
    assert ball.update(12, [target(12, 50.0)]) is True and ball.why == "acquired"
    assert ball.update(13, [target(13, 54.0)]) is True and ball.why == "confirmed"


def test_an_impossibly_fast_return_is_dropped(lib):
    """The association gate bounds the jump first, so the ceiling is tested
    with a lower one: 5 bins in 3 ms is 78 m/s against a 60 m/s ceiling."""
    ball = Ball(lib, maxSpeedMps=60.0)
    ball.arm()
    ball.update(8, [target(8, 48.0)])
    assert ball.update(9, [target(9, 53.0)]) is False
    assert ball.why == "toofast" and ball.track.core.count == 0


def test_a_confirmed_flight_that_leaves_the_window_is_done(lib):
    ball, _ = fly(lib, frames=4)
    assert ball.update(12, []) is False and ball.why == "coasted"
    assert ball.update(13, []) is False and ball.why == "lost"
    assert ball.track.done == 1
    assert ball.update(14, [target(14, 80.0)]) is False and ball.why == "lost"
    # The launch is still readable from what was gathered.
    assert ball.launch()[0] == 4


def test_the_strongest_return_does_not_steal_a_confirmed_flight(lib):
    ball, _ = fly(lib, frames=3)
    last = ball.track.core.lastBin
    step = ball.track.core.velocityBinsPerFrame
    decoy = target(11, ORIGIN_BIN + 0.5, confidence=0.99)  # the club, still at the tee
    real = target(11, last + step, confidence=0.6)
    assert ball.update(11, [decoy, real]) is True
    assert ball.track.core.lastBin == pytest.approx(last + step)


def test_reset_and_rearm_forget_the_flight_but_keep_the_counters(lib):
    ball, _ = fly(lib, frames=4)
    lib.l3_ball_track_reset(ctypes.byref(ball.track))
    assert ball.track.armed == 0 and ball.track.core.count == 0 and ball.track.confirmed == 0
    assert ball.track.counters[WHY["acquired"]] == 1
    ball.arm(origin_bin=50.0, impact_us=99)
    assert (
        ball.track.armed == 1
        and ball.track.originBin == 50.0
        and ball.track.impactTimestampUs == 99
    )
    assert ball.track.cfg.launchPoints == 6


def test_why_names_and_formats(lib):
    for index, name in enumerate(fw.BALL_TRACK_WHY_NAMES):
        assert lib.l3_ball_track_why_name(index).decode() == name
    ball, _ = fly(lib, hla_deg=2.0, vla_deg=12.0)
    status = fw.c_text(lib.l3_ball_track_format_status, ctypes.byref(ball.track))
    assert status.startswith(
        "balltrack armed=1 confirmed=1 done=0 why=tracked count=6 origin=47.00"
    )
    assert f" impact={IMPACT_US} acq=1 slow=0 fast=0 lost=0" in status
    _, launch = ball.launch()
    text = fw.c_text(lib.l3_launch_format, ctypes.byref(launch))
    assert text.startswith("launch points=6 speed=60.") or text.startswith(
        "launch points=6 speed=59."
    )
    assert " hla=2." in text and " vla=12." in text and text.endswith(" why=ok valid=shv")
    empty = fw.Launch()
    assert fw.c_text(lib.l3_launch_format, ctypes.byref(empty)).endswith(" valid=none")


def test_the_departure_band_excludes_the_follow_through_behind_the_ball(lib):
    """Recorded shot 001: after impact the clubhead carried on at 15 m/s
    (bins 48.6, 49.9, 50.9) while the ball was at 51.7 then 54.1. Only
    candidates a bin or more beyond the origin, then beyond the first point,
    are offered, so the club is never the more confident choice."""
    ball = Ball(lib)
    ball.arm(origin_bin=49.0)
    assert (
        ball.update(10, [target(10, 48.6, confidence=0.64), target(10, 51.7, confidence=0.55)])
        is True
    )
    assert ball.track.core.lastBin == pytest.approx(51.7)
    assert (
        ball.update(11, [target(11, 48.4, confidence=0.86), target(11, 54.1, confidence=0.82)])
        is True
    )
    assert ball.why == "confirmed" and ball.track.core.lastBin == pytest.approx(54.1)
    # Once flying, nothing behind the last point is offered to the track, so
    # the static return at the origin cannot pull it back when the ball fades.
    assert (
        ball.update(12, [target(12, 49.9, confidence=0.84), target(12, 56.0, confidence=0.40)])
        is True
    )
    assert ball.track.core.lastBin == pytest.approx(56.0)
    assert ball.update(13, [target(13, 49.0, confidence=0.9)]) is False
    assert ball.why == "coasted"
    assert ball.update(14, [target(14, 61.2, confidence=0.3)]) is True
    assert ball.why == "tracked"


def test_scattered_launch_angles_fall_back_to_the_radial_speed(lib):
    """Shot 003 gave 40 m/s radial with azimuths that swung by 30 degrees; the
    launch must report the radial speed and no angles, not a 54 m/s ball at
    -27 degrees."""
    ball = Ball(lib)
    ball.arm()
    for frame, (b, az) in enumerate(
        [(50.7, 0.1), (53.3, -0.4), (55.8, 0.5), (58.4, -0.5), (61.1, 0.4), (63.7, -0.3)], start=9
    ):
        assert ball.update(frame, [target(frame, b)])
        ball.set_angles(az, 0.2)
    used, launch = ball.launch()
    assert used == 6 and launch.speedValid
    assert not launch.hlaValid and not launch.vlaValid
    assert launch.speedMps == pytest.approx(launch.radialSpeedMps)
    assert launch.speedMps == pytest.approx(2.6 * BIN_M / (FRAME_US * 1e-6), rel=0.05)


def test_the_confirmation_frame_offers_only_the_band_beyond_the_first_point(lib):
    """Recorded shot 002: one point at 52.2 predicts 52.2; the nearest candidate
    was a static return at 51.9 while the ball had moved on to 54.6. From a
    single point only departure can be predicted, so only candidates a bin or
    more beyond it are offered for confirmation."""
    ball = Ball(lib)
    ball.arm(origin_bin=49.0)
    assert (
        ball.update(9, [target(9, 48.3, confidence=0.95), target(9, 52.2, confidence=0.5)]) is True
    )
    assert (
        ball.update(
            10,
            [
                target(10, 48.4, confidence=0.9),
                target(10, 54.6, confidence=0.4),
                target(10, 51.9, confidence=0.3),
            ],
        )
        is True
    )
    assert ball.why == "confirmed" and ball.track.core.lastBin == pytest.approx(54.6)
    # A single point with nothing departing beyond it coasts rather than
    # confirming on the static return.
    ball = Ball(lib)
    ball.arm(origin_bin=49.0)
    ball.update(9, [target(9, 52.2)])
    assert ball.update(10, [target(10, 51.9), target(10, 52.4)]) is False
    assert ball.why == "coasted"


def test_the_appended_target_is_reported_by_its_index_into_the_caller_list(lib):
    ball = Ball(lib)
    assert ball.track.lastTargetIndex == fw.TRACK_NO_TARGET
    ball.arm(origin_bin=49.0)
    assert (
        ball.update(9, [target(9, 48.3, confidence=0.95), target(9, 52.2, confidence=0.5)]) is True
    )
    assert ball.track.lastTargetIndex == 1, "the caller's index, not the filtered candidate's"
    assert (
        ball.update(10, [target(10, 48.4), target(10, 51.9), target(10, 54.6, confidence=0.4)])
        is True
    )
    assert ball.track.lastTargetIndex == 2
    assert ball.update(11, []) is False
    assert ball.track.lastTargetIndex == fw.TRACK_NO_TARGET


# --- the hypothesis search (l3_ball_hyp.c) ------------------------------------


def hyp_track(lib, **overrides):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    cfg.useHypotheses = 1
    for name, value in overrides.items():
        setattr(cfg, name, value)
    track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(track), ctypes.byref(cfg))
    return track


def run_joint(lib, track, scene, on_frame=None):
    origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
    lib.l3_ball_track_arm(
        ctypes.byref(track), scene.origin_bin, ctypes.byref(origin), scene.gate_us
    )
    whys = []
    for f in scene.build():
        arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
        lib.l3_ball_track_update_joint(
            ctypes.byref(track), arr, len(f.targets), f.frame, f.timestamp_us, f.club_index
        )
        whys.append(fw.BALL_TRACK_WHY_NAMES[track.why])
        if on_frame is not None:
            on_frame(track, f)
    return whys


def core_bins(lib, track):
    out = []
    for i in range(track.core.count):
        p = fw.TrackPoint()
        lib.l3_track_point(ctypes.byref(track.core), i, ctypes.byref(p))
        out.append(round(p.rangeBin, 3))
    return out


def launch_of(lib, track):
    out = fw.Launch()
    used = lib.l3_ball_track_launch(ctypes.byref(track), ctypes.byref(out))
    return used, out


def test_the_hypothesis_search_is_off_by_default_until_evaluated(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.useHypotheses, cfg.skipClubClaim) == (0, 1)
    assert cfg.hyps.classifyPoints == 4


def test_the_hypotheses_share_the_core_geometry(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    cfg.core.binWidthM = 0.05
    cfg.core.velocitySpanMps = 12.0
    track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(track), ctypes.byref(cfg))
    assert (track.hyps.cfg.binWidthM, track.hyps.cfg.velocitySpanMps) == pytest.approx((0.05, 12.0))


def test_the_ball_track_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallTrack) == lib.l3_ball_track_struct_bytes()


def test_with_the_search_off_the_joint_update_is_todays_update(lib):
    scene = TwoTracks()
    a = hyp_track(lib, useHypotheses=0)
    run_joint(lib, a, scene)
    b = fw.BallTrack()
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    lib.l3_ball_track_init(ctypes.byref(b), ctypes.byref(cfg))
    origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
    lib.l3_ball_track_arm(ctypes.byref(b), scene.origin_bin, ctypes.byref(origin), scene.gate_us)
    for f in scene.build():
        arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
        lib.l3_ball_track_update(ctypes.byref(b), arr, len(f.targets), f.frame, f.timestamp_us)
    assert bytes(a.core) == bytes(b.core)
    assert list(a.counters) == list(b.counters)


def test_beside_the_club_the_ball_track_is_the_ball(lib):
    scene = TwoTracks()
    track = hyp_track(lib)
    whys = run_joint(lib, track, scene)
    assert whys[:3] == ["searching"] * 3
    assert whys[3] == "confirmed" and set(whys[4:]) == {"tracked"}
    frames = scene.build()
    assert core_bins(lib, track) == [round(f.ball_bin, 3) for f in frames]
    used, launch = launch_of(lib, track)
    assert used >= 3 and launch.speedMps == pytest.approx(42.0, rel=0.05)


def test_a_lone_ball_is_found_without_a_club_track(lib):
    track = hyp_track(lib)
    run_joint(lib, track, TwoTracks(club_visible=False))
    used, launch = launch_of(lib, track)
    assert track.confirmed and launch.speedMps == pytest.approx(42.0, rel=0.05)


def test_without_a_ball_there_is_no_launch(lib):
    scene = TwoTracks(missing_ball=tuple(range(1, 9)), extras=[(48.0, 20000.0, 0.8)])
    track = hyp_track(lib)
    whys = run_joint(lib, track, scene)
    assert set(whys) == {"searching"}
    assert not track.confirmed
    assert launch_of(lib, track)[0] == 0


def test_a_confirmed_ball_skips_the_club_claim_while_another_candidate_is_in_gate(lib):
    scene = TwoTracks(frames=5)
    track = hyp_track(lib)
    run_joint(lib, track, scene)
    assert track.confirmed
    step = 42.0 * 0.002 / BIN_M
    expected = 46.0 + step * 6
    club = obs(6, 12000, expected - 0.1, 9000.0, 40.0)
    ball = obs(6, 12000, expected + 0.6, 1500.0, 42.0)
    arr = (fw.TargetObs * 2)(club, ball)
    assert lib.l3_ball_track_update_joint(ctypes.byref(track), arr, 2, 6, 12000, 0) == 1
    assert track.lastTargetIndex == 1
    # The club's claim alone in the gate: the two share a bin, and it is taken.
    lone = obs(7, 14000, 46.0 + step * 7, 9000.0, 40.0)
    arr = (fw.TargetObs * 1)(lone)
    assert lib.l3_ball_track_update_joint(ctypes.byref(track), arr, 1, 7, 14000, 0) == 1


def test_angles_on_hypothesis_points_survive_adoption(lib):
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION

    def angle_every_new_point(track, _frame):
        for i in range(fw.BALL_HYP_MAX):
            if track.hyps.hyp[i].lastTargetIndex != fw.BALL_HYP_NONE:
                lib.l3_ball_hyps_set_angles(ctypes.byref(track.hyps), i, 0.02, 0.2, both, 1.0)

    track = hyp_track(lib)
    run_joint(lib, track, TwoTracks(frames=4), on_frame=angle_every_new_point)
    assert track.confirmed
    for i in range(3):  # the points adopted from earlier frames
        p = fw.TrackPoint()
        lib.l3_track_point(ctypes.byref(track.core), i, ctypes.byref(p))
        assert p.anglesValid == both


@pytest.mark.parametrize(("ball_mps", "missing"), [(95.0, ()), (60.0, (2,))])
def test_adoption_takes_every_classified_ball(lib, ball_mps, missing):
    """Review finding #3: the hypothesis's points were replayed through the
    core's frame-counted 6-bin gate, which refused a fast ball or one with a
    missed frame; a classified hypothesis is adopted on its classification frame."""
    track = hyp_track(lib)
    whys = run_joint(
        lib, track, TwoTracks(frame_us=3000, frames=8, ball_mps=ball_mps, missing_ball=missing)
    )
    assert track.confirmed
    assert whys.index("confirmed") == 3 + len(missing)  # the frame of the fourth point
    used, launch = launch_of(lib, track)
    assert used >= 3 and launch.speedMps == pytest.approx(ball_mps, rel=0.05)


def test_a_confirmed_ball_leaves_no_hypothesis_pointing_at_a_target(lib):
    """Review finding #4: once the ball is confirmed the hypotheses stop, so no
    stale index sends the board computing angles for the wrong target."""
    stale = []

    def after(track, _frame):
        if track.confirmed:
            stale.extend(
                track.hyps.hyp[i].lastTargetIndex
                for i in range(fw.BALL_HYP_MAX)
                if track.hyps.hyp[i].lastTargetIndex != fw.BALL_HYP_NONE
            )

    track = hyp_track(lib)
    run_joint(lib, track, TwoTracks(frames=8), on_frame=after)
    assert track.confirmed and stale == []


# --- the Pi detector's fastest-credible rule on the track -------------------------------------


def unclaimed(scene):
    """The scene's frames with no club claim: the club track lost the club."""
    frames = scene.build()
    for f in frames:
        f.club_index = fw.TRACK_NO_TARGET
    return frames


@pytest.mark.parametrize("decel", [300.0, 1500.0])
def test_fastest_credible_keeps_an_unclaimed_follow_through_off_the_ball(lib, decel):
    """20260927_183542_142_009: the club track was inactive after impact, so no
    claim separated the returns and the search adopted the follow-through."""
    scene = TwoTracks(frames=10, club_decel_mps2=decel, club_stat=1400.0)

    def launch_speed(**hyps):
        track = hyp_track(lib)
        for name, value in hyps.items():
            setattr(track.cfg.hyps, name, value)
            setattr(track.hyps.cfg, name, value)
        origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
        lib.l3_ball_track_arm(
            ctypes.byref(track), scene.origin_bin, ctypes.byref(origin), scene.gate_us
        )
        for f in unclaimed(scene):
            arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
            lib.l3_ball_track_update_joint(
                ctypes.byref(track), arr, len(f.targets), f.frame, f.timestamp_us, f.club_index
            )
        return launch_of(lib, track)[1].speedMps

    assert launch_speed() < 30.0, "without the rule the follow-through is the ball"
    assert launch_speed(fastBallMps=30.0) == pytest.approx(42.0, rel=0.1)


# --- late-flight launch angles (2026-09-29 spec) ------------------------------------


BOTH = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION


def test_points_without_angles_give_no_angles(lib):
    ball, _ = fly(lib, speed=45.0, vla_deg=14.0, frames=14, angles=False)
    _, launch = ball.launch()
    assert launch.speedValid and not launch.vlaValid and not launch.hlaValid


def test_scattered_angles_are_still_rejected(lib):
    ball, _ = fly(lib, speed=45.0, vla_deg=14.0, frames=14, angles=False)
    core = ball.track.core
    for index in range(core.count):
        jitter = 25.0 * DEG if index % 2 else -25.0 * DEG
        lib.l3_track_set_point_angles(ctypes.byref(core), index, jitter, jitter, BOTH, 1.0)
    _, launch = ball.launch()
    assert launch.speedValid and not launch.vlaValid


def test_the_per_frame_launch_is_speed_only(lib):
    """l3_ball_track_launch runs every post frame: it must not fit a direction."""
    ball, _ = fly(lib, speed=60.0, hla_deg=3.0, vla_deg=12.0)
    out = fw.Launch()
    assert lib.l3_ball_track_launch(ctypes.byref(ball.track), ctypes.byref(out)) == 6
    assert out.speedValid and not out.hlaValid and not out.vlaValid
    assert out.angleWhy == 0, "none: the direction fit has not run"


def test_reconstruct_sets_the_direction_the_velocity_and_the_launch_position(lib):
    ball, (vx, vy, vz) = fly(lib, speed=60.0, hla_deg=3.0, vla_deg=12.0, frames=8)
    _, launch = ball.launch()
    assert launch.hlaValid and launch.vlaValid
    assert fw.BALL_FIT_WHY_NAMES[launch.angleWhy] == "ok"
    assert launch.anglesAccepted == 8, "fly() sets every point's angles"
    assert launch.angleRmsRad == pytest.approx(0.0, abs=0.01)
    assert launch.hlaRad / DEG == pytest.approx(3.0, abs=0.3)
    assert launch.vlaRad / DEG == pytest.approx(12.0, abs=0.3)
    speed = launch.speedMps
    assert (launch.velocity.x, launch.velocity.y, launch.velocity.z) == pytest.approx(
        (vx / 60.0 * speed, vy / 60.0 * speed, vz / 60.0 * speed), abs=0.3
    )
    assert (launch.launchPosition.x, launch.launchPosition.y, launch.launchPosition.z) == pytest.approx(
        ORIGIN, abs=1e-3
    )


def test_reconstruct_on_an_unconfirmed_track_leaves_everything_unfiltered(lib):
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, ORIGIN_BIN + 3.0)])
    out = fw.Launch()
    assert lib.l3_ball_track_reconstruct(ctypes.byref(ball.track), ctypes.byref(out)) == 0
    assert out.angleWhy == 0 and not out.hlaValid
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(ball.track.core), 0, ctypes.byref(point))
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED


def test_the_launch_line_reports_the_direction_fit(lib):
    ball, _ = fly(lib, speed=60.0, hla_deg=0.0, vla_deg=12.0, frames=8)
    _, launch = ball.launch()
    text = fw.c_text(lib.l3_launch_format, ctypes.byref(launch))
    assert " angles=8 rms=" in text and " why=ok valid=shv" in text
    assert "late=" not in text


def test_defaults_carry_the_direction_fit_constants(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    reference = fw.BallFitCfg()
    lib.l3_ball_fit_cfg_defaults(ctypes.byref(reference))
    assert bytes(cfg.fit) == bytes(reference)


# --- seeded from the ball-leave fallback ---------------------------------------
#
# The fallback (l3_leave.c) fires about two frames after launch, having seen
# the ball step outward at a ball's speed. By the first post frame the ball is
# 4-6 bins out and too smeared (confidence 0.0-0.17) for the tracker to start
# on it, so its own two points start the flight instead.


def seed(ball, first, second) -> int:
    return ball.lib.l3_ball_track_seed(
        ctypes.byref(ball.track), ctypes.byref(first), ctypes.byref(second)
    )


def test_a_seeded_flight_is_confirmed_and_tracking_carries_on_at_any_confidence(lib):
    ball = Ball(lib)
    ball.arm()
    # 2.8 bins a 3 ms frame: ~44 m/s.
    assert seed(ball, target(7, 50.0, confidence=0.02), target(8, 52.8, confidence=0.0)) == 1
    assert ball.track.confirmed == 1 and ball.track.core.count == 2
    assert ball.why == "confirmed"
    assert ball.update(9, [target(9, 55.6, confidence=0.0)])
    assert ball.why == "tracked" and ball.track.core.count == 3


def test_a_seeded_flight_gives_a_launch_from_the_seed_on(lib):
    ball = Ball(lib)
    ball.arm()
    seed(ball, target(7, 50.0), target(8, 52.8))
    ball.update(9, [target(9, 55.6)])
    used, launch = ball.launch()
    assert used == 3
    assert launch.speedMps == pytest.approx(2.8 * BIN_M / (FRAME_US * 1e-6), rel=0.02)


def test_seeding_needs_an_armed_track(lib):
    ball = Ball(lib)
    assert seed(ball, target(7, 50.0), target(8, 52.8)) == 0
    assert ball.track.core.count == 0 and ball.track.confirmed == 0


@pytest.mark.parametrize(("second_bin", "why"), [(50.1, "tooslow"), (50.0 + 7.0, "toofast")])
def test_a_seed_still_passes_the_departure_speed_checks(lib, second_bin, why):
    """0.1 bins a frame is ~1.6 m/s; 7 bins a frame ~109 m/s."""
    ball = Ball(lib)
    ball.arm()
    assert seed(ball, target(7, 50.0), target(8, second_bin)) == 0
    assert ball.why == why
    assert ball.track.core.count == 0 and ball.track.confirmed == 0


def test_seeding_a_confirmed_flight_does_nothing(lib):
    ball, _ = fly(lib, frames=3)
    count = ball.track.core.count
    assert seed(ball, target(20, 60.0), target(21, 62.8)) == 0
    assert ball.track.core.count == count


# --- a smeared ball starts a flight --------------------------------------------
#
# A departing ball moves a few bins within a frame, so its return is smeared:
# on the labelled swings its first points after the fire read confidence
# 0.04-0.13, under the core's 0.2, and the club's follow-through (0.9) was
# taken as the ball a few frames later (20260809_110338, 20260824_111428,
# ~21 m/s reported for ~46). The range rate the second point must show is
# what refuses slow things; the ball tracker's first point needs little
# confidence.


def test_the_ball_trackers_first_point_needs_little_confidence(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.core.minConfidence == pytest.approx(0.05)
    club = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(club))
    assert club.minConfidence == pytest.approx(0.2), "the club tracker is unchanged"


def test_a_smeared_departing_ball_is_acquired_and_confirmed(lib):
    ball = Ball(lib)
    ball.arm()
    assert ball.update(8, [target(8, 49.5, confidence=0.06)])
    assert ball.why == "acquired"
    assert ball.update(9, [target(9, 52.3, confidence=0.04)])
    assert ball.why == "confirmed"


def test_a_smeared_slow_return_never_becomes_a_flight(lib):
    """Creeping 0.3 bins a frame (~5 m/s): never offered past the departure
    band beyond its first point, it coasts and is dropped, never confirmed."""
    ball = Ball(lib)
    ball.arm()
    for frame in range(8, 14):
        ball.update(frame, [target(frame, 49.5 + 0.3 * (frame - 8), confidence=0.06)])
        assert ball.track.confirmed == 0, frame


def test_noise_under_the_floor_confidence_does_not_start_a_flight(lib):
    ball = Ball(lib)
    ball.arm()
    assert not ball.update(8, [target(8, 49.5, confidence=0.02)])
    assert ball.why == "nocandidate"


def test_a_confident_departure_displaces_an_unconfirmed_smeared_first_point(lib):
    """20260916_184748: a stray 0.11 return at 53.8 was taken on frame 9; the
    ball (51.6, 0.9) arrived behind it on frame 10, was never offered, and had
    left the gate when the stray failed to confirm, so the club was taken
    (20 m/s for a 33 m/s ball). An unconfirmed first point gives way to a
    confident return in the origin gate."""
    ball = Ball(lib)
    ball.arm(origin_bin=49.0)
    assert ball.update(9, [target(9, 53.8, confidence=0.11)])
    assert ball.update(10, [target(10, 51.6, confidence=0.9)])
    assert ball.why == "acquired" and ball.track.core.count == 1
    assert ball.track.core.lastBin == pytest.approx(51.6)
    assert ball.update(11, [target(11, 54.1, confidence=0.95)])
    assert ball.why == "confirmed"


def test_a_confirming_point_is_taken_before_any_displacement(lib):
    """The smeared ball confirmed on its next frame keeps the flight, even with
    a confident return (the club's follow-through) in the origin gate."""
    ball = Ball(lib)
    ball.arm(origin_bin=46.0)
    ball.update(10, [target(10, 50.5, confidence=0.06)])
    assert ball.update(11, [target(11, 47.6, confidence=0.92), target(11, 53.3, confidence=0.04)])
    assert ball.why == "confirmed" and ball.track.core.lastBin == pytest.approx(53.3)


def test_a_weak_return_does_not_displace_a_first_point(lib):
    ball = Ball(lib)
    ball.arm(origin_bin=49.0)
    ball.update(9, [target(9, 53.8, confidence=0.11)])
    ball.update(10, [target(10, 51.6, confidence=0.15)])
    assert ball.track.core.count == 1 and ball.track.core.lastBin == pytest.approx(53.8)


def test_the_displacing_confidence_is_the_club_trackers(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.displaceConfidence == pytest.approx(0.2)


def test_an_adopted_hypothesis_keeps_its_points_angle_confidence(lib):
    """Adoption re-seeds the core from the hypothesis's points; each keeps the
    confidence its angles were measured with, or the direction fit would drop them."""
    ball = Ball(lib, useHypotheses=1)
    ball.arm()
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
    rng = ORIGIN_BIN + 2.0
    for frame in range(8, 16):
        rng += 3.0
        arr = (fw.TargetObs * 1)(target(frame, rng, doppler=12.0))
        lib.l3_ball_track_update_joint(ctypes.byref(ball.track), arr, 1, frame, frame * FRAME_US, 0xFFFFFFFF)
        for i in range(fw.BALL_HYP_MAX):
            lib.l3_ball_hyps_set_angles(ctypes.byref(ball.track.hyps), i, 0.02, 0.2, both, 0.61)
        if ball.track.confirmed:
            break
    assert ball.track.confirmed, "the hypothesis search never adopted the ball"
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(ball.track.core), 0, ctypes.byref(point))
    assert point.anglesValid == both
    assert point.angleConfidence == pytest.approx(0.61)


def test_a_launch_call_after_reconstruct_resets_the_angles(lib):
    """The contract the board relies on: l3_ball_track_launch rebuilds the whole
    l3_launch_t, so callers must not call it again once reconstruct has run
    (l3_dump.c guards the per-frame call with gShotResultReady)."""
    ball, _ = fly(lib, speed=60.0, hla_deg=3.0, vla_deg=12.0, frames=8)
    _, launch = ball.launch()
    assert launch.hlaValid and launch.vlaValid
    lib.l3_ball_track_launch(ctypes.byref(ball.track), ctypes.byref(launch))
    assert not launch.hlaValid and not launch.vlaValid
    assert launch.angleWhy == 0 and launch.anglesAccepted == 0
