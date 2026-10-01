"""Tests for the IWR6843 club track, firmware/iwr6843/l3_club_track.c.

Built with the host C compiler (openflight.iwr6843.firmware_host, which
also holds the ctypes mirrors) and driven through ctypes with synthetic
target lists: a clubhead closing on the ball at a steady bins-per-frame rate,
decoys the association must ignore, gaps to coast over, and a range-over-time
fit that yields the club's radial speed. Bins are global range-FFT bins.
"""

from __future__ import annotations

import ctypes
import math

import pytest

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.firmware_host import (
    ANGLE_AZIMUTH,
    ANGLE_ELEVATION,
    TRACK_NO_TARGET,
    TRACK_POINTS as POINTS,
    TRACK_WHY_NAMES as WHY,
    ClubTrack as Track,
    Delivery,
    TargetObs as Target,
    TrackCfg as Cfg,
    TrackPoint as Point,
    build_firmware_library,
    host_compiler,
)

BIN_M = 6.0 / 128
FRAME_US = 3000


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def target(
    frame: int,
    range_bin: float,
    *,
    confidence: float = 0.9,
    doppler: float = 3.0,  # a mover; acquisition prefers |Doppler| >= 1 m/s
    energy: float = 5000.0,
    snr: float = 20.0,
) -> Target:
    t = Target()
    t.frame = frame
    t.timestampUs = frame * FRAME_US
    t.peakBin = int(round(range_bin))
    t.rangeBin = range_bin
    t.energy = energy
    t.peak = energy / 4
    t.stat = t.peak
    t.snr = snr
    t.coherence = 0.9
    t.dopplerAliasMps = doppler
    t.confidence = confidence
    return t


class Tracker:
    def __init__(self, lib, **overrides):
        self.lib = lib
        self.cfg = Cfg()
        lib.l3_track_cfg_defaults(ctypes.byref(self.cfg))
        for name, value in overrides.items():
            setattr(self.cfg, name, value)
        self.track = Track()
        lib.l3_track_init(ctypes.byref(self.track), ctypes.byref(self.cfg))

    def update(self, frame: int, targets: list[Target]) -> bool:
        arr = (Target * max(1, len(targets)))(*targets)
        return bool(
            self.lib.l3_track_update(
                ctypes.byref(self.track), arr, len(targets), frame, frame * FRAME_US
            )
        )

    def points(self) -> list[Point]:
        out = []
        for i in range(self.track.count):
            p = Point()
            assert self.lib.l3_track_point(ctypes.byref(self.track), i, ctypes.byref(p)) == 1
            out.append(p)
        return out

    def why(self) -> str:
        return self.lib.l3_track_why_name(self.track.why).decode()

    def fit(self, max_points: int = 8):
        slope = ctypes.c_float()
        residual = ctypes.c_float()
        used = self.lib.l3_track_fit(
            ctypes.byref(self.track), max_points, ctypes.byref(slope), ctypes.byref(residual)
        )
        return used, slope.value, residual.value

    def status(self, dest: int = 48) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_track_format_status(ctypes.byref(self.track), dest, buf, len(buf))
        return buf.value.decode()


def test_defaults_describe_the_wide_profile(lib):
    tr = Tracker(lib)
    assert tr.cfg.binWidthM == pytest.approx(BIN_M)
    assert (tr.cfg.gateBins, tr.cfg.maxMisses) == (3.0, 2)
    assert tr.cfg.velocitySpanMps == pytest.approx(2 * 0.00484 / (4 * 135e-6), rel=0.01)


def test_a_steady_approach_becomes_one_continuous_track(lib):
    """Bins 22, 25, 27, 30, 33 ... 48: the trajectory the ten captures showed."""
    tr = Tracker(lib)
    path = [22, 25, 27, 30, 33, 35, 38, 40, 43, 45, 46, 47, 48]
    for frame, b in enumerate(path, start=1):
        assert tr.update(frame, [target(frame, float(b))]) is True
    assert tr.track.active == 1 and tr.track.count == len(path)
    assert tr.why() == "associated"
    assert [round(p.rangeBin) for p in tr.points()] == path
    assert tr.track.counters[WHY.index("acquired")] == 1, "acquired once, never re-acquired"


def test_association_follows_the_prediction_not_the_strongest_return(lib):
    """The hands light up 12 bins behind the club, stronger and more confident."""
    tr = Tracker(lib)
    for frame, b in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b))])
    decoy = target(4, 14.0, confidence=1.0, energy=50000.0)
    club = target(4, 26.0, confidence=0.6)
    assert tr.update(4, [decoy, club]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(26.0)
    assert tr.why() == "associated"


def test_inside_the_gate_range_error_doppler_continuity_and_quality_all_count(lib):
    tr = Tracker(lib)
    for frame, b in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b), doppler=3.0)])
    # Two candidates half a bin either side of the prediction (26), so range
    # alone cannot separate them: one with a Doppler half a span away from
    # the track's, one with continuous Doppler and better quality.
    a = target(4, 25.5, confidence=0.5, doppler=-5.5)
    b = target(4, 26.5, confidence=0.9, doppler=3.1)
    tr.update(4, [a, b])
    assert tr.points()[-1].rangeBin == pytest.approx(26.5)
    # With Doppler and quality ignored the nearer-in-range candidate wins.
    tr = Tracker(lib, weightVelocity=0.0, weightQuality=0.0)
    for frame, b_ in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b_), doppler=3.0)])
    tr.update(4, [target(4, 25.6, confidence=0.5, doppler=-5.5), b])
    assert tr.points()[-1].rangeBin == pytest.approx(25.6)


def test_a_missing_frame_is_coasted_then_the_club_is_picked_up_where_predicted(lib):
    tr = Tracker(lib)
    for frame, b in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b))])
    assert tr.update(4, []) is False
    assert tr.why() == "coasted" and tr.track.active == 1
    assert tr.track.predictedBin == pytest.approx(26.0, abs=0.5)
    assert tr.update(5, [target(5, 28.0)]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(28.0)
    assert tr.track.count == 4


def test_too_many_misses_drop_the_track_and_a_new_one_is_acquired(lib):
    tr = Tracker(lib)
    for frame, b in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b))])
    tr.update(4, [])
    tr.update(5, [])
    assert tr.update(6, []) is False
    assert tr.why() == "dropped" and tr.track.active == 0
    assert tr.update(7, [target(7, 40.0)]) is True
    assert tr.why() == "acquired"


def test_acquisition_needs_confidence_and_takes_the_most_confident(lib):
    tr = Tracker(lib)
    assert tr.update(1, [target(1, 30.0, confidence=0.1)]) is False
    assert tr.why() == "idle"
    assert tr.update(2, [target(2, 30.0, confidence=0.4), target(2, 40.0, confidence=0.8)]) is True
    assert tr.points()[0].rangeBin == pytest.approx(40.0)


def test_history_keeps_the_newest_32_points(lib):
    tr = Tracker(lib, gateBins=100.0)
    for frame in range(1, 41):
        tr.update(frame, [target(frame, 20.0 + 1.1 * frame)])
    assert tr.track.count == POINTS and tr.track.total == 40
    assert tr.points()[0].frame == 9 and tr.points()[-1].frame == 40


def test_fit_recovers_the_range_rate_as_speed(lib):
    """2.0 bins per 3 ms frame = 31.25 m/s radial at 46.875 mm bins."""
    tr = Tracker(lib)
    for frame in range(1, 9):
        tr.update(frame, [target(frame, 20.0 + 2.0 * frame)])
    used, slope, residual = tr.fit(8)
    assert used == 8
    assert slope == pytest.approx(2.0 / 3e-3, rel=0.01)
    assert residual == pytest.approx(0.0, abs=0.05)
    assert tr.lib.l3_track_speed_mps(ctypes.byref(tr.track), 8) == pytest.approx(31.25, rel=0.01)
    assert tr.points()[-1].radialVelocityMps == pytest.approx(31.25, rel=0.05)


def test_fit_needs_three_points_and_reports_scatter(lib):
    tr = Tracker(lib)
    tr.update(1, [target(1, 20.0)])
    tr.update(2, [target(2, 22.0)])
    assert tr.fit()[0] == 0 and tr.lib.l3_track_speed_mps(ctypes.byref(tr.track), 8) == 0.0
    tr.update(3, [target(3, 25.0)])  # off the line by a bin
    used, _slope, residual = tr.fit()
    assert used == 3 and residual > 0.2


def test_reset_forgets_the_track_but_keeps_configuration_and_counters(lib):
    tr = Tracker(lib)
    for frame, b in enumerate([20, 22, 24], start=1):
        tr.update(frame, [target(frame, float(b))])
    tr.lib.l3_track_reset(ctypes.byref(tr.track))
    assert tr.track.active == 0 and tr.track.count == 0
    assert tr.track.counters[WHY.index("acquired")] == 1
    assert tr.cfg.gateBins == 3.0


def test_status_and_point_lines_read_without_float_printf(lib):
    tr = Tracker(lib)
    for frame in range(1, 6):
        tr.update(frame, [target(frame, 30.0 + 2.0 * frame, doppler=-1.5)])
    status = tr.status(dest=48)
    assert status.startswith(
        "clubtrack active=1 why=associated count=5 total=5 misses=0 bin=40.00 dest=48 dist=8.00 "
    )
    assert " speed=31.25 fit=5 residual=0.00 " in status
    buf = ctypes.create_string_buffer(256)
    tr.lib.l3_track_format_point(ctypes.byref(tr.points()[-1]), 48, buf, len(buf))
    line = buf.value.decode()
    assert line.startswith(
        "p frame=5 t=15000 bin=40.00 dist=8.00 range=1.88 vr=31.25 vd=-1.50 coh=90 conf=0.90"
    )
    assert line.endswith("angles=none")
    for code, name in enumerate(WHY):
        assert tr.lib.l3_track_why_name(code).decode() == name


# --- 3D delivery ---------------------------------------------------------------

DEG = math.pi / 180.0


def straight_line_track(
    lib,
    *,
    speed: float = 40.0,
    path_deg: float = 0.0,
    attack_deg: float = 0.0,
    frames: int = 8,
    angles: int = ANGLE_AZIMUTH | ANGLE_ELEVATION,
    start=(1.2, -0.05, -0.20),
    frame_s: float = FRAME_US * 1e-6,
    confidence: float = 0.9,
    cal_attitude: dict | None = None,
):
    """A clubhead on a straight line through the golf frame at constant velocity,
    observed as (range, azimuth, elevation) each frame. The delivery fit must
    give back the velocity vector, so path and attack angles are the inputs
    the assertions read against."""
    tr = Tracker(lib)
    if cal_attitude:
        for name, value in cal_attitude.items():
            setattr(tr.track.cfg.cal, name, value)
    vx = speed * math.cos(attack_deg * DEG) * math.cos(path_deg * DEG)
    vy = speed * math.cos(attack_deg * DEG) * math.sin(path_deg * DEG)
    vz = speed * math.sin(attack_deg * DEG)
    cal = tr.track.cfg.cal
    truth = []
    for frame in range(1, frames + 1):
        s = (frame - 1) * frame_s
        golf = fw.Vec3(start[0] + vx * s, start[1] + vy * s, start[2] + vz * s)
        radar = fw.Vec3()
        lib.l3_frames_golf_to_radar(ctypes.byref(cal), ctypes.byref(golf), ctypes.byref(radar))
        sph = fw.Spherical()
        lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        tgt = target(frame, sph.rangeM / BIN_M, confidence=confidence)
        assert tr.update(frame, [tgt]) is True
        assert (
            lib.l3_track_set_angles(
                ctypes.byref(tr.track), sph.azimuthRad, sph.elevationRad, angles, 1.0
            )
            == 1
        )
        truth.append((golf.x, golf.y, golf.z))
    return tr, (vx, vy, vz), truth


def delivery(lib, tr, max_points: int = POINTS):
    out = Delivery()
    used = lib.l3_track_delivery(ctypes.byref(tr.track), max_points, ctypes.byref(out))
    return used, out


def test_points_carry_golf_frame_positions_from_range_and_angles(lib):
    tr, _, truth = straight_line_track(lib, frames=4)
    for point, (x, y, z) in zip(tr.points(), truth, strict=True):
        assert (point.position.x, point.position.y, point.position.z) == pytest.approx(
            (x, y, z), abs=2e-3
        )
        assert point.anglesValid == ANGLE_AZIMUTH | ANGLE_ELEVATION


def test_set_angles_targets_the_point_the_last_update_appended(lib):
    tr = Tracker(lib)
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.1, 0.2, 3, 1.0) == 0, "nothing yet"
    tr.update(1, [target(1, 20.0)])
    assert tr.track.lastTargetIndex == 0
    tr.update(2, [target(2, 30.0, confidence=0.3), target(2, 22.0)])
    assert tr.track.lastTargetIndex == 1, "the associated target, not the first"
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.1, -0.2, ANGLE_ELEVATION, 1.0) == 1
    newest = tr.points()[-1]
    assert newest.anglesValid == ANGLE_ELEVATION
    assert newest.elevationRad == pytest.approx(-0.2)
    # Azimuth not measured: the position sits on boresight in y.
    assert newest.position.y == pytest.approx(0.0)
    assert newest.position.z == pytest.approx(22.0 * BIN_M * math.sin(-0.2), abs=1e-4)
    assert tr.update(3, []) is False
    assert tr.track.lastTargetIndex == TRACK_NO_TARGET
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.0, 0.0, 3, 1.0) == 0


def test_a_range_only_point_sits_on_boresight_with_the_range_bias_removed(lib):
    tr = Tracker(lib)
    tr.track.cfg.cal.rangeBiasM = 0.066
    tr.update(1, [target(1, 40.0)])
    point = tr.points()[-1]
    assert point.position.x == pytest.approx(40.0 * BIN_M - 0.066, abs=1e-5)
    assert point.position.y == 0.0 and point.position.z == 0.0


@pytest.mark.parametrize("path_deg,attack_deg", [(0.0, 0.0), (4.0, -3.0), (-6.0, 2.5), (2.0, -5.0)])
def test_delivery_recovers_the_velocity_vector_speed_path_and_attack(lib, path_deg, attack_deg):
    tr, (vx, vy, vz), _ = straight_line_track(lib, path_deg=path_deg, attack_deg=attack_deg)
    used, out = delivery(lib, tr)
    assert used == 8 and out.points == 8 and out.azimuthPoints == 8 and out.elevationPoints == 8
    assert (out.velocity.x, out.velocity.y, out.velocity.z) == pytest.approx((vx, vy, vz), abs=0.3)
    assert out.speedMps == pytest.approx(40.0, abs=0.3)
    assert out.pathRad / DEG == pytest.approx(path_deg, abs=0.3)
    assert out.attackRad / DEG == pytest.approx(attack_deg, abs=0.3)
    assert out.speedValid and out.pathValid and out.attackValid
    assert out.residualM < 0.005
    assert out.confidence > 0.8
    assert out.timestampUs == 8 * FRAME_US


def test_delivery_signs_follow_the_frame_conventions(lib):
    """Right of the target line is positive path (in-to-out); up is positive attack."""
    _, in_to_out = delivery(lib, straight_line_track(lib, path_deg=5.0)[0])
    _, out_to_in = delivery(lib, straight_line_track(lib, path_deg=-5.0)[0])
    assert in_to_out.pathRad > 0 > out_to_in.pathRad
    _, descending = delivery(lib, straight_line_track(lib, attack_deg=-4.0)[0])
    _, ascending = delivery(lib, straight_line_track(lib, attack_deg=4.0)[0])
    assert descending.attackRad < 0 < ascending.attackRad


def test_delivery_is_read_in_the_golf_frame_whatever_the_radar_attitude(lib):
    """The same swing seen by a pitched, yawed radar: the calibration undoes it."""
    attitude = {"radarPitchRad": 10.0 * DEG, "radarYawRad": -3.0 * DEG, "radarRollRad": 2.0 * DEG}
    tr, (vx, vy, vz), _ = straight_line_track(
        lib, path_deg=3.0, attack_deg=-4.0, cal_attitude=attitude
    )
    _, out = delivery(lib, tr)
    assert (out.velocity.x, out.velocity.y, out.velocity.z) == pytest.approx((vx, vy, vz), abs=0.3)
    assert out.pathRad / DEG == pytest.approx(3.0, abs=0.3)
    assert out.attackRad / DEG == pytest.approx(-4.0, abs=0.3)


def test_radial_speed_is_the_projection_the_range_walk_alone_can_see(lib):
    """Down the boresight the two agree; off it the radial speed is smaller."""
    _, on_axis = delivery(lib, straight_line_track(lib, start=(1.2, 0.0, 0.0))[0])
    assert on_axis.radialSpeedMps == pytest.approx(on_axis.speedMps, abs=0.2)
    _, off_axis = delivery(lib, straight_line_track(lib, path_deg=20.0, start=(1.0, -0.6, 0.0))[0])
    assert off_axis.radialSpeedMps < off_axis.speedMps - 1.0


def test_elevation_only_points_give_speed_and_attack_but_no_path(lib):
    tr, (vx, _, vz), _ = straight_line_track(
        lib, path_deg=4.0, attack_deg=-3.0, angles=ANGLE_ELEVATION
    )
    used, out = delivery(lib, tr)
    assert used == 8 and out.azimuthPoints == 0 and out.elevationPoints == 8
    assert out.speedValid and out.attackValid and not out.pathValid
    assert out.pathRad == 0.0
    assert out.attackRad / DEG == pytest.approx(-3.0, abs=0.5)
    # Without azimuth the position is on boresight, so vy reads as nothing.
    assert out.velocity.y == pytest.approx(0.0, abs=1e-3)
    assert out.velocity.z == pytest.approx(vz, abs=0.3)
    assert out.velocity.x == pytest.approx(
        math.hypot(vx, 40.0 * math.cos(-3.0 * DEG) * math.sin(4.0 * DEG)), abs=0.5
    )


def test_range_only_points_give_the_radial_speed_and_nothing_angular(lib):
    tr, _, _ = straight_line_track(lib, angles=0)
    used, out = delivery(lib, tr)
    assert used == 8 and out.azimuthPoints == 0 and out.elevationPoints == 0
    assert out.speedValid and not out.pathValid and not out.attackValid
    assert out.speedMps == pytest.approx(out.radialSpeedMps, abs=1e-3)
    assert out.velocity.y == 0.0 and out.velocity.z == 0.0


def test_measured_angles_never_mix_with_assumed_boresight(lib):
    """Five angled points and three range-only ones: the fit uses the five, so
    the boresight assumption cannot bend the path."""
    tr = Tracker(lib)
    for frame in range(1, 9):
        bin_ = 20.0 + 2.0 * frame
        tr.update(frame, [target(frame, bin_)])
        if frame <= 5:
            lib.l3_track_set_angles(
                ctypes.byref(tr.track), 6.0 * DEG, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION, 1.0
            )
    used, out = delivery(lib, tr)
    assert used == 5 and out.points == 5 and out.azimuthPoints == 5
    assert out.pathValid
    # A target at constant azimuth moving along range: path equals that azimuth.
    assert out.pathRad / DEG == pytest.approx(6.0, abs=0.2)


def test_too_few_angled_points_give_a_radial_only_delivery_never_a_mix(lib):
    """Six points walking 2 bins a frame, only two of them angled and wildly
    inconsistent (the 2026-09-27 ball: one boresight point and two noise
    angles made 339 m/s). With under three angled points the fit cannot read
    a direction, so it must not blend boresight and angled positions: the
    speed is the range walk's and nothing angular is claimed."""
    tr = Tracker(lib)
    wild = {2: (35.0 * DEG, -25.0 * DEG), 4: (-40.0 * DEG, 30.0 * DEG)}
    for frame in range(1, 7):
        tr.update(frame, [target(frame, 20.0 + 2.0 * frame)])
        if frame in wild:
            az, el = wild[frame]
            lib.l3_track_set_angles(
                ctypes.byref(tr.track), az, el, ANGLE_AZIMUTH | ANGLE_ELEVATION, 1.0
            )
    used, out = delivery(lib, tr)
    walk_mps = 2.0 * BIN_M / (FRAME_US * 1e-6)
    assert used == 6
    assert out.radialSpeedMps == pytest.approx(walk_mps, rel=1e-3)
    assert out.speedValid and out.speedMps == pytest.approx(out.radialSpeedMps, abs=1e-3)
    assert (out.velocity.x, out.velocity.y, out.velocity.z) == pytest.approx(
        (out.radialSpeedMps, 0.0, 0.0), abs=1e-3
    )
    assert not out.pathValid and not out.attackValid
    assert out.azimuthPoints == 0 and out.elevationPoints == 0


def test_delivery_needs_three_points_and_honours_max_points(lib):
    tr, _, _ = straight_line_track(lib, frames=2)
    used, out = delivery(lib, tr)
    assert used == 0 and not out.speedValid and out.points == 0
    tr, _, _ = straight_line_track(lib, frames=12)
    used, out = delivery(lib, tr, max_points=5)
    assert used == 5 and out.points == 5


def test_delivery_confidence_falls_with_scatter_few_points_and_weak_targets(lib):
    _, clean = delivery(lib, straight_line_track(lib)[0])
    _, few = delivery(lib, straight_line_track(lib, frames=3)[0])
    _, weak = delivery(lib, straight_line_track(lib, confidence=0.4)[0])
    assert few.confidence < clean.confidence
    assert weak.confidence < clean.confidence
    tr = Tracker(lib)
    rng = [22.0, 23.0, 27.0, 26.0, 31.0, 30.0]  # two bins of scatter around a line
    for frame, b in enumerate(rng, start=1):
        tr.update(frame, [target(frame, b)])
    _, scattered = delivery(lib, tr)
    assert scattered.confidence < clean.confidence
    assert scattered.residualM > 0.02


def test_delivery_format_names_the_metrics_and_their_validity(lib):
    _, full = delivery(lib, straight_line_track(lib, path_deg=2.0, attack_deg=-3.0)[0])
    text = fw.c_text(lib.l3_track_format_delivery, ctypes.byref(full))
    assert text.startswith("delivery points=8 az=8 el=8 speed=40.")
    assert " path=2.0" in text and " attack=-3.0" in text and text.endswith(" valid=spa")
    _, radial = delivery(lib, straight_line_track(lib, angles=0)[0])
    assert fw.c_text(lib.l3_track_format_delivery, ctypes.byref(radial)).endswith(" valid=s")
    empty = Delivery()
    assert fw.c_text(lib.l3_track_format_delivery, ctypes.byref(empty)).endswith(" valid=none")


def test_reset_forgets_the_last_target_but_keeps_the_calibration(lib):
    tr = Tracker(lib)
    tr.track.cfg.cal.radarPitchRad = 0.1
    tr.update(1, [target(1, 20.0)])
    lib.l3_track_reset(ctypes.byref(tr.track))
    assert tr.track.lastTargetIndex == TRACK_NO_TARGET
    assert tr.track.cfg.cal.radarPitchRad == pytest.approx(0.1)


# --- lessons from the first recorded swings ------------------------------------


def test_acquisition_prefers_a_mover_over_a_stronger_stationary_return(lib):
    """Recorded swings: hands or body at bin 44 read as the most confident
    target in every frame while the club passed at 3 to 7 m/s of Doppler."""
    tr = Tracker(lib)
    body = target(1, 44.0, confidence=0.95, doppler=0.3)
    club = target(1, 40.0, confidence=0.6, doppler=4.7)
    assert tr.update(1, [body, club]) is True
    assert tr.points()[-1].rangeBin == 40.0 and tr.track.lastTargetIndex == 1
    # With nothing moving, the most confident target is still acquired.
    tr = Tracker(lib)
    assert (
        tr.update(
            1,
            [
                target(1, 44.0, confidence=0.95, doppler=0.3),
                target(1, 40.0, confidence=0.6, doppler=0.2),
            ],
        )
        is True
    )
    assert tr.points()[-1].rangeBin == 44.0
    # The preference can be switched off.
    tr = Tracker(lib, minAcquireDopplerMps=0.0)
    assert tr.update(1, [body, club]) is True
    assert tr.points()[-1].rangeBin == 44.0
    cfg = Cfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.minAcquireDopplerMps == pytest.approx(1.0)
    assert cfg.maxAngleResidualM == pytest.approx(2 * BIN_M)


def test_angles_that_do_not_fit_a_line_are_dropped_in_favour_of_the_radial_speed(lib):
    """A fast ball crossing bins within a burst gives azimuths that scatter by
    tens of degrees; the delivery keeps the range walk and no direction."""
    tr = Tracker(lib)
    for frame, (b, az) in enumerate(
        [(50.0, 0.0), (52.6, 0.4), (55.2, -0.4), (57.8, 0.5), (60.4, -0.5), (63.0, 0.3)], start=1
    ):
        tr.update(frame, [target(frame, b)])
        lib.l3_track_set_angles(
            ctypes.byref(tr.track), az, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION, 1.0
        )
    used, out = delivery(lib, tr)
    assert used == 6 and out.speedValid
    assert not out.pathValid and not out.attackValid
    assert out.azimuthPoints == 0 and out.elevationPoints == 0
    assert out.speedMps == pytest.approx(out.radialSpeedMps)
    assert out.speedMps == pytest.approx(2.6 * BIN_M / (FRAME_US * 1e-6), rel=0.05)
    assert out.velocity.y == 0.0 and out.velocity.z == 0.0
    # Consistent angles keep the direction.
    tr = Tracker(lib)
    for frame, b in enumerate([50.0, 52.6, 55.2, 57.8, 60.4, 63.0], start=1):
        tr.update(frame, [target(frame, b)])
        lib.l3_track_set_angles(
            ctypes.byref(tr.track), 0.05, 0.2, ANGLE_AZIMUTH | ANGLE_ELEVATION, 1.0
        )
    _, steady = delivery(lib, tr)
    assert steady.pathValid and steady.attackValid
    # The guard can be disabled.
    tr = Tracker(lib, maxAngleResidualM=0.0)
    for frame, (b, az) in enumerate([(50.0, 0.0), (52.6, 0.4), (55.2, -0.4), (57.8, 0.5)], start=1):
        tr.update(frame, [target(frame, b)])
        lib.l3_track_set_angles(
            ctypes.byref(tr.track), az, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION, 1.0
        )
    _, raw = delivery(lib, tr)
    assert raw.pathValid


def test_a_track_that_never_approaches_gives_way_to_the_club(lib):
    """Recorded 2026-09-27 14:43:41 (shot 13): a hand/body return next to
    the ball reads +3 to +6 m/s of Doppler, so the mover preference acquires
    it on frame 2 while the real club is still short of the watch region.
    It then sits at bins 43.1-43.9 for eight frames, and because association
    only looks around the prediction, the club (SNR 30-200, bins 29.4 to
    39.7 at about a bin per frame) is never picked up. The trigger fires
    correctly on the club at frame 13; the club track reports no approach."""
    tr = Tracker(lib)
    body = {2: 43.1, 3: 43.5, 4: 43.6, 5: 43.9, 6: 43.9, 7: 43.9, 8: 43.8, 9: 43.7, 10: 43.8}
    club = {
        3: 29.1, 4: 29.4, 5: 29.4, 6: 30.4, 7: 31.3, 8: 33.4, 9: 34.1,
        10: 35.7, 11: 36.6, 12: 38.6, 13: 39.7,
    }  # fmt: skip
    club_doppler = {3: 0.4, 4: 3.2, 5: 6.0, 6: 7.9, 7: -8.8, 8: -7.5, 9: -5.9, 10: -4.6,
                    11: -3.5, 12: -3.3, 13: -2.4}  # fmt: skip
    for frame in range(2, 14):
        targets = []
        if frame in club:
            targets.append(target(frame, club[frame], confidence=0.9, doppler=club_doppler[frame]))
        if frame in body:
            targets.append(target(frame, body[frame], confidence=0.6, doppler=5.5))
        tr.update(frame, targets)

    points = tr.points()
    newest = [p.rangeBin for p in points if p.frame >= 9]
    assert newest, f"no points after frame 9: {[(p.frame, p.rangeBin) for p in points]}"
    assert all(b < 41.0 for b in newest), "the track is still on the return at the ball"
    assert newest[-1] == pytest.approx(39.7), "the club's last frame is on the track"
    used, slope, _ = tr.fit(6)
    assert used >= 3 and slope > 0, "the fitted track closes on the ball"


def test_club_rule_defaults_and_the_ball_tracker_keeps_them_off(lib):
    """The club moves into ascending bins, at most two points in the same bin."""
    cfg = Cfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.ascendingOnly, cfg.maxSameBinPoints) == (1, 2)
    ball = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(ball))
    assert (ball.core.ascendingOnly, ball.core.maxSameBinPoints) == (0, 0)
    assert ball.core.approachMaxSameBinPoints == 0


def _feed_stationary(tr, frames, bin_=43.5, doppler=5.5):
    for frame in frames:
        tr.update(frame, [target(frame, bin_, confidence=0.9, doppler=doppler)])


def test_a_third_point_in_the_same_bin_triggers_release(lib):
    """A third consecutive point in the same bin exceeds the limit: the track
    is released immediately (not coasted) so the club can be re-acquired from
    the same frame's other targets without waiting for a drop."""
    tr = Tracker(lib, approachMaxSameBinPoints=2)
    _feed_stationary(tr, range(1, 3))  # two points in bin 44 (43.5 rounds up): allowed
    assert tr.track.active == 1 and tr.track.count == 2
    # Third offer in same bin: appended then released; a mover well clear of the
    # released bin (>gateBins=3 away from 44) is re-acquired on the same frame.
    mover = target(3, 49.0, doppler=5.5)
    assert tr.update(3, [target(3, 43.6, doppler=5.5), mover]) is True
    assert tr.why() == "acquired" and tr.points()[0].rangeBin == pytest.approx(49.0)
    assert tr.track.count == 1  # fresh track, one point


def test_the_repeat_count_is_consecutive_and_per_rounded_bin(lib):
    tr = Tracker(lib, approachMaxSameBinPoints=2, standingFrames=0)
    # 30.2 and 30.4 share bin 30, 30.6 is bin 31: a club crossing a bin
    # boundary slowly is not a repeat.
    for frame, bin_ in enumerate([30.2, 30.4, 30.6, 31.2, 31.9, 32.8], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert tr.track.counters[WHY.index("released")] == 0 and tr.track.count == 6


def test_release_lets_a_mover_be_acquired_when_the_decoy_is_stuck(lib):
    """When the track is released after the same-bin limit, the real club at a
    higher bin is re-acquired on the same frame rather than waiting for a drop."""
    tr = Tracker(lib, approachMaxSameBinPoints=2)
    for frame, bin_ in enumerate([43.1, 43.9], start=2):  # bins 43, 44
        tr.update(frame, [target(frame, bin_, doppler=5.5)])
    tr.update(4, [target(4, 44.2, doppler=5.5)])  # 44 again: 2 in bin 44, kept
    assert tr.track.active == 1
    # Frame 5: same-bin 44 is appended (3rd → exceeds limit), track released,
    # then re-acquired from the moving target at 49 (well clear of released bin 44)
    # on the same call.
    assert tr.update(5, [target(5, 44.1, doppler=5.4), target(5, 49.0, doppler=5.5)]) is True
    assert tr.why() == "acquired" and tr.points()[0].rangeBin == pytest.approx(49.0)


@pytest.mark.parametrize("override", [{"approachMaxSameBinPoints": 0}])
def test_the_repeat_rule_can_be_switched_off(lib, override):
    tr = Tracker(lib, **override)
    _feed_stationary(tr, range(1, 20))
    assert tr.track.active == 1 and tr.track.count == 19
    assert tr.track.counters[WHY.index("released")] == 0


def test_same_bin_release_without_alternatives_causes_idle_then_reacquisition(lib):
    """When the track is released and no alternative is available, the track
    becomes idle (active=0). On the next frame with a moving target it reacquires."""
    tr = Tracker(lib, maxMisses=3, approachMaxSameBinPoints=2)
    _feed_stationary(tr, range(1, 3))  # 2 in bin 44: sameBinCount = 2
    assert tr.track.count == 2
    # 3rd same-bin: appended, released, no alternative → reacquisition fails → released/idle
    tr.update(3, [target(3, 43.6, doppler=5.5)])
    assert tr.track.active == 0 and tr.why() in ("idle", "released")
    # Next frame: a mover well clear of the released bin (>gateBins=3 away from 44)
    assert tr.update(4, [target(4, 49.0, doppler=5.5)]) is True
    assert tr.why() == "acquired"


def test_strength_term_prefers_high_snr_target(lib):
    """With weightStrength > 0 the association score penalises low-SNR targets.
    Two candidates at the same range, velocity, and confidence: the one with
    higher SNR (stronger MTI residual) wins.  We distinguish the winner by
    energy, which is copied verbatim into TrackPoint."""
    tr = Tracker(lib, weightStrength=1.0)
    # Establish a track
    for frame, bin_ in enumerate([30.0, 31.0], start=1):
        tr.update(frame, [target(frame, bin_)])
    # Frame 3: two equidistant targets differ only in SNR (and energy as tracer)
    weak = target(3, 32.0, snr=5.0, energy=100.0)  # strengthMisfit = 1/5  = 0.20
    strong = target(3, 32.0, snr=50.0, energy=9999.0)  # strengthMisfit = 1/50 = 0.02
    assert tr.update(3, [weak, strong]) is True
    # The stronger MTI return wins (lower score)
    assert tr.points()[-1].energy == pytest.approx(9999.0)


def test_strength_term_off_does_not_change_default_selection(lib):
    """With weightStrength=0 the result is the same as without the term."""
    tr = Tracker(lib, weightStrength=0.0)
    for frame, bin_ in enumerate([30.0, 31.0], start=1):
        tr.update(frame, [target(frame, bin_)])
    # Frame 3: only one candidate; should associate normally
    assert tr.update(3, [target(3, 32.0)]) is True
    assert tr.track.count == 3


def test_association_only_takes_the_same_bin_or_higher(lib):
    tr = Tracker(lib)
    for frame, bin_ in enumerate([30.0, 31.1, 32.2], start=1):
        tr.update(frame, [target(frame, bin_)])
    # Nearest the prediction, but a lower bin than the last point: never the club.
    assert tr.update(4, [target(4, 31.4), target(4, 35.0, confidence=0.5)]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(35.0)
    # Only lower bins on offer: coasted, not associated.
    assert tr.update(5, [target(5, 33.0)]) is False and tr.why() == "coasted"
    # The same rounded bin passes (sub-bin jitter is not a retreat).
    tr2 = Tracker(lib, approachMaxSameBinPoints=2)
    for frame, bin_ in enumerate([30.0, 31.3, 31.1], start=1):
        tr2.update(frame, [target(frame, bin_)])
    assert tr2.track.count == 3


def test_a_wobble_before_the_downswing_is_not_followed(lib):
    """The club only ascends: a dip coasts, then the track drops or resumes."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([30.0, 29.2, 28.6, 30.8, 31.9], start=1):
        tr.update(frame, [target(frame, bin_)])
    bins = [round(p.rangeBin, 1) for p in tr.points()]
    assert bins == [30.0, 30.8, 31.9]


def test_ascending_can_be_switched_off(lib):
    tr = Tracker(lib, ascendingOnly=0, approachMaxSameBinPoints=2)
    for frame, bin_ in enumerate([30.0, 29.2, 28.6], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert tr.track.count == 3


def test_same_bin_release_sets_released_valid_and_prevents_re_taking_same_bin(lib):
    """When released due to same-bin excess, releasedValid is set so that
    reacquisition skips the released bin (preventing an immediate re-grab)."""
    tr = Tracker(lib, approachMaxSameBinPoints=2)
    _feed_stationary(tr, range(1, 3))  # 2 points in bin 44
    tr.update(3, [target(3, 43.6, doppler=5.5)])  # 3rd same-bin: released
    assert tr.track.releasedValid == 1  # release state is set
    assert tr.track.releasedBin == pytest.approx(44, abs=2)  # last tracked bin
    # A mover well clear of the released bin (>gateBins=3 away from 44) is acquired
    assert tr.update(4, [target(4, 49.0, doppler=5.5)]) is True
    assert tr.why() == "acquired"


def _follow(tr, frame, targets):
    arr = (Target * max(1, len(targets)))(*targets)
    return bool(
        tr.lib.l3_track_follow(
            ctypes.byref(tr.track), arr, len(targets), frame, frame * FRAME_US, None
        )
    )


def test_follow_continues_an_active_track_by_association(lib):
    """After impact the club track carries on: the first of the two tracks."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([30.0, 31.1, 32.2, 33.3], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert _follow(tr, 5, [target(5, 40.0, confidence=0.95), target(5, 34.0)]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(34.0) and tr.why() == "associated"


def test_follow_never_acquires_and_never_releases(lib):
    tr = Tracker(lib, standingFrames=0)
    assert _follow(tr, 1, [target(1, 40.0)]) is False
    assert tr.track.active == 0 and tr.track.count == 0
    # A return holding one bin after impact: two points, then refused (the
    # club never holds a bin for three) -- coasted and dropped, never released.
    for frame, bin_ in enumerate([30.0, 31.1, 32.2, 33.3], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert _follow(tr, 5, [target(5, 33.4)]) is True  # the second point in bin 33
    for frame in range(6, 10):
        assert _follow(tr, frame, [target(frame, 33.4)]) is False
    assert tr.track.counters[WHY.index("released")] == 0 and tr.track.count == 5
    assert tr.track.active == 0 and tr.why() == "dropped"


def test_follow_takes_the_strongest_return_the_club_not_the_prediction(lib):
    """After impact the club slows while the weaker ball carries on at least
    at the club's pace: where both are in reach, the stronger is the club."""
    tr = Tracker(lib, standingFrames=0)
    for frame, bin_ in enumerate([40.6, 43.1, 45.6, 48.1], start=1):  # ~2.5 bins/frame
        tr.update(frame, [target(frame, bin_)])
    club = [47.4, 48.6, 49.6, 50.4, 51.0]
    ball = [50.6, 53.4, 55.8, 58.4, 61.2]
    for frame, (c, b) in enumerate(zip(club, ball), start=5):
        strong, weak = target(frame, c), target(frame, b)
        strong.stat, weak.stat = 5000.0, 400.0
        assert _follow(tr, frame, [weak, strong]) is True
        assert tr.track.lastTargetIndex == 1
    assert [round(p.rangeBin, 1) for p in tr.points()[-5:]] == club


def test_follow_looks_one_bin_behind_the_last_point_and_no_further(lib):
    def fresh():
        tr = Tracker(lib, standingFrames=0)
        for frame, bin_ in enumerate([40.0, 41.0, 42.0], start=1):
            tr.update(frame, [target(frame, bin_)])
        return tr

    tr = fresh()
    assert _follow(tr, 4, [target(4, 41.1)]) is True  # 0.9 behind: the stalling club
    tr = fresh()
    assert _follow(tr, 4, [target(4, 40.8)]) is False  # 1.2 behind: not the club
    assert tr.why() == "coasted"
    tr = fresh()
    assert _follow(tr, 4, [target(4, 43.4)]) is True  # prediction 43.0, within half a bin
    tr = fresh()
    # Faster than the club arrived: the ball's, not the club's, however strong.
    strong = target(4, 43.8)
    strong.stat = 1e9
    assert _follow(tr, 4, [strong]) is False


def test_follow_never_takes_a_third_point_in_the_same_bin(lib):
    """Recorded 2026-08-24 12:04:08: after impact a strong near-stationary
    return sits at 38.2-38.4 (SNR 350-970) beside the club's weaker
    follow-through. The club never holds a bin for more than two points, so
    the stall is not the club."""
    tr = Tracker(lib, standingFrames=0)
    for frame, bin_ in enumerate([32.9, 34.4, 35.9, 37.4], start=1):  # 1.5 bins/frame
        tr.update(frame, [target(frame, bin_)])
    stall = [38.2, 38.2, 38.2, 38.3, 38.3]
    through = [None, 39.6, 40.1, 41.2, 42.3]
    got = []
    for frame, (s_, f_) in enumerate(zip(stall, through), start=5):
        stall_t = target(frame, s_)
        stall_t.stat = 9000.0
        targets = [stall_t]
        if f_ is not None:
            club_t = target(frame, f_)
            club_t.stat = 4000.0
            targets.append(club_t)
        assert _follow(tr, frame, targets) is True
        got.append(round(tr.points()[-1].rangeBin, 1))
    # Two points in bin 38 are allowed (the stronger stall wins them); the
    # third is refused and the weaker follow-through is the club from then on.
    assert got == [38.2, 38.2, 40.1, 41.2, 42.3]


def test_follow_caps_speed_at_the_impact_speed_not_the_decaying_estimate(lib):
    """Sitting on a stall shrinks the smoothed velocity; the cap on how far
    ahead the club may be stays the speed it had at impact."""
    tr = Tracker(lib, standingFrames=0)
    for frame, bin_ in enumerate([32.9, 34.4, 35.9, 37.4], start=1):  # 1.5 bins/frame
        tr.update(frame, [target(frame, bin_)])
    assert _follow(tr, 5, [target(5, 38.1)]) is True  # stalling
    assert _follow(tr, 6, [target(6, 38.4)]) is True
    # 38.4 + 1.5 + 0.5 = 40.4: 40.1 is reachable at the impact speed ...
    assert _follow(tr, 7, [target(7, 40.1)]) is True
    # ... and 42.2 (2.1 bins in one frame) is faster than the club arrived.
    assert _follow(tr, 8, [target(8, 42.2)]) is False


def test_wrapped_diff_goes_the_short_way_round(lib):
    f = lib.l3_track_wrapped_diff
    assert f(1.0, -1.0, 18.0) == pytest.approx(2.0)
    assert f(8.5, -8.5, 18.0) == pytest.approx(1.0)
    assert f(33.6, -2.4, 18.0) == pytest.approx(0.0, abs=1e-4)  # 33.6 m/s reads as -2.4
    assert f(3.0, 1.0, 0.0) == 0.0


def _approach_at(lib, samples):
    """A club track from (timestamp_us, bin) samples, one per frame."""
    tr = Tracker(lib)
    for frame, (ts, bin_) in enumerate(samples, start=1):
        t = target(frame, bin_)
        t.timestampUs = ts
        arr = (Target * 1)(t)
        lib.l3_track_update(ctypes.byref(tr.track), arr, 1, frame, ts)
    return tr


def _follow_at(tr, frame, timestamp_us, bins):
    targets = []
    for b in bins:
        t = target(frame, b)
        t.timestampUs = timestamp_us
        targets.append(t)
    arr = (Target * max(1, len(targets)))(*targets)
    return bool(
        tr.lib.l3_track_follow(ctypes.byref(tr.track), arr, len(targets), frame, timestamp_us, None)
    )


def test_follow_caps_the_club_by_elapsed_time_not_frames(lib):
    """750 bins/s at impact: a frame retained 4 ms after the last lets the club be
    3 bins on (+0.5 of jitter), whatever the frame count says."""
    history = [(0, 30.0), (2000, 31.5), (4000, 33.0), (6000, 34.5)]
    tr = _approach_at(lib, history)
    assert _follow_at(tr, 5, 10000, [37.3]) is True
    tr = _approach_at(lib, history)
    assert _follow_at(tr, 5, 10000, [38.2]) is False
    assert tr.track.followBinsPerS == pytest.approx(750.0, rel=1e-3)


def test_follow_with_two_points_at_impact_caps_by_their_rate(lib):
    """Recorded 2026-08-24 12:04:08 at tee bin 39: the club track was
    reacquired two frames before impact, too few points for a fit; the cap
    must still let the club move at the rate those two points show."""
    tr = _approach_at(lib, [(0, 30.0), (3000, 31.5)])  # 500 bins/s
    assert tr.track.count == 2
    assert _follow_at(tr, 3, 6000, [32.9]) is True  # 31.5 + 1.5 (+0.5)
    assert tr.track.followBinsPerS == pytest.approx(500.0, rel=1e-3)


def test_append_point_locates_and_counts(lib):
    tracker = Tracker(lib)
    track = tracker.track
    point = Point()
    point.frame = 7
    point.timestampUs = 21_000
    point.rangeBin = 40.0
    point.rangeM = 40.0 * track.cfg.binWidthM
    point.confidence = 0.9
    lib.l3_track_append_point(ctypes.byref(track), ctypes.byref(point))
    assert track.count == 1
    assert track.lastBin == pytest.approx(40.0)
    stored = tracker.points()[0]
    assert (stored.position.x, stored.position.y, stored.position.z) != (0.0, 0.0, 0.0)


def _approach_then_follow(tr, doppler):
    """A club arriving at ~2.3 bins/frame with the given aliased Doppler."""
    for frame, bin_ in enumerate([32.0, 34.3, 36.6, 38.9], start=1):
        tr.update(frame, [target(frame, bin_, doppler=doppler)])


def test_follow_refuses_a_stronger_return_reading_another_velocity(lib):
    """Recorded 2026-08-24 12:09:34: after impact the shaft's return (SNR ~500,
    Doppler -2..-8) is far stronger than the clubhead (SNR 60-140, Doppler +6
    decaying), which carries on from its impact Doppler. Doppler continuity, not
    strength, picks the club."""
    tr = Tracker(lib)
    _approach_then_follow(tr, doppler=6.0)
    shaft, club = target(5, 40.0, doppler=-4.0), target(5, 41.0, doppler=5.2)
    shaft.stat, club.stat = 9000.0, 300.0
    assert _follow(tr, 5, [shaft, club]) is True
    assert tr.track.lastTargetIndex == 1
    assert tr.points()[-1].rangeBin == pytest.approx(41.0)


def test_follow_takes_no_return_when_none_reads_the_clubs_velocity(lib):
    tr = Tracker(lib)
    _approach_then_follow(tr, doppler=6.0)
    assert _follow(tr, 5, [target(5, 40.0, doppler=-4.0)]) is False
    assert tr.why() == "coasted"


def test_follow_allows_the_clubs_doppler_to_fall_but_barely_to_rise(lib):
    """After impact the club only slows. A return whose Doppler wraps to a rise
    over the last point's is faster than the club was, so it is not the club:
    +8.5 at impact, -7.3 later wraps to +10.9."""
    tr = Tracker(lib, standingFrames=0)
    _approach_then_follow(tr, doppler=8.5)
    assert _follow(tr, 5, [target(5, 40.0, doppler=-7.3)]) is False  # wraps to a rise of 2.5
    assert _follow(tr, 6, [target(6, 40.0, doppler=6.2)]) is True  # a fall of 2.3: slowing
    assert (
        _follow(tr, 7, [target(7, 41.0, doppler=7.4)]) is True
    )  # a rise of 1.2 is measurement noise
    assert _follow(tr, 8, [target(8, 42.0, doppler=9.0)]) is False  # a rise of 1.6 is not


def test_a_second_point_in_the_same_bin_releases_the_approach_and_the_club_is_taken(lib):
    """Recorded 2026-08-24 12:12:26: before the swing a strong return stands at
    bin 37 (SNR 50-280, Doppler ~3, the hands or body) and outranks the weak
    club sweeping up from bin 21. The club moves several bins a frame, so its
    approach never holds a bin for two points; the stall is released at its
    second point and the club, in the same frame's targets, is taken."""
    tr = Tracker(lib)
    stall = target(1, 36.9, doppler=2.7, snr=50.0)
    assert tr.update(1, [stall, target(1, 21.1, doppler=-6.9, snr=23.0)]) is True
    assert tr.points()[0].rangeBin == pytest.approx(36.9)  # the stronger: acquired first
    stall2 = target(2, 37.2, doppler=3.0, snr=57.0)
    assert tr.update(2, [stall2, target(2, 22.7, doppler=-2.2, snr=11.0)]) is True
    assert tr.why() == "acquired" and tr.points()[0].rangeBin == pytest.approx(22.7)


def test_the_approach_same_bin_limit_is_its_own_setting(lib):
    """The follow phase keeps its own limit (the stall beside the ball gets two
    points there); the approach is stricter."""
    cfg = Cfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.approachMaxSameBinPoints, cfg.maxSameBinPoints) == (1, 2)


# --- standing returns: a bin occupied frame after frame is not the club --------


def test_acquisition_skips_a_return_that_has_stood_in_its_bin(lib):
    """Captures 2026-09-19 (17:52:42 and 15 more): a return at bin 49, SNR 40-90
    and 2-4 m/s Doppler on every frame (the hands or body), outranks the weak
    club at acquisition. The club moves; a return that has stood in its bin for
    the frames before is not it, however strong and however fast it reads."""
    tr = Tracker(lib)
    for frame in range(1, 4):  # too weak to acquire, but seen: it stands
        assert tr.update(frame, [target(frame, 49.0, confidence=0.1, doppler=3.0)]) is False
    stand = target(4, 49.1, confidence=0.9, doppler=3.2, snr=60.0)
    club = target(4, 44.0, confidence=0.5, doppler=5.0, snr=8.0)
    assert tr.update(4, [stand, club]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(44.0)


def test_a_bin_seen_only_now_is_a_candidate(lib):
    """The club is above threshold in any one bin for about a frame: a strong
    return that appears for the first time is acquired as before."""
    tr = Tracker(lib)
    stand = target(1, 49.1, confidence=0.9, doppler=3.2, snr=60.0)
    club = target(1, 44.0, confidence=0.5, doppler=5.0, snr=8.0)
    assert tr.update(1, [stand, club]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(49.1)


def test_follow_takes_the_club_over_a_stall_that_wobbles_between_two_bins(lib):
    """Captures 2026-09-19: after impact the stall near the ball (SNR 86-250,
    under 1 m/s) wobbles between two rounded bins, so the three-in-one-bin rule
    never fires, and 'the strongest' kept the track on it while the club, SNR
    6-39, moved on. Once the stall has stood for two frames it is not a candidate."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([30.0, 31.6, 33.2, 34.8], start=1):
        tr.update(frame, [target(frame, bin_, doppler=3.0)])
    taken = []
    for frame, (stall_bin, club_bin) in enumerate(
        zip([36.2, 36.6, 36.2, 36.6, 36.2], [35.4, 36.8, 37.9, 38.9, 39.9]), start=5
    ):
        # Dopplers agree: the Doppler gate is its own rule, tested above.
        stall = target(frame, stall_bin, doppler=2.8)
        club = target(frame, club_bin, doppler=2.5)
        stall.stat, club.stat = 9000.0, 300.0
        assert _follow(tr, frame, [stall, club]) is True
        taken.append(tr.track.lastTargetIndex)
    assert taken == [0, 0, 1, 1, 1], "the stall (0) until it has stood, then the club (1)"


def test_the_standing_rule_can_be_switched_off(lib):
    tr = Tracker(lib, standingFrames=0)
    for frame in range(1, 4):
        tr.update(frame, [target(frame, 49.0, confidence=0.1, doppler=3.0)])
    stand = target(4, 49.1, confidence=0.9, doppler=3.2, snr=60.0)
    club = target(4, 44.0, confidence=0.5, doppler=5.0, snr=8.0)
    assert tr.update(4, [stand, club]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(49.1)


def test_the_ball_tracker_core_has_no_standing_rule(lib):
    ball = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(ball))
    assert ball.core.standingFrames == 0
    cfg = Cfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.standingFrames == 2


def test_a_club_at_ordinary_speeds_is_never_standing(lib):
    """A club sweeping 1.5-2.5 bins a frame is in any bin for about a frame:
    approach and follow carry on with the rule on, from 1.5 bins a frame up."""
    for bins_per_frame in (1.5, 2.0, 2.5):
        tr = Tracker(lib)
        for frame in range(1, 6):
            assert tr.update(frame, [target(frame, 25.0 + bins_per_frame * frame)]) is True
        for frame in range(6, 9):
            assert _follow(tr, frame, [target(frame, 25.0 + bins_per_frame * frame)]) is True
        assert tr.track.count == 8, bins_per_frame


def test_a_moving_track_keeps_the_club_as_it_passes_through_a_standing_bin(lib):
    """Capture 2026-08-24 12:09:34 #011: a static return at bin 33 (SNR 10) stood
    on every frame; the club sweeping up at 2.5 bins a frame was there at 33.2
    and 34.7, merged with it. A track already moving at club speed whose
    prediction lands in a standing bin keeps that point -- only a track sitting
    on or beside the standing return is denied it."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([26.2, 28.7, 31.2], start=1):
        stand = target(frame, 33.1, confidence=0.5, doppler=0.4, snr=10.0)
        assert tr.update(frame, [target(frame, bin_, doppler=3.0), stand]) is True
    club = target(4, 33.6, doppler=3.0)
    stand = target(4, 33.1, confidence=0.5, doppler=0.4, snr=10.0)
    assert tr.update(4, [club, stand]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(33.6)
    assert tr.update(5, [target(5, 36.0, doppler=3.0), stand]) is True


def test_a_track_on_a_standing_return_gets_no_such_pass(lib):
    """The same standing bin, but the track was acquired on a return that does
    not move: a target in its bin is only its own, never an approach."""
    tr = Tracker(lib, approachMaxSameBinPoints=0)
    for frame in range(1, 4):
        stand = target(frame, 33.1, doppler=3.0)
        assert tr.update(frame, [stand]) is True
    stand = target(4, 34.0, doppler=3.0)  # a step onto a neighbouring bin of the standing return
    tr.update(4, [stand])
    assert tr.points()[-1].rangeBin != pytest.approx(34.0)


BOTH_ANGLES = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION


def test_set_point_angles_stores_the_angle_confidence_and_unfilters(lib):
    tr = Tracker(lib)
    for frame, rng in ((1, 30.0), (2, 31.0), (3, 32.0)):
        assert tr.update(frame, [target(frame, rng)])
    assert (
        lib.l3_track_set_point_angles(ctypes.byref(tr.track), 1, 0.1, 0.2, BOTH_ANGLES, 0.37) == 1
    )
    point = tr.points()[1]
    assert point.angleConfidence == pytest.approx(0.37)
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED and point.filterAccepted == 0
    assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
        point.position.x,
        point.position.y,
        point.position.z,
    )


def test_an_appended_point_starts_unfiltered_with_no_angle_confidence(lib):
    tr = Tracker(lib)
    assert tr.update(1, [target(1, 30.0)])
    point = tr.points()[0]
    assert point.angleConfidence == 0.0
    assert point.filterHypothesis == fw.FILTER_HYP_UNFILTERED


def test_track_cfg_defaults_fill_the_reconstruction_constants(lib):
    cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    assert cfg.kf.accelSigmaMps2 == pytest.approx(1500.0)
    assert cfg.kf.rangeSigmaM == pytest.approx(0.03)
    assert cfg.kf.angleSigmaRad == pytest.approx(math.radians(15.0))
    assert cfg.kf.minAngleConfidence == pytest.approx(0.05)
    assert cfg.kf.chi2Gate == pytest.approx(9.21)
    assert cfg.kf.initPositionSigmaM == pytest.approx(0.5)
    assert cfg.kf.initVelocitySigmaMps == pytest.approx(50.0)


# --- candidate approaches (acquisition past the golfer) -----------------------
#
# The golfer's body sits in the bins short of the ball, returns far stronger
# than the club (snr 100-870 against 10-96 on the 2026-09-19 captures) and
# reads as a slow mover, so single-frame acquisition took it. The club's
# approach advances ~2 bins a frame (0.8-3.6 on the labels); the body stays
# put. Acquisition now holds candidates and starts a track only on one whose
# next point makes an approach step. The club's aliased Doppler is too
# unsteady frame to frame (median change 2.3 m/s, a tenth 7-8) to gate on; it
# only ranks the pairs. Off by default (see l3_track_cfg_defaults): these
# tests turn it on at a 70 m/s club's step.

CANDIDATES = {"acquireMaxStepBins": 4.5}

CLUB_DOPPLER = -8.6  # an approaching club, aliased near the span's edge


def body(frame: int, range_bin: float = 45.0) -> Target:
    """The golfer: strong, confident, a slow mover, in one place."""
    return target(frame, range_bin, confidence=0.95, doppler=3.0, energy=90000.0, snr=300.0)


def weak_club(frame: int, range_bin: float, doppler: float = CLUB_DOPPLER) -> Target:
    return target(frame, range_bin, confidence=0.1, doppler=doppler, energy=800.0, snr=1.5)


def test_candidate_defaults_describe_the_labelled_approaches(lib):
    tr = Tracker(lib, **CANDIDATES)
    assert tr.cfg.acquireMinStepBins == pytest.approx(0.75)
    assert Tracker(lib).cfg.acquireMaxStepBins == 0.0, "off by default"
    assert tr.cfg.acquireExpectedStepBins == pytest.approx(2.0)
    assert tr.cfg.acquireDopplerTolMps >= 0.5 * tr.cfg.velocitySpanMps, "no Doppler gate"
    assert tr.cfg.acquireMinConfidence == pytest.approx(0.0)


def test_a_weak_club_stepping_in_is_acquired_over_the_stronger_golfer(lib):
    tr = Tracker(lib, **CANDIDATES)
    assert tr.update(1, [body(1), weak_club(1, 30.0)]) is False
    assert tr.track.active == 0, "one frame cannot tell a club from the body"
    assert tr.update(2, [body(2, 45.2), weak_club(2, 32.3)]) is True
    assert tr.why() == "acquired"
    assert [p.rangeBin for p in tr.points()] == [pytest.approx(30.0), pytest.approx(32.3)]
    assert tr.track.velocityBinsPerFrame == pytest.approx(2.3)
    assert tr.track.lastTargetIndex == 1, "this frame's club target was appended"
    assert tr.update(3, [body(3, 45.1), weak_club(3, 34.6)]) is True
    assert tr.why() == "associated"


def test_the_golfer_standing_in_place_never_starts_a_track(lib):
    tr = Tracker(lib, **CANDIDATES)
    for frame, b in enumerate([45.0, 45.3, 44.9, 45.2, 45.4, 45.0], start=1):
        assert tr.update(frame, [body(frame, b)]) is False
    assert tr.track.active == 0
    assert tr.track.counters[WHY.index("acquired")] == 0


def test_a_doppler_gate_when_set_refuses_a_step_that_reads_another_velocity(lib):
    tr = Tracker(lib, **CANDIDATES, acquireDopplerTolMps=3.0)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, [weak_club(2, 32.0, doppler=CLUB_DOPPLER + 6.0)]) is False
    assert tr.track.active == 0


def test_without_a_gate_a_step_with_another_doppler_still_confirms(lib):
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, [weak_club(2, 32.0, doppler=CLUB_DOPPLER + 6.0)]) is True


def test_the_doppler_agreement_wraps_across_the_alias_span(lib):
    """-8.8 and +8.8 m/s are 0.3 m/s apart on an alias span of ~17.9 m/s."""
    tr = Tracker(lib, **CANDIDATES, acquireDopplerTolMps=3.0)
    span = tr.cfg.velocitySpanMps
    tr.update(1, [weak_club(1, 30.0, doppler=-0.5 * span + 0.15)])
    assert tr.update(2, [weak_club(2, 32.0, doppler=0.5 * span - 0.15)]) is True


def test_of_two_confirming_steps_the_confident_one_near_the_expected_step_wins(lib):
    """20260824_120934: from the club at 22.2 a flicker at 25.7 (confidence
    0.02) with a closer Doppler beat the club at 23.5 (0.96), and the track ran
    ahead of the club."""
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [target(1, 22.2, doppler=7.7, confidence=0.92)])
    club = target(2, 23.9, doppler=-6.0, confidence=0.96, snr=14.0)
    flicker = target(2, 25.7, doppler=8.5, confidence=0.02, snr=1.1)
    assert tr.update(2, [flicker, club]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(23.9)
    assert tr.track.lastTargetIndex == 1


def test_of_two_equal_steps_the_closer_doppler_wins(lib):
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    far = weak_club(2, 32.0, doppler=CLUB_DOPPLER + 7.0)
    near = weak_club(2, 32.0, doppler=CLUB_DOPPLER + 0.5)
    assert tr.update(2, [far, near]) is True
    assert tr.track.lastTargetIndex == 1


@pytest.mark.parametrize("step", [0.4, 6.0, -2.0])
def test_only_an_approach_sized_step_forward_confirms(lib, step):
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, [weak_club(2, 30.0 + step)]) is False
    assert tr.track.active == 0


def test_a_candidate_confirms_across_one_missed_frame(lib):
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, []) is False
    assert tr.update(3, [weak_club(3, 34.4)]) is True
    assert tr.track.velocityBinsPerFrame == pytest.approx(2.2)
    assert [p.frame for p in tr.points()] == [1, 3]


def test_a_candidate_two_frames_stale_is_forgotten(lib):
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    tr.update(2, [])
    tr.update(3, [])
    assert tr.update(4, [weak_club(4, 36.6)]) is False


def test_candidates_below_the_candidate_confidence_are_not_held(lib):
    tr = Tracker(lib, **CANDIDATES, acquireMinConfidence=0.2)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, [weak_club(2, 32.3)]) is False


def test_the_released_return_does_not_come_back_as_a_candidate(lib):
    """Releasing a stuck track must not re-seed a candidate on that return."""
    tr = Tracker(lib, approachMaxSameBinPoints=1, standingFrames=0)
    tr.update(1, [target(1, 40.0)])
    tr.update(2, [target(2, 40.1)])  # one too many in one bin: released
    assert tr.track.active == 0
    tr.track.cfg.acquireMaxStepBins = CANDIDATES["acquireMaxStepBins"]  # its own copy
    tr.update(3, [target(3, 40.2)])
    assert tr.update(4, [target(4, 42.3)]) is False


def test_confirmation_off_acquires_on_the_first_frame(lib):
    tr = Tracker(lib)
    assert tr.update(1, [target(1, 30.0)]) is True
    assert tr.why() == "acquired"


def test_a_hop_between_two_still_returns_is_not_an_approach_step(lib):
    """20260919_175322 (no swing): still returns at 34.3 and 37.4 on every
    frame; the step between them confirmed a track that fired. The club steps
    into a bin nothing held the frame before."""
    tr = Tracker(lib, **CANDIDATES)
    for frame in range(1, 6):
        still = [
            target(frame, 34.3, doppler=-0.6, confidence=0.88),
            target(frame, 37.4, doppler=-0.6, confidence=0.88),
        ]
        assert tr.update(frame, still) is False
    assert tr.track.counters[WHY.index("acquired")] == 0


def test_the_club_stepping_past_a_still_return_is_still_acquired(lib):
    tr = Tracker(lib, **CANDIDATES)
    still = lambda frame: target(frame, 45.0, doppler=-0.6, confidence=0.88)  # noqa: E731
    tr.update(1, [still(1), weak_club(1, 30.0)])
    assert tr.update(2, [still(2), weak_club(2, 32.1)]) is True


def test_a_slow_club_step_within_its_own_last_bin_still_confirms(lib):
    """A 0.8-bin step lands within a bin of the club's own last return."""
    tr = Tracker(lib, **CANDIDATES)
    tr.update(1, [weak_club(1, 30.0)])
    assert tr.update(2, [weak_club(2, 30.8)]) is True
