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
) -> Target:
    t = Target()
    t.frame = frame
    t.timestampUs = frame * FRAME_US
    t.peakBin = int(round(range_bin))
    t.rangeBin = range_bin
    t.energy = energy
    t.peak = energy / 4
    t.stat = t.peak
    t.snr = 20.0
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
        tr.update(frame, [target(frame, 20.0 + 0.5 * frame)])
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
                ctypes.byref(tr.track), sph.azimuthRad, sph.elevationRad, angles
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
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.1, 0.2, 3) == 0, "nothing yet"
    tr.update(1, [target(1, 20.0)])
    assert tr.track.lastTargetIndex == 0
    tr.update(2, [target(2, 30.0, confidence=0.3), target(2, 22.0)])
    assert tr.track.lastTargetIndex == 1, "the associated target, not the first"
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.1, -0.2, ANGLE_ELEVATION) == 1
    newest = tr.points()[-1]
    assert newest.anglesValid == ANGLE_ELEVATION
    assert newest.elevationRad == pytest.approx(-0.2)
    # Azimuth not measured: the position sits on boresight in y.
    assert newest.position.y == pytest.approx(0.0)
    assert newest.position.z == pytest.approx(22.0 * BIN_M * math.sin(-0.2), abs=1e-4)
    assert tr.update(3, []) is False
    assert tr.track.lastTargetIndex == TRACK_NO_TARGET
    assert lib.l3_track_set_angles(ctypes.byref(tr.track), 0.0, 0.0, 3) == 0


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
                ctypes.byref(tr.track), 6.0 * DEG, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION
            )
    used, out = delivery(lib, tr)
    assert used == 5 and out.points == 5 and out.azimuthPoints == 5
    assert out.pathValid
    # A target at constant azimuth moving along range: path equals that azimuth.
    assert out.pathRad / DEG == pytest.approx(6.0, abs=0.2)


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
        lib.l3_track_set_angles(ctypes.byref(tr.track), az, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION)
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
        lib.l3_track_set_angles(ctypes.byref(tr.track), 0.05, 0.2, ANGLE_AZIMUTH | ANGLE_ELEVATION)
    _, steady = delivery(lib, tr)
    assert steady.pathValid and steady.attackValid
    # The guard can be disabled.
    tr = Tracker(lib, maxAngleResidualM=0.0)
    for frame, (b, az) in enumerate([(50.0, 0.0), (52.6, 0.4), (55.2, -0.4), (57.8, 0.5)], start=1):
        tr.update(frame, [target(frame, b)])
        lib.l3_track_set_angles(ctypes.byref(tr.track), az, 0.0, ANGLE_AZIMUTH | ANGLE_ELEVATION)
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


def _feed_stationary(tr, frames, bin_=43.5, doppler=5.5):
    for frame in frames:
        tr.update(frame, [target(frame, bin_, confidence=0.9, doppler=doppler)])


def test_a_third_point_in_the_same_bin_releases_the_track(lib):
    tr = Tracker(lib)
    _feed_stationary(tr, range(1, 3))  # two points in bin 44 (43.5 rounds up): allowed
    assert tr.track.active == 1 and tr.track.count == 2
    assert tr.update(3, [target(3, 43.6, doppler=5.5)]) is False  # a third: not the club
    assert tr.track.active == 0 and tr.why() == "released"
    assert tr.track.counters[WHY.index("released")] == 1
    assert tr.track.releasedValid == 1 and tr.track.releasedBin == pytest.approx(43.6)
    # The same return is not taken back while it stands there ...
    assert tr.update(4, [target(4, 44.0, doppler=5.0)]) is False
    assert tr.why() == "idle"
    # ... but a club sweeping through its range at another Doppler is.
    assert tr.update(5, [target(5, 42.0, doppler=-3.0)]) is True
    assert tr.why() == "acquired" and tr.track.releasedValid == 0


def test_the_repeat_count_is_consecutive_and_per_rounded_bin(lib):
    tr = Tracker(lib)
    # 30.2 and 30.4 share bin 30, 30.6 is bin 31: a club crossing a bin
    # boundary slowly is not a repeat.
    for frame, bin_ in enumerate([30.2, 30.4, 30.6, 31.2, 31.9, 32.8], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert tr.track.counters[WHY.index("released")] == 0 and tr.track.count == 6


def test_a_repeat_releases_on_the_frame_the_third_point_arrives(lib):
    """The check is made as the point is offered, so the new frame's other
    targets can seed the next track at once."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([43.1, 43.9], start=2):  # bins 43, 44
        tr.update(frame, [target(frame, bin_, doppler=5.5)])
    tr.update(4, [target(4, 44.2, doppler=5.5)])  # 44 again: 2 in bin 44, kept
    assert tr.track.active == 1
    tr.update(5, [target(5, 44.1, doppler=5.4), target(5, 30.4, doppler=7.9)])
    assert tr.track.counters[WHY.index("released")] == 1
    assert tr.why() == "acquired" and tr.points()[-1].rangeBin == pytest.approx(30.4)


@pytest.mark.parametrize("override", [{"maxSameBinPoints": 0}])
def test_the_repeat_rule_can_be_switched_off(lib, override):
    tr = Tracker(lib, **override)
    _feed_stationary(tr, range(1, 20))
    assert tr.track.active == 1 and tr.track.count == 19
    assert tr.track.counters[WHY.index("released")] == 0


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
    tr2 = Tracker(lib)
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
    tr = Tracker(lib, ascendingOnly=0)
    for frame, bin_ in enumerate([30.0, 29.2, 28.6], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert tr.track.count == 3


def test_reset_forgets_a_release(lib):
    tr = Tracker(lib)
    _feed_stationary(tr, range(1, 4))
    assert tr.track.releasedValid == 1
    lib.l3_track_reset(ctypes.byref(tr.track))
    assert tr.track.releasedValid == 0
    assert tr.update(4, [target(4, 43.5, doppler=5.5)]) is True  # may be acquired again


def _follow(tr, frame, targets):
    arr = (Target * max(1, len(targets)))(*targets)
    return bool(
        tr.lib.l3_track_follow(ctypes.byref(tr.track), arr, len(targets), frame, frame * FRAME_US)
    )


def test_follow_continues_an_active_track_by_association(lib):
    """After impact the club track carries on: the first of the two tracks."""
    tr = Tracker(lib)
    for frame, bin_ in enumerate([30.0, 31.1, 32.2, 33.3], start=1):
        tr.update(frame, [target(frame, bin_)])
    assert _follow(tr, 5, [target(5, 40.0, confidence=0.95), target(5, 34.0)]) is True
    assert tr.points()[-1].rangeBin == pytest.approx(34.0) and tr.why() == "associated"


def test_follow_never_acquires_and_never_releases(lib):
    tr = Tracker(lib)
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
    tr = Tracker(lib)
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
        tr = Tracker(lib)
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
    tr = Tracker(lib)
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
    tr = Tracker(lib)
    for frame, bin_ in enumerate([32.9, 34.4, 35.9, 37.4], start=1):  # 1.5 bins/frame
        tr.update(frame, [target(frame, bin_)])
    assert _follow(tr, 5, [target(5, 38.1)]) is True  # stalling
    assert _follow(tr, 6, [target(6, 38.4)]) is True
    # 38.4 + 1.5 + 0.5 = 40.4: 40.1 is reachable at the impact speed ...
    assert _follow(tr, 7, [target(7, 40.1)]) is True
    # ... and 42.2 (2.1 bins in one frame) is faster than the club arrived.
    assert _follow(tr, 8, [target(8, 42.2)]) is False
