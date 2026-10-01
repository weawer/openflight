"""Tests for the tee-anchored ball direction fit, firmware/iwr6843/l3_ball_fit.c.

Over the ~30 ms a ball track spans, gravity moves it ~3 mm, so its path is a
straight line from the tee. The fit takes each point's range as exact, the
tee as the anchor, and finds only the direction (HLA, VLA) that best explains
the points' measured angles, scoring each against both the direct return and
its floor reflection. Angles further than gateK sigma from the fit are
rejected. Identity calibration throughout: the radar frame is the golf frame.
"""

from __future__ import annotations

import ctypes
import math
import random

import pytest

from openflight.iwr6843 import firmware_host as fw

DEG = math.pi / 180.0
BIN_M = 6.0 / 128
FRAME_US = 3000
IMPACT_US = 21_700
ORIGIN_BIN = 38.0
BOTH = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
HYP = {name: index for index, name in enumerate(fw.FILTER_HYP_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def defaults(lib, **overrides) -> fw.BallFitCfg:
    cfg = fw.BallFitCfg()
    lib.l3_ball_fit_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    return cfg


def direction(hla_deg, vla_deg):
    h, v = hla_deg * DEG, vla_deg * DEG
    return (math.cos(v) * math.cos(h), math.cos(v) * math.sin(h), math.sin(v))


class Shot:
    """A ball leaving the fit's tee anchor along (hla, vla): one point per frame
    on a track core, each with its true direction as the measured angles."""

    def __init__(self, lib, *, hla_deg, vla_deg, speed=50.0, frames=8, first_frame=8, cfg=None):
        self.lib = lib
        self.cfg = cfg or defaults(lib)
        tcfg = fw.TrackCfg()
        lib.l3_track_cfg_defaults(ctypes.byref(tcfg))
        tcfg.maxAngleResidualM = 0.0  # the raw line fit, for comparisons, never gives up
        self.core = fw.ClubTrack()
        lib.l3_track_init(ctypes.byref(self.core), ctypes.byref(tcfg))
        self.origin = fw.Vec3(ORIGIN_BIN * BIN_M, 0.0, 0.0)
        height = self.cfg.teeBallHeightM - self.cfg.radarHeightM
        rng = ORIGIN_BIN * BIN_M
        self.tee = (math.sqrt(rng * rng - height * height), 0.0, height)
        u = direction(hla_deg, vla_deg)
        self.truth = []
        for index, frame in enumerate(range(first_frame, first_frame + frames)):
            s = speed * (frame * FRAME_US - IMPACT_US) * 1e-6
            p = tuple(self.tee[i] + s * u[i] for i in range(3))
            self.truth.append(p)
            point = fw.TrackPoint()
            point.frame = frame
            point.timestampUs = frame * FRAME_US
            point.rangeM = math.sqrt(sum(c * c for c in p))
            point.rangeBin = point.rangeM / BIN_M
            lib.l3_track_append_point(ctypes.byref(self.core), ctypes.byref(point))
            self.measure(index, p)

    def angles_of(self, golf):
        cal = self.core.cfg.cal
        radar = fw.Vec3()
        self.lib.l3_frames_golf_to_radar(
            ctypes.byref(cal), ctypes.byref(fw.Vec3(*golf)), ctypes.byref(radar)
        )
        sph = fw.Spherical()
        self.lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        return sph.azimuthRad, sph.elevationRad

    def measure(self, index, golf, *, az_err=0.0, el_err=0.0, confidence=1.0, flags=BOTH):
        az, el = self.angles_of(golf)
        assert (
            self.lib.l3_track_set_point_angles(
                ctypes.byref(self.core), index, az + az_err, el + el_err, flags, confidence
            )
            == 1
        )

    def reflect(self, index):
        x, y, z = self.truth[index]
        self.measure(index, (x, y, -2.0 * self.cfg.radarHeightM - z))

    def run(self, origin=None):
        fit = fw.BallFit()
        accepted = self.lib.l3_ball_fit_run(
            ctypes.byref(self.cfg),
            ctypes.byref(origin or self.origin),
            ctypes.byref(self.core),
            ctypes.byref(fit),
        )
        return accepted, fit

    def point(self, index) -> fw.TrackPoint:
        point = fw.TrackPoint()
        assert self.lib.l3_track_point(ctypes.byref(self.core), index, ctypes.byref(point)) == 1
        return point

    def raw_line_direction(self):
        """The unconstrained 3D line fit over the raw points (l3_delivery_fit)."""
        out = fw.Delivery()
        n = self.core.count
        self.lib.l3_track_delivery_range(ctypes.byref(self.core), 0, n, n, ctypes.byref(out))
        v = out.velocity
        return math.degrees(math.atan2(v.y, v.x)), math.degrees(math.atan2(v.z, math.hypot(v.x, v.y)))


def why(fit) -> str:
    return fw.BALL_FIT_WHY_NAMES[fit.why]


def test_defaults_are_the_measured_scatter_and_a_three_level_grid(lib):
    cfg = defaults(lib)
    assert cfg.angleSigmaRad == pytest.approx(12.0 * DEG)
    assert cfg.gateK == pytest.approx(2.5) and cfg.huberK == pytest.approx(1.5)
    assert cfg.minAccepted == 4 and cfg.maxRmsRad == pytest.approx(15.0 * DEG)
    assert cfg.imageSepMinRad == pytest.approx(2.0 * DEG)
    assert cfg.radarHeightM == pytest.approx(0.152) and cfg.teeBallHeightM == pytest.approx(0.04)
    assert (cfg.hlaMinRad, cfg.hlaMaxRad) == pytest.approx((-45.0 * DEG, 45.0 * DEG))
    assert (cfg.vlaMinRad, cfg.vlaMaxRad) == pytest.approx((-10.0 * DEG, 60.0 * DEG))
    assert cfg.gridSteps == 10 and cfg.gridLevels == 3
    assert cfg.maxAngleSigmaRad == pytest.approx(3.0 * DEG)


def test_why_names_match_the_firmware(lib):
    for index, name in enumerate(fw.BALL_FIT_WHY_NAMES):
        assert lib.l3_ball_fit_why_name(index).decode() == name


@pytest.mark.parametrize(
    "hla_deg,vla_deg", [(0.0, 12.0), (3.0, 10.0), (-4.5, 15.0), (1.2, 25.0), (8.0, 35.0)]
)
def test_noise_free_points_give_back_the_direction_and_the_line(lib, hla_deg, vla_deg):
    shot = Shot(lib, hla_deg=hla_deg, vla_deg=vla_deg)
    accepted, fit = shot.run()
    assert why(fit) == "ok" and fit.valid == 1 and accepted == 8 == fit.accepted
    assert fit.hlaRad / DEG == pytest.approx(hla_deg, abs=0.3)
    assert fit.vlaRad / DEG == pytest.approx(vla_deg, abs=0.3)
    assert (fit.tee.x, fit.tee.y, fit.tee.z) == pytest.approx(shot.tee, abs=1e-4)
    for sigma in (fit.hlaSigmaRad, fit.vlaSigmaRad):
        assert math.isfinite(sigma) and 0.0 < sigma < 1.0 * DEG
    for index, truth in enumerate(shot.truth):
        point = shot.point(index)
        assert point.filterAccepted == 1 and point.filterHypothesis == HYP["direct"]
        filtered = (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z)
        assert filtered == pytest.approx(truth, abs=0.01)


def test_noisy_angles_fit_better_than_the_unconstrained_line(lib):
    """Fixed-seed Monte Carlo: 1 deg angle noise per point. The tee anchor and
    the exact ranges leave two unknowns, so the fit beats the free 3D line.
    If the 3 deg bound fails with a correct implementation, report the RMS
    achieved; do not loosen the bound silently."""
    rng = random.Random(20261001)
    fit_err, line_err = [], []
    for _ in range(40):
        shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
        for index, truth in enumerate(shot.truth):
            shot.measure(index, truth, az_err=rng.gauss(0, 1.0) * DEG, el_err=rng.gauss(0, 1.0) * DEG)
        _, fit = shot.run()
        assert fit.valid
        fit_err.append(math.hypot(fit.hlaRad / DEG - 2.0, fit.vlaRad / DEG - 14.0))
        hla, vla = shot.raw_line_direction()
        line_err.append(math.hypot(hla - 2.0, vla - 14.0))
    fit_rms = math.sqrt(sum(e * e for e in fit_err) / len(fit_err))
    line_rms = math.sqrt(sum(e * e for e in line_err) / len(line_err))
    assert fit_rms <= 3.0
    assert fit_rms < 0.5 * line_rms, (fit_rms, line_rms)


def test_wild_angles_are_gated_out_and_marked(lib):
    shot = Shot(lib, hla_deg=1.0, vla_deg=15.0)
    for index in (2, 5):
        shot.measure(index, shot.truth[index], az_err=40.0 * DEG, el_err=-40.0 * DEG)
    accepted, fit = shot.run()
    assert fit.valid and accepted == 6
    assert fit.hlaRad / DEG == pytest.approx(1.0, abs=0.5)
    assert fit.vlaRad / DEG == pytest.approx(15.0, abs=0.5)
    for index in range(8):
        point = shot.point(index)
        if index in (2, 5):
            assert point.filterAccepted == 0 and point.filterHypothesis == HYP["none"]
        else:
            assert point.filterAccepted == 1
    wild = shot.point(2)
    assert (wild.filteredPosition.x, wild.filteredPosition.y, wild.filteredPosition.z) == pytest.approx(
        shot.truth[2], abs=0.01
    ), "a rejected point still lies on the fitted line at its range"


def test_floor_reflections_count_for_the_same_direction(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=20.0)
    for index in (0, 2, 4, 6):
        shot.reflect(index)
    accepted, fit = shot.run()
    assert fit.valid and accepted == 8
    assert fit.vlaRad / DEG == pytest.approx(20.0, abs=0.5)
    labels = [shot.point(i).filterHypothesis for i in range(8)]
    assert all(labels[i] in (HYP["image"], HYP["ambiguous"]) for i in (0, 2, 4, 6))
    assert sum(labels[i] == HYP["image"] for i in (0, 2, 4, 6)) >= 2
    assert all(labels[i] == HYP["direct"] for i in (1, 3, 5, 7))


def test_too_few_angles_report_no_direction_and_leave_points_unfiltered(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    for index in range(3, 8):
        shot.measure(index, shot.truth[index], flags=0, confidence=0.0)
    accepted, fit = shot.run()
    assert accepted == 0 and fit.valid == 0 and why(fit) == "few_angles"
    for index in range(8):
        point = shot.point(index)
        assert point.filterHypothesis == HYP["unfiltered"]
        assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
            point.position.x,
            point.position.y,
            point.position.z,
        )


def test_pure_noise_reports_no_direction(lib):
    """Noise can be explained by an extreme direction under this geometry; what
    matters is that no direction is reported."""
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    for index, truth in enumerate(shot.truth):
        sign = 1.0 if index % 2 else -1.0
        shot.measure(index, truth, az_err=sign * 25.0 * DEG, el_err=-sign * 25.0 * DEG)
    _, fit = shot.run()
    assert fit.valid == 0 and why(fit) in ("scatter", "grid_edge")
    assert all(shot.point(i).filterHypothesis == HYP["unfiltered"] for i in range(8))


def test_scattered_angles_report_scatter(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0, cfg=defaults(lib, maxRmsRad=5.0 * DEG))
    for index, truth in enumerate(shot.truth):
        sign = 1.0 if index % 2 else -1.0
        shot.measure(index, truth, az_err=sign * 8.0 * DEG, el_err=-sign * 8.0 * DEG)
    _, fit = shot.run()
    assert fit.valid == 0 and why(fit) == "scatter", (
        fit.hlaRad / DEG, fit.vlaRad / DEG, fit.rmsRad / DEG, fit.accepted)
    assert all(shot.point(i).filterHypothesis == HYP["unfiltered"] for i in range(8))


def test_a_best_fit_on_the_search_limit_is_not_reported(lib):
    shot = Shot(lib, hla_deg=60.0, vla_deg=12.0)
    _, fit = shot.run()
    assert fit.valid == 0 and why(fit) == "grid_edge"


def test_points_short_of_the_tee_are_skipped(lib):
    """Frames before impact put the line behind the tee: no forward root."""
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0, first_frame=6, frames=10)
    accepted, fit = shot.run()
    assert fit.valid
    behind = [i for i, p in enumerate(shot.truth) if math.dist(p, (0, 0, 0)) <= math.dist(shot.tee, (0, 0, 0))]
    assert behind, "the setup must put a point short of the tee"
    for index in behind:
        assert shot.point(index).filterHypothesis == HYP["unfiltered"]
    assert accepted == 10 - len(behind)


def test_a_zero_confidence_angle_has_no_say(lib):
    clean = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    _, reference = clean.run()
    shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    shot.measure(3, shot.truth[3], az_err=30.0 * DEG, el_err=30.0 * DEG, confidence=0.0)
    _, fit = shot.run()
    assert (fit.hlaRad, fit.vlaRad) == (reference.hlaRad, reference.vlaRad)
    point = shot.point(3)
    assert point.filterAccepted == 0 and point.filterHypothesis == HYP["none"]


def test_a_point_with_one_angle_is_not_weighted(lib):
    shot = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    shot.measure(4, shot.truth[4], el_err=30.0 * DEG, flags=fw.ANGLE_ELEVATION)
    _, fit = shot.run()
    assert fit.valid and fit.used == 7
    assert shot.point(4).filterHypothesis == HYP["none"]


def test_a_zero_origin_has_no_tee(lib):
    """Review Focus 3: armed with no destination measured."""
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0)
    accepted, fit = shot.run(origin=fw.Vec3(0.0, 0.0, 0.0))
    assert accepted == 0 and why(fit) == "no_tee"
    assert all(shot.point(i).filterHypothesis == HYP["unfiltered"] for i in range(8))


def test_the_search_stays_inside_its_evaluation_budget(lib):
    cfg = defaults(lib)
    assert lib.l3_ball_fit_max_evaluations(ctypes.byref(cfg)) == 2 * 3 * 11 * 11 + 9
    clean = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    _, fit = clean.run()
    assert fit.evaluations == 3 * 11 * 11 + 9, "nothing gated: the refit is skipped"
    wild = Shot(lib, hla_deg=2.0, vla_deg=14.0)
    wild.measure(2, wild.truth[2], az_err=40.0 * DEG, el_err=40.0 * DEG)
    _, fit = wild.run()
    assert fit.evaluations == 2 * 3 * 11 * 11 + 9


def noisy_shots(lib, noise_deg, count, seed, **cfg):
    rng = random.Random(seed)
    for _ in range(count):
        shot = Shot(lib, hla_deg=2.0, vla_deg=14.0, cfg=defaults(lib, **cfg))
        for index, truth in enumerate(shot.truth):
            shot.measure(
                index, truth, az_err=rng.gauss(0, noise_deg) * DEG, el_err=rng.gauss(0, noise_deg) * DEG
            )
        _, fit = shot.run()
        yield fit, math.hypot(fit.hlaRad / DEG - 2.0, fit.vlaRad / DEG - 14.0)


def test_realistic_angle_scatter_never_reports_a_confident_wrong_direction(lib):
    """At the measured 12 deg scatter the geometry (the radar looks down the
    flight line) cannot support a direction: the fit must say so, not guess."""
    wrong = valid = 0
    for fit, err in noisy_shots(lib, 12.0, 200, 20261002):
        if fit.valid:
            valid += 1
            assert fit.hlaSigmaRad <= 3.0 * DEG + 1e-6 and fit.vlaSigmaRad <= 3.0 * DEG + 1e-6
            wrong += err > 9.0
    assert wrong <= 10


def test_small_angle_noise_still_reports_a_direction(lib):
    results = list(noisy_shots(lib, 1.0, 100, 20261003))
    valid = [(f, e) for f, e in results if f.valid]
    assert len(valid) >= 95
    for fit, err in valid:
        assert err <= 3.0 * max(fit.hlaSigmaRad, fit.vlaSigmaRad) / DEG + 0.5


def test_too_few_minaccepted_is_clamped(lib):
    shot = Shot(lib, hla_deg=0.0, vla_deg=12.0, cfg=defaults(lib, minAccepted=0))
    for index in range(2, 8):
        shot.measure(index, shot.truth[index], flags=0, confidence=0.0)
    accepted, fit = shot.run()
    assert accepted == 0 and why(fit) == "few_angles"
