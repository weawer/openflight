"""Tests for the club's trajectory reconstruction, firmware/iwr6843/l3_track_kf.c.

A constant-velocity EKF over the club track's held points, then an RTS
smoother: range is trusted (small sigma), azimuth and elevation are weak
(large sigma, scaled by 1 / angle confidence) and pass a 2-dof chi-square
gate on their innovation; a point that fails still updates on its range. The
clubhead here swings on a 1.1 m arc at ~24 m/s, identity calibration.
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
BOTH = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
HYP = {name: index for index, name in enumerate(fw.FILTER_HYP_NAMES)}
WHY = {name: index for index, name in enumerate(fw.TRACK_KF_WHY_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def arc(t_s: float) -> tuple[float, float, float]:
    """A clubhead on a 1.1 m arc, sweeping 0.6 rad in 27 ms toward the ball."""
    theta = -0.6 + 22.0 * t_s
    return (1.2 + 1.1 * math.sin(theta), 0.1 * math.sin(theta), 1.0 - 1.1 * math.cos(theta))


class Club:
    def __init__(self, lib, *, frames=10, start_us=FRAME_US, skip=(), step_us=FRAME_US, **kf):
        self.lib = lib
        cfg = fw.TrackCfg()
        lib.l3_track_cfg_defaults(ctypes.byref(cfg))
        cfg.maxAngleResidualM = 0.0
        for name, value in kf.items():
            setattr(cfg.kf, name, value)
        self.track = fw.ClubTrack()
        lib.l3_track_init(ctypes.byref(self.track), ctypes.byref(cfg))
        self.truth = []
        self.times = []
        for k in range(frames):
            if k in skip:
                continue
            stamp = (start_us + k * step_us) & 0xFFFFFFFF
            p = arc(k * step_us * 1e-6)
            point = fw.TrackPoint()
            point.frame = k
            point.timestampUs = stamp
            point.rangeM = math.sqrt(sum(c * c for c in p))
            point.rangeBin = point.rangeM / BIN_M
            lib.l3_track_append_point(ctypes.byref(self.track), ctypes.byref(point))
            self.truth.append(p)
            self.times.append(stamp)
        for index, p in enumerate(self.truth):
            self.measure(index, p)

    def measure(self, index, golf, *, az_err=0.0, el_err=0.0, range_err=0.0, confidence=1.0, flags=BOTH):
        radar = fw.Vec3()
        self.lib.l3_frames_golf_to_radar(
            ctypes.byref(self.track.cfg.cal), ctypes.byref(fw.Vec3(*golf)), ctypes.byref(radar)
        )
        sph = fw.Spherical()
        self.lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(sph))
        if range_err:
            # Range noise: nudge the held ring slot's rangeM through the ctypes
            # mirror; set_point_angles below relocates the point from it.
            slot = (self.track.next + fw.TRACK_POINTS - self.track.count + index) % fw.TRACK_POINTS
            self.track.points[slot].rangeM = sph.rangeM + range_err
        assert (
            self.lib.l3_track_set_point_angles(
                ctypes.byref(self.track), index, sph.azimuthRad + az_err, sph.elevationRad + el_err, flags, confidence
            )
            == 1
        )

    def run(self, cfg=None):
        work = ctypes.create_string_buffer(self.lib.l3_track_kf_work_bytes())
        out = fw.TrackKfResult()
        accepted = self.lib.l3_track_kf_run(
            ctypes.byref(cfg or self.track.cfg.kf), ctypes.byref(self.track), work, ctypes.byref(out)
        )
        return accepted, out

    def point(self, index) -> fw.TrackPoint:
        point = fw.TrackPoint()
        assert self.lib.l3_track_point(ctypes.byref(self.track), index, ctypes.byref(point)) == 1
        return point

    def errors(self, attr):
        out = []
        for index, truth in enumerate(self.truth):
            v = getattr(self.point(index), attr)
            out.append(math.dist((v.x, v.y, v.z), truth))
        return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def test_defaults_come_from_one_place(lib):
    cfg = fw.TrackKfCfg()
    lib.l3_track_kf_cfg_defaults(ctypes.byref(cfg))
    track_cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
    assert bytes(track_cfg.kf) == bytes(cfg)
    assert cfg.accelSigmaMps2 == pytest.approx(1500.0)


def test_why_names_match_the_firmware(lib):
    for index, name in enumerate(fw.TRACK_KF_WHY_NAMES):
        assert lib.l3_track_kf_why_name(index).decode() == name


def test_the_work_area_fits_the_mss(lib):
    assert 0 < lib.l3_track_kf_work_bytes() <= 12 * 1024


def test_noise_free_points_are_reproduced(lib):
    club = Club(lib)
    accepted, out = club.run()
    assert out.why == WHY["ok"] and out.points == 10 and accepted == 9 == out.accepted
    assert max(club.errors("filteredPosition")) < 0.05
    assert club.point(0).filterHypothesis == HYP["none"], "the first point seeds the state"
    assert all(club.point(i).filterHypothesis == HYP["direct"] for i in range(1, 10))


def test_the_smoothed_track_beats_the_raw_angles(lib):
    """Fixed-seed: 10 deg angle noise, 1 cm range noise, 20 swings."""
    rng = random.Random(7)
    raw, smoothed = [], []
    for _ in range(20):
        club = Club(lib)
        for index, p in enumerate(club.truth):
            club.measure(
                index, p, az_err=rng.gauss(0, 10.0) * DEG, el_err=rng.gauss(0, 10.0) * DEG,
                range_err=rng.gauss(0, 0.01),
            )
        club.run()
        raw += club.errors("position")
        smoothed += club.errors("filteredPosition")
    # measured 0.56 (2026-10-01); the requirement is smoothed below raw
    assert rms(smoothed) < 0.65 * rms(raw), (rms(smoothed), rms(raw))


def test_an_angle_jump_is_gated_but_its_range_still_counts(lib):
    club = Club(lib)
    # Floor multipath corrupts both axes. At angleSigma 15 deg a 40/40 jump measured chi2 7.8:
    # inside the noise model, so the filter down-weights it; 60/60 is beyond it (chi2 ~17.5).
    club.measure(5, club.truth[5], az_err=60.0 * DEG, el_err=60.0 * DEG)
    club.run()
    jumped = club.point(5)
    assert jumped.filterAccepted == 0 and jumped.filterHypothesis == HYP["none"]
    f = jumped.filteredPosition
    assert math.dist((f.x, f.y, f.z), club.truth[5]) < 0.1
    assert math.hypot(f.x, f.y, f.z) == pytest.approx(jumped.rangeM, abs=0.02)
    assert all(club.point(i).filterAccepted == 1 for i in range(1, 10) if i != 5)


def test_a_gap_uses_the_real_time_step(lib):
    club = Club(lib, frames=12, skip=(5, 6))
    club.run()
    assert max(club.errors("filteredPosition")) < 0.08


def test_timestamps_wrapping_past_2_to_the_32_filter_the_same(lib):
    """Review Focus 2."""
    plain = Club(lib)
    plain.run()
    wrapped = Club(lib, start_us=2**32 - 4 * FRAME_US)
    assert wrapped.times[4] < wrapped.times[3], "the setup must wrap mid-track"
    wrapped.run()
    for index in range(10):
        a, b = plain.point(index).filteredPosition, wrapped.point(index).filteredPosition
        assert (a.x, a.y, a.z) == pytest.approx((b.x, b.y, b.z), abs=1e-4)


def test_a_repeated_timestamp_neither_crashes_nor_goes_nan(lib):
    """Review Focus 4."""
    club = Club(lib, frames=6)
    point = fw.TrackPoint()
    lib.l3_track_point(ctypes.byref(club.track), 5, ctypes.byref(point))
    lib.l3_track_append_point(ctypes.byref(club.track), ctypes.byref(point))
    club.truth.append(club.truth[5])
    club.measure(6, club.truth[6])
    _, out = club.run()
    assert out.why == WHY["ok"]
    for index in range(7):
        f = club.point(index).filteredPosition
        assert all(math.isfinite(c) for c in (f.x, f.y, f.z))


def test_fewer_than_three_points_are_left_unfiltered(lib):
    club = Club(lib, frames=2)
    accepted, out = club.run()
    assert accepted == 0 and out.why == WHY["few_points"]
    assert all(club.point(i).filterHypothesis == HYP["unfiltered"] for i in range(2))


def test_a_degenerate_filter_resets_to_raw_without_nans(lib):
    """No process noise, no initial uncertainty and zero measurement variance:
    the first range update has S = 0, which is divergence, not a crash."""
    club = Club(lib, accelSigmaMps2=0.0, initPositionSigmaM=0.0, initVelocitySigmaMps=0.0, rangeSigmaM=0.0)
    accepted, out = club.run()
    assert accepted == 0 and out.why == WHY["diverged"]
    for index in range(10):
        point = club.point(index)
        assert point.filterHypothesis == HYP["unfiltered"]
        assert (point.filteredPosition.x, point.filteredPosition.y, point.filteredPosition.z) == (
            point.position.x,
            point.position.y,
            point.position.z,
        )


def test_the_filtered_delivery_reads_the_reconstruction(lib):
    rng = random.Random(11)
    raw_err, filtered_err = [], []
    for _ in range(20):
        club = Club(lib)
        for index, p in enumerate(club.truth):
            club.measure(index, p, az_err=rng.gauss(0, 10.0) * DEG, el_err=rng.gauss(0, 10.0) * DEG)
        club.run()
        # The delivery is a line over the newest 8 points (indices 2..9): their chord.
        start, end = arc(2 * FRAME_US * 1e-6), arc(9 * FRAME_US * 1e-6)
        truth_path = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        raw, filtered = fw.Delivery(), fw.Delivery()
        lib.l3_track_delivery(ctypes.byref(club.track), 8, ctypes.byref(raw))
        lib.l3_track_delivery_filtered(ctypes.byref(club.track), 8, ctypes.byref(filtered))
        if raw.pathValid:
            raw_err.append(math.degrees(raw.pathRad) - truth_path)
        assert filtered.pathValid
        filtered_err.append(math.degrees(filtered.pathRad) - truth_path)
    assert rms(filtered_err) < rms(raw_err)


def test_an_unfiltered_track_delivers_exactly_as_before(lib):
    club = Club(lib)
    raw, filtered = fw.Delivery(), fw.Delivery()
    lib.l3_track_delivery(ctypes.byref(club.track), 8, ctypes.byref(raw))
    lib.l3_track_delivery_filtered(ctypes.byref(club.track), 8, ctypes.byref(filtered))
    assert bytes(raw) == bytes(filtered)


def test_a_gated_point_still_counts_in_the_filtered_delivery(lib):
    club = Club(lib)
    club.measure(5, club.truth[5], az_err=60.0 * DEG, el_err=60.0 * DEG)
    club.run()
    assert club.point(5).filterAccepted == 0
    out = fw.Delivery()
    lib.l3_track_delivery_filtered(ctypes.byref(club.track), 8, ctypes.byref(out))
    assert out.points == 8 and out.pathValid


def _filtered_at_5(lib, confidence, err_deg, **kf):
    club = Club(lib, **kf)
    club.measure(5, club.truth[5], az_err=err_deg * DEG, el_err=err_deg * DEG, confidence=confidence)
    club.run()
    return club


def test_a_low_confidence_angle_counts_for_less(lib):
    high = _filtered_at_5(lib, 1.0, 30.0)
    low = _filtered_at_5(lib, 0.1, 30.0)
    assert low.errors("filteredPosition")[5] < high.errors("filteredPosition")[5]


def test_confidence_below_the_floor_uses_the_floor(lib):
    floor = _filtered_at_5(lib, 0.05, 30.0)
    below = _filtered_at_5(lib, 0.01, 30.0)
    assert bytes(floor.point(5).filteredPosition) == bytes(below.point(5).filteredPosition)


def test_a_failed_smoother_solve_unfilters(lib):
    club = Club(lib, accelSigmaMps2=0.0, initVelocitySigmaMps=0.0)
    accepted, out = club.run()
    assert accepted == 0 and out.why == WHY["diverged"]
    for index in range(10):
        point = club.point(index)
        assert point.filterHypothesis == HYP["unfiltered"]
        f = point.filteredPosition
        assert all(math.isfinite(c) for c in (f.x, f.y, f.z))


def test_a_point_with_one_angle_updates_on_range_only(lib):
    club = Club(lib)
    club.measure(4, club.truth[4], flags=fw.ANGLE_AZIMUTH)
    club.run()
    point = club.point(4)
    assert point.filterAccepted == 0 and point.filterHypothesis == HYP["none"]
    f = point.filteredPosition
    assert math.hypot(f.x, f.y, f.z) == pytest.approx(point.rangeM, abs=0.02)
