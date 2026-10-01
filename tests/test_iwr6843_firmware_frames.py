"""Tests for the IWR6843 coordinate frames and calibration, firmware/iwr6843/l3_frames.c.

The frame and sign conventions every metric depends on are defined in
l3_frames.h; these tests are their executable statement, checked against
numpy rotations and the host's own conventions (ballistics: x downrange, y
right, z up; club path and horizontal launch positive right; vertical angles
positive up).
"""

from __future__ import annotations

import ctypes
import math

import numpy as np
import pytest

from openflight.iwr6843 import firmware_host as fw

DEG = math.pi / 180.0


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def cal(lib, **attitude) -> fw.RadarCal:
    c = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(c), 8)
    for name, value in attitude.items():
        setattr(c, name, value)
    return c


def vec(v: fw.Vec3) -> tuple[float, float, float]:
    return (v.x, v.y, v.z)


def radar_to_golf(lib, c, xyz):
    radar = fw.Vec3(*xyz)
    golf = fw.Vec3()
    lib.l3_frames_radar_to_golf(ctypes.byref(c), ctypes.byref(radar), ctypes.byref(golf))
    return vec(golf)


def test_identity_calibration_is_unit_corrections_and_level_attitude(lib):
    c = cal(lib)
    assert c.virtualElements == 8
    assert list(c.correctionRe) == [1.0] * 8 and list(c.correctionIm) == [0.0] * 8
    assert c.radarPitchRad == c.radarYawRad == c.radarRollRad == c.rangeBiasM == 0.0
    lib.l3_cal_identity(ctypes.byref(c), 99)
    assert c.virtualElements == 8, "clamped to the array size"


@pytest.mark.parametrize(
    "range_m,az_deg,el_deg",
    [(2.0, 0.0, 0.0), (1.5, 10.0, 0.0), (1.5, 0.0, -12.0), (3.0, -20.0, 15.0)],
)
def test_spherical_round_trip_matches_the_header_definition(lib, range_m, az_deg, el_deg):
    sph = fw.Spherical(range_m, az_deg * DEG, el_deg * DEG)
    radar = fw.Vec3()
    lib.l3_frames_from_spherical(ctypes.byref(sph), ctypes.byref(radar))
    expected = (
        range_m * math.cos(el_deg * DEG) * math.cos(az_deg * DEG),
        range_m * math.cos(el_deg * DEG) * math.sin(az_deg * DEG),
        range_m * math.sin(el_deg * DEG),
    )
    assert vec(radar) == pytest.approx(expected, abs=1e-5)
    back = fw.Spherical()
    lib.l3_frames_to_spherical(ctypes.byref(radar), ctypes.byref(back))
    assert (back.rangeM, back.azimuthRad, back.elevationRad) == pytest.approx(
        (range_m, az_deg * DEG, el_deg * DEG), abs=1e-5
    )


def test_azimuth_is_positive_to_the_right_and_elevation_positive_up(lib):
    right = fw.Vec3(1.0, 0.5, 0.0)
    up = fw.Vec3(1.0, 0.0, 0.5)
    sph = fw.Spherical()
    lib.l3_frames_to_spherical(ctypes.byref(right), ctypes.byref(sph))
    assert sph.azimuthRad > 0 and sph.elevationRad == pytest.approx(0.0)
    lib.l3_frames_to_spherical(ctypes.byref(up), ctypes.byref(sph))
    assert sph.elevationRad > 0 and sph.azimuthRad == pytest.approx(0.0)


def test_level_attitude_makes_radar_and_golf_frames_identical(lib):
    c = cal(lib)
    assert radar_to_golf(lib, c, (1.0, 0.2, -0.3)) == pytest.approx((1.0, 0.2, -0.3))


def test_a_nose_up_radar_sees_a_level_target_below_boresight(lib):
    """Pitch: the calibration's tilt (10.4 deg on the reference rig)."""
    pitch = 10.4 * DEG
    c = cal(lib, radarPitchRad=pitch)
    # A target 2 m ahead on the radar's own horizontal plane appears at -pitch.
    sph = fw.Spherical(2.0, 0.0, -pitch)
    radar = fw.Vec3()
    lib.l3_frames_from_spherical(ctypes.byref(sph), ctypes.byref(radar))
    assert radar_to_golf(lib, c, vec(radar)) == pytest.approx((2.0, 0.0, 0.0), abs=1e-5)
    # And boresight itself points up in the golf frame.
    x, _, z = radar_to_golf(lib, c, (1.0, 0.0, 0.0))
    assert z == pytest.approx(math.sin(pitch)) and x == pytest.approx(math.cos(pitch))


def test_a_radar_yawed_right_sees_the_target_line_to_its_left(lib):
    yaw = 5.0 * DEG
    c = cal(lib, radarYawRad=yaw)
    on_line_in_radar = (math.cos(yaw), -math.sin(yaw), 0.0)  # azimuth -yaw
    assert radar_to_golf(lib, c, on_line_in_radar) == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)
    _, y, _ = radar_to_golf(lib, c, (1.0, 0.0, 0.0))
    assert y > 0, "boresight points right of the target line"


def test_a_radar_rolled_right_side_down_sees_a_target_on_its_right_lower(lib):
    roll = 8.0 * DEG
    c = cal(lib, radarRollRad=roll)
    golf = fw.Vec3(0.0, 1.0, 0.0)
    radar = fw.Vec3()
    lib.l3_frames_golf_to_radar(ctypes.byref(c), ctypes.byref(golf), ctypes.byref(radar))
    assert radar.y == pytest.approx(math.cos(roll)) and radar.z == pytest.approx(-math.sin(roll))


def test_radar_to_golf_matches_a_numpy_rotation_and_inverts_exactly(lib):
    roll, pitch, yaw = 3.0 * DEG, 10.0 * DEG, -4.0 * DEG
    c = cal(lib, radarRollRad=roll, radarPitchRad=pitch, radarYawRad=yaw)
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array(
        [[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]]
    )
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    rotation = rz @ ry @ rx
    rng = np.random.default_rng(3)
    for _ in range(20):
        p = rng.normal(size=3)
        golf = radar_to_golf(lib, c, tuple(p))
        assert golf == pytest.approx(tuple(rotation @ p), abs=1e-5)
        back = fw.Vec3()
        lib.l3_frames_golf_to_radar(
            ctypes.byref(c), ctypes.byref(fw.Vec3(*golf)), ctypes.byref(back)
        )
        assert vec(back) == pytest.approx(tuple(p), abs=1e-5)


def test_observe_removes_the_range_bias_but_not_the_angle_offsets(lib):
    """l3_angle_estimate already removed the offsets (the azimuth one as a
    phase); observing them again would count them twice."""
    c = cal(lib, rangeBiasM=0.066, azimuthOffsetRad=2.0 * DEG, elevationOffsetRad=-1.0 * DEG)
    golf = fw.Vec3()
    lib.l3_frames_observe(ctypes.byref(c), 2.066, 0.0, 0.0, ctypes.byref(golf))
    assert vec(golf) == pytest.approx((2.0, 0.0, 0.0), abs=1e-5)
    lib.l3_frames_observe(ctypes.byref(c), 2.066, 3.0 * DEG, -2.0 * DEG, ctypes.byref(golf))
    expected = (
        2.0 * math.cos(-2.0 * DEG) * math.cos(3.0 * DEG),
        2.0 * math.cos(-2.0 * DEG) * math.sin(3.0 * DEG),
        2.0 * math.sin(-2.0 * DEG),
    )
    assert vec(golf) == pytest.approx(expected, abs=1e-5)
    lib.l3_frames_observe(ctypes.byref(c), 0.01, 0.0, 0.0, ctypes.byref(golf))
    assert vec(golf) == pytest.approx((0.0, 0.0, 0.0)), "a range under the bias clamps at zero"


def test_velocity_angles_follow_the_documented_sign_conventions(lib):
    # Right of the target line is positive: in-to-out for a right-hander.
    in_to_out = fw.Vec3(40.0, 40.0 * math.tan(4.0 * DEG), 0.0)
    assert lib.l3_frames_horizontal_rad(ctypes.byref(in_to_out)) == pytest.approx(4.0 * DEG)
    out_to_in = fw.Vec3(40.0, -40.0 * math.tan(4.0 * DEG), 0.0)
    assert lib.l3_frames_horizontal_rad(ctypes.byref(out_to_in)) == pytest.approx(-4.0 * DEG)
    # Up is positive: an ascending club, a ball launched upward.
    ascending = fw.Vec3(40.0, 0.0, 40.0 * math.tan(3.0 * DEG))
    assert lib.l3_frames_vertical_rad(ctypes.byref(ascending)) == pytest.approx(3.0 * DEG)
    descending = fw.Vec3(40.0, 0.0, -40.0 * math.tan(3.0 * DEG))
    assert lib.l3_frames_vertical_rad(ctypes.byref(descending)) == pytest.approx(-3.0 * DEG)
    assert lib.l3_frames_speed(ctypes.byref(fw.Vec3(3.0, 4.0, 12.0))) == pytest.approx(13.0)
    zero = fw.Vec3()
    assert lib.l3_frames_horizontal_rad(ctypes.byref(zero)) == 0.0
    assert lib.l3_frames_vertical_rad(ctypes.byref(zero)) == 0.0


def test_horizontal_launch_convention_agrees_with_the_ballistics_world_frame():
    """ballistics.simulate: vy = v cos(la_v) sin(la_h), y right; the same atan2."""
    la_h = 2.5 * DEG
    vx, vy = math.cos(la_h), math.sin(la_h)
    assert math.atan2(vy, vx) == pytest.approx(la_h)


def test_element_calibration_round_trips_gain_and_phase(lib):
    c = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(c), 8)
    assert lib.l3_cal_set_element(ctypes.byref(c), 3, 1.25, 0.4) == 0
    gain = ctypes.c_float()
    phase = ctypes.c_float()
    assert lib.l3_cal_element(ctypes.byref(c), 3, ctypes.byref(gain), ctypes.byref(phase)) == 0
    assert gain.value == pytest.approx(1.25, abs=1e-5)
    assert phase.value == pytest.approx(0.4, abs=1e-5)
    # The stored correction is exp(-j phase) / gain, as l3_angle applies it.
    assert c.correctionRe[3] == pytest.approx(math.cos(-0.4) / 1.25, abs=1e-6)
    assert c.correctionIm[3] == pytest.approx(math.sin(-0.4) / 1.25, abs=1e-6)
    # Untouched elements read as unit gain, zero phase.
    assert lib.l3_cal_element(ctypes.byref(c), 0, ctypes.byref(gain), ctypes.byref(phase)) == 0
    assert (gain.value, phase.value) == (1.0, 0.0)
    assert lib.l3_cal_set_element(ctypes.byref(c), 8, 1.0, 0.0) == -1, "past the array"
    assert lib.l3_cal_set_element(ctypes.byref(c), 1, 0.0, 0.0) == -1, "a gain must be positive"
    assert lib.l3_cal_set_element(ctypes.byref(c), 1, -1.0, 0.0) == -1
    assert lib.l3_cal_element(ctypes.byref(c), 9, ctypes.byref(gain), ctypes.byref(phase)) == -1
    # Setting an element beyond the count in force extends it.
    lib.l3_cal_identity(ctypes.byref(c), 4)
    assert lib.l3_cal_set_element(ctypes.byref(c), 6, 1.0, 0.1) == 0
    assert c.virtualElements == 7


def test_calibration_formats_attitude_offsets_and_elements(lib):
    c = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(c), 8)
    c.azimuthOffsetRad, c.elevationOffsetRad, c.rangeBiasM = 0.05, -0.02, 0.031
    c.radarPitchRad, c.radarYawRad, c.radarRollRad = math.radians(3.0), math.radians(-1.5), 0.0
    text = fw.c_text(lib.l3_cal_format, ctypes.byref(c))
    assert text == "cal elems=8 az0=0.05 el0=-0.02 pitch=3.00 yaw=-1.50 roll=0.00 bias=0.03"
    lib.l3_cal_set_element(ctypes.byref(c), 2, 0.9, -0.25)
    assert (
        fw.c_text(lib.l3_cal_format_element, ctypes.byref(c), 2) == "elem 2 gain=0.90 phase=-0.25"
    )
    assert fw.c_text(lib.l3_cal_format_element, ctypes.byref(c), 0) == "elem 0 gain=1.00 phase=0.00"
    assert fw.c_text(lib.l3_cal_format_element, ctypes.byref(c), 9) == "elem 9 invalid"
