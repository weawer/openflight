"""Tests for the IWR6843 impact anchor, firmware/iwr6843/l3_ball_anchor.c.

The ball search back-projects to where and when the ball was struck: the tee
bin, and the impact time from the club's approach when that fit is tight, else
the gate time with the gate's tolerance.
"""

from __future__ import annotations

import ctypes

import pytest
from iwr6843_twotrack import BIN_M, obs

from openflight.iwr6843 import firmware_host as fw

TEE = 46.0
GATE_US = 30_000


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def fit_cfg(lib):
    cfg = fw.ImpactFitCfg()
    lib.l3_impact_fit_cfg_defaults(ctypes.byref(cfg))
    return cfg


def approach(lib, mps=30.0, frames=8, frame_us=3000, impact_us=GATE_US, jitter=()):
    """A club track closing on the tee at mps, reaching it at impact_us."""
    cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    track = fw.ClubTrack()
    lib.l3_track_init(ctypes.byref(track), ctypes.byref(cfg))
    for k in range(frames):
        ts = impact_us - (frames - k) * frame_us
        bin_ = TEE - mps * (impact_us - ts) * 1e-6 / BIN_M + (jitter[k] if k < len(jitter) else 0.0)
        target = obs(k + 1, ts, bin_, 9000.0, mps)
        lib.l3_track_update(ctypes.byref(track), ctypes.byref(target), 1, k + 1, ts)
    return track


def make(lib, club=None, max_sigma=3000.0, accept=TEE + 3.0):
    out = fw.BallAnchor()
    lib.l3_ball_anchor_make(
        TEE, accept, GATE_US, 15_000, ctypes.byref(fit_cfg(lib)),
        None if club is None else ctypes.byref(club), max_sigma, ctypes.byref(out),
    )
    return out


def test_the_struct_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallAnchor) == lib.l3_ball_anchor_struct_bytes()


def test_without_a_club_the_gate_anchors_impact(lib):
    a = make(lib)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert (a.anchorUs, a.gateUs, a.anchorTolUs) == (GATE_US, GATE_US, 15_000)
    assert (a.anchorBin, a.acceptFromBin) == (TEE, TEE + 3.0)


def test_a_clean_club_approach_anchors_impact(lib):
    club = approach(lib, impact_us=GATE_US - 4000)  # the gate fired 4 ms late
    assert club.count >= 4
    a = make(lib, club)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "club"
    assert a.anchorUs == pytest.approx(GATE_US - 4000, abs=300)
    assert a.gateUs == GATE_US
    assert a.anchorTolUs == max(2000, round(3 * a.anchorSigmaUs))


def test_a_loose_club_fit_falls_back_to_the_gate(lib):
    club = approach(lib, frames=4, jitter=(0.0, 2.5, -2.5, 2.5))
    a = make(lib, club, max_sigma=50.0)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert a.anchorUs == GATE_US


def test_too_few_club_points_fall_back_to_the_gate(lib):
    a = make(lib, approach(lib, frames=1))
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert a.anchorUs == GATE_US and a.anchorUs != 0


def test_a_zero_sigma_limit_never_uses_the_club(lib):
    a = make(lib, approach(lib), max_sigma=0.0)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
