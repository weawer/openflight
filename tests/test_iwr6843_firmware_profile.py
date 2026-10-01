"""Tests for the IWR6843 profiling counters and adaptive capture windows,
firmware/iwr6843/l3_profile.c and l3_adaptive.c."""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw

STAGE = {name: index for index, name in enumerate(fw.PROFILE_STAGE_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def profile(lib, ticks_per_us=200) -> fw.Profile:
    p = fw.Profile()
    lib.l3_profile_init(ctypes.byref(p), ticks_per_us)
    return p


def test_profile_accumulates_count_last_mean_and_max_per_stage(lib):
    p = profile(lib)
    for ticks in (200 * 300, 200 * 310, 200 * 290):
        lib.l3_profile_add(ctypes.byref(p), STAGE["residual"], ticks)
    lib.l3_profile_add(ctypes.byref(p), STAGE["angle"], 200 * 50)
    lib.l3_profile_frame(ctypes.byref(p))
    assert p.stage[STAGE["residual"]].count == 3
    assert lib.l3_profile_mean_us(ctypes.byref(p), STAGE["residual"]) == 300
    assert lib.l3_profile_max_us(ctypes.byref(p), STAGE["residual"]) == 310
    assert lib.l3_profile_mean_us(ctypes.byref(p), STAGE["angle"]) == 50
    assert lib.l3_profile_frame_us(ctypes.byref(p)) == 350
    assert p.frames == 1
    text = fw.c_text(lib.l3_profile_format, ctypes.byref(p), STAGE["residual"])
    assert text == "perf residual n=3 last=290 mean=300 max=310"
    summary = fw.c_text(lib.l3_profile_format_summary, ctypes.byref(p))
    assert summary == "perf frames=1 total=350us clock=200"


def test_the_dss_wait_is_shown_but_not_added_to_the_frame_twice(lib):
    """dspwait is the MSS blocked on the DSS inside the residual stage: the
    frame total already counts it there."""
    p = profile(lib)
    lib.l3_profile_add(ctypes.byref(p), STAGE["residual"], 200 * 400)
    lib.l3_profile_add(ctypes.byref(p), STAGE["dspwait"], 200 * 350)
    assert lib.l3_profile_frame_us(ctypes.byref(p)) == 400
    text = fw.c_text(lib.l3_profile_format, ctypes.byref(p), STAGE["dspwait"])
    assert text == "perf dspwait n=1 last=350 mean=350 max=350"


def test_dspwait_is_the_last_stage():
    """Appended, so every earlier stage keeps its index in "triggerLog perf"."""
    assert fw.PROFILE_STAGE_NAMES[-1] == "dspwait"
    assert fw.PROFILE_STAGE_NAMES.index("balltrack") == 7


def test_profile_sum_saturates_and_marks_the_mean(lib):
    p = profile(lib, ticks_per_us=1)
    lib.l3_profile_add(ctypes.byref(p), STAGE["trigger"], 0xFFFFFFF0)
    lib.l3_profile_add(ctypes.byref(p), STAGE["trigger"], 0x100)
    stage = p.stage[STAGE["trigger"]]
    assert stage.sumTicks == 0xFFFFFFFF and stage.sumOverflow == 1
    assert (
        fw.c_text(lib.l3_profile_format, ctypes.byref(p), STAGE["trigger"]).split()[4].endswith("+")
    )


def test_profile_reset_keeps_the_clock_and_ignores_bad_stages(lib):
    p = profile(lib, ticks_per_us=150)
    lib.l3_profile_add(ctypes.byref(p), 99, 1000)
    lib.l3_profile_add(ctypes.byref(p), STAGE["impact"], 1500)
    lib.l3_profile_reset(ctypes.byref(p))
    assert p.ticksPerUs == 150 and p.stage[STAGE["impact"]].count == 0
    assert lib.l3_profile_mean_us(ctypes.byref(p), 99) == 0
    assert fw.c_text(lib.l3_profile_format, ctypes.byref(p), 99) == "perf ?"
    zero = fw.Profile()
    lib.l3_profile_init(ctypes.byref(zero), 0)
    assert zero.ticksPerUs == 1, "a zero clock cannot divide"
    for index, name in enumerate(fw.PROFILE_STAGE_NAMES):
        assert lib.l3_profile_stage_name(index).decode() == name


def adaptive(lib, **overrides) -> fw.AdaptiveCfg:
    cfg = fw.AdaptiveCfg()
    lib.l3_adaptive_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    return cfg


def windows(lib, cfg, ball_bin, *, fft=128, pre=53, impact=53, post=53):
    out = fw.AdaptiveWindows()
    ok = lib.l3_adaptive_windows(
        ctypes.byref(cfg), ball_bin, fft, pre, impact, post, ctypes.byref(out)
    )
    return ok, out


def test_adaptive_defaults_are_off_with_a_metre_of_approach(lib):
    cfg = adaptive(lib)
    assert cfg.enabled == 0 and cfg.approachBins == 24 and cfg.marginBins == 4
    ok, _ = windows(lib, cfg, 46)
    assert ok == 0, "disabled computes nothing"


def test_adaptive_windows_follow_the_locked_ball(lib):
    """A ball at bin 46 (2.16 m, as the ten captures showed) instead of the
    configured 20/32/47 windows built for a tee at 1.57 m."""
    ok, out = windows(lib, adaptive(lib, enabled=1), 46)
    assert ok == 1
    assert out.preStart == 46 - 24
    assert out.impactStart == 46 - 4 and out.postStart == 46 - 4
    assert out.lateStart == out.postStart + 53 // 2
    assert lib.l3_adaptive_differs(ctypes.byref(out), 20, 32, 32, 47) == 1
    assert lib.l3_adaptive_differs(ctypes.byref(out), 22, 42, 42, 68) == 0


def test_adaptive_windows_stay_inside_the_fft(lib):
    cfg = adaptive(lib, enabled=1)
    _, near = windows(lib, cfg, 10)
    assert near.preStart == 0 and near.impactStart == 6
    _, far = windows(lib, cfg, 120)
    assert far.postStart == 128 - 53 and far.lateStart == 128 - 53
    ok, _ = windows(lib, cfg, 128)
    assert ok == 0, "outside the FFT"
    ok, _ = windows(lib, cfg, 46, pre=0)
    assert ok == 0, "a plan without a pre window"
    _, no_impact = windows(lib, cfg, 46, impact=0)
    assert no_impact.impactStart == 42, "unphased plans use the post width"


def test_adaptive_format_names_every_window(lib):
    cfg = adaptive(lib, enabled=1)
    _, out = windows(lib, cfg, 46)
    text = fw.c_text(lib.l3_adaptive_format, ctypes.byref(cfg), ctypes.byref(out))
    assert text == "adaptive enabled=1 approach=24 margin=4 pre=22 impact=42 post=42 late=68"


def test_reconstruct_is_a_stage_but_not_a_per_frame_cost(lib):
    """It runs once per shot (the ball fit at RESULT): its mean would inflate the
    per-frame budget the MSS reports."""
    assert fw.PROFILE_STAGE_NAMES.index("reconstruct") == 8
    assert fw.PROFILE_STAGE_NAMES[-1] == "dspwait"
    p = profile(lib)
    lib.l3_profile_add(ctypes.byref(p), STAGE["residual"], 200 * 300)
    lib.l3_profile_add(ctypes.byref(p), STAGE["reconstruct"], 200 * 900)
    lib.l3_profile_frame(ctypes.byref(p))
    assert lib.l3_profile_frame_us(ctypes.byref(p)) == 300
