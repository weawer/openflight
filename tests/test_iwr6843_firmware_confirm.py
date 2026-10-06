"""Tests for the ball-flight confirmation of a self-trigger fire,
firmware/iwr6843/l3_confirm.c.

In confirm mode a club or leave fire is only a candidate: the host is told
once the ball track holds a flight (consecutive points stepping outward at a
ball's speed, Doppler running on smoothly, the range rate wrapping onto the
Doppler), and told to release the ring when none shows within the window. A
backswing or an empty lane never launches a ball.
"""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw

VERDICT = {name: index for index, name in enumerate(fw.CONFIRM_VERDICT_NAMES)}
WHY = {name: index for index, name in enumerate(fw.CONFIRM_WHY_NAMES)}
BIN_M = 6.0 / 128.0
SPAN = 2.0 * fw.OBS_WAVELENGTH_M / (4.0 * 135.0e-6)
FRAME_US = 2000
CANDIDATE_US = 100_000


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def config(lib, **overrides) -> fw.ConfirmCfg:
    cfg = fw.ConfirmCfg()
    lib.l3_confirm_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    return cfg


def armed(lib, **overrides) -> fw.Confirm:
    confirm = fw.Confirm()
    lib.l3_confirm_init(ctypes.byref(confirm), ctypes.byref(config(lib, **overrides)))
    lib.l3_confirm_arm(ctypes.byref(confirm), CANDIDATE_US)
    return confirm


def wrap(speed: float) -> float:
    """A speed as the aliased Doppler reads it."""
    return speed - SPAN * round(speed / SPAN)


def flight(
    speed_mps: float,
    n: int,
    *,
    start_bin: float = 44.0,
    start_us: int = CANDIDATE_US + FRAME_US,
    doppler=None,
) -> list[tuple[int, float, float]]:
    """``n`` points of a target receding at ``speed_mps`` from ``start_bin``:
    (timestamp, bin, aliased Doppler)."""
    points = []
    for i in range(n):
        t = start_us + i * FRAME_US
        b = start_bin + speed_mps * i * FRAME_US * 1e-6 / BIN_M
        points.append((t, b, wrap(speed_mps) if doppler is None else doppler[i]))
    return points


def update(lib, confirm, points, now_us: int) -> int:
    array = (fw.TrackPoint * max(1, len(points)))()
    for slot, (t, b, d) in zip(array, points):
        slot.timestampUs, slot.rangeBin, slot.rangeM, slot.dopplerAliasMps = t, b, b * BIN_M, d
    fit_list = fw.FitList(array, len(points))
    return lib.l3_confirm_update(
        ctypes.byref(confirm),
        fw.fit_reader(lib),
        ctypes.cast(ctypes.byref(fit_list), ctypes.c_void_p),
        len(points),
        now_us,
    )


def text(lib, confirm) -> str:
    return fw.c_text(lib.l3_confirm_format, ctypes.byref(confirm))


def test_the_settings_mirror_the_c_struct():
    assert [name for name, _ in fw.ConfirmCfg._fields_] == [
        "enabled",
        "windowUs",
        "points",
        "minSpeedMps",
        "maxSpeedMps",
        "dopplerStepMps",
        "rateDopplerMps",
        "binWidthM",
        "velocitySpanMps",
    ]


def test_defaults_are_off_and_usable(lib):
    cfg = config(lib)
    assert cfg.enabled == 0
    assert cfg.windowUs == 24000
    assert cfg.points == 3
    assert cfg.minSpeedMps == pytest.approx(15.0)
    assert cfg.maxSpeedMps == pytest.approx(90.0)
    assert cfg.velocitySpanMps == pytest.approx(SPAN, rel=1e-5)
    assert lib.l3_confirm_cfg_check(ctypes.byref(cfg)) == 0


@pytest.mark.parametrize(
    "overrides",
    [
        {"points": 2},
        {"points": fw.CONFIRM_MAX_POINTS + 1},
        {"windowUs": 0},
        {"windowUs": fw.CONFIRM_MAX_WINDOW_US + 1},
        {"minSpeedMps": 0.0},
        {"maxSpeedMps": 10.0},
        {"dopplerStepMps": 0.0},
        {"rateDopplerMps": -1.0},
        {"binWidthM": 0.0},
        {"velocitySpanMps": 0.0},
        {"minSpeedMps": float("nan")},
    ],
)
def test_unusable_settings_are_refused(lib, overrides):
    assert lib.l3_confirm_cfg_check(ctypes.byref(config(lib, **overrides))) == -1


def test_an_idle_rule_ignores_points(lib):
    confirm = fw.Confirm()
    lib.l3_confirm_init(ctypes.byref(confirm), ctypes.byref(config(lib)))
    assert update(lib, confirm, flight(45.0, 3), CANDIDATE_US + 8000) == VERDICT["idle"]
    assert confirm.confirmed == 0 and confirm.rejected == 0


@pytest.mark.parametrize("speed", [25.0, 45.0, 70.0])
def test_a_ball_flight_confirms(lib, speed):
    confirm = armed(lib)
    points = flight(speed, 3)
    now = points[-1][0]
    assert update(lib, confirm, points, now) == VERDICT["confirmed"]
    assert confirm.why == WHY["flight"]
    assert confirm.speedMps == pytest.approx(speed, rel=0.01)
    assert confirm.decidedUs == now
    assert confirm.confirmed == 1
    assert "verdict=confirmed why=flight" in text(lib, confirm)
    assert f"dt={now - CANDIDATE_US}" in text(lib, confirm)


def test_two_points_are_not_yet_a_flight(lib):
    confirm = armed(lib)
    assert update(lib, confirm, flight(45.0, 2), CANDIDATE_US + 4000) == VERDICT["pending"]
    assert confirm.why == WHY["few"]


def test_no_points_wait_until_the_window_then_reject(lib):
    confirm = armed(lib)
    assert update(lib, confirm, [], CANDIDATE_US + 24000) == VERDICT["pending"]
    assert update(lib, confirm, [], CANDIDATE_US + 24001) == VERDICT["rejected"]
    assert confirm.why == WHY["timeout"]
    assert confirm.rejected == 1


def test_a_standing_return_never_confirms(lib):
    """The takeaway captures' ball tracks: a return at the tee, Doppler ~0."""
    confirm = armed(lib)
    points = flight(0.0, 6, doppler=[-0.2] * 6)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["still"]


def test_a_slow_mover_is_not_a_ball(lib):
    confirm = armed(lib)
    points = flight(10.0, 4)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["still"]


def test_a_step_inward_breaks_the_flight(lib):
    confirm = armed(lib)
    points = flight(45.0, 3)
    t, b, d = points[2]
    points[2] = (t, points[0][1] - 0.5, d)
    assert update(lib, confirm, points, t) == VERDICT["pending"]
    assert confirm.why == WHY["still"]


def test_a_jump_faster_than_any_ball_is_not_a_flight(lib):
    confirm = armed(lib)
    points = flight(120.0, 3)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["fast"]


def test_noise_doppler_breaks_the_flight(lib):
    """Points chained through noise read any Doppler frame to frame."""
    confirm = armed(lib)
    points = flight(45.0, 3, doppler=[wrap(45.0), wrap(45.0) + 6.0, wrap(45.0) - 3.0])
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["jump"]


def test_doppler_continuity_is_judged_round_the_alias(lib):
    """8.8 then -8.8 m/s is a 0.3 m/s change across the wrap, not 17.6."""
    confirm = armed(lib)
    speed = SPAN * 3 + 8.8  # aliases to +8.8 (close to the edge)
    doppler = [8.8, -(SPAN - 8.8 - 0.1), 8.7]
    points = flight(speed, 3, doppler=doppler)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["confirmed"]


def test_a_range_rate_that_does_not_wrap_onto_the_doppler_is_rejected(lib):
    confirm = armed(lib)
    points = flight(45.0, 3, doppler=[wrap(45.0) + 7.0] * 3)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["rate"]


def test_the_rate_check_can_be_turned_off(lib):
    confirm = armed(lib, rateDopplerMps=0.0)
    points = flight(45.0, 3, doppler=[wrap(45.0) + 7.0] * 3)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["confirmed"]


def test_an_earlier_bad_point_does_not_hide_a_later_flight(lib):
    """The leave fallback's seed or a club claim may precede the ball's run."""
    confirm = armed(lib)
    points = [(CANDIDATE_US, 43.0, 4.0)] + flight(45.0, 3, start_bin=44.0)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["confirmed"]


def test_the_newest_runs_reason_is_reported(lib):
    confirm = armed(lib)
    points = flight(45.0, 2) + [(CANDIDATE_US + 7 * FRAME_US, 44.0, -0.1)]
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["pending"]
    assert confirm.why == WHY["still"]


def test_a_decided_verdict_stands_until_rearmed(lib):
    confirm = armed(lib)
    points = flight(45.0, 3)
    assert update(lib, confirm, points, points[-1][0]) == VERDICT["confirmed"]
    assert update(lib, confirm, [], CANDIDATE_US + 90_000) == VERDICT["confirmed"]
    assert lib.l3_confirm_end(ctypes.byref(confirm), CANDIDATE_US + 90_000) == VERDICT["confirmed"]
    lib.l3_confirm_rearm(ctypes.byref(confirm))
    assert confirm.verdict == VERDICT["idle"]
    assert confirm.confirmed == 1  # counters survive


def test_the_post_movie_ending_rejects_a_pending_candidate(lib):
    confirm = armed(lib)
    assert lib.l3_confirm_end(ctypes.byref(confirm), CANDIDATE_US + 6000) == VERDICT["rejected"]
    assert confirm.why == WHY["ended"]
    assert "verdict=rejected why=ended" in text(lib, confirm)


def test_a_new_candidate_restarts_the_window(lib):
    confirm = armed(lib)
    assert lib.l3_confirm_end(ctypes.byref(confirm), CANDIDATE_US) == VERDICT["rejected"]
    lib.l3_confirm_arm(ctypes.byref(confirm), CANDIDATE_US + 500_000)
    assert confirm.verdict == VERDICT["pending"]
    assert confirm.rejected == 1


def test_the_window_is_wrap_safe(lib):
    confirm = fw.Confirm()
    lib.l3_confirm_init(ctypes.byref(confirm), ctypes.byref(config(lib)))
    start = 0xFFFF_FFFF - 5000
    lib.l3_confirm_arm(ctypes.byref(confirm), start)
    assert update(lib, confirm, [], (start + 10_000) & 0xFFFF_FFFF) == VERDICT["pending"]
    assert update(lib, confirm, [], (start + 30_000) & 0xFFFF_FFFF) == VERDICT["rejected"]


def test_names_cover_every_code(lib):
    for index, name in enumerate(fw.CONFIRM_VERDICT_NAMES):
        assert lib.l3_confirm_verdict_name(index).decode() == name
    for index, name in enumerate(fw.CONFIRM_WHY_NAMES):
        assert lib.l3_confirm_why_name(index).decode() == name
    assert lib.l3_confirm_why_name(len(fw.CONFIRM_WHY_NAMES)).decode() == "?"
