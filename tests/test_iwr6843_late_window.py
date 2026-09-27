"""Late-window planning and measurement for outdoor range and on-course shots."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
import pytest

from openflight.clubs.physics import CLUB_PHYSICS
from openflight.clubs.types import ClubType
from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.dump import (
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    pack_dump,
)
from openflight.iwr6843.late_window import (
    CHIRP_MAX_RANGE_M,
    IMPACT_WINDOW_S,
    LATE_METHOD,
    LateLook,
    LateWindowPlan,
    capture_late_window,
    descent_angle_deg,
    expected_slant_m,
    frame_times_s,
    late_gate_m,
    late_period_s,
    long_range_cfg,
    long_range_span_m,
    measured_ranges,
    net_gate_m,
    plan_late_window,
)
from openflight.iwr6843.monitor import tee_global_bin
from openflight.iwr6843.runtime import IWR6843Runtime

# Amateur TrackMan averages. Driver is the longest flight, not the timing limit.
_DRIVER = dict(ball_speed_mph=160.0, launch_angle_deg=12.0, spin_rpm=2500.0)
# l3sparse power cube plus track cells at 1,041,667 baud. A CP2105 stall is extra.
_SPARSE_DUMP_S = 1.0
_SPAN_M = 128.0  # 1 m per bin keeps the synthetic ranges readable


def _plan(t0: float = 2.0, t1: float = 2.4, r0: float = 40.0, r1: float = 60.0) -> LateWindowPlan:
    return LateWindowPlan(
        enabled=True,
        reason="outdoor",
        apex_t_s=(t0 + t1) / 2,
        looks=(
            LateLook(t_s=t0, downrange_m=r0 - 1.5, height_m=20.0, slant_range_m=r0),
            LateLook(t_s=t1, downrange_m=r1 - 1.5, height_m=19.0, slant_range_m=r1),
        ),
    )


def _late_dump(
    ball_bins: list[int | None],
    *,
    clutter_bin: int | None = None,
    clutter_power: float = 1e4,
    ball_power: float = 30.0,
    other_bins: list[int] | None = None,
    other_power: float = 300.0,
    bins: int = 128,
    frame_time_offsets_us: tuple[int, ...] | None = None,
) -> bytes:
    """One frame per entry; the ball moves bins, the clutter stays put."""
    frames = len(ball_bins)
    rng = np.random.default_rng(3)
    cube = (
        rng.normal(size=(frames, 6, 4, bins)) + 1j * rng.normal(size=(frames, 6, 4, bins))
    ) * 0.1
    if clutter_bin is not None:
        cube[:, :, :, clutter_bin] += clutter_power
    for frame, ball in enumerate(ball_bins):
        if ball is not None:
            cube[frame, :, :, ball] += ball_power
    for frame, other in enumerate(other_bins or []):
        cube[frame, :, :, other] += other_power
    if frame_time_offsets_us is None:
        return pack_dump(cube.astype(np.complex64), n_tx=2)
    return pack_dump(
        np.round(cube).astype(np.complex64),
        n_tx=2,
        version=6,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=(0,) * frames,
        range_bin_counts=(bins,) * frames,
        frame_time_offsets_us=frame_time_offsets_us,
    )


# --- planning ---------------------------------------------------------------


def test_enable_flag_bin_comes_from_the_tee_distance():
    cfg = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
    assert tee_global_bin(1.575, cfg) == 34


def test_net_keeps_the_gate():
    assert net_gate_m("net", 4.6) == 4.6


@pytest.mark.parametrize("flight", ["range", "course"])
def test_open_flight_drops_the_net_gate(flight):
    assert net_gate_m(flight, 4.6) is None


def test_net_skips_the_late_window():
    plan = plan_late_window("net", **_DRIVER)

    assert plan.enabled is False
    assert plan.reason == "net"
    assert plan.looks == ()


@pytest.mark.parametrize("mode", ["outdoor", "on_course"])
def test_open_flight_schedules_looks_around_apex(mode):
    plan = plan_late_window(mode, **_DRIVER)

    assert plan.enabled is True
    assert plan.apex_t_s is not None
    pre, post = plan.looks
    assert pre.t_s < plan.apex_t_s < post.t_s
    assert post.downrange_m > pre.downrange_m
    assert pre.t_s > IMPACT_WINDOW_S
    assert pre.slant_range_m > CHIRP_MAX_RANGE_M
    assert post.slant_range_m > CHIRP_MAX_RANGE_M


@pytest.mark.parametrize(
    ("speed", "launch", "reason"),
    [(0.0, 12.0, "no-flight"), (150.0, 0.0, "no-flight"), (150.0, -3.0, "no-flight")],
)
def test_no_flight_disables_the_window(speed, launch, reason):
    plan = plan_late_window("outdoor", speed, launch, 2500.0)

    assert plan.enabled is False
    assert plan.reason == reason


def test_every_club_leaves_time_to_arm_after_the_sparse_dump():
    """The lob wedge apexes first. A driver-only check hides that budget."""
    tightest = None
    for club, physics in CLUB_PHYSICS.items():
        if club is ClubType.UNKNOWN:
            continue
        plan = plan_late_window(
            "outdoor",
            physics.average_ball_speed_mph,
            physics.optimal_launch_deg,
            physics.typical_spin_rpm,
        )
        assert plan.enabled is True
        pre, post = plan.looks
        assert pre.t_s < plan.apex_t_s < post.t_s
        assert pre.slant_range_m > CHIRP_MAX_RANGE_M
        spare_s = pre.t_s - (IMPACT_WINDOW_S + _SPARSE_DUMP_S)
        assert spare_s > 0.0, club.name
        if tightest is None or spare_s < tightest[0]:
            tightest = (spare_s, club)

    spare_s, club = tightest
    assert club in {ClubType.SW, ClubType.LW}
    assert spare_s < 1.0


def test_late_profile_keeps_a_pretrigger_ring_across_both_looks():
    plan = plan_late_window("outdoor", **_DRIVER)
    cfg = long_range_cfg(plan)
    phase = next(line for line in cfg.splitlines() if line.startswith("phaseCaptureCfg"))
    pre_frames = int(phase.split()[3])
    assert pre_frames >= 8
    assert "captureFormat iq16" in cfg
    # Eight frames at the late period must span both looks.
    assert pre_frames * late_period_s(plan) >= plan.looks[-1].t_s - plan.looks[0].t_s


def test_expected_slant_interpolates_between_looks_and_clamps_outside():
    plan = _plan()

    assert expected_slant_m(plan, 2.2) == pytest.approx(50.0)
    assert expected_slant_m(plan, 1.0) == pytest.approx(40.0)
    assert expected_slant_m(plan, 9.0) == pytest.approx(60.0)


def test_gate_never_narrower_than_its_floor():
    assert late_gate_m(5.0) == pytest.approx(1.5)
    assert late_gate_m(60.0) == pytest.approx(6.0)


# --- timing -----------------------------------------------------------------------


def test_newest_frame_is_stamped_at_the_freeze_not_after_the_transfer():
    times = frame_times_s({}, 4, freeze_t_s=3.0, period_s=0.1)

    assert times == pytest.approx([2.7, 2.8, 2.9, 3.0])


def test_frame_times_follow_the_dump_offsets_when_present():
    meta = {"frame_time_offsets_us": (0, 90_000, 200_000)}

    assert frame_times_s(meta, 3, freeze_t_s=5.0, period_s=0.1) == pytest.approx([4.8, 4.89, 5.0])


def test_capture_stamps_samples_from_the_freeze_even_with_a_slow_dump():
    """A 2.6 s UART transfer must not shift every sample 2.6 s late."""
    plan = _plan()
    period = late_period_s(plan)
    clock = {"t": 0.0}

    class Radar:
        def __init__(self):
            self.configs = []

        def send_config(self, path):
            self.configs.append(path)
            clock["t"] += 0.5

        def read_dump(self):
            clock["t"] += 2.6
            return _late_dump([40, 47, 53, 60])

    radar = Radar()
    measured = capture_late_window(
        radar,
        plan,
        impact_timestamp=0.0,
        tee_range_m=1.5,
        restore_cfg="impact.cfg",
        now=lambda: clock["t"],
        sleep=lambda seconds: clock.__setitem__("t", clock["t"] + seconds),
    )

    freeze = plan.looks[-1].t_s + period
    assert measured["freeze_t_s"] == pytest.approx(freeze)
    assert measured["ranges"][-1]["t_s"] == pytest.approx(freeze)
    assert measured["timing_s"]["dump_s"] == pytest.approx(2.6)
    assert measured["timing_s"]["retune_s"] == pytest.approx(0.5)
    assert measured["timing_s"]["restore_s"] == pytest.approx(0.5)
    assert measured["method"] == LATE_METHOD
    assert radar.configs[0] != "impact.cfg" and radar.configs[-1] == "impact.cfg"
    assert len(radar.configs) == 2


def test_capture_skips_when_the_first_look_has_passed():
    plan = plan_late_window("outdoor", **_DRIVER)

    class Radar:
        def send_config(self, _path):
            raise AssertionError("retune")

    assert (
        capture_late_window(
            Radar(),
            plan,
            impact_timestamp=0.0,
            tee_range_m=1.5,
            restore_cfg="impact.cfg",
            now=lambda: plan.looks[0].t_s,
            sleep=lambda _seconds: None,
        )
        is None
    )


def test_capture_restores_the_impact_profile_when_the_dump_fails():
    plan = _plan()
    configs = []

    class Radar:
        def send_config(self, path):
            configs.append(path)

        def read_dump(self):
            raise TimeoutError("dump stalled")

    with pytest.raises(TimeoutError):
        capture_late_window(
            Radar(),
            plan,
            impact_timestamp=0.0,
            tee_range_m=1.5,
            restore_cfg="impact.cfg",
            now=lambda: 0.0,
            sleep=lambda _seconds: None,
        )
    assert configs[-1] == "impact.cfg"


# --- range measurement -------------------------------------------------------------


def test_static_clutter_is_removed_before_the_peak():
    plan = _plan(t0=0.0, t1=0.3, r0=40.0, r1=60.0)
    raw = _late_dump([40, 47, 53, 60], clutter_bin=50, clutter_power=1e4)

    ranges = measured_ranges(raw, _SPAN_M, plan, freeze_t_s=0.3, period_s=0.1)

    assert [sample["slant_range_m"] for sample in ranges] == [40.0, 47.0, 53.0, 60.0]


def test_mover_outside_the_gate_is_ignored():
    plan = _plan(t0=0.0, t1=0.3, r0=40.0, r1=60.0)
    # A ten-times-stronger second mover (a bird, a cart) far from the prediction.
    raw = _late_dump([40, 47, 53, 60], other_bins=[100, 110, 90, 105])

    ranges = measured_ranges(raw, _SPAN_M, plan, freeze_t_s=0.3, period_s=0.1)

    assert [sample["slant_range_m"] for sample in ranges] == [40.0, 47.0, 53.0, 60.0]


def test_gate_outside_the_captured_window_reports_no_range():
    plan = _plan(t0=0.0, t1=0.3, r0=400.0, r1=410.0)
    raw = _late_dump([40, 47, 53, 60])

    ranges = measured_ranges(raw, _SPAN_M, plan, freeze_t_s=0.3, period_s=0.1)

    assert all(sample["slant_range_m"] is None for sample in ranges)


def test_varying_range_windows_are_refused():
    cube = np.ones((2, 4, 4, 8), dtype=np.complex64)
    raw = pack_dump(
        cube,
        n_tx=2,
        version=6,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=(0, 4),
        range_bin_counts=(8, 8),
        frame_time_offsets_us=(0, 1000),
    )

    with pytest.raises(ValueError, match="one range window"):
        measured_ranges(raw, _SPAN_M, _plan(), freeze_t_s=1.0, period_s=0.1)


# --- descent ---------------------------------------------------------------------


def test_descent_is_the_downward_chord_of_the_two_looks():
    plan = plan_late_window("outdoor", **_DRIVER)
    before, after = plan.looks
    ranges = [
        {"t_s": before.t_s, "slant_range_m": before.slant_range_m},
        {"t_s": after.t_s, "slant_range_m": after.slant_range_m * 0.98},
    ]
    descent = descent_angle_deg(plan, ranges, tee_range_m=1.5, max_time_error_s=0.1)
    assert descent is not None
    assert descent > 0.0


def test_descent_needs_a_sample_near_each_look():
    plan = _plan()
    ranges = [
        {"t_s": 2.0, "slant_range_m": 40.0},
        {"t_s": 3.5, "slant_range_m": 60.0},
    ]

    assert descent_angle_deg(plan, ranges, 1.5, max_time_error_s=0.15) is None


def test_descent_skips_frames_without_a_range():
    plan = _plan()
    ranges = [
        {"t_s": 2.0, "slant_range_m": 40.0},
        {"t_s": 2.4, "slant_range_m": None},
    ]

    assert descent_angle_deg(plan, ranges, 1.5, max_time_error_s=0.15) is None


def test_descent_rejects_a_ball_that_moved_backwards():
    plan = _plan()
    ranges = [
        {"t_s": 2.0, "slant_range_m": 60.0},
        {"t_s": 2.4, "slant_range_m": 40.0},
    ]

    assert descent_angle_deg(plan, ranges, 1.5, max_time_error_s=0.15) is None


def test_long_range_span_covers_the_far_look():
    assert long_range_span_m(10.0) == pytest.approx(20.0)
    assert long_range_span_m(100.0) == pytest.approx(115.0)


# --- runtime -----------------------------------------------------------------------


class _Monitor:
    """Capture monitor double that runs submitted jobs on a thread."""

    config_path = "impact.cfg"

    def __init__(self, *, running: bool = True):
        self.running = running
        self.events: list[str] = []
        self.thread: threading.Thread | None = None

    def submit(self, name, job):
        if not self.running:
            return False
        self.events.append(f"submit:{name}")
        self.thread = threading.Thread(target=job, args=("radar",))
        self.thread.start()
        return True

    def run_on_other_profile(self, job):
        self.events.append("trigger-off")
        try:
            job("radar")
        finally:
            self.events.append("trigger-on")


def _runtime(flight: str, monitor=None, tee: float | None = 1.5) -> IWR6843Runtime:
    return IWR6843Runtime(
        capture_monitor=monitor or _Monitor(),
        calibration=SimpleNamespace(tee_range_m=tee),
        net_range_m=4.6,
        flight_mode=flight,
    )


def test_runtime_plans_nothing_in_a_net():
    runtime = _runtime("net")

    assert (
        runtime.plan_late_window(ball_speed_mph=150.0, launch_angle_deg=12.0, spin_rpm=2500.0)
        is None
    )


def test_runtime_plans_nothing_without_a_launch_angle():
    runtime = _runtime("range")

    assert (
        runtime.plan_late_window(ball_speed_mph=150.0, launch_angle_deg=None, spin_rpm=2500.0)
        is None
    )


def test_runtime_plan_uses_the_default_tee_when_uncalibrated():
    plan = _runtime("course", tee=None).plan_late_window(
        ball_speed_mph=150.0, launch_angle_deg=12.0, spin_rpm=None
    )
    expected = plan_late_window("on_course", 150.0, 12.0, 0.0, DEFAULT_TEE_RANGE_M)

    assert plan == expected


def test_runtime_measures_on_the_worker_with_the_trigger_paused(monkeypatch):
    monitor = _Monitor()
    runtime = _runtime("range", monitor)
    results = []
    captured = {}

    def fake_capture(radar, plan, **kwargs):
        captured.update(kwargs, radar=radar, plan=plan)
        monitor.events.append("capture")
        return {"descent_deg": 41.0, "timing_s": {}}

    monkeypatch.setattr("openflight.iwr6843.runtime.capture_late_window", fake_capture)
    plan = _plan()

    assert runtime.measure_late_window(plan, impact_timestamp=12.0, on_measured=results.append)
    monitor.thread.join(1.0)

    assert monitor.events == ["submit:late-window", "trigger-off", "capture", "trigger-on"]
    assert results == [{"descent_deg": 41.0, "timing_s": {}}]
    assert captured["impact_timestamp"] == 12.0
    assert captured["restore_cfg"] == "impact.cfg"
    assert captured["tee_range_m"] == 1.5


def test_runtime_reports_a_failed_measurement_as_none(monkeypatch):
    monitor = _Monitor()
    runtime = _runtime("range", monitor)
    results = []

    def failing_capture(*_args, **_kwargs):
        raise TimeoutError("dump stalled")

    monkeypatch.setattr("openflight.iwr6843.runtime.capture_late_window", failing_capture)

    runtime.measure_late_window(_plan(), impact_timestamp=0.0, on_measured=results.append)
    monitor.thread.join(1.0)

    assert results == [None]
    assert monitor.events[-1] == "trigger-on"


def test_runtime_refuses_when_the_monitor_is_stopped():
    runtime = _runtime("range", _Monitor(running=False))

    assert runtime.measure_late_window(_plan(), impact_timestamp=0.0, on_measured=print) is False
