"""Tests for impact from the tracks either side of the tee band,
firmware/iwr6843/l3_impact_fit.c.

Scene used throughout: ball at rest at 2.20 m, impact at t = 30 000 us,
band +/- 6 bins (+/- 0.281 m). Club in at 30 m/s, club out at 25 m/s, ball
out at 60 m/s; every point lies outside the band.
"""

from __future__ import annotations

import ctypes
import re

import pytest

from openflight.iwr6843 import firmware_host as fw

BIN_M = 6.0 / 128
BALL_M = 2.20
IMPACT_US = 30_000.0
CLUB_IN, CLUB_OUT, BALL_OUT = 0, 1, 2
WHY = {name: i for i, name in enumerate(fw.FIT_WHY_NAMES)}
VERDICT = {name: i for i, name in enumerate(fw.FIT_VERDICT_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def cfg(lib, **overrides) -> fw.ImpactFitCfg:
    c = fw.ImpactFitCfg()
    lib.l3_impact_fit_cfg_defaults(ctypes.byref(c))
    for name, value in overrides.items():
        setattr(c, name, value)
    return c


def line(speed_mps: float, times_us, *, noise_m=()) -> list[tuple[float, float]]:
    """(t_us, r_m) on the line through (IMPACT_US, BALL_M) at speed_mps."""
    noise = list(noise_m) + [0.0] * len(times_us)
    return [
        (t, BALL_M + speed_mps * (t - IMPACT_US) * 1e-6 + noise[i]) for i, t in enumerate(times_us)
    ]


def point_list(samples) -> fw.FitList:
    arr = (fw.TrackPoint * max(1, len(samples)))()
    for i, (t, r) in enumerate(samples):
        arr[i].frame = i
        arr[i].timestampUs = int(round(t))
        arr[i].rangeM = r
        arr[i].rangeBin = r / BIN_M
    out = fw.FitList(ctypes.cast(arr, ctypes.POINTER(fw.TrackPoint)), len(samples))
    out.keep = arr  # keep the array alive as long as the list
    return out


def estimate(lib, which, samples, **overrides) -> fw.FitEstimate:
    lst = point_list(samples)
    out = fw.FitEstimate()
    lib.l3_impact_fit_track(
        ctypes.byref(cfg(lib, **overrides)),
        which,
        fw.fit_reader(lib),
        ctypes.byref(lst),
        len(samples),
        BALL_M,
        ctypes.byref(out),
    )
    return out


CLUB_IN_T = (9_000, 12_000, 15_000, 18_000)
CLUB_OUT_T = (42_000, 45_000, 48_000, 51_000)
BALL_OUT_T = (36_000, 39_000, 42_000, 45_000)


def test_defaults_are_the_specs(lib):
    c = cfg(lib)
    assert c.bandBins == 6.0  # on by default: the 2026-09-28 capture's ridge is 6 bins wide
    assert c.bandSearchBins == 10.0
    assert c.fitPoints == 4 and c.minPoints == 3
    # 17 m/s: a late backswing swings downrange past the ball's range too, but
    # rarely that fast radially; a full downswing is 30-50 m/s.
    assert (c.clubMinMps, c.clubMaxMps, c.clubOutMaxRatio) == (17.0, 70.0, pytest.approx(1.10))
    assert (c.ballMinMps, c.ballMaxMps) == (15.0, 90.0)
    assert (c.gateSigmas, c.minSigmaUs) == (3.0, 500.0)
    assert c.maxSigmaUs == 3000.0  # one 3 ms frame
    assert c.binWidthM == pytest.approx(BIN_M)


@pytest.mark.parametrize(
    "which, speed, times",
    [(CLUB_IN, 30.0, CLUB_IN_T), (CLUB_OUT, 25.0, CLUB_OUT_T), (BALL_OUT, 60.0, BALL_OUT_T)],
)
def test_clean_line_crosses_the_ball_at_impact(lib, which, speed, times):
    e = estimate(lib, which, line(speed, times))
    assert e.why == WHY["ok"]
    assert e.points == 4
    assert e.speedMps == pytest.approx(speed, rel=1e-4)
    assert e.timeUs == pytest.approx(IMPACT_US, abs=2.0)


def test_sigma_floor_is_a_bin_of_quantisation_over_speed(lib):
    e = estimate(lib, CLUB_IN, line(30.0, CLUB_IN_T))
    floor_us = BIN_M / 12**0.5 / 30.0 * 1e6
    assert e.sigmaUs == pytest.approx(floor_us, rel=1e-3)


def test_sigma_grows_with_extrapolation_distance(lib):
    noise = (0.01, -0.01, 0.012, -0.008)
    near = estimate(lib, BALL_OUT, line(60.0, BALL_OUT_T, noise_m=noise))
    far_t = tuple(t + 15_000 for t in BALL_OUT_T)
    far = estimate(lib, BALL_OUT, line(60.0, far_t, noise_m=noise))
    assert near.why == far.why == WHY["ok"]
    assert far.sigmaUs > near.sigmaUs


def test_club_in_uses_its_last_k_points_and_outs_their_first_k(lib):
    # A stray early club-in point and a stray late ball point, both far off the line.
    club = [(0.0, 0.5)] + line(30.0, CLUB_IN_T)
    ball = line(60.0, BALL_OUT_T) + [(60_000.0, 9.0)]
    e_in = estimate(lib, CLUB_IN, club)
    e_out = estimate(lib, BALL_OUT, ball)
    assert e_in.timeUs == pytest.approx(IMPACT_US, abs=2.0)
    assert e_out.timeUs == pytest.approx(IMPACT_US, abs=2.0)


def test_strided_timestamps_are_honoured(lib):
    e = estimate(lib, BALL_OUT, line(60.0, (36_000, 42_000, 48_000, 54_000)))
    assert e.timeUs == pytest.approx(IMPACT_US, abs=2.0)


LATE_BASE_US = 1_800_000_000  # ~30 minutes of uptime; exactly representable in float


def shifted(samples, base_us):
    """The samples with base_us added to every timestamp, wrapped to uint32."""
    return [((t + base_us) % 2**32, r) for t, r in samples]


def test_a_late_session_line_keeps_its_timing(lib):
    # At 1.8e9 us a float's step is 128 us: converting each timestamp to float
    # before subtracting smeared the 3 ms spacing. dt is taken in integers from
    # the first point, so only the final absolute time is float-rounded. The
    # crossing is placed on a float-representable time (first point + 3200 us)
    # so the check is on the offset from the first point, to 2 us.
    ball_t = (26_800, 29_800, 32_800, 35_800)  # crossing at 30 000 = first + 3200
    # The first point lands on LATE_BASE_US; 3200 us is 25 float steps past it.
    e = estimate(lib, BALL_OUT, shifted(line(60.0, ball_t), LATE_BASE_US - ball_t[0]))
    assert e.why == WHY["ok"]
    assert e.speedMps == pytest.approx(60.0, rel=1e-4)
    assert abs((e.timeUs - LATE_BASE_US) - (IMPACT_US - ball_t[0])) <= 2.0


def test_a_line_straddling_the_uint32_wrap_is_fitted(lib):
    # Timestamps wrap at 2**32 us (~71.6 min); dt from the first point in
    # int32 keeps the line straight across it.
    base = 2**32 - 40_000  # the wrap falls between the 2nd and 3rd ball points
    e = estimate(lib, BALL_OUT, shifted(line(60.0, BALL_OUT_T), base))
    assert e.why == WHY["ok"]
    assert e.speedMps == pytest.approx(60.0, rel=1e-4)


def test_no_points_is_missing(lib):
    assert estimate(lib, CLUB_IN, []).why == WHY["missing"]


def test_two_points_are_too_few(lib):
    e = estimate(lib, CLUB_IN, line(30.0, CLUB_IN_T[:2]))
    assert e.why == WHY["few_points"] and e.points == 2


def test_points_that_do_not_spread_in_time_are_nonfinite(lib):
    same = [(12_000.0, 1.6), (12_000.0, 1.7), (12_000.0, 1.8)]
    assert estimate(lib, CLUB_IN, same).why == WHY["nonfinite"]


@pytest.mark.parametrize(
    "which, times", [(CLUB_IN, CLUB_IN_T), (CLUB_OUT, CLUB_OUT_T), (BALL_OUT, BALL_OUT_T)]
)
def test_moving_toward_the_radar_is_the_wrong_direction(lib, which, times):
    assert estimate(lib, which, line(-20.0, times)).why == WHY["wrong_direction"]


@pytest.mark.parametrize(
    "which, speed, times",
    [
        (CLUB_IN, 9.0, CLUB_IN_T),
        (CLUB_IN, 71.0, CLUB_IN_T),
        (CLUB_OUT, 71.0, CLUB_OUT_T),
        (BALL_OUT, 14.0, BALL_OUT_T),
        (BALL_OUT, 91.0, BALL_OUT_T),
    ],
)
def test_speeds_outside_the_bounds_are_rejected(lib, which, speed, times):
    assert estimate(lib, which, line(speed, times)).why == WHY["speed_bounds"]


@pytest.mark.parametrize("speed", [12.0, 15.0, 16.9])
def test_a_backswing_speed_club_in_is_rejected(lib, speed):
    # The club heading downrange into the top of the backswing crosses the
    # ball's range above the ball; at backswing speed it must not fire.
    assert estimate(lib, CLUB_IN, line(speed, CLUB_IN_T)).why == WHY["speed_bounds"]


def test_the_club_in_floor_itself_is_accepted(lib):
    assert estimate(lib, CLUB_IN, line(17.0, CLUB_IN_T)).why == WHY["ok"]


def test_crawling_club_out_is_too_uncertain_to_time(lib):
    # After impact the club only slows and there is no speed floor but "moving
    # away"; at 3 m/s, though, a bin of quantisation is 4.5 ms, over the cap.
    e = estimate(lib, CLUB_OUT, line(3.0, CLUB_OUT_T))
    assert e.why == WHY["uncertain"]
    assert e.sigmaUs > 3000.0


def test_slow_club_out_is_accepted(lib):
    # 10 m/s: a bin of quantisation is 1.35 ms, inside the cap.
    assert estimate(lib, CLUB_OUT, line(10.0, CLUB_OUT_T)).why == WHY["ok"]


def test_a_count_beyond_the_list_is_missing_not_garbage(lib):
    samples = line(30.0, CLUB_IN_T[:2])
    lst = point_list(samples)
    out = fw.FitEstimate()
    lib.l3_impact_fit_track(
        ctypes.byref(cfg(lib)),
        CLUB_IN,
        fw.fit_reader(lib),
        ctypes.byref(lst),
        4,
        BALL_M,
        ctypes.byref(out),
    )
    assert out.why == WHY["missing"]


def test_span_after_reads_only_points_appended_after_a_frame(lib):
    track_cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
    track = fw.ClubTrack()
    lib.l3_track_init(ctypes.byref(track), ctypes.byref(track_cfg))
    for frame, (t, r) in enumerate(line(30.0, (3_000, 6_000, 9_000, 12_000, 15_000))):
        p = fw.TrackPoint()
        p.frame, p.timestampUs, p.rangeM, p.rangeBin = frame, int(t), r, r / BIN_M
        lib.l3_track_append_point(ctypes.byref(track), ctypes.byref(p))

    span = fw.FitSpan()
    lib.l3_fit_span_after(ctypes.byref(track), 2, ctypes.byref(span))

    assert (span.first, span.count) == (3, 2)
    out = fw.TrackPoint()
    assert lib.l3_fit_span_point(ctypes.byref(span), 0, ctypes.byref(out)) == 1
    assert out.frame == 3
    assert lib.l3_fit_span_point(ctypes.byref(span), 2, ctypes.byref(out)) == 0


def solved(
    lib, estimates: dict[int, tuple[str, float, float, float]], trigger_us=27_500, **overrides
):
    """estimates: track -> (why, timeUs, sigmaUs, speedMps). others missing."""
    fit = fw.ImpactFit()
    lib.l3_impact_fit_reset(ctypes.byref(fit))
    for which, (why, t, sigma, speed) in estimates.items():
        e = fit.track[which]
        e.why, e.timeUs, e.sigmaUs, e.speedMps, e.points = WHY[why], t, sigma, speed, 4
    lib.l3_impact_fit_solve(ctypes.byref(cfg(lib, **overrides)), ctypes.byref(fit), trigger_us)
    return fit


def test_three_agreeing_tracks_are_consistent_and_weighted_by_inverse_variance(lib):
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 30_000, 400, 30),
            CLUB_OUT: ("ok", 30_300, 400, 25),
            BALL_OUT: ("ok", 30_100, 200, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    w = [1 / 400**2, 1 / 400**2, 1 / 200**2]
    expected = (30_000 * w[0] + 30_300 * w[1] + 30_100 * w[2]) / sum(w)
    assert fit.impactUs == pytest.approx(expected, abs=0.5)
    assert fit.spreadUs == pytest.approx(300)
    assert fit.droppedTrack == fw.FIT_NO_TRACK
    assert fit.refinedMinusTriggerUs == pytest.approx(expected - 27_500, abs=0.5)


def test_gate_uses_the_half_millisecond_floor(lib):
    # sigmas of 50 us would gate at 150 us; the 500 us floor gates at 1.5 ms.
    fit = solved(lib, {CLUB_IN: ("ok", 30_000, 50, 30), BALL_OUT: ("ok", 31_000, 50, 60)})
    assert fit.verdict == VERDICT["consistent"]


def test_one_outlier_of_three_is_dropped_and_the_rest_fused(lib):
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 30_000, 300, 30),
            CLUB_OUT: ("ok", 38_000, 300, 25),
            BALL_OUT: ("ok", 30_200, 300, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == CLUB_OUT
    assert fit.track[CLUB_OUT].why == WHY["dropped"]
    assert fit.impactUs == pytest.approx(30_100, abs=0.5)
    assert fit.spreadUs == pytest.approx(200)


def test_a_sharp_outlier_does_not_mask_itself_by_dragging_the_mean(lib):
    # The review's probe: the ball's small sigma pulls the three-way mean to
    # ~34.9 ms, so judged against that mean the two club tracks look like the
    # outliers. Judged pair by pair, only the club pair agrees and the ball
    # fails the gate around the club pair's own mean.
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 30_000, 800, 30),
            CLUB_OUT: ("ok", 30_100, 800, 25),
            BALL_OUT: ("ok", 35_000, 100, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == BALL_OUT
    assert fit.track[BALL_OUT].why == WHY["dropped"]
    assert fit.track[CLUB_IN].why == fit.track[CLUB_OUT].why == WHY["ok"]
    assert fit.impactUs == pytest.approx(30_050, abs=0.5)
    assert fit.spreadUs == pytest.approx(100)


@pytest.mark.parametrize("outlier", [CLUB_IN, CLUB_OUT, BALL_OUT])
def test_each_track_as_the_sharp_outlier_is_the_one_dropped(lib, outlier):
    speeds = {CLUB_IN: 30, CLUB_OUT: 25, BALL_OUT: 60}
    good = iter((30_000, 30_100))
    estimates = {}
    for which in (CLUB_IN, CLUB_OUT, BALL_OUT):
        if which == outlier:
            estimates[which] = ("ok", 35_000, 100, speeds[which])
        else:
            estimates[which] = ("ok", next(good), 800, speeds[which])
    fit = solved(lib, estimates)
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == outlier
    assert fit.track[outlier].why == WHY["dropped"]
    assert fit.impactUs == pytest.approx(30_050, abs=0.5)


def test_two_agreeing_pairs_keep_the_tighter_one(lib):
    # club_in/club_out and club_out/ball_out both agree; the three together do
    # not. The club_out/ball_out pair disagrees less, so club_in is dropped.
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 30_000, 500, 30),
            CLUB_OUT: ("ok", 31_400, 500, 25),
            BALL_OUT: ("ok", 31_600, 100, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == CLUB_IN


def test_equal_pairs_tie_to_leaving_out_the_earlier_track(lib):
    # club_in and ball_out sit symmetrically either side of club_out; the two
    # pairs that include club_out disagree equally, so the pair leaving out
    # club_in (the earlier index) wins and club_in is dropped.
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 28_000, 500, 30),
            CLUB_OUT: ("ok", 30_000, 500, 25),
            BALL_OUT: ("ok", 32_000, 500, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == CLUB_IN
    assert fit.impactUs == pytest.approx(31_000, abs=0.5)


def test_three_agreeing_around_their_mean_drop_nothing_even_with_a_sharp_one(lib):
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 30_000, 800, 30),
            CLUB_OUT: ("ok", 30_100, 800, 25),
            BALL_OUT: ("ok", 30_300, 100, 60),
        },
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.droppedTrack == fw.FIT_NO_TRACK


def test_two_disagreeing_tracks_are_inconsistent_and_take_the_smaller_sigma(lib):
    fit = solved(lib, {CLUB_IN: ("ok", 30_000, 600, 30), BALL_OUT: ("ok", 36_000, 200, 60)})
    assert fit.verdict == VERDICT["inconsistent"]
    assert fit.impactUs == pytest.approx(36_000)


def test_three_all_disagreeing_are_inconsistent(lib):
    fit = solved(
        lib,
        {
            CLUB_IN: ("ok", 20_000, 300, 30),
            CLUB_OUT: ("ok", 30_000, 200, 25),
            BALL_OUT: ("ok", 40_000, 300, 60),
        },
    )
    assert fit.verdict == VERDICT["inconsistent"]
    assert fit.impactUs == pytest.approx(30_000)


def test_one_track_is_single(lib):
    fit = solved(lib, {BALL_OUT: ("ok", 30_000, 200, 60)})
    assert fit.verdict == VERDICT["single_track"]
    assert fit.impactUs == pytest.approx(30_000)
    assert fit.spreadUs == 0.0


def test_no_track_is_none_and_reports_nothing(lib):
    fit = solved(lib, {})
    assert fit.verdict == VERDICT["none"]
    assert (fit.impactUs, fit.refinedMinusTriggerUs) == (0.0, 0.0)


def test_club_out_faster_than_club_in_breaks_physics(lib):
    fit = solved(lib, {CLUB_IN: ("ok", 30_000, 300, 30), CLUB_OUT: ("ok", 30_000, 300, 34)})
    assert fit.track[CLUB_OUT].why == WHY["physics"]
    assert fit.verdict == VERDICT["single_track"]


def test_ball_not_faster_than_club_out_breaks_physics(lib):
    fit = solved(lib, {CLUB_OUT: ("ok", 30_000, 300, 25), BALL_OUT: ("ok", 30_000, 300, 25)})
    assert fit.track[BALL_OUT].why == WHY["physics"]
    assert fit.verdict == VERDICT["single_track"]


def run(lib, club_in=(), club_out=(), ball_out=(), no_lock=0, trigger_us=27_500) -> fw.ImpactFit:
    """l3_impact_fit_run with club out and ball out as tracks (spans), club in as a list."""

    def span_of(samples):
        track_cfg = fw.TrackCfg()
        lib.l3_track_cfg_defaults(ctypes.byref(track_cfg))
        track = fw.ClubTrack()
        lib.l3_track_init(ctypes.byref(track), ctypes.byref(track_cfg))
        for frame, (t, r) in enumerate(samples):
            p = fw.TrackPoint()
            p.frame, p.timestampUs, p.rangeM, p.rangeBin = frame, int(t), r, r / BIN_M
            lib.l3_track_append_point(ctypes.byref(track), ctypes.byref(p))
        span = fw.FitSpan(ctypes.pointer(track), 0, len(samples))
        span.keep = track
        return span

    fit = fw.ImpactFit()
    lst = point_list(club_in)
    lib.l3_impact_fit_run(
        ctypes.byref(cfg(lib)),
        ctypes.byref(lst) if club_in else None,
        ctypes.byref(span_of(club_out)) if club_out else None,
        ctypes.byref(span_of(ball_out)) if ball_out else None,
        BALL_M,
        no_lock,
        trigger_us,
        ctypes.byref(fit),
    )
    return fit


def test_run_on_the_clean_scene_recovers_impact(lib):
    fit = run(
        lib,
        club_in=line(30.0, CLUB_IN_T),
        club_out=line(25.0, CLUB_OUT_T),
        ball_out=line(60.0, BALL_OUT_T),
    )
    assert fit.verdict == VERDICT["consistent"]
    assert fit.impactUs == pytest.approx(IMPACT_US, abs=5.0)
    assert fit.refinedMinusTriggerUs == pytest.approx(IMPACT_US - 27_500, abs=5.0)


def test_club_in_missing_uses_the_outgoing_tracks(lib):
    fit = run(lib, club_out=line(25.0, CLUB_OUT_T), ball_out=line(60.0, BALL_OUT_T))
    assert fit.track[CLUB_IN].why == WHY["missing"]
    assert fit.verdict == VERDICT["consistent"]
    assert fit.impactUs == pytest.approx(IMPACT_US, abs=5.0)


def test_ball_missing_fuses_the_club_either_side(lib):
    fit = run(lib, club_in=line(30.0, CLUB_IN_T), club_out=line(25.0, CLUB_OUT_T))
    assert fit.track[BALL_OUT].why == WHY["missing"]
    assert fit.verdict == VERDICT["consistent"]


def test_no_lock_is_carried(lib):
    assert run(lib, ball_out=line(60.0, BALL_OUT_T), no_lock=1).noLock == 1


def test_format_names_the_verdict_and_every_track(lib):
    fit = run(lib, club_in=line(30.0, CLUB_IN_T), ball_out=line(60.0, BALL_OUT_T))
    text = fw.c_text(lib.l3_impact_fit_format, ctypes.byref(fit), cap=240)
    match = re.match(r"impactfit verdict=consistent t=(-?\d+) ", text)
    assert match is not None
    assert abs(int(match.group(1)) - 30_000) <= 5
    assert "club_in=ok:" in text and "club_out=missing" in text and "ball_out=ok:" in text
    assert "dropped=- nolock=0" in text


# --- the sigma cap: an estimate too uncertain to place impact -----------------

NOISY_M = (0.05, -0.05, 0.06, -0.04)
FAR_BALL_T = tuple(t + 60_000 for t in BALL_OUT_T)


def test_a_noisy_far_extrapolated_line_over_the_cap_is_uncertain(lib):
    e = estimate(lib, BALL_OUT, line(60.0, FAR_BALL_T, noise_m=NOISY_M))
    assert e.why == WHY["uncertain"]
    assert e.sigmaUs > 3000.0
    # Kept for diagnostics: the time, its sigma and the speed are still filled.
    assert e.points == 4 and e.speedMps > 0.0
    assert abs(e.timeUs - IMPACT_US) < 10 * e.sigmaUs


def test_the_same_line_is_ok_with_the_cap_off_or_above_its_sigma(lib):
    samples = line(60.0, FAR_BALL_T, noise_m=NOISY_M)
    assert estimate(lib, BALL_OUT, samples, maxSigmaUs=0.0).why == WHY["ok"]
    assert estimate(lib, BALL_OUT, samples, maxSigmaUs=1e9).why == WHY["ok"]


def test_uncertain_code_follows_dropped_so_earlier_codes_keep_their_numbers():
    assert fw.FIT_WHY_NAMES.index("uncertain") == fw.FIT_WHY_NAMES.index("dropped") + 1
    assert fw.FIT_WHY_NAMES[:8] == (
        "ok",
        "missing",
        "few_points",
        "wrong_direction",
        "speed_bounds",
        "physics",
        "nonfinite",
        "dropped",
    )


def test_c_names_the_uncertain_code(lib):
    assert lib.l3_impact_fit_why_name(WHY["uncertain"]) == b"uncertain"


def test_the_fusion_never_uses_an_uncertain_track(lib):
    fit = run(
        lib,
        club_in=line(30.0, CLUB_IN_T),
        ball_out=line(60.0, FAR_BALL_T, noise_m=NOISY_M),
    )
    assert fit.track[BALL_OUT].why == WHY["uncertain"]
    assert fit.verdict == VERDICT["single_track"]
    assert fit.impactUs == pytest.approx(fit.track[CLUB_IN].timeUs)
    assert fit.droppedTrack == fw.FIT_NO_TRACK


def test_format_prints_an_uncertain_track_with_its_time_and_sigma(lib):
    fit = run(
        lib,
        club_in=line(30.0, CLUB_IN_T),
        ball_out=line(60.0, FAR_BALL_T, noise_m=NOISY_M),
    )
    text = fw.c_text(lib.l3_impact_fit_format, ctypes.byref(fit), cap=240)
    assert re.search(r"ball_out=uncertain:-?\d+\+-\d+", text), text


# --- rounding a float time to whole microseconds -------------------------------

ROUND_CASES = [
    (0.0, 0),
    (-5.0, 0),
    (float("nan"), 0),
    (float("inf"), 0),
    (1.4, 1),
    (1.5, 2),
    (29_999.5, 30_000),
    # An odd whole number past 2**23: "+ 0.5F" in float rounds it to even.
    (8_388_609.0, 8_388_609),
    (1_800_000_128.0, 1_800_000_128),
    # Past the uint32 wrap a fitted time folds back like the timestamps did.
    (2.0**32 + 1024.0, 1024),
]


@pytest.mark.parametrize("us, expected", ROUND_CASES)
def test_c_rounds_microseconds_half_up_and_folds_the_wrap(lib, us, expected):
    assert lib.l3_round_us(ctypes.c_float(us).value) == expected


@pytest.mark.parametrize("us, expected", ROUND_CASES)
def test_python_rounds_microseconds_like_the_c(us, expected):
    assert fw.round_us(ctypes.c_float(us).value) == expected


def test_format_clamps_huge_values_instead_of_overflowing_int(lib):
    fit = fw.ImpactFit()
    lib.l3_impact_fit_reset(ctypes.byref(fit))
    fit.verdict = VERDICT["inconsistent"]
    fit.impactUs, fit.spreadUs, fit.refinedMinusTriggerUs = 1.0e12, 1.0e12, -1.0e12
    e = fit.track[BALL_OUT]
    e.why, e.timeUs, e.sigmaUs = WHY["uncertain"], 1.0e12, 1.0e12
    text = fw.c_text(lib.l3_impact_fit_format, ctypes.byref(fit), cap=400)
    assert " t=0 " in text  # not a time the uint32 clock can hold
    assert "spreadus=2147483647 " in text
    assert "dtrigus=-2147483647 " in text
    assert "ball_out=uncertain:0+-2147483647" in text
