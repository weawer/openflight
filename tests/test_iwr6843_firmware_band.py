"""Tests for the tee band, firmware/iwr6843/l3_band.c."""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def span(lo: float, hi: float) -> fw.Band:
    """A valid band over [lo, hi], both edges inside."""
    return fw.Band(1, lo, hi)


# The band these tests were written around: 47 +- 6.
BAND_41_53 = (41.0, 53.0)


def no_band() -> fw.Band:
    return fw.Band()


def targets(*bins: float):
    arr = (fw.TargetObs * len(bins))()
    for i, b in enumerate(bins):
        arr[i].rangeBin = b
        arr[i].snr = 100.0 - i  # strongest first, as l3_obs_extract ranks them
    return arr


def test_an_invalid_band_contains_nothing(lib):
    b = no_band()
    assert b.valid == 0
    assert lib.l3_band_contains(ctypes.byref(b), 47.0) == 0
    assert lib.l3_band_contains(ctypes.byref(b), 0.0) == 0


def test_edges_are_inside(lib):
    b = span(*BAND_41_53)
    assert lib.l3_band_contains(ctypes.byref(b), 41.0) == 1
    assert lib.l3_band_contains(ctypes.byref(b), 53.0) == 1
    assert lib.l3_band_contains(ctypes.byref(b), 40.99) == 0
    assert lib.l3_band_contains(ctypes.byref(b), 53.01) == 0


def test_filter_removes_every_in_band_target_and_keeps_order(lib):
    b = span(*BAND_41_53)
    arr = targets(47.0, 30.0, 41.0, 60.0, 52.9, 35.5)

    kept = lib.l3_band_filter(ctypes.byref(b), arr, 6)

    assert kept == 3
    assert [arr[i].rangeBin for i in range(kept)] == [30.0, 60.0, 35.5]
    assert [arr[i].snr for i in range(kept)] == [99.0, 97.0, 95.0]


def test_filter_with_everything_in_band_keeps_nothing(lib):
    b = span(*BAND_41_53)
    arr = targets(44.0, 47.0, 50.0)
    assert lib.l3_band_filter(ctypes.byref(b), arr, 3) == 0


def test_disabled_band_filters_nothing(lib):
    b = no_band()
    arr = targets(47.0, 48.0)
    assert lib.l3_band_filter(ctypes.byref(b), arr, 2) == 2


def test_keep_short_keeps_only_targets_short_of_the_band_in_order(lib):
    """Before impact the club approaches the ball: only returns short of the
    band can be it; the band and everything beyond it are dropped."""
    b = span(*BAND_41_53)
    arr = targets(47.0, 30.0, 41.0, 60.0, 40.5, 53.0, 35.5)

    kept = lib.l3_band_keep_short(ctypes.byref(b), arr, 7)

    assert kept == 3
    assert [arr[i].rangeBin for i in range(kept)] == [30.0, 40.5, 35.5]
    assert [arr[i].snr for i in range(kept)] == [99.0, 96.0, 94.0]


def test_keep_short_drops_the_low_edge_itself(lib):
    b = span(*BAND_41_53)
    arr = targets(41.0)
    assert lib.l3_band_keep_short(ctypes.byref(b), arr, 1) == 0


def test_keep_short_with_an_invalid_band_keeps_everything(lib):
    b = no_band()
    arr = targets(47.0, 30.0, 60.0)
    assert lib.l3_band_keep_short(ctypes.byref(b), arr, 3) == 3
    assert [arr[i].rangeBin for i in range(3)] == [47.0, 30.0, 60.0]


def arm_at(lib, ball, bin_: float) -> None:
    """Arm the ball track at bin_ with the gate (time 0) as the anchor."""
    anchor = fw.BallAnchor(
        anchorBin=bin_,
        acceptFromBin=bin_,
        anchorTolUs=ball.cfg.gateTolUs,
    )
    lib.l3_ball_track_arm(ctypes.byref(ball), ctypes.byref(anchor), ctypes.byref(fw.Vec3()))


def test_ball_track_armed_at_band_edge_acquires_a_departing_ball(lib):
    """The ball's first points beyond the band are inside the tracker's
    origin gate only when it is armed at the band's far edge."""
    b = span(*BAND_41_53)
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    ball = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(ball), ctypes.byref(cfg))
    arm_at(lib, ball, b.hiBin)
    acquired = False
    # 2.5 bins per 3 ms frame (39 m/s): inside the core's 3-bin association gate.
    for frame, bin_ in enumerate((54.5, 57.0, 59.5, 62.0), start=1):
        arr = targets(bin_)
        arr[0].frame, arr[0].timestampUs, arr[0].confidence = frame, frame * 3000, 0.9
        arr[0].dopplerAliasMps = 5.0
        n = lib.l3_band_filter(ctypes.byref(b), arr, 1)
        acquired |= bool(
            lib.l3_ball_track_update_joint(
                ctypes.byref(ball), arr, n, frame, frame * 3000, fw.TRACK_NO_TARGET
            )
        )
    assert acquired
    assert ball.core.count >= 3


def depart(lib, b, arm_bin: float, bins) -> fw.BallTrack:
    """A ball departing through bins (one per 3 ms frame) with the band filter
    applied, the tracker armed at arm_bin; the tracker after the last frame."""
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    ball = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(ball), ctypes.byref(cfg))
    arm_at(lib, ball, arm_bin)
    for frame, bin_ in enumerate(bins, start=1):
        arr = targets(bin_)
        arr[0].frame, arr[0].timestampUs, arr[0].confidence = frame, frame * 3000, 0.9
        arr[0].dopplerAliasMps = 5.0
        n = lib.l3_band_filter(ctypes.byref(b), arr, 1)
        lib.l3_ball_track_update_joint(
            ctypes.byref(ball), arr, n, frame, frame * 3000, fw.TRACK_NO_TARGET
        )
    return ball


# First seen 2.5 bins past the band's far edge (53), then 2.5 bins a frame:
# 8.5 bins past the ball at 47, beyond the tracker's 8-bin origin gate.
FIRST_SEEN_PAST_THE_BAND = (55.5, 58.0, 60.5, 63.0)


def test_the_same_ball_is_acquired_when_armed_at_the_band_edge(lib):
    b = span(*BAND_41_53)
    ball = depart(lib, b, b.hiBin, FIRST_SEEN_PAST_THE_BAND)
    assert ball.confirmed == 1
    assert ball.core.count == 4


def test_armed_at_the_ball_the_band_hides_every_point_its_origin_gate_would_take(lib):
    """Why the edge matters: armed at the ball (47) the origin gate reaches
    55, the band hides up to 53, and the ball first shows at 55.5 -- so the
    departing ball is never acquired."""
    b = span(*BAND_41_53)
    ball = depart(lib, b, 47.0, FIRST_SEEN_PAST_THE_BAND)
    assert ball.core.count == 0
    assert ball.confirmed == 0


def obs_row(values, stat="peak"):
    """l3_bin_obs_t per bin carrying `values` as the peak statistic."""
    arr = (fw.BinObs * len(values))()
    for i, v in enumerate(values):
        arr[i].peak = v
        arr[i].energy = v
    return arr


STAT = fw.STAT_NAMES["peak"]


def noisy_map(lib, first_bin, values, updates=8):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    row = obs_row(values)
    for _ in range(updates):
        lib.l3_band_noise_update(ctypes.byref(noise), STAT, first_bin, row, len(values))
    return noise


def place(lib, noise, centre, width, search=10.0):
    out = fw.Band()
    lib.l3_band_place(ctypes.byref(noise), centre, search, width, ctypes.byref(out))
    return out


def test_noise_map_is_an_ema_of_the_statistic(lib):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 20, obs_row([16.0, 0.0]), 2)
    assert (noise.firstBin, noise.count, noise.updates) == (20, 2, 1)
    assert noise.avg[0] == pytest.approx(16.0)  # the first frame seeds the map
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 20, obs_row([0.0, 16.0]), 2)
    assert noise.avg[0] == pytest.approx(15.0) and noise.avg[1] == pytest.approx(1.0)


def test_noise_map_restarts_when_the_window_moves(lib):
    noise = noisy_map(lib, 20, [5.0] * 10)
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 32, obs_row([1.0] * 10), 10)
    assert (noise.firstBin, noise.updates) == (32, 1)
    assert noise.avg[0] == pytest.approx(1.0)


def test_placement_takes_the_noisiest_contiguous_run(lib):
    values = [1.0] * 53
    for b in range(24, 29):  # global bins 44..48 (first bin 20)
        values[b] = 50.0
    band = place(lib, noisy_map(lib, 20, values), centre=47.0, width=5.0)
    assert (band.valid, band.loBin, band.hiBin) == (1, 44.0, 48.0)


def test_placement_stays_in_the_search_window(lib):
    values = [1.0] * 53
    values[50] = 1000.0  # global 70: outside 47 +/- 10
    band = place(lib, noisy_map(lib, 20, values), centre=47.0, width=5.0)
    assert 37.0 <= band.loBin and band.hiBin <= 57.0


def test_ties_go_to_the_run_nearest_the_centre(lib):
    band = place(lib, noisy_map(lib, 20, [3.0] * 53), centre=47.0, width=5.0)
    assert (band.loBin, band.hiBin) == (45.0, 49.0)


def peaked(first_bin=20, size=53, peak=range(31, 36)):
    values = [1.0] * size
    for b in peak:  # global bins 51..55 with first bin 20
        values[b] = 50.0
    return values


def test_without_history_the_band_is_centred(lib):
    values = peaked()
    band = place(lib, noisy_map(lib, 20, values, updates=7), centre=47.0, width=5.0)
    assert (band.loBin, band.hiBin) == (45.0, 49.0)
    # With history: the ridge (51..55) is entirely beyond the centre, so the
    # band slides to the ridge-side edge and still holds the centre.
    band = place(lib, noisy_map(lib, 20, values, updates=8), centre=47.0, width=5.0)
    assert (band.loBin, band.hiBin) == (47.0, 51.0)


def test_width_wider_than_the_window_falls_back_to_centred(lib):
    band = place(lib, noisy_map(lib, 40, [3.0] * 6), centre=42.0, width=12.0, search=3.0)
    assert band.valid == 1
    assert band.hiBin - band.loBin == 11.0


def test_even_width_centred_is_deterministic(lib):
    # centred: lo = round(centre) - (width - 1) // 2 = 47 - 1
    band = place(lib, noisy_map(lib, 20, peaked(), updates=0), centre=47.0, width=4.0)
    assert (band.loBin, band.hiBin) == (46.0, 49.0)


def test_search_window_is_clamped_to_the_map(lib):
    # map covers global bins 40..50, narrower than 48 +/- 10; peak at its edge.
    # Holding the centre allows starts 44..48; the map ends the last start at 46.
    values = [1.0] * 11
    for b in range(6, 11):  # global 46..50
        values[b] = 50.0
    band = place(lib, noisy_map(lib, 40, values), centre=48.0, width=5.0)
    assert (band.valid, band.loBin, band.hiBin) == (1, 46.0, 50.0)


def test_noise_map_restarts_when_the_count_changes(lib):
    noise = noisy_map(lib, 20, [5.0] * 10)
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 20, obs_row([1.0] * 8), 8)
    assert (noise.count, noise.updates) == (8, 1)
    assert noise.avg[0] == pytest.approx(1.0)


def test_empty_update_is_a_no_op(lib):
    noise = noisy_map(lib, 20, [5.0] * 10)
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 30, obs_row([1.0]), 0)
    assert (noise.firstBin, noise.count, noise.updates) == (20, 10, 8)


def test_zero_width_is_no_band(lib):
    assert place(lib, noisy_map(lib, 20, [3.0] * 53), centre=47.0, width=0.0).valid == 0


@pytest.mark.parametrize(
    "ridge, expected",
    [
        (range(33, 38), (29.0, 33.0)),  # entirely beyond the ball: slides to the far edge
        (range(22, 27), (25.0, 29.0)),  # entirely short of it: slides to the near edge
        (range(27, 32), (27.0, 31.0)),  # around it: the ridge itself
    ],
)
def test_the_band_always_holds_the_centre(lib, ridge, expected):
    """start <= round(centre) <= start + width - 1: the ball's own bin is
    always inside the band, wherever the ridge lies (final-review ruling)."""
    values = [1.0] * 40
    for b in ridge:  # map first bin 10
        values[b - 10] = 50.0
    band = place(lib, noisy_map(lib, 10, values), centre=29.0, width=5.0)
    assert (band.valid, band.loBin, band.hiBin) == (1, *expected)
    assert band.loBin <= 29.0 <= band.hiBin


def test_holding_the_centre_rounds_it(lib):
    """centre 28.6 rounds to 29: a ridge beyond puts the band at 29..33."""
    values = [1.0] * 40
    for b in range(33, 38):
        values[b - 10] = 50.0
    band = place(lib, noisy_map(lib, 10, values), centre=28.6, width=5.0)
    assert (band.loBin, band.hiBin) == (29.0, 33.0)


# --- span updates (the scan plan, l3_scan.h) ---------------------------------------
#
# An armed frame scores only some bins (the club's approach, the fallback's
# stretch, a chunk of the band's interior), so idle frames feed the map span by
# span. The map stays keyed to the frame's window; placement needs history on
# every bin it might place the band over.


def update_span(lib, noise, span_first, values, window=(20, 53)):
    lib.l3_band_noise_update_span(
        ctypes.byref(noise), STAT, window[0], window[1], span_first, obs_row(values), len(values)
    )


def test_a_span_update_keys_the_map_to_the_window_and_touches_only_its_bins(lib):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    update_span(lib, noise, 30, [16.0] * 5)
    assert (noise.firstBin, noise.count) == (20, 53)
    assert [noise.seen[i] for i in range(8, 17)] == [0, 0, 1, 1, 1, 1, 1, 0, 0]
    assert noise.avg[10] == pytest.approx(16.0), "the first update seeds the bin"
    update_span(lib, noise, 30, [0.0] * 5)
    assert noise.avg[10] == pytest.approx(15.0) and noise.seen[10] == 2


def test_a_span_update_on_a_moved_window_restarts_the_map(lib):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    update_span(lib, noise, 30, [16.0] * 5)
    update_span(lib, noise, 40, [2.0] * 3, window=(32, 53))
    assert (noise.firstBin, noise.count) == (32, 53)
    assert noise.seen[30 - 32 + 32] == 0 and noise.seen[8] == 1


def test_a_span_is_clipped_to_the_window(lib):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    update_span(lib, noise, 70, [5.0] * 6)  # 70..75, window ends at 72
    assert [noise.seen[i] for i in range(49, 53)] == [0, 1, 1, 1]


def test_whole_window_updates_count_history_on_every_bin(lib):
    noise = noisy_map(lib, 20, [3.0] * 53, updates=3)
    assert {noise.seen[i] for i in range(53)} == {3}


def span_map(lib, values, first=20, updates=8, skip=None):
    """A map fed span by span over the bins placement reads, as idle frames do."""
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    for n in range(updates):
        for start in range(first, first + len(values), 4):
            if skip is not None and skip == start and n == updates - 1:
                continue
            chunk = values[start - first : start - first + 4]
            update_span(lib, noise, start, chunk, window=(first, len(values)))
    return noise


def test_placement_from_span_updates_finds_the_ridge(lib):
    band = place(lib, span_map(lib, peaked()), centre=47.0, width=5.0)
    assert (band.loBin, band.hiBin) == (47.0, 51.0)


def test_placement_needs_history_on_every_bin_it_might_cover(lib):
    """One chunk (global 48..51) a round short: the band stays centred."""
    band = place(lib, span_map(lib, peaked(), skip=48), centre=47.0, width=5.0)
    assert (band.loBin, band.hiBin) == (45.0, 49.0)


# --- the clutter map: expected return and spread per bin -----------------------
#
# The map is fed only on frames with no club track, so it learns the scene at
# address: the golfer's body in the bins short of the ball, still scatterers.
# A pre-impact target that does not beat its bin's expected return by
# `sigmas` spreads is that clutter, not the club (2026-10-01: the body out-
# returned the club by 10-30 dB and took its track).


def clutter_target(peak_bin: int, stat: float):
    t = fw.TargetObs()
    t.peakBin = peak_bin
    t.rangeBin = float(peak_bin)
    t.stat = stat
    t.snr = stat
    return t


def clutter_filter(lib, noise, sigmas, *items):
    arr = (fw.TargetObs * max(1, len(items)))(*items)
    kept = lib.l3_band_clutter_filter(ctypes.byref(noise), sigmas, arr, len(items))
    return [arr[i].peakBin for i in range(kept)]


def alternating_map(lib, first_bin=40, size=11, low=90.0, high=110.0, updates=64):
    """Every bin swings between low and high: mean 100, spread ~10."""
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    for k in range(updates):
        value = low if k % 2 == 0 else high
        lib.l3_band_noise_update(
            ctypes.byref(noise), STAT, first_bin, obs_row([value] * size), size
        )
    return noise


def test_noise_map_keeps_a_spread_the_ema_of_the_deviation(lib):
    noise = fw.BandNoise()
    lib.l3_band_noise_reset(ctypes.byref(noise))
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 20, obs_row([16.0]), 1)
    assert noise.dev[0] == pytest.approx(0.0)  # a seed has no spread yet
    lib.l3_band_noise_update(ctypes.byref(noise), STAT, 20, obs_row([0.0]), 1)
    # |0 - 16| against the mean before this update, at the map's 1/16
    assert noise.dev[0] == pytest.approx(1.0)


def test_the_spread_settles_on_the_scenes_swing(lib):
    noise = alternating_map(lib)
    assert noise.avg[5] == pytest.approx(100.0, abs=1.5)
    assert noise.dev[5] == pytest.approx(10.0, rel=0.15)


def test_clutter_filter_drops_what_the_bin_usually_returns_and_keeps_order(lib):
    noise = alternating_map(lib)
    kept = clutter_filter(
        lib,
        noise,
        3.0,
        clutter_target(45, 125.0),  # within 3 spreads of 100: the scene
        clutter_target(46, 150.0),  # well over: something new
        clutter_target(44, 60.0),
        clutter_target(47, 140.0),
    )
    assert kept == [46, 47]


def test_clutter_filter_keeps_bins_the_map_has_not_learned(lib):
    noise = alternating_map(lib, first_bin=40, size=11)
    kept = clutter_filter(lib, noise, 3.0, clutter_target(30, 50.0), clutter_target(60, 50.0))
    assert kept == [30, 60]


def test_clutter_filter_needs_the_bins_history(lib):
    noise = alternating_map(lib, updates=4)
    assert clutter_filter(lib, noise, 3.0, clutter_target(45, 101.0)) == [45]


def test_clutter_filter_with_no_sigmas_keeps_everything(lib):
    noise = alternating_map(lib)
    assert clutter_filter(lib, noise, 0.0, clutter_target(45, 101.0)) == [45]
