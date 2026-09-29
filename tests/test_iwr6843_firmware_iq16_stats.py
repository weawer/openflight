"""Tests for the exact IQ16 channel statistics (firmware/iwr6843/l3_iq16_stats.c)
and the parabolic sub-bin range (l3_observation.c).

The integer path must equal Python's exact arithmetic bit for bit before the
one float conversion, and reproduce the float residual pass to within its
rounding; the parabola must find a sampled main lobe's peak and never leave
the centre bin's half.
"""

from __future__ import annotations

import ctypes
import math

import numpy as np
import pytest

from openflight.iwr6843 import firmware_host as fw


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def channel(lib, values: list[tuple[int, int]], stride: int = 2):
    """values: (im, re) per loop, laid out with `stride` words between loops."""
    loops = len(values)
    words = np.zeros(loops * stride + 2, dtype=np.int16)
    for loop, (im, re) in enumerate(values):
        words[loop * stride] = im
        words[loop * stride + 1] = re
    out = fw.Iq16ChannelStats()
    status = lib.l3_iq16_channel_stats(
        words.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)), loops, stride, ctypes.byref(out)
    )
    return status, out


def exact_reference(values: list[tuple[int, int]]):
    loops = len(values)
    sum_im = sum(v[0] for v in values)
    sum_re = sum(v[1] for v in values)
    power = []
    energy = r1re = r1im = 0
    prev = None
    for im, re in values:
        sim, sre = loops * im - sum_im, loops * re - sum_re
        p = sim * sim + sre * sre
        power.append(p)
        energy += p
        if prev is not None:
            r1re += sre * prev[1] + sim * prev[0]
            r1im += sim * prev[1] - sre * prev[0]
        prev = (sim, sre)
    return sum_im, sum_re, energy, power, r1re, r1im


@pytest.mark.parametrize("seed", [1, 2, 3])
@pytest.mark.parametrize("loops", [2, 12, 16])
def test_channel_stats_are_exact_integers(lib, seed, loops):
    rng = np.random.default_rng(seed)
    values = [(int(a), int(b)) for a, b in rng.integers(-32768, 32768, size=(loops, 2))]
    status, out = channel(
        lib, values, stride=8 * 2 * 53
    )  # a 53-bin, 4-RX, 2-TX frame's loop stride
    assert status == 0
    sum_im, sum_re, energy, power, r1re, r1im = exact_reference(values)
    assert (out.sumIm, out.sumRe) == (sum_im, sum_re)
    assert out.energy == energy
    assert list(out.loopPower[:loops]) == power
    assert (out.r1Re, out.r1Im) == (r1re, r1im)
    assert all(v == 0 for v in out.loopPower[loops:])


def test_full_scale_extremes_do_not_overflow(lib):
    values = [(-32768, 32767), (32767, -32768)] * 8  # 16 loops of the widest swing
    status, out = channel(lib, values)
    assert status == 0
    _, _, energy, power, _, _ = exact_reference(values)
    assert out.energy == energy and list(out.loopPower) == power
    assert 2**40 < out.energy < 2**63


def test_channel_stats_reject_bad_loop_counts(lib):
    assert channel(lib, [])[0] == -1
    assert channel(lib, [(1, 1)] * 17)[0] == -1


def test_bin_stats_sum_channels_and_finish_in_physical_units(lib):
    rng = np.random.default_rng(9)
    loops = 12
    channels = [
        [(int(a), int(b)) for a, b in rng.integers(-3000, 3000, size=(loops, 2))] for _ in range(8)
    ]
    bin_stats = fw.Iq16BinStats()
    lib.l3_iq16_bin_stats_init(ctypes.byref(bin_stats), loops)
    for values in channels:
        _, ch = channel(lib, values)
        lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(ch))
    assert bin_stats.channels == 8
    energy, peak, loop0, r1re, r1im = (ctypes.c_float() for _ in range(5))
    per_loop = (ctypes.c_float * 16)()
    lib.l3_iq16_bin_stats_finish(
        ctypes.byref(bin_stats),
        ctypes.byref(energy),
        ctypes.byref(peak),
        ctypes.byref(loop0),
        ctypes.byref(r1re),
        ctypes.byref(r1im),
        per_loop,
    )
    # The float residual pass: mean removed, squares summed over channels.
    ref_energy = ref_r1re = ref_r1im = 0.0
    ref_power = [0.0] * loops
    for values in channels:
        arr = np.array(values, dtype=np.float64)
        resid = arr - arr.mean(axis=0)
        p = (resid**2).sum(axis=1)
        ref_power = [a + b for a, b in zip(ref_power, p)]
        ref_energy += p.sum()
        c = resid[:, 1] + 1j * resid[:, 0]
        r1 = (c[1:] * np.conj(c[:-1])).sum()
        ref_r1re += r1.real
        ref_r1im += r1.imag
    assert energy.value == pytest.approx(ref_energy, rel=1e-6)
    assert peak.value == pytest.approx(max(ref_power), rel=1e-6)
    assert loop0.value == pytest.approx(ref_power[0], rel=1e-6)
    assert r1re.value == pytest.approx(ref_r1re, rel=1e-6, abs=1e-3)
    assert r1im.value == pytest.approx(ref_r1im, rel=1e-6, abs=1e-3)
    assert [per_loop[i] for i in range(loops)] == pytest.approx(ref_power, rel=1e-6)
    # A channel with another loop count is refused, not mixed in.
    _, odd = channel(lib, [(1, 2)] * 4)
    lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(odd))
    assert bin_stats.channels == 8


# --- range window --------------------------------------------------------------


def windowed_channel(lib, bins: list[list[tuple[int, int]]], window: int, stride: int = 6):
    """bins: per loop, the (im, re) of bins k-1, k, k+1, laid out as the board
    lays out adjacent bins of one channel. Stats are taken at bin k."""
    loops = len(bins)
    words = np.zeros(loops * stride + 6, dtype=np.int16)
    for loop, triple in enumerate(bins):
        for offset, (im, re) in enumerate(triple):
            words[loop * stride + 2 * offset] = im
            words[loop * stride + 2 * offset + 1] = re
    centre = ctypes.cast(
        words.ctypes.data + 2 * ctypes.sizeof(ctypes.c_int16), ctypes.POINTER(ctypes.c_int16)
    )
    out = fw.Iq16ChannelStats()
    status = lib.l3_iq16_channel_stats_windowed(centre, loops, stride, window, ctypes.byref(out))
    return status, out


def finish(lib, bin_stats):
    energy, peak, loop0, r1re, r1im = (ctypes.c_float() for _ in range(5))
    lib.l3_iq16_bin_stats_finish(
        ctypes.byref(bin_stats),
        ctypes.byref(energy),
        ctypes.byref(peak),
        ctypes.byref(loop0),
        ctypes.byref(r1re),
        ctypes.byref(r1im),
        None,
    )
    return energy.value, peak.value, loop0.value, r1re.value, r1im.value


def test_the_kernel_is_the_periodic_hann_window_in_time():
    """The claim the firmware rests on, in numpy alone: for a 128-point FFT of
    128 samples, X[k] - (X[k-1] + X[k+1]) / 2 is the FFT of the signal times
    2 x the periodic Hann window, for every bin (cyclically)."""
    n = 128
    rng = np.random.default_rng(4)
    x = rng.normal(size=n) + 1j * rng.normal(size=n)
    spectrum = np.fft.fft(x)
    kernel = spectrum - 0.5 * (np.roll(spectrum, 1) + np.roll(spectrum, -1))
    hann = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)
    np.testing.assert_allclose(kernel, np.fft.fft(2 * hann * x), atol=1e-9)


@pytest.mark.parametrize("seed", [1, 2])
@pytest.mark.parametrize("loops", [2, 12, 16])
def test_hann_channel_stats_are_exact_integers(lib, seed, loops):
    rng = np.random.default_rng(seed)
    bins = [
        [(int(a), int(b)) for a, b in rng.integers(-32768, 32768, size=(3, 2))]
        for _ in range(loops)
    ]
    status, out = windowed_channel(lib, bins, fw.RANGE_WINDOW_HANN)
    assert status == 0 and out.window == fw.RANGE_WINDOW_HANN
    doubled = [
        (2 * mid[0] - left[0] - right[0], 2 * mid[1] - left[1] - right[1])
        for left, mid, right in bins
    ]
    sum_im, sum_re, energy, power, r1re, r1im = exact_reference(doubled)
    assert (out.sumIm, out.sumRe) == (sum_im, sum_re)
    assert out.energy == energy
    assert list(out.loopPower[:loops]) == power
    assert (out.r1Re, out.r1Im) == (r1re, r1im)


def test_no_window_through_the_windowed_entry_is_the_plain_statistics(lib):
    rng = np.random.default_rng(5)
    bins = [
        [(int(a), int(b)) for a, b in rng.integers(-32768, 32768, size=(3, 2))] for _ in range(12)
    ]
    status, windowed = windowed_channel(lib, bins, fw.RANGE_WINDOW_NONE)
    _, plain = channel(lib, [triple[1] for triple in bins])
    assert status == 0 and windowed.window == fw.RANGE_WINDOW_NONE
    assert bytes(windowed) == bytes(plain)


def test_hann_full_scale_extremes_do_not_overflow(lib):
    # The widest windowed swing: centre and both neighbours at opposite rails.
    high = [(-32768, -32768), (32767, 32767), (-32768, -32768)]
    low = [(32767, 32767), (-32768, -32768), (32767, 32767)]
    bins = [high, low] * 8
    status, out = windowed_channel(lib, bins, fw.RANGE_WINDOW_HANN)
    assert status == 0
    doubled = [
        (2 * mid[0] - left[0] - right[0], 2 * mid[1] - left[1] - right[1])
        for left, mid, right in bins
    ]
    _, _, energy, power, _, _ = exact_reference(doubled)
    assert out.energy == energy and list(out.loopPower) == power
    # Eight channels of this still fit int64 and double's exact integers.
    assert 8 * out.energy < 2**53


def test_an_unknown_window_is_refused(lib):
    status, _ = windowed_channel(lib, [[(1, 1)] * 3] * 4, 7)
    assert status == -1


def test_bin_stats_refuse_a_channel_with_another_window(lib):
    rng = np.random.default_rng(6)
    bins = [
        [(int(a), int(b)) for a, b in rng.integers(-3000, 3000, size=(3, 2))] for _ in range(12)
    ]
    _, hann = windowed_channel(lib, bins, fw.RANGE_WINDOW_HANN)
    _, plain = windowed_channel(lib, bins, fw.RANGE_WINDOW_NONE)
    bin_stats = fw.Iq16BinStats()
    lib.l3_iq16_bin_stats_init(ctypes.byref(bin_stats), 12)
    lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(hann))
    lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(plain))
    assert bin_stats.channels == 1 and bin_stats.window == fw.RANGE_WINDOW_HANN


def _moving_tone_spectra(tone_bin: float, loops: int, amplitude: float, n: int = 128):
    """Range FFT of a tone at tone_bin whose phase advances each loop (a mover,
    so the burst-MTI residual keeps it), quantized to int16: [loops, n]."""
    samples = np.arange(n)
    spectra = []
    for loop in range(loops):
        x = np.exp(2j * np.pi * (tone_bin * samples / n + 0.23 * loop))
        spectra.append(np.fft.fft(x) * amplitude / n)
    return np.round(np.array(spectra))


def _bin_energy(lib, spectra: np.ndarray, k: int, window: int) -> float:
    bins = [[(int(s[j].imag), int(s[j].real)) for j in (k - 1, k, k + 1)] for s in spectra]
    _, stats = windowed_channel(lib, bins, window)
    bin_stats = fw.Iq16BinStats()
    lib.l3_iq16_bin_stats_init(ctypes.byref(bin_stats), len(spectra))
    lib.l3_iq16_bin_stats_add(ctypes.byref(bin_stats), ctypes.byref(stats))
    return finish(lib, bin_stats)[0]


def test_hann_keeps_an_on_bin_return_and_buries_its_sidelobes(lib):
    """A strong mover between bins 40 and 41: unwindowed, bin 46 (5.5 bins
    away) sits about 25 dB under it, where a ball could hide; through the
    window it falls by at least another 25 dB. An on-bin return keeps its
    energy (the window is gain-compensated)."""
    loops = 12
    off_bin = _moving_tone_spectra(40.5, loops, 20000.0)
    for window in (fw.RANGE_WINDOW_NONE, fw.RANGE_WINDOW_HANN):
        assert _bin_energy(lib, off_bin, 40, window) > 0.0
    plain_ratio = _bin_energy(lib, off_bin, 46, fw.RANGE_WINDOW_NONE) / _bin_energy(
        lib, off_bin, 40, fw.RANGE_WINDOW_NONE
    )
    hann_ratio = _bin_energy(lib, off_bin, 46, fw.RANGE_WINDOW_HANN) / _bin_energy(
        lib, off_bin, 40, fw.RANGE_WINDOW_HANN
    )
    assert 10 * math.log10(plain_ratio) == pytest.approx(-21.0, abs=4.0)
    assert 10 * math.log10(hann_ratio) < 10 * math.log10(plain_ratio) - 25.0
    on_bin = _moving_tone_spectra(40.0, loops, 20000.0)
    assert _bin_energy(lib, on_bin, 40, fw.RANGE_WINDOW_HANN) == pytest.approx(
        _bin_energy(lib, on_bin, 40, fw.RANGE_WINDOW_NONE), rel=1e-3
    )


# --- parabolic sub-bin ---------------------------------------------------------


def test_parabolic_offset_recovers_a_gaussian_lobe_peak_exactly(lib):
    for true_offset in (-0.45, -0.25, 0.0, 0.1, 0.3, 0.49):
        # A Gaussian main lobe sampled at -1, 0, +1 bins around the true peak:
        # a parabola in the log domain, so the vertex is the peak.
        width = 1.2
        left, centre, right = (
            math.exp(-((x - true_offset) ** 2) / width) for x in (-1.0, 0.0, 1.0)
        )
        got = lib.l3_obs_parabolic_offset(left, centre, right)
        assert got == pytest.approx(true_offset, abs=1e-3), true_offset
    assert lib.l3_obs_parabolic_offset(1.0, 5.0, 1.0) == 0.0


def _lobe_powers(offset: float, window: np.ndarray | None = None, n: int = 128, k: int = 47):
    x = np.exp(2j * np.pi * (k + offset) * np.arange(n) / n)
    if window is not None:
        x = x * window
    power = np.abs(np.fft.fft(x)) ** 2
    return float(power[k - 1]), float(power[k]), float(power[k + 1])


def test_parabolic_offset_bias_on_the_range_fft_lobe_is_bounded(lib):
    """The unwindowed 128-point range FFT's sinc^2 lobe is not a parabola in
    any domain; the log fit's worst error is documented here so the trackers'
    range noise floor is known (about 8 mm at 46.9 mm per bin). A Hann window
    would make it almost exact, which is a waveform decision for later."""
    worst_plain = max(
        abs(lib.l3_obs_parabolic_offset(*_lobe_powers(off)) - off)
        for off in np.linspace(-0.49, 0.49, 25)
    )
    assert worst_plain < 0.18
    worst_hann = max(
        abs(lib.l3_obs_parabolic_offset(*_lobe_powers(off, np.hanning(128))) - off)
        for off in np.linspace(-0.49, 0.49, 25)
    )
    assert worst_hann < 0.02


def test_parabolic_offset_is_clamped_and_falls_back_on_non_peaks(lib):
    assert lib.l3_obs_parabolic_offset(4.0, 4.0, 4.0) == 0.0, "flat"
    assert lib.l3_obs_parabolic_offset(6.0, 2.0, 6.0) == 0.0, "a valley"
    assert lib.l3_obs_parabolic_offset(1.0, 10.0, 9.99) == pytest.approx(0.499, abs=2e-3)
    assert lib.l3_obs_parabolic_offset(9.99, 10.0, 1.0) == pytest.approx(-0.499, abs=2e-3)
    # Not a peak at the centre but still concave: the vertex lies past the
    # neighbour, and the guard keeps it inside the centre bin's half.
    assert lib.l3_obs_parabolic_offset(0.0, 10.0, 15.0) == 0.5
    assert lib.l3_obs_parabolic_offset(15.0, 10.0, 0.0) == -0.5


def _bins(values: dict[int, float], first=20, count=16, noise=100.0):
    obs = (fw.BinObs * count)()
    for i in range(count):
        peak = values.get(first + i, noise)
        obs[i].peak = peak
        obs[i].energy = 4.0 * peak
        obs[i].loop0 = peak / 3.0
        obs[i].r1Re = 0.9 * obs[i].energy
    return obs


def _extract(lib, params, obs, out):
    return lib.l3_obs_extract(
        ctypes.byref(params), 1, 1000, 20, obs, 16, 100.0, out, fw.OBS_MAX_TARGETS
    )


def test_extract_uses_the_parabola_when_asked_and_the_centroid_otherwise(lib):
    obs = _bins({25: 4000.0, 26: 10000.0, 27: 7000.0})
    out = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    parabolic = fw.ObsParams(fw.STAT_PEAK, 6.0, 135e-6, fw.SUBBIN_PARABOLIC)
    centroid = fw.ObsParams(fw.STAT_PEAK, 6.0, 135e-6, fw.SUBBIN_CENTROID)
    assert _extract(lib, parabolic, obs, out) == 1
    parabola = out[0].rangeBin
    left, centre, right = (math.log(v) for v in (4000.0, 10000.0, 7000.0))
    expected = 26 + 0.5 * (left - right) / (left - 2 * centre + right)
    assert parabola == pytest.approx(expected, abs=1e-4)
    assert _extract(lib, centroid, obs, out) == 1
    centre = out[0].rangeBin
    assert centre != pytest.approx(parabola, abs=1e-3)
    assert 26.0 < centre < 26.5 and 26.0 < parabola < 26.5
    # A peak on the region's edge has no left neighbour: the centroid stands in.
    edge = _bins({20: 10000.0, 21: 3000.0})
    assert _extract(lib, parabolic, edge, out) == 1
    assert 20.0 <= out[0].rangeBin < 20.5


def test_asymmetric_neighbours_move_both_estimators_the_same_way(lib):
    """A stronger return two bins away lifts the weaker target's inner neighbour:
    both estimators lean toward it (the lobe really is off-centre), and both
    stay inside the peak bin's half."""
    obs = _bins({30: 30000.0, 31: 2000.0, 32: 3000.0, 33: 250.0})
    out = (fw.TargetObs * fw.OBS_MAX_TARGETS)()
    leans = {}
    for mode, name in ((fw.SUBBIN_PARABOLIC, "parabolic"), (fw.SUBBIN_CENTROID, "centroid")):
        params = fw.ObsParams(fw.STAT_PEAK, 6.0, 135e-6, mode)
        found = _extract(lib, params, obs, out)
        assert found == 2
        weaker = next(out[i] for i in range(found) if out[i].peakBin == 32)
        leans[name] = 32.0 - weaker.rangeBin
    assert 0.0 < leans["parabolic"] <= 0.5 and 0.0 < leans["centroid"] <= 0.5
    # A symmetric peak stays on its bin either way.
    symmetric = _bins({28: 900.0, 29: 5000.0, 30: 900.0})
    for mode in (fw.SUBBIN_PARABOLIC, fw.SUBBIN_CENTROID):
        params = fw.ObsParams(fw.STAT_PEAK, 6.0, 135e-6, mode)
        assert _extract(lib, params, symmetric, out) == 1
        assert out[0].rangeBin == pytest.approx(29.0, abs=1e-4)
