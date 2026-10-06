"""Capture windows placed on the tee: monitor.tee_relative_config.

The shipped configs hard-code impact/ball windows (32/47) built for a tee at
1.57 m. A tee at 1.36 m (bin 29) then falls short of the impact window and
the capture keeps no club at impact and no ball leaving the tee.
"""

from __future__ import annotations

import ctypes
from pathlib import Path

import pytest

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.monitor import (
    CAPTURE_MARGIN_BINS,
    IWR6843CaptureMonitor,
    tee_global_bin,
    tee_relative_config,
)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
DEFAULT_CONFIG = CONFIG_DIR / "iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
FFT_BINS = 128
PHASE = "phaseCaptureCfg 20 53 9 32 53 7 47 53 47 8 1"


def phase_values(lines: list[str]) -> list[int]:
    (line,) = [line for line in lines if line.startswith("phaseCaptureCfg")]
    return [int(field) for field in line.split()[1:]]


def windows(lines: list[str]) -> dict[str, tuple[int, int]]:
    """(first, last) global bin of each phase window."""
    v = phase_values(lines)
    return {
        "pre": (v[0], v[0] + v[1] - 1),
        "impact": (v[3], v[3] + v[4] - 1),
        "post": (v[6], v[6] + v[7] - 1),
        "late": (v[8], v[8] + v[7] - 1),
    }


def test_impact_and_ball_windows_start_short_of_a_136m_tee():
    """The reported capture: tee at bin 29, impact window started at 32."""
    out = tee_relative_config(["captureFormat iq16", PHASE, "sensorStart"], 29)

    w = windows(out)
    assert w["impact"][0] == 29 - CAPTURE_MARGIN_BINS
    assert w["post"][0] == 29 - CAPTURE_MARGIN_BINS
    assert w["late"][0] == 29 - CAPTURE_MARGIN_BINS + 53 // 2
    for name in ("impact", "post"):
        first, last = w[name]
        assert first < 29 <= last, f"{name} window {w[name]} must hold the tee and bins short of it"


def test_pre_window_frames_widths_and_stride_are_kept():
    v = phase_values(tee_relative_config([PHASE], 29))
    assert v[:3] == [20, 53, 9]
    assert v[4:6] == [53, 7]
    assert v[7] == 53
    assert v[9:] == [8, 1]


def test_other_lines_keep_their_order_and_adaptive_follows_from_the_tee():
    lines = ["% comment", "captureFormat iq16", PHASE, "lowPower 0 0", "sensorStart"]

    out = tee_relative_config(lines, 29)

    assert out[:2] == lines[:2]
    # The firmware's lock-follow keeps the configured pre window when the
    # ball sits on the tee: approach = tee - preStart.
    assert out[2] == f"captureCfg adaptive 1 {29 - 20} {CAPTURE_MARGIN_BINS}"
    assert out[3].startswith("phaseCaptureCfg ")
    assert out[4:] == lines[3:]


def test_an_existing_adaptive_line_is_the_config_authors_and_is_kept():
    lines = ["captureCfg adaptive 0 24 4", PHASE]

    out = tee_relative_config(lines, 29)

    assert [line for line in out if line.startswith("captureCfg adaptive")] == [
        "captureCfg adaptive 0 24 4"
    ]


def test_windows_clip_to_the_fft_near_the_far_end():
    out = tee_relative_config(["phaseCaptureCfg 70 53 9 90 53 7 90 53 100 8 1"], 110)

    w = windows(out)
    assert w["impact"] == (FFT_BINS - 53, FFT_BINS - 1)
    assert w["post"] == (FFT_BINS - 53, FFT_BINS - 1)
    assert w["late"] == (FFT_BINS - 53, FFT_BINS - 1)


def test_windows_clip_at_bin_zero_for_a_tee_inside_the_margin():
    w = windows(tee_relative_config(["phaseCaptureCfg 0 53 9 32 53 7 47 53 47 8 1"], 2))
    assert w["impact"][0] == 0
    assert w["post"][0] == 0
    assert w["late"][0] == 53 // 2


def test_narrow_impact_window_still_holds_the_tee():
    w = windows(tee_relative_config(["phaseCaptureCfg 20 53 9 32 8 7 47 53 47 8 1"], 29))
    assert w["impact"] == (25, 32)


def test_tee_outside_the_pre_window_is_refused():
    with pytest.raises(ValueError, match="outside the pre-impact window"):
        tee_relative_config([PHASE], 19)
    with pytest.raises(ValueError, match="outside the pre-impact window"):
        tee_relative_config([PHASE], 73)


def test_config_without_phase_capture_is_refused():
    with pytest.raises(ValueError, match="phaseCaptureCfg"):
        tee_relative_config(["captureCfg 20 53 32 53 47 8"], 29)


def test_two_phase_capture_lines_are_refused():
    with pytest.raises(ValueError, match="phaseCaptureCfg"):
        tee_relative_config([PHASE, PHASE], 29)


def test_malformed_phase_capture_is_refused():
    with pytest.raises(ValueError, match="11 values"):
        tee_relative_config(["phaseCaptureCfg 20 53 9"], 29)


@pytest.mark.parametrize(
    "config",
    sorted(
        p for p in CONFIG_DIR.glob("iwr6843_l3dump_*.cfg") if "phaseCaptureCfg" in p.read_text()
    ),
    ids=lambda p: p.stem,
)
@pytest.mark.parametrize("tee_m", [1.36, 1.575, 2.0])
def test_every_shipped_profile_covers_the_tee_and_passes_the_firmware_limits(config, tee_m):
    lines = config.read_text(encoding="utf-8").splitlines()
    tee = tee_global_bin(tee_m, config)

    w = windows(tee_relative_config(lines, tee))

    for name in ("impact", "post"):
        assert w[name][0] < tee <= w[name][1], f"{config.name} {name} {w[name]}"
    for first, last in w.values():
        # l3_cli_phaseCaptureCfg: start + bins <= N_SAMPLES.
        assert 0 <= first and last < FFT_BINS


# --- parity with the firmware's l3_adaptive_windows --------------------------


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


@pytest.mark.parametrize("tee", [0, 2, 29, 34, 46, 72])
@pytest.mark.parametrize("impact_bins", [8, 53])
def test_windows_match_the_firmware_lock_follow_for_a_ball_on_the_tee(lib, tee, impact_bins):
    """So a lock on the tee leaves the windows the Pi sent where they are."""
    pre_start = max(0, tee - 20)
    line = f"phaseCaptureCfg {pre_start} 53 9 32 {impact_bins} 7 47 53 47 8 1"
    v = phase_values(tee_relative_config([line], tee))

    cfg = fw.AdaptiveCfg()
    lib.l3_adaptive_cfg_defaults(ctypes.byref(cfg))
    cfg.enabled = 1
    cfg.approachBins = tee - pre_start
    cfg.marginBins = CAPTURE_MARGIN_BINS
    out = fw.AdaptiveWindows()
    assert lib.l3_adaptive_windows(
        ctypes.byref(cfg), tee, FFT_BINS, 53, impact_bins, 53, ctypes.byref(out)
    )

    assert (v[0], v[3], v[6], v[8]) == (
        out.preStart,
        out.impactStart,
        out.postStart,
        out.lateStart,
    )


# --- the monitor sends the rewritten config ---------------------------------


class ConfigRadar:
    port = "/dev/fake-iwr6843"

    def __init__(self):
        self.sent = []

    def firmware_version(self):
        return None

    def send_config(self, path, lines=None):
        self.sent.append((path, lines))

    def set_tee_band(self, bins, min_span_us=None):
        return True

    def set_ball_snr(self, snr):
        return True

    def set_radar_cal(self, args):
        return True

    def set_elements(self, phases, gains):
        return True

    def stop_sensor(self):
        pass

    def close(self):
        pass


def test_monitor_sends_tee_relative_windows_when_it_knows_the_tee(tmp_path):
    radar = ConfigRadar()
    monitor = IWR6843CaptureMonitor(
        config_path=DEFAULT_CONFIG,
        output_dir=tmp_path,
        radar=radar,
        button_factory=lambda *_a, **_k: None,
        tee_range_m=1.36,
    )

    monitor.start(armed=False)
    monitor.stop()

    (path, lines) = radar.sent[0]
    assert path == str(DEFAULT_CONFIG)
    assert windows(lines)["impact"][0] == 29 - CAPTURE_MARGIN_BINS


def test_monitor_without_a_tee_sends_the_file_unchanged(tmp_path):
    radar = ConfigRadar()
    monitor = IWR6843CaptureMonitor(
        config_path=DEFAULT_CONFIG,
        output_dir=tmp_path,
        radar=radar,
        button_factory=lambda *_a, **_k: None,
    )

    monitor.start(armed=False)
    monitor.stop()

    assert radar.sent[0] == (str(DEFAULT_CONFIG), None)


def test_monitor_refuses_a_tee_outside_the_capture_before_touching_the_radar(tmp_path):
    radar = ConfigRadar()
    monitor = IWR6843CaptureMonitor(
        config_path=DEFAULT_CONFIG,
        output_dir=tmp_path,
        radar=radar,
        button_factory=lambda *_a, **_k: None,
        tee_range_m=0.5,
    )

    with pytest.raises(ValueError, match="outside the first capture window"):
        monitor.start(armed=False)
    assert radar.sent == []
