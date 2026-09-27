"""Behavioural tests for the IWR6843 self-trigger detector, firmware/iwr6843/l3_trigger.c.

The detector is plain C with no hardware access, so it is built here with the
host C compiler and driven through ctypes with synthetic frames: a noise floor
across the watch region plus a "clubhead" whose range bin and Doppler are
chosen per frame. Every test states what a real swing (or non-swing) looks
like to the radar and asserts the detector's verdict and its telemetry.
"""

from __future__ import annotations

import ctypes
import math
import shutil
import subprocess
from pathlib import Path

import pytest

FIRMWARE_DIR = Path(__file__).parents[1] / "firmware" / "iwr6843"
SOURCE = FIRMWARE_DIR / "l3_trigger.c"

# Mirror l3_trigger.h.
MAX_BINS = 64
LOG_DEPTH = 128
TRACE_DEPTH = 64
TRACE_RATIO = 2.0
COUNT_TOTAL = 12
NO_BIN = 0xFF
STATE_IDLE, STATE_TRACKING, STATE_FIRED = 0, 1, 2
WHY = {
    name: index
    for index, name in enumerate(
        [
            "quiet",
            "acquired",
            "advanced",
            "jumped",
            "missed",
            "lost",
            "lowcoh",
            "slowdop",
            "young",
            "slow",
            "fired",
        ]
    )
}
COUNT = {
    name: index
    for index, name in enumerate(
        [
            "frames",
            "cand",
            "acq",
            "adv",
            "jump",
            "miss",
            "lost",
            "lowcoh",
            "slowdop",
            "young",
            "slow",
            "fired",
        ]
    )
}
WAVELENGTH_M = 0.00484
LOOP_PERIOD_S = 135e-6  # 3 TX x (7 us idle + 38 us ramp) on the wide profile

# Fixture geometry: tee at local bin 20, 12 approach bins, gate +/- 3 -> the
# region is bins 8..23 and the gate is bins 17..23.
TEE = 20
APPROACH = 12
GATE = 3
NOISE = 100.0
# Strongest loop as a fraction of the 12-loop energy for a target present in
# every loop, and for noise: 1/12 each, a little more for the maximum.
PEAK_FRACTION = 0.25
# Loop 0 alone, as the first detector probed it: one loop of twelve.
LOOP0_FRACTION = 1.0 / 12.0


class Cfg(ctypes.Structure):
    _fields_ = [
        ("teeBin", ctypes.c_uint32),
        ("snr", ctypes.c_float),
        ("trackFrames", ctypes.c_uint32),
        ("approachBins", ctypes.c_uint32),
        ("gateBins", ctypes.c_uint32),
        ("minCoherence", ctypes.c_float),
        ("minStepBins", ctypes.c_float),
        ("stat", ctypes.c_uint32),
        ("minSpeedMps", ctypes.c_float),
    ]


STAT_ENERGY, STAT_PEAK = 0, 1


class Obs(ctypes.Structure):
    _fields_ = [
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("r1Re", ctypes.c_float),
        ("r1Im", ctypes.c_float),
    ]


class Trace(ctypes.Structure):
    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("gap", ctypes.c_uint16),
        ("bin", ctypes.c_uint8),
        ("state", ctypes.c_uint8),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("loop0", ctypes.c_float),
        ("floor", ctypes.c_float),
        ("threshold", ctypes.c_float),
        ("coherencePct", ctypes.c_uint8),
        ("dest", ctypes.c_uint8),
    ]


class Record(ctypes.Structure):
    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("gap", ctypes.c_uint16),
        ("state", ctypes.c_uint8),
        ("why", ctypes.c_uint8),
        ("bin", ctypes.c_uint8),
        ("age", ctypes.c_uint8),
        ("velocityCms", ctypes.c_int16),
        ("energy", ctypes.c_float),
        ("peak", ctypes.c_float),
        ("floor", ctypes.c_float),
        ("coherencePct", ctypes.c_uint8),
        ("dest", ctypes.c_uint8),
    ]


class Trig(ctypes.Structure):
    _fields_ = [
        ("cfg", Cfg),
        ("state", ctypes.c_uint8),
        ("traceEnabled", ctypes.c_uint8),
        ("floor", ctypes.c_float),
        ("loopPeriodS", ctypes.c_float),
        ("trackBin", ctypes.c_uint8),
        ("trackStartBin", ctypes.c_uint8),
        ("trackAge", ctypes.c_uint8),
        ("trackMisses", ctypes.c_uint8),
        ("trackStartFrame", ctypes.c_uint32),
        ("counters", ctypes.c_uint32 * COUNT_TOTAL),
        ("quietSince", ctypes.c_uint32),
        ("logNext", ctypes.c_uint32),
        ("logCount", ctypes.c_uint32),
        ("log", Record * LOG_DEPTH),
        ("traceQuiet", ctypes.c_uint32),
        ("traceNext", ctypes.c_uint32),
        ("traceCount", ctypes.c_uint32),
        ("trace", Trace * TRACE_DEPTH),
        ("maxFirstBin", ctypes.c_uint32),
        ("maxBins", ctypes.c_uint32),
        ("maxStat", ctypes.c_float * MAX_BINS),
        ("maxFrame", ctypes.c_uint32 * MAX_BINS),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("no C compiler for the firmware detector")
    out = tmp_path_factory.mktemp("l3_trigger") / "l3_trigger.so"
    subprocess.run(
        [
            compiler,
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-shared",
            "-fPIC",
            "-O1",
            "-o",
            str(out),
            str(SOURCE),
            "-lm",
        ],
        check=True,
        cwd=FIRMWARE_DIR,
    )
    library = ctypes.CDLL(str(out))
    library.l3_trig_cfg_defaults.argtypes = [ctypes.POINTER(Cfg)]
    library.l3_trig_cfg_check.argtypes = [ctypes.POINTER(Cfg)]
    library.l3_trig_cfg_check.restype = ctypes.c_int32
    library.l3_trig_init.argtypes = [ctypes.POINTER(Trig), ctypes.POINTER(Cfg), ctypes.c_float]
    library.l3_trig_rearm.argtypes = [ctypes.POINTER(Trig)]
    library.l3_trig_region.argtypes = [
        ctypes.POINTER(Cfg),
        ctypes.c_uint32,  # tee (global)
        ctypes.c_uint32,  # window start (global)
        ctypes.c_uint32,  # bins in the window
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.POINTER(ctypes.c_uint32),
    ]
    library.l3_trig_region.restype = ctypes.c_int32
    library.l3_trig_update.argtypes = [
        ctypes.POINTER(Trig),
        ctypes.c_uint32,  # frame
        ctypes.c_uint32,  # tee (global)
        ctypes.c_uint32,  # global bin of obs[0]
        ctypes.POINTER(Obs),
        ctypes.c_uint32,
    ]
    library.l3_trig_update.restype = ctypes.c_int32
    library.l3_trig_log_count.argtypes = [ctypes.POINTER(Trig)]
    library.l3_trig_log_count.restype = ctypes.c_uint32
    library.l3_trig_log_get.argtypes = [
        ctypes.POINTER(Trig),
        ctypes.c_uint32,
        ctypes.POINTER(Record),
    ]
    library.l3_trig_log_get.restype = ctypes.c_int32
    for name in ("l3_trig_format_summary", "l3_trig_format_config"):
        getattr(library, name).argtypes = [ctypes.POINTER(Trig), ctypes.c_char_p, ctypes.c_uint32]
        getattr(library, name).restype = ctypes.c_int32
    library.l3_trig_format_record.argtypes = [
        ctypes.POINTER(Record),
        ctypes.c_char_p,
        ctypes.c_uint32,
    ]
    library.l3_trig_format_record.restype = ctypes.c_int32
    library.l3_trig_why_name.argtypes = [ctypes.c_uint8]
    library.l3_trig_why_name.restype = ctypes.c_char_p
    library.l3_trig_trace_clear.argtypes = [ctypes.POINTER(Trig)]
    library.l3_trig_trace_enable.argtypes = [ctypes.POINTER(Trig), ctypes.c_uint8]
    library.l3_trig_trace_enable.restype = None
    library.l3_trig_trace_count.argtypes = [ctypes.POINTER(Trig)]
    library.l3_trig_trace_count.restype = ctypes.c_uint32
    library.l3_trig_trace_get.argtypes = [
        ctypes.POINTER(Trig),
        ctypes.c_uint32,
        ctypes.POINTER(Trace),
    ]
    library.l3_trig_trace_get.restype = ctypes.c_int32
    library.l3_trig_format_trace_header.argtypes = [
        ctypes.POINTER(Trig),
        ctypes.c_char_p,
        ctypes.c_uint32,
    ]
    library.l3_trig_format_trace_header.restype = ctypes.c_int32
    library.l3_trig_format_trace.argtypes = [
        ctypes.POINTER(Trace),
        ctypes.c_char_p,
        ctypes.c_uint32,
    ]
    library.l3_trig_format_trace.restype = ctypes.c_int32
    library.l3_trig_format_maxhold.argtypes = [
        ctypes.POINTER(Trig),
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_char_p,
        ctypes.c_uint32,
    ]
    library.l3_trig_format_maxhold.restype = ctypes.c_int32
    return library


def make_cfg(lib, **overrides) -> Cfg:
    cfg = Cfg()
    lib.l3_trig_cfg_defaults(ctypes.byref(cfg))
    values = {
        "teeBin": TEE,
        "snr": 6.0,
        "trackFrames": 2,
        "approachBins": APPROACH,
        "gateBins": GATE,
    }
    values.update(overrides)
    for name, value in values.items():
        setattr(cfg, name, value)
    return cfg


class Detector:
    """One detector instance plus a frame builder for its watch region.

    Bins are global. The fixture's window starts at global bin 0 unless
    ``window_start`` says otherwise, so the local offsets equal the global
    bins and the scenarios read plainly; the windowed tests set a start.
    """

    def __init__(
        self,
        lib,
        cfg: Cfg,
        bin_count: int = 53,
        loop_period_s: float = LOOP_PERIOD_S,
        window_start: int = 0,
        tee: int | None = None,
        trace_enabled: bool = True,
    ):
        self.lib = lib
        self.cfg = cfg
        self.tee = cfg.teeBin if tee is None else tee
        self.window_start = window_start
        self.trig = Trig()
        lib.l3_trig_init(ctypes.byref(self.trig), ctypes.byref(cfg), loop_period_s)
        if trace_enabled:
            lib.l3_trig_trace_enable(ctypes.byref(self.trig), 1)
        first = ctypes.c_uint32()
        count = ctypes.c_uint32()
        assert (
            lib.l3_trig_region(
                ctypes.byref(cfg),
                self.tee,
                window_start,
                bin_count,
                ctypes.byref(first),
                ctypes.byref(count),
            )
            == 1
        )
        self.first_local = first.value
        self.first = window_start + first.value  # global bin of obs[0]
        self.count = count.value
        self.frame = 0

    def feed(
        self,
        targets: dict[int, float] | None = None,
        *,
        noise: float = NOISE,
        coherence: float = 0.9,
        velocity_mps: float = 0.0,
        peak_fraction: float = PEAK_FRACTION,
        target_peak_fraction: float | None = None,
        loop0_fraction: float = LOOP0_FRACTION,
    ) -> bool:
        """One frame: noise everywhere, plus targets {local_bin: energy}.

        Noise varies by a few percent per bin so the median is exercised.
        Each bin's strongest-loop power is peak_fraction of its energy
        (target_peak_fraction for targets), so with the default fixture the
        peak and energy statistics see the same ratios. The lag-1
        autocorrelation of each target carries the coherence and the Doppler
        phase for velocity_mps at the fixture loop period.
        """
        if target_peak_fraction is None:
            target_peak_fraction = peak_fraction
        obs = (Obs * self.count)()
        for index in range(self.count):
            obs[index].energy = noise * (1.0 + 0.04 * ((index * 7 + self.frame) % 5 - 2))
            obs[index].peak = obs[index].energy * peak_fraction
            obs[index].loop0 = obs[index].energy * loop0_fraction
            obs[index].r1Re = 0.0
            obs[index].r1Im = 0.0
        phase = 4.0 * math.pi * velocity_mps * LOOP_PERIOD_S / WAVELENGTH_M
        for local_bin, energy in (targets or {}).items():
            index = local_bin - self.first
            assert 0 <= index < self.count, f"bin {local_bin} outside the region"
            obs[index].energy = energy
            obs[index].peak = energy * target_peak_fraction
            obs[index].loop0 = energy * loop0_fraction
            obs[index].r1Re = coherence * energy * math.cos(phase)
            obs[index].r1Im = coherence * energy * math.sin(phase)
        self.frame += 1
        return bool(
            self.lib.l3_trig_update(
                ctypes.byref(self.trig), self.frame, self.tee, self.first, obs, self.count
            )
        )

    def traces(self) -> list[Trace]:
        out = []
        for index in range(self.lib.l3_trig_trace_count(ctypes.byref(self.trig))):
            entry = Trace()
            assert (
                self.lib.l3_trig_trace_get(ctypes.byref(self.trig), index, ctypes.byref(entry)) == 1
            )
            out.append(entry)
        return out

    def trace_header(self) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_trace_header(ctypes.byref(self.trig), buf, len(buf))
        return buf.value.decode()

    def trace_line(self, entry: Trace) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_trace(ctypes.byref(entry), buf, len(buf))
        return buf.value.decode()

    def maxhold_line(self, start: int, count: int) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_maxhold(ctypes.byref(self.trig), start, count, buf, len(buf))
        return buf.value.decode()

    def records(self) -> list[Record]:
        out = []
        for index in range(self.lib.l3_trig_log_count(ctypes.byref(self.trig))):
            record = Record()
            assert (
                self.lib.l3_trig_log_get(ctypes.byref(self.trig), index, ctypes.byref(record)) == 1
            )
            out.append(record)
        return out

    def whys(self) -> list[str]:
        return [self.lib.l3_trig_why_name(record.why).decode() for record in self.records()]

    def counter(self, name: str) -> int:
        return self.trig.counters[COUNT[name]]

    def summary(self) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_summary(ctypes.byref(self.trig), buf, len(buf))
        return buf.value.decode()

    def config_line(self) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_config(ctypes.byref(self.trig), buf, len(buf))
        return buf.value.decode()

    def record_line(self, record: Record) -> str:
        buf = ctypes.create_string_buffer(256)
        self.lib.l3_trig_format_record(ctypes.byref(record), buf, len(buf))
        return buf.value.decode()


CLUB = 60.0 * NOISE  # comfortably above floor * snr


def detector(lib, **overrides) -> Detector:
    return Detector(lib, make_cfg(lib, **overrides))


# --- configuration -----------------------------------------------------------


def test_defaults_are_the_documented_ones_and_pass_the_check(lib):
    cfg = make_cfg(lib)
    assert (cfg.approachBins, cfg.gateBins) == (12, 3)
    assert cfg.minCoherence == 0.0
    assert cfg.minStepBins == pytest.approx(1.0)
    assert cfg.stat == STAT_PEAK
    assert lib.l3_trig_cfg_check(ctypes.byref(cfg)) == 0


@pytest.mark.parametrize(
    "overrides",
    [
        {"snr": 0.5},
        {"snr": float("nan")},
        {"trackFrames": 0},
        {"approachBins": 0},
        {"approachBins": MAX_BINS + 1},
        {"gateBins": APPROACH},  # gate must sit inside the watched approach
        {"minCoherence": 1.5},
        {"minCoherence": -0.1},
        {"minStepBins": -1.0},
        {"stat": 2},
        {"minSpeedMps": -1.0},
        {"teeBin": 253, "gateBins": 3},  # record stores bins in a byte, 0xFF = none
    ],
)
def test_config_the_detector_cannot_run_is_rejected(lib, overrides):
    cfg = make_cfg(lib, **overrides)
    assert lib.l3_trig_cfg_check(ctypes.byref(cfg)) != 0


@pytest.mark.parametrize(
    ("tee", "bin_count", "expected"),
    [
        (TEE, 53, (TEE - APPROACH, APPROACH + GATE + 1)),  # whole region fits
        (5, 53, (0, 5 + GATE + 1)),  # approach clipped at bin 0
        (50, 53, (38, 53 - 38)),  # gate clipped at the window end
        (52, 53, (40, 13)),  # tee on the last bin
    ],
)
def test_region_clips_to_the_capture_window(lib, tee, bin_count, expected):
    cfg = make_cfg(lib, teeBin=tee)
    first = ctypes.c_uint32()
    count = ctypes.c_uint32()
    assert (
        lib.l3_trig_region(
            ctypes.byref(cfg), tee, 0, bin_count, ctypes.byref(first), ctypes.byref(count)
        )
        == 1
    )
    assert (first.value, count.value) == expected


@pytest.mark.parametrize("bin_count", [0, TEE, TEE - 3])
def test_region_is_empty_when_the_tee_is_outside_the_window(lib, bin_count):
    cfg = make_cfg(lib)
    first = ctypes.c_uint32()
    count = ctypes.c_uint32()
    assert (
        lib.l3_trig_region(
            ctypes.byref(cfg), TEE, 0, bin_count, ctypes.byref(first), ctypes.byref(count)
        )
        == 0
    )
    assert count.value == 0


def test_region_converts_a_global_tee_into_the_windows_offsets(lib):
    """Wide profile: window 20..72, tee 1.575 m = global bin 34 -> local 14, region local 2..17."""
    cfg = make_cfg(lib, teeBin=34)
    first = ctypes.c_uint32()
    count = ctypes.c_uint32()
    assert (
        lib.l3_trig_region(ctypes.byref(cfg), 34, 20, 53, ctypes.byref(first), ctypes.byref(count))
        == 1
    )
    assert (first.value, count.value) == (2, 16)
    # The same tee against the late window (47..99) is not visible.
    assert (
        lib.l3_trig_region(ctypes.byref(cfg), 34, 47, 53, ctypes.byref(first), ctypes.byref(count))
        == 0
    )
    # A ball found at global bin 48 (2.25 m) in the pre window: local 28, region 16..31.
    assert (
        lib.l3_trig_region(ctypes.byref(cfg), 48, 20, 53, ctypes.byref(first), ctypes.byref(count))
        == 1
    )
    assert (first.value, count.value) == (16, 16)


def test_records_and_trace_report_global_bins_in_a_windowed_frame(lib):
    det = Detector(lib, make_cfg(lib, teeBin=34), window_start=20)
    assert (det.first_local, det.first) == (2, 22)
    assert det.feed({27: CLUB}) is False  # global bins throughout
    assert det.feed({29: CLUB}) is False
    assert det.feed({31: CLUB}) is True, "31 is inside the gate 31..37 around tee 34"
    assert [record.bin for record in det.records()] == [27, 29, 31]
    assert det.traces()[-1].bin == 31
    assert det.trig.maxFirstBin == 22


def test_destination_can_differ_from_the_configured_tee(lib):
    """Following the ball detector: the gate moves to where the ball actually is."""
    det = Detector(lib, make_cfg(lib, teeBin=34), window_start=20, tee=48)
    assert det.first == 36
    for local_bin in [40, 43, 46]:  # the gate is 45..51 around the ball, not around 34
        fired = det.feed({local_bin: CLUB})
    assert fired is True
    records = det.records()
    assert [record.dest for record in records] == [48, 48, 48]
    assert [record.dest - record.bin for record in records] == [8, 5, 2], "bins short of impact"
    assert det.traces()[-1].dest == 48


# --- swings that must fire ---------------------------------------------------


def test_full_swing_fires_when_the_track_enters_the_gate(lib):
    """A driver's clubhead closes ~2.5 bins per frame; nothing is required after the gate."""
    det = detector(lib)
    path = [9, 11, 14, 16, 19]
    fired_at = None
    for local_bin in path:
        if det.feed({local_bin: CLUB}):
            fired_at = local_bin
            break
    assert fired_at == 19, "first bin inside the gate (>= 17) fires"
    assert det.trig.state == STATE_FIRED
    assert det.whys() == ["acquired", "advanced", "advanced", "advanced", "fired"]
    assert det.counter("fired") == 1
    last = det.records()[-1]
    assert (last.bin, last.age) == (19, 5)


def test_a_fired_detector_ignores_further_frames_until_rearmed(lib):
    det = detector(lib)
    for local_bin in [12, 15, 18]:
        det.feed({local_bin: CLUB})
    assert det.trig.state == STATE_FIRED
    frames_before = det.counter("frames")
    assert det.feed({21: CLUB}) is False
    assert det.counter("frames") == frames_before
    lib.l3_trig_rearm(ctypes.byref(det.trig))
    assert det.trig.state == STATE_IDLE
    assert det.counter("fired") == 1, "rearm keeps the counters"
    assert len(det.records()) == 3, "and the log"
    assert det.feed({10: CLUB}) is False
    assert det.feed({13: CLUB}) is False
    assert det.feed({17: CLUB}) is True
    assert det.counter("fired") == 2


def test_fast_club_seen_twice_fires_on_the_second_frame(lib):
    """Two frames of consistent approach are enough by default; no long history needed."""
    det = detector(lib)
    assert det.feed({12: CLUB}) is False
    assert det.feed({18: CLUB}) is True
    assert det.whys() == ["acquired", "fired"]


def test_club_first_seen_inside_the_gate_waits_one_frame_then_fires(lib):
    """Too young on first sight, not rejected for good."""
    det = detector(lib)
    assert det.feed({18: CLUB}) is False
    assert det.whys() == ["young"]
    assert det.feed({21: CLUB}) is True


@pytest.mark.parametrize("bins", [[18] * 10, [21, 20, 19, 18, 17]])
def test_stationary_or_receding_gate_return_cannot_bypass_approach_rate(lib, bins):
    det = detector(lib)
    for bin_index in bins:
        assert det.feed({bin_index: CLUB}) is False
    assert det.counter("slow") > 0


def test_repeated_gate_bin_then_one_bin_drift_does_not_fire(lib):
    det = detector(lib)
    for _ in range(10):
        assert det.feed({18: CLUB}) is False
    assert det.feed({19: CLUB}) is False
    assert det.counter("slow") > 0


def test_track_frames_of_one_fires_on_first_sight_in_the_gate(lib):
    det = detector(lib, trackFrames=1)
    assert det.feed({18: CLUB}) is True
    assert det.whys() == ["fired"]


def test_one_missing_frame_does_not_break_the_track(lib):
    """A weak frame mid-downswing is bridged rather than restarting the track."""
    det = detector(lib)
    for local_bin in [10, 12]:
        assert det.feed({local_bin: CLUB}) is False
    assert det.feed() is False  # club invisible this frame
    assert det.trig.state == STATE_TRACKING
    assert det.feed({15: CLUB}) is False
    assert det.feed({18: CLUB}) is True
    assert det.whys() == ["acquired", "advanced", "missed", "advanced", "fired"]
    assert det.records()[-1].age == 4, "misses do not count as observations"


def test_fast_backswing_restarts_the_track_and_the_downswing_still_fires(lib):
    """Retreating faster than the jitter window restarts the track; no order is required."""
    det = detector(lib)
    for local_bin in [16, 13, 10]:  # away from the tee, 3 bins per frame
        assert det.feed({local_bin: CLUB}) is False
    assert det.whys() == ["acquired", "jumped", "jumped"]
    assert det.feed({13: CLUB}) is False
    assert det.feed({17: CLUB}) is True
    assert det.whys()[-2:] == ["advanced", "fired"]


def test_slow_backswing_within_the_jitter_window_does_not_poison_the_approach_rate(lib):
    """A 2-bin retreat continues the track; the rate is measured from the turnaround."""
    det = detector(lib)
    for local_bin in [16, 14, 12]:
        assert det.feed({local_bin: CLUB}) is False
    assert det.whys() == ["acquired", "advanced", "advanced"]
    assert det.trig.trackStartBin == 12, "the nearest point is the reference"
    assert det.feed({14: CLUB}) is False
    assert det.feed({17: CLUB}) is True, "5 bins over 2 frames from the turnaround"


def test_scatterer_wander_toward_the_radar_keeps_the_track(lib):
    det = detector(lib)
    for local_bin in [10, 13, 12, 14, 12]:  # -1 and -2 bins are jitter, not a retreat
        assert det.feed({local_bin: CLUB}) is False
    assert det.whys() == ["acquired", "advanced", "advanced", "advanced", "advanced"]


def test_a_stronger_return_elsewhere_does_not_steal_the_track(lib):
    """Clubhead, shaft, hands and reflections swap as the strongest bin; follow the track."""
    det = detector(lib)
    assert det.feed({10: CLUB}) is False
    assert det.feed({12: CLUB}) is False
    # The hands, three times stronger, light up behind the club at bin 9;
    # outside the track's window (10..20), so the club at 15 keeps the track.
    assert det.feed({15: CLUB, 9: 3 * CLUB}) is False
    assert det.whys()[-1] == "advanced"
    assert det.records()[-1].bin == 15
    assert det.feed({18: CLUB, 9: 3 * CLUB}) is True
    assert det.records()[-1].bin == 18


def test_inside_the_window_the_strongest_return_leads(lib):
    """Two returns both plausible as the club: the stronger one is the club."""
    det = detector(lib)
    det.feed({10: CLUB})
    det.feed({12: CLUB})
    det.feed({15: CLUB, 16: 3 * CLUB})  # both within 10..20 of the track at 12
    assert det.whys()[-1] == "advanced"
    assert det.records()[-1].bin == 16


def test_without_a_continuation_the_strongest_return_starts_a_new_track(lib):
    det = detector(lib)
    det.feed({10: CLUB})
    det.feed({12: CLUB})
    assert det.feed({22: 3 * CLUB}) is False  # nothing near 12, so this is a jump
    assert det.whys()[-1] == "young", "jumped into the gate at age 1"
    assert det.trig.trackStartBin == 22


def test_the_ball_bin_needs_no_motion_for_the_trigger_to_arm(lib):
    """MTI suppresses the stationary ball; the tee bin sits at the noise floor throughout."""
    det = detector(lib)
    tee_index = TEE - det.first
    for local_bin in [11, 14, 17]:
        fired = det.feed({local_bin: CLUB})
    assert fired is True
    # The frame builder never raised the tee bin above noise.
    assert all(record.bin != TEE for record in det.records())
    assert tee_index < det.count


# --- non-swings that must not fire -------------------------------------------


def test_noise_alone_never_fires_and_logs_nothing(lib):
    det = detector(lib)
    for _ in range(200):
        assert det.feed() is False
    assert det.trig.state == STATE_IDLE
    assert det.counter("frames") == 200
    assert det.counter("cand") == 0
    assert det.records() == []
    assert det.trig.floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)


def test_a_person_walking_up_to_the_ball_is_too_slow(lib):
    """~1 bin per 5 frames (~3 m/s radial) reaches the gate but is not a clubhead."""
    det = detector(lib)
    local_bin = 10
    fired = False
    for frame in range(60):
        fired |= det.feed({local_bin: CLUB})
        if frame % 5 == 4:
            local_bin += 1
    assert fired is False
    assert det.counter("slow") > 0
    assert det.counter("fired") == 0
    assert det.trig.state == STATE_TRACKING


def test_min_step_of_zero_lets_a_slow_target_fire(lib):
    """The speed test is a tunable, so a putt-speed target can be admitted deliberately."""
    det = detector(lib, minStepBins=0.0)
    local_bin = 14
    fired = False
    for frame in range(40):
        fired |= det.feed({local_bin: CLUB})
        if fired:
            break
        if frame % 5 == 4:
            local_bin += 1
    assert fired is True


def test_a_return_that_jumps_more_than_eight_bins_restarts_the_track(lib):
    """Hands, shaft and clubhead swapping as the strongest bin must not be one target."""
    det = detector(lib)
    assert det.feed({9: CLUB}) is False
    assert det.feed({19: CLUB}) is False, "10-bin jump lands in the gate but with age 1"
    assert det.whys() == ["acquired", "young"]
    assert det.trig.trackStartBin == 19


def test_two_missing_frames_drop_the_track(lib):
    det = detector(lib)
    det.feed({10: CLUB})
    det.feed()
    assert det.trig.state == STATE_TRACKING
    det.feed()
    assert det.trig.state == STATE_IDLE
    assert det.whys() == ["acquired", "missed", "lost"]
    assert det.counter("lost") == 1


def test_candidates_below_snr_times_floor_are_not_candidates(lib):
    det = detector(lib, snr=6.0)
    for local_bin in [11, 14, 17]:
        assert det.feed({local_bin: 5.5 * NOISE}) is False
    assert det.counter("cand") == 0
    for local_bin in [11, 14, 17]:
        fired = det.feed({local_bin: 6.5 * NOISE})
    assert fired is True


def test_peak_statistic_sees_a_club_present_for_only_part_of_the_frame(lib):
    """Two strong loops out of twelve: modest energy, large strongest-loop power."""
    # Energy 3x noise energy; its peak is a whole loop's worth, 12x the noise peak.
    part_frame = {"targets": {12: 3.0 * NOISE}, "target_peak_fraction": 1.0}
    peak = detector(lib, stat=STAT_PEAK)
    peak.feed(part_frame["targets"], target_peak_fraction=part_frame["target_peak_fraction"])
    assert peak.counter("cand") == 1
    energy = detector(lib, stat=STAT_ENERGY)
    energy.feed(part_frame["targets"], target_peak_fraction=part_frame["target_peak_fraction"])
    assert energy.counter("cand") == 0


def test_energy_statistic_is_selectable_and_floors_in_its_own_units(lib):
    det = detector(lib, stat=STAT_ENERGY)
    for _ in range(20):
        det.feed()
    assert det.trig.floor == pytest.approx(NOISE, rel=0.05)
    peak = detector(lib, stat=STAT_PEAK)
    for _ in range(20):
        peak.feed()
    assert peak.trig.floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)
    assert "stat=energy" in det.config_line()
    assert "stat=peak" in peak.config_line()


# --- adaptive floor ----------------------------------------------------------


def test_floor_follows_the_room_and_the_threshold_with_it(lib):
    """Fixed absolute levels break with enclosure, mounting and gain; the floor adapts."""
    det = detector(lib, snr=4.0)
    for _ in range(20):
        det.feed()
    assert det.trig.floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)
    assert det.feed({12: 5.0 * NOISE}) is False
    assert det.counter("cand") == 1, "5x the quiet floor is a candidate"
    for _ in range(60):
        det.feed(noise=10.0 * NOISE)
    assert det.trig.floor == pytest.approx(10.0 * NOISE * PEAK_FRACTION, rel=0.05)
    # The step itself reads as a candidate for the few frames the floor
    # takes to catch up (a 10x jump clears 4x the old floor); the range
    # track, not the threshold, is what keeps that from firing.
    settled = det.counter("cand")
    assert settled <= 8
    assert det.counter("fired") == 0
    det.feed({12: 5.0 * NOISE}, noise=10.0 * NOISE)
    assert det.counter("cand") == settled, "the same energy is under the raised floor"


def test_floor_ignores_the_club_occupying_a_few_bins(lib):
    det = detector(lib)
    for _ in range(20):
        det.feed()
    for local_bin in [9, 10, 11, 12]:
        det.feed({local_bin: CLUB, local_bin + 1: CLUB, local_bin + 2: CLUB})
    assert det.trig.floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)


def test_floor_never_drops_to_zero(lib):
    det = detector(lib)
    for _ in range(50):
        det.feed(noise=0.0)
    assert det.trig.floor == pytest.approx(1.0)


# --- Doppler telemetry -------------------------------------------------------


def test_coherence_gate_rejects_an_incoherent_strong_bin_and_logs_why(lib):
    det = detector(lib, minCoherence=0.5)
    assert det.feed({12: CLUB}, coherence=0.1) is False
    assert det.trig.state == STATE_IDLE
    assert det.whys() == ["lowcoh"]
    assert det.records()[0].bin == 12, "the rejected bin is still logged for tuning"
    assert det.records()[0].coherencePct == 10
    assert det.feed({12: CLUB}, coherence=0.8) is False
    assert det.whys()[-1] == "acquired"


def test_coherence_rejection_outranks_the_miss_it_causes(lib):
    det = detector(lib, minCoherence=0.5)
    det.feed({12: CLUB})
    det.feed({14: CLUB}, coherence=0.2)
    det.feed({16: CLUB}, coherence=0.2)
    assert det.whys() == ["acquired", "lowcoh", "lowcoh"]
    assert [record.state for record in det.records()] == [
        STATE_TRACKING,
        STATE_TRACKING,
        STATE_IDLE,
    ]


def test_coherence_gate_is_off_by_default(lib):
    det = detector(lib)
    assert det.feed({12: CLUB}, coherence=0.0) is False
    assert det.whys() == ["acquired"]


@pytest.mark.parametrize("velocity", [0.0, 2.5, -4.0, 8.0])
def test_unaliased_velocity_is_read_back_in_cm_per_s(lib, velocity):
    det = detector(lib)
    det.feed({12: CLUB}, velocity_mps=velocity)
    assert det.records()[0].velocityCms == pytest.approx(velocity * 100.0, abs=3)


def test_velocity_beyond_the_unambiguous_span_aliases(lib):
    """+/- lambda / (4 T) is ~9 m/s here: a 12 m/s club reads as 12 - 17.9 m/s.

    That is why the trigger gates on range rate across frames, not on the
    Doppler sign, and why velocity is a readout rather than a condition.
    """
    span = WAVELENGTH_M / (2.0 * LOOP_PERIOD_S)
    det = detector(lib)
    det.feed({12: CLUB}, velocity_mps=12.0)
    assert det.records()[0].velocityCms == pytest.approx((12.0 - span) * 100.0, abs=3)


def test_doppler_speed_gate_is_off_by_default(lib):
    det = detector(lib)
    assert det.feed({12: CLUB}, velocity_mps=0.0) is False
    assert det.whys() == ["acquired"]


def test_doppler_speed_gate_rejects_a_body_and_keeps_a_club(lib):
    """A person sways at well under 1 m/s; a clubhead aliases to |v| spread over 0..9 m/s."""
    det = detector(lib, minSpeedMps=1.5)
    assert det.feed({12: CLUB}, velocity_mps=0.4) is False
    assert det.whys() == ["slowdop"]
    assert det.trig.state == STATE_IDLE
    assert det.records()[0].velocityCms == pytest.approx(40, abs=3)
    assert det.counter("slowdop") == 1
    assert det.feed({12: CLUB}, velocity_mps=-3.0) is False
    assert det.whys()[-1] == "acquired"
    assert det.feed({15: CLUB}, velocity_mps=12.0) is False, "aliases to about -5.9 m/s: fast"
    assert det.whys()[-1] == "advanced"


def test_doppler_speed_gate_counts_as_a_miss_for_a_live_track(lib):
    det = detector(lib, minSpeedMps=1.5)
    det.feed({10: CLUB}, velocity_mps=4.0)
    det.feed({12: CLUB}, velocity_mps=4.0)
    assert det.feed({15: CLUB}, velocity_mps=0.2) is False
    assert det.whys()[-1] == "slowdop"
    assert det.trig.state == STATE_TRACKING, "one slow frame is bridged like a miss"
    assert det.feed({18: CLUB}, velocity_mps=4.0) is True


def test_doppler_speed_gate_needs_a_loop_period(lib):
    det = Detector(lib, make_cfg(lib, minSpeedMps=1.5), loop_period_s=0.0)
    assert det.feed({12: CLUB}, velocity_mps=0.0) is False
    assert det.whys() == ["acquired"], "no timing, no Doppler: the gate does not apply"


def test_no_loop_period_means_no_velocity(lib):
    det = Detector(lib, make_cfg(lib), loop_period_s=0.0)
    det.feed({12: CLUB}, velocity_mps=5.0)
    assert det.records()[0].velocityCms == 0


# --- flight recorder ---------------------------------------------------------


def test_records_carry_the_quiet_gap_before_them(lib):
    det = detector(lib)
    for _ in range(10):
        det.feed()
    det.feed({12: CLUB})
    for _ in range(3):
        det.feed()  # missed, lost, quiet
    det.feed({9: CLUB})
    gaps = [record.gap for record in det.records()]
    assert gaps == [10, 0, 0, 1]


def test_log_keeps_the_newest_frames_when_full(lib):
    det = detector(lib, minStepBins=0.0, trackFrames=200)
    for _ in range(LOG_DEPTH + 20):
        det.feed({12: CLUB})  # every frame logs, never fires (too young)
    records = det.records()
    assert len(records) == LOG_DEPTH
    assert records[0].frame == 21
    assert records[-1].frame == LOG_DEPTH + 20
    assert "records=128" in det.summary()


def test_record_carries_energy_floor_state_and_age(lib):
    det = detector(lib)
    for _ in range(8):
        det.feed()
    det.feed({12: CLUB})
    det.feed({15: CLUB})
    record = det.records()[-1]
    assert record.energy == pytest.approx(CLUB)
    assert record.peak == pytest.approx(CLUB * PEAK_FRACTION)
    assert record.floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)
    assert (record.state, record.age, record.bin) == (STATE_TRACKING, 2, 15)


# --- raw-input trace ---------------------------------------------------------


def test_trace_records_the_strongest_bin_of_frames_the_log_never_sees(lib):
    """A club at 3x the floor is no candidate at snr 6, but the trace shows it was offered."""
    det = detector(lib, snr=6.0)
    for _ in range(10):
        det.feed()
    for local_bin in [10, 12, 15]:
        assert det.feed({local_bin: 3.0 * NOISE}) is False
    assert det.records() == [], "below snr: nothing in the log"
    traces = det.traces()
    assert [entry.bin for entry in traces] == [10, 12, 15]
    assert traces[0].gap == 10
    assert traces[0].energy == pytest.approx(3.0 * NOISE)
    assert traces[0].peak == pytest.approx(3.0 * NOISE * PEAK_FRACTION)
    assert traces[0].loop0 == pytest.approx(3.0 * NOISE * LOOP0_FRACTION)
    assert traces[0].floor == pytest.approx(NOISE * PEAK_FRACTION, rel=0.05)
    assert traces[0].threshold == pytest.approx(traces[0].floor * 6.0), "floor x snr in force"
    assert traces[0].coherencePct == 90


def test_trace_bar_is_twice_the_floor_so_noise_stays_out(lib):
    det = detector(lib)
    for _ in range(50):
        det.feed()
    assert det.traces() == []
    det.feed({12: 1.9 * NOISE})
    assert det.traces() == []
    det.feed({12: 2.2 * NOISE})
    assert [entry.bin for entry in det.traces()] == [12]


def test_trace_follows_the_regions_strongest_bin_not_the_track(lib):
    det = detector(lib)
    det.feed({10: CLUB})
    det.feed({12: CLUB})
    det.feed({15: CLUB, 9: 3 * CLUB})  # the track keeps 15; the trace reports 9
    assert det.records()[-1].bin == 15
    assert det.traces()[-1].bin == 9


def test_max_hold_keeps_the_largest_statistic_per_bin_with_its_frame(lib):
    det = detector(lib)
    for _ in range(5):
        det.feed()
    det.feed({12: 4.0 * NOISE})  # frame 6
    det.feed({12: 3.0 * NOISE, 13: 2.6 * NOISE})  # frame 7
    trig = det.trig
    assert trig.maxFirstBin == det.first
    assert trig.maxBins == det.count
    index = 12 - det.first
    assert trig.maxStat[index] == pytest.approx(4.0 * NOISE * PEAK_FRACTION)
    assert trig.maxFrame[index] == 6
    assert trig.maxStat[index + 1] == pytest.approx(2.6 * NOISE * PEAK_FRACTION)
    assert trig.maxFrame[index + 1] == 7
    line = det.maxhold_line(index, 2)
    assert (
        line
        == f"trigmax 12:{4.0 * NOISE * PEAK_FRACTION:.0f}@6 13:{2.6 * NOISE * PEAK_FRACTION:.0f}@7"
    )


def test_trace_is_opt_in_and_can_be_disabled_without_changing_detection(lib):
    det = Detector(lib, make_cfg(lib), trace_enabled=False)
    assert det.feed({12: CLUB}) is False
    assert det.traces() == []
    assert det.trig.maxBins == 0

    lib.l3_trig_trace_enable(ctypes.byref(det.trig), 1)
    assert det.feed({15: CLUB}) is False
    assert [entry.bin for entry in det.traces()] == [15]

    lib.l3_trig_trace_enable(ctypes.byref(det.trig), 0)
    assert det.feed({18: CLUB}) is True
    assert [entry.bin for entry in det.traces()] == [15]


def test_trace_clear_empties_trace_and_maxima_but_keeps_the_log_and_arm(lib):
    det = detector(lib)
    det.feed({12: CLUB})
    assert det.traces() and det.records()
    lib.l3_trig_trace_clear(ctypes.byref(det.trig))
    assert det.traces() == []
    assert det.trig.maxBins == 0
    assert len(det.records()) == 1
    assert det.trig.state == STATE_TRACKING
    det.feed({15: CLUB})
    assert [entry.bin for entry in det.traces()] == [15]


def test_trace_keeps_the_newest_frames_when_full(lib):
    det = detector(lib, minStepBins=0.0, trackFrames=200)
    for _ in range(TRACE_DEPTH + 10):
        det.feed({12: CLUB})
    traces = det.traces()
    assert len(traces) == TRACE_DEPTH
    assert traces[0].frame == 11
    assert traces[-1].frame == TRACE_DEPTH + 10


def test_trace_header_and_lines_read_without_float_printf(lib):
    det = detector(lib)
    for _ in range(3):
        det.feed()
    det.feed({12: 4.0 * NOISE})
    header = det.trace_header()
    assert header.startswith("trigtrace state=idle stat=peak floor=")
    assert f"bar=2.0x frames=4 region={det.first}+{det.count} entries=1" in header
    line = det.trace_line(det.traces()[0])
    assert line.startswith(
        "t frame=4 gap=3 bin=12 dest=20 dist=8 state=idle energy=400 peak=100 loop0=33 floor="
    )
    # floor ~25 (peak units): threshold ~150, energy/floor ~16, peak/floor ~4.
    assert " thr=" in line and " e/f=16." in line and " p/f=4.0 coh=90" in line


# --- text output (integer-only printf) --------------------------------------


def test_summary_line_reports_state_floor_and_counters(lib):
    det = detector(lib)
    for local_bin in [11, 14, 17]:
        det.feed({local_bin: CLUB})
    summary = det.summary()
    assert summary.startswith("trig state=fired floor=")
    for token in ("frames=3", "cand=3", "acq=1", "adv=1", "fired=1", "records=3"):
        assert token in summary
    assert len(summary) < 160


def test_config_line_echoes_the_arming_parameters(lib):
    det = detector(lib, snr=6.5, minStepBins=1.25)
    line = det.config_line()
    assert line == (
        "trigcfg tee=20 snr=6.50 track=2 approach=12 gate=3 mincoh=0.00 minstep=1.25 "
        "stat=peak minspeed=0.00 loopus=135.0"
    )


def test_record_line_is_human_readable_without_float_printf(lib):
    det = detector(lib)
    for _ in range(4):
        det.feed()
    det.feed({12: CLUB}, velocity_mps=2.5)
    line = det.record_line(det.records()[0])
    assert line.startswith(
        "frame=5 gap=4 state=tracking why=acquired bin=12 dest=20 dist=8 age=1 "
        "energy=6000 peak=1500 floor="
    )
    assert " v=2.5" in line and line.endswith("coh=90")


def test_record_line_shows_a_dash_when_no_bin_was_seen(lib):
    det = detector(lib)
    det.feed({12: CLUB})
    det.feed()
    line = det.record_line(det.records()[1])
    assert "why=missed bin=- dest=20 dist=- age=1" in line


def test_why_names_cover_every_reason(lib):
    for name, code in WHY.items():
        assert lib.l3_trig_why_name(code).decode() == name
    assert lib.l3_trig_why_name(len(WHY)).decode() == "?"
