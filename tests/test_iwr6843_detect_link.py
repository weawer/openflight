"""The host side of the live DSS detector (``trackCfg detectCore``,
``triggerLog perf``, ``triggerLog timing``): openflight.iwr6843.dsp_link's
parsers and the driver's commands.

Round trips through the firmware's own formatters (l3_detect_core_format,
l3_timing_format_*) built on the host, so the C and the regexes cannot drift
apart; and literal replies for what only the board prints.
"""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dsp_link import (
    DetectCoreStatus,
    DetectMismatch,
    DspLinkError,
    TimingStat,
    parse_detect_core,
    parse_detect_timing,
)

DETECT = (
    "detect core=dss active=mss latched=1 mss=10 dss=240 verify=0 ineligible=2 "
    "failures=3 fallbacks=3 streak=3 latches=1 mismatches=0 "
    "dss_inv_us=4/9 dss_score_us=310/402"
)
TIMING = (
    "timing frames=250 budget_us=3000 over_budget=1 depth_max=2 ring=12 "
    "margin_last_us=28100 margin_min_us=24000 margin_negative=0\n"
    "timing wait n=250 last=40 min=10 mean=35 max=900\n"
    "timing score n=240 last=320 min=290 mean=330 max=480\n"
    "timing service n=250 last=900 min=600 mean=950 max=3100\n"
    "timing latency n=250 last=950 min=620 mean=990 max=4000\n"
    "timing arrival n=249 last=3000 min=2998 mean=3000 max=3004\n"
)
TIMELINE = (
    "timeline slot=3 epoch=41 core=dss wait_us=40 score_us=320 service_us=900 "
    "latency_us=940 depth=0 flags=-\n"
    "timeline slot=12 epoch=post core=fallback wait_us=20 score_us=- service_us=700 "
    "latency_us=720 depth=1 flags=post|stale\n"
)


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


# --- detect core ----------------------------------------------------------------


def test_the_detect_line_is_parsed_field_by_field():
    status = parse_detect_core(DETECT + "\n")
    assert status == DetectCoreStatus(
        requested="dss",
        active="mss",
        latched=True,
        mss=10,
        dss=240,
        verify=0,
        ineligible=2,
        failures=3,
        fallbacks=3,
        streak=3,
        latches=1,
        mismatches=0,
        dss_inv_us_last=4,
        dss_inv_us_max=9,
        dss_score_us_last=310,
        dss_score_us_max=402,
        first_mismatch=None,
    )


def test_the_first_mismatch_is_named_by_field():
    text = DETECT.replace("mismatches=0", "mismatches=2") + " first_mismatch=7:13:3\n"
    assert parse_detect_core(text).first_mismatch == DetectMismatch(slot=7, bin=13, field="r1Re")


def test_an_unknown_mismatch_field_is_kept_as_its_number():
    text = DETECT + " first_mismatch=7:13:9\n"
    assert parse_detect_core(text).first_mismatch.field == "9"


@pytest.mark.parametrize(
    "text",
    [
        "Error: detectCore verify needs an IQ16 ring (captureFormat iq16)\n",
        "Error: detectCore verify needs the DSS link (trackCfg dsp status)\n",
        "Error: trackCfg detectCore dss|verify\n",
    ],
)
def test_a_refusal_is_raised_with_its_reason(text):
    with pytest.raises(DspLinkError, match="needs|dss\\|verify"):
        parse_detect_core(text)


def test_firmware_without_the_detect_core_is_an_error():
    """An older image answers with trackCfg's own usage line."""
    with pytest.raises(DspLinkError):
        parse_detect_core("Error: trackCfg <loopPeriodS> ...\n")
    with pytest.raises(DspLinkError):
        parse_detect_core("Done\n")


def test_the_c_detect_line_round_trips(lib):
    core = fw.DetectCore()
    lib.l3_detect_core_init(ctypes.byref(core))
    lib.l3_detect_core_set(ctypes.byref(core), fw.DETECT_CORE_VERIFY, 1)
    routed = lib.l3_detect_core_route(ctypes.byref(core), 1)
    lib.l3_detect_core_report(ctypes.byref(core), routed, fw.DETECT_OUTCOME_MISMATCH, 600, 60000)
    lib.l3_detect_core_note_mismatch(ctypes.byref(core), 4, 22, 5)
    lib.l3_detect_core_route(ctypes.byref(core), 0)
    text = fw.c_text(lib.l3_detect_core_format, ctypes.byref(core), 600, cap=320)
    status = parse_detect_core(text + "\nDone\n")
    assert (status.requested, status.active, status.verify, status.ineligible) == (
        "verify",
        "verify",
        1,
        1,
    )
    assert (status.mismatches, status.dss_score_us_max) == (1, 100)
    assert status.first_mismatch == DetectMismatch(slot=4, bin=22, field="set")


# --- detect timing ------------------------------------------------------------------


def test_the_timing_is_parsed():
    timing = parse_detect_timing(DETECT + "\n" + TIMING + "Done\n")
    assert (timing.frames, timing.budget_us, timing.over_budget, timing.ring) == (250, 3000, 1, 12)
    assert (timing.margin_last_us, timing.margin_min_us, timing.margin_negative) == (
        28100,
        24000,
        0,
    )
    assert timing.stats["service"] == TimingStat(count=250, last=900, min=600, mean=950, max=3100)
    assert set(timing.stats) == {"wait", "score", "service", "latency", "arrival"}
    assert timing.timeline == ()
    assert timing.keeps_up


def test_the_timeline_is_parsed():
    timing = parse_detect_timing(TIMING + TIMELINE)
    first, second = timing.timeline
    assert (first.slot, first.epoch, first.core, first.score_us, first.flags) == (
        3,
        41,
        "dss",
        320,
        frozenset(),
    )
    assert (second.epoch, second.core, second.score_us, second.flags) == (
        None,
        "fallback",
        None,
        frozenset({"post", "stale"}),
    )


def test_no_frame_yet_is_none():
    assert parse_detect_timing(DETECT + "\ntiming frames=0 (no frame decided yet)\nDone\n") is None


def test_margins_before_any_pre_impact_frame_are_none():
    text = TIMING.replace(
        "margin_last_us=28100 margin_min_us=24000", "margin_last_us=- margin_min_us=-"
    )
    timing = parse_detect_timing(text)
    assert timing.margin_last_us is None and timing.margin_min_us is None


def test_a_negative_margin_does_not_keep_up():
    text = TIMING.replace(
        "margin_min_us=24000 margin_negative=0", "margin_min_us=-150 margin_negative=2"
    )
    timing = parse_detect_timing(text)
    assert timing.margin_min_us == -150 and not timing.keeps_up


def test_mean_service_at_the_budget_does_not_keep_up():
    text = TIMING.replace(
        "timing service n=250 last=900 min=600 mean=950",
        "timing service n=250 last=900 min=600 mean=3000",
    )
    assert not parse_detect_timing(text).keeps_up


def test_no_service_yet_does_not_keep_up():
    text = TIMING.replace("timing service n=250", "timing service n=0")
    assert not parse_detect_timing(text).keeps_up


def test_a_reply_without_the_summary_is_an_error():
    with pytest.raises(DspLinkError):
        parse_detect_timing("perf frames=12 total=400us clock=200\nDone\n")
    with pytest.raises(DspLinkError):
        parse_detect_timing("Error: triggerLog [trace|track]\n")


def test_the_c_timing_lines_round_trip(lib):
    t = fw.Timing()
    lib.l3_timing_init(ctypes.byref(t), 200, 3000, 12)
    events = []
    for k in range(3):
        acquired = k * 3000 * 200
        e = fw.TimingEvent(
            slot=k,
            epoch=k + 1,
            acquired=acquired,
            dequeued=acquired + 40 * 200,
            scoreStart=acquired + 50 * 200,
            scoreEnd=acquired + 350 * 200,
            decided=acquired + 900 * 200,
            core=fw.DETECT_CORE_DSS,
            flags=fw.TIMING_FLAG_SCORED | (fw.TIMING_FLAG_FIRED if k == 2 else 0),
            depth=k,
        )
        lib.l3_timing_record(ctypes.byref(t), ctypes.byref(e))
        events.append(e)
    lines = [fw.c_text(lib.l3_timing_format_summary, ctypes.byref(t), cap=200)]
    lines += [
        fw.c_text(lib.l3_timing_format_stat, ctypes.byref(t), k, cap=200)
        for k in range(len(fw.TIMING_STAT_NAMES))
    ]
    lines += [
        fw.c_text(lib.l3_timing_format_event, ctypes.byref(t), ctypes.byref(e), cap=200)
        for e in events
    ]
    timing = parse_detect_timing("\n".join(lines) + "\nDone\n")
    assert timing.frames == 3 and timing.depth_max == 2
    assert timing.stats["arrival"] == TimingStat(count=2, last=3000, min=3000, mean=3000, max=3000)
    assert timing.stats["score"].mean == 300
    assert [e.latency_us for e in timing.timeline] == [900, 900, 900]
    assert timing.timeline[2].flags == frozenset({"fired"})
    assert timing.keeps_up


# --- the driver ----------------------------------------------------------------------


class _Radar(IWR6843Radar):
    def __init__(self, reply: str):  # pylint: disable=super-init-not-called
        self.sent: list[tuple[str, float]] = []
        self._reply = reply

    def cmd(self, line, timeout=1.0):
        self.sent.append((line, timeout))
        return self._reply


@pytest.mark.parametrize(
    ("core", "line"),
    [
        (None, "trackCfg detectCore"),
        ("dss", "trackCfg detectCore dss"),
        ("verify", "trackCfg detectCore verify"),
    ],
)
def test_the_driver_reads_or_chooses_the_core(core, line):
    radar = _Radar(DETECT + "\nDone\n")
    assert radar.detect_core(core).requested == "dss"
    assert radar.sent == [(line, 2.0)]


@pytest.mark.parametrize("core", ["DSS", "dsp", "", "mss"])
def test_the_driver_refuses_an_unknown_core_before_sending(core):
    """mss is not a choice: the MSS scores only the frames the DSS cannot take."""
    radar = _Radar(DETECT)
    with pytest.raises(ValueError):
        radar.detect_core(core)
    assert radar.sent == []


def test_the_driver_raises_the_boards_refusal():
    radar = _Radar("Error: detectCore verify needs an IQ16 ring (captureFormat iq16)\n")
    with pytest.raises(DspLinkError, match="IQ16"):
        radar.detect_core("verify")


def test_the_driver_reads_the_timing():
    radar = _Radar(DETECT + "\n" + TIMING + TIMELINE + "Done\n")
    timing = radar.detect_timing()
    assert timing.frames == 250 and len(timing.timeline) == 2
    assert radar.sent == [("triggerLog timing", 2.0)]
