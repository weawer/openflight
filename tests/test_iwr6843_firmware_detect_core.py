"""Tests for the detect core, firmware/iwr6843/l3_detect_core.c.

"trackCfg detectCore dss|verify" chooses how the detector's bins are
scored; l3_detect_core decides per frame and keeps the record:

- dss (the default): the DSS scores; a DSS that fails a frame has the MSS
  score it (a fallback), and three failures in a row latch the MSS until
  the core is chosen again
- verify: both score and must agree bit for bit; never latches
- mss is not a choice: the MSS scores only the frames the DSS cannot take
  (ineligible: not an IQ16 ring frame, or the link busy or down), the
  fallbacks, and every frame once latched
- verify is refused for a capture the DSS cannot read; dss is not, its
  frames go to the MSS as ineligible
"""

from __future__ import annotations

import ctypes

import pytest

from openflight.iwr6843 import firmware_host as fw

MSS, DSS, VERIFY = fw.DETECT_CORE_MSS, fw.DETECT_CORE_DSS, fw.DETECT_CORE_VERIFY
OK, FAILED, MISMATCH = fw.DETECT_OUTCOME_OK, fw.DETECT_OUTCOME_FAILED, fw.DETECT_OUTCOME_MISMATCH


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def core(lib, which: int | None = None) -> fw.DetectCore:
    out = fw.DetectCore()
    lib.l3_detect_core_init(ctypes.byref(out))
    if which is not None:
        assert lib.l3_detect_core_set(ctypes.byref(out), which, 1) == 0
    return out


def route(lib, c: fw.DetectCore, eligible: bool = True) -> int:
    return lib.l3_detect_core_route(ctypes.byref(c), int(eligible))


def report(lib, c: fw.DetectCore, routed: int, outcome: int, inv=0, score=0) -> int:
    return lib.l3_detect_core_report(ctypes.byref(c), routed, outcome, inv, score)


def fail_dss_frame(lib, c: fw.DetectCore) -> int:
    routed = route(lib, c)
    assert routed == DSS
    return report(lib, c, routed, FAILED)


def formatted(lib, c: fw.DetectCore, clock_mhz: int = 600) -> str:
    return fw.c_text(lib.l3_detect_core_format, ctypes.byref(c), clock_mhz, cap=320)


# --- choosing a core ---------------------------------------------------------------


def test_it_starts_on_the_dss(lib):
    c = core(lib)
    assert (c.requested, c.active, c.latched, c.failLimit) == (DSS, DSS, 0, 3)
    assert route(lib, c) == DSS
    assert (c.dssFrames, c.mssFrames) == (1, 0)


def test_verify_needs_a_capture_the_dss_can_read(lib):
    c = core(lib)
    assert lib.l3_detect_core_set(ctypes.byref(c), VERIFY, 0) == -1
    assert (c.requested, c.active) == (DSS, DSS), "a refusal changes nothing"


def test_dss_is_allowed_on_any_capture(lib):
    """An IQ8 or compact capture: every frame goes to the MSS as ineligible."""
    c = core(lib, VERIFY)
    assert lib.l3_detect_core_set(ctypes.byref(c), DSS, 0) == 0
    assert (c.requested, c.active) == (DSS, DSS)
    assert route(lib, c, eligible=False) == MSS
    assert (c.ineligible, c.mssFrames, c.dssFrames) == (1, 1, 0)


@pytest.mark.parametrize("eligible", [0, 1])
def test_the_mss_cannot_be_chosen(lib, eligible):
    c = core(lib, VERIFY)
    assert lib.l3_detect_core_set(ctypes.byref(c), MSS, eligible) == -1
    assert (c.requested, c.active) == (VERIFY, VERIFY), "a refusal changes nothing"


def test_the_mss_cannot_be_chosen_to_clear_a_latch(lib):
    """Only choosing dss or verify again clears it."""
    c = core(lib, DSS)
    for _ in range(3):
        fail_dss_frame(lib, c)
    assert lib.l3_detect_core_set(ctypes.byref(c), MSS, 1) == -1
    assert (c.latched, c.active) == (1, MSS)


def test_an_unknown_core_is_refused(lib):
    c = core(lib, VERIFY)
    assert lib.l3_detect_core_set(ctypes.byref(c), 3, 1) == -1
    assert c.active == VERIFY


@pytest.mark.parametrize("which", [DSS, VERIFY])
def test_each_core_routes_its_frames(lib, which):
    c = core(lib, which)
    assert route(lib, c) == which
    assert (c.mssFrames, c.dssFrames, c.verifyFrames) == tuple(
        int(which == k) for k in (MSS, DSS, VERIFY)
    )


# --- frames the DSS cannot take ---------------------------------------------------------


@pytest.mark.parametrize("which", [DSS, VERIFY])
def test_an_ineligible_frame_goes_to_the_mss_counted(lib, which):
    c = core(lib, which)
    assert route(lib, c, eligible=False) == MSS
    assert (c.ineligible, c.mssFrames, c.failures, c.failStreak) == (1, 1, 0, 0)


def test_ineligible_frames_never_latch(lib):
    """Not a DSS failure: a format change, or the link busy with a CLI command."""
    c = core(lib, DSS)
    for _ in range(10):
        route(lib, c, eligible=False)
    assert (c.latched, c.active, c.ineligible) == (0, DSS, 10)


def test_a_latched_frame_is_not_ineligible(lib):
    """Latched, every frame goes to the MSS anyway: not the frame's doing."""
    c = core(lib, DSS)
    for _ in range(3):
        fail_dss_frame(lib, c)
    assert route(lib, c, eligible=False) == MSS
    assert c.ineligible == 0 and c.mssFrames == 1


# --- DSS failures, fallbacks and the latch -------------------------------------------------


def test_a_failed_dss_frame_is_a_fallback(lib):
    c = core(lib, DSS)
    assert fail_dss_frame(lib, c) == 0
    assert (c.failures, c.fallbacks, c.failStreak, c.latched) == (1, 1, 1, 0)


def test_three_failures_in_a_row_latch_the_mss(lib):
    c = core(lib, DSS)
    assert [fail_dss_frame(lib, c) for _ in range(3)] == [0, 0, 1]
    assert (c.latched, c.active, c.requested, c.latches) == (1, MSS, DSS, 1)
    assert route(lib, c) == MSS, "latched: the next frame goes to the MSS"


def test_a_success_resets_the_streak(lib):
    c = core(lib, DSS)
    fail_dss_frame(lib, c)
    fail_dss_frame(lib, c)
    report(lib, c, route(lib, c), OK)
    assert c.failStreak == 0
    fail_dss_frame(lib, c)
    fail_dss_frame(lib, c)
    assert c.latched == 0 and c.failures == 4


def test_an_ineligible_frame_between_failures_does_not_reset_the_streak(lib):
    """Only an answer from the DSS proves it is working again."""
    c = core(lib, DSS)
    fail_dss_frame(lib, c)
    fail_dss_frame(lib, c)
    route(lib, c, eligible=False)
    assert fail_dss_frame(lib, c) == 1


def test_choosing_the_core_again_clears_the_latch(lib):
    c = core(lib, DSS)
    for _ in range(3):
        fail_dss_frame(lib, c)
    assert lib.l3_detect_core_set(ctypes.byref(c), DSS, 1) == 0
    assert (c.latched, c.active, c.failStreak) == (0, DSS, 0)
    assert c.latches == 1, "the history stays until the counts are reset"


def test_a_latch_is_reported_once(lib):
    c = core(lib, DSS)
    latched = [fail_dss_frame(lib, c) for _ in range(3)]
    latched.append(report(lib, c, DSS, FAILED))  # a late report after the latch
    assert latched == [0, 0, 1, 0] and c.latches == 1


def test_verify_failures_never_latch(lib):
    """verify: the MSS scored the frame anyway; a failing DSS costs nothing."""
    c = core(lib, VERIFY)
    for _ in range(10):
        assert report(lib, c, route(lib, c), FAILED) == 0
    assert (c.failures, c.fallbacks, c.latched, c.active) == (10, 0, 0, VERIFY)


def test_mss_frames_are_not_reported(lib):
    c = core(lib)
    assert report(lib, c, MSS, FAILED) == 0
    assert c.failures == 0


# --- verify mismatches ------------------------------------------------------------------------


def test_a_mismatch_is_counted_and_the_first_kept(lib):
    c = core(lib, VERIFY)
    report(lib, c, route(lib, c), MISMATCH)
    lib.l3_detect_core_note_mismatch(ctypes.byref(c), 7, 12, 3)
    report(lib, c, route(lib, c), MISMATCH)
    lib.l3_detect_core_note_mismatch(ctypes.byref(c), 8, 2, 0)
    assert c.mismatches == 2
    assert (c.mismatchSlot, c.mismatchBin, c.mismatchField) == (7, 12, 3)


def test_a_mismatch_is_not_a_failure(lib):
    c = core(lib, VERIFY)
    report(lib, c, route(lib, c), MISMATCH)
    assert (c.failures, c.failStreak) == (0, 0)


# --- DSS cycles ---------------------------------------------------------------------------------


def test_the_dss_cycles_keep_last_and_max(lib):
    c = core(lib, DSS)
    report(lib, c, route(lib, c), OK, inv=600, score=6000)
    report(lib, c, route(lib, c), OK, inv=300, score=12000)
    assert (c.dssInvCyclesLast, c.dssInvCyclesMax) == (300, 600)
    assert (c.dssScoreCyclesLast, c.dssScoreCyclesMax) == (12000, 12000)


def test_a_failed_frame_leaves_the_cycles_alone(lib):
    c = core(lib, DSS)
    report(lib, c, route(lib, c), OK, inv=600, score=6000)
    report(lib, c, route(lib, c), FAILED, inv=9999, score=9999)
    assert (c.dssInvCyclesLast, c.dssScoreCyclesLast) == (600, 6000)


# --- counts ------------------------------------------------------------------------------------


def test_reset_counts_keeps_the_choice_and_the_latch(lib):
    c = core(lib, DSS)
    for _ in range(3):
        fail_dss_frame(lib, c)
    lib.l3_detect_core_note_mismatch(ctypes.byref(c), 1, 2, 3)
    lib.l3_detect_core_reset_counts(ctypes.byref(c))
    assert (c.requested, c.active, c.latched, c.failLimit) == (DSS, MSS, 1, 3)
    assert (c.failures, c.fallbacks, c.latches, c.dssFrames, c.haveMismatch) == (0, 0, 0, 0, 0)


# --- names and the status line --------------------------------------------------------------


@pytest.mark.parametrize(("which", "name"), list(enumerate(fw.DETECT_CORE_NAMES)))
def test_every_core_has_a_name(lib, which, name):
    """mss too: the status line prints it as the active core once latched."""
    assert lib.l3_detect_core_name(which).decode() == name


@pytest.mark.parametrize("which", [DSS, VERIFY])
def test_the_choices_parse(lib, which):
    out = ctypes.c_uint32(99)
    assert lib.l3_detect_core_parse(fw.DETECT_CORE_NAMES[which].encode(), ctypes.byref(out)) == 0
    assert out.value == which


@pytest.mark.parametrize("name", [b"mss", b"MSS", b"", b"dsp", b"verify "])
def test_other_names_are_refused(lib, name):
    out = ctypes.c_uint32(99)
    assert lib.l3_detect_core_parse(name, ctypes.byref(out)) == -1
    assert out.value == 99


def test_null_names_are_refused(lib):
    assert lib.l3_detect_core_parse(None, None) == -1
    assert lib.l3_detect_core_name(3) is None


def test_the_status_line(lib):
    c = core(lib, DSS)
    report(lib, c, route(lib, c), OK, inv=1200, score=180000)
    for _ in range(3):
        fail_dss_frame(lib, c)
    route(lib, c, eligible=False)
    assert formatted(lib, c) == (
        "detect core=dss active=mss latched=1 mss=1 dss=4 verify=0 ineligible=0 "
        "failures=3 fallbacks=3 streak=3 latches=1 mismatches=0 "
        "dss_inv_us=2/2 dss_score_us=300/300"
    )


def test_the_status_line_names_the_first_mismatch(lib):
    c = core(lib, VERIFY)
    report(lib, c, route(lib, c), MISMATCH)
    lib.l3_detect_core_note_mismatch(ctypes.byref(c), 5, 17, 1)
    assert formatted(lib, c).endswith(
        " mismatches=1 dss_inv_us=0/0 dss_score_us=0/0 first_mismatch=5:17:1"
    )


def test_a_zero_clock_does_not_divide_by_zero(lib):
    c = core(lib, DSS)
    report(lib, c, route(lib, c), OK, inv=100, score=100)
    assert "dss_score_us=0/0" in formatted(lib, c, clock_mhz=0)


def test_a_short_buffer_is_truncated_not_overrun(lib):
    c = core(lib, VERIFY)
    lib.l3_detect_core_note_mismatch(ctypes.byref(c), 5, 17, 1)
    buffer = ctypes.create_string_buffer(b"\xff" * 48)
    lib.l3_detect_core_format(ctypes.byref(c), 600, buffer, 40)
    assert len(buffer.value) == 39 and buffer.raw[40:48] == b"\xff" * 8


def test_the_widest_line_fits_the_boards_buffer(lib):
    """l3_dump.c prints it from a 320-byte buffer (L3_DETECT_LINE_BYTES)."""
    c = core(lib, VERIFY)
    c.latched = 1
    for name in (
        "failStreak",
        "mssFrames",
        "dssFrames",
        "verifyFrames",
        "ineligible",
        "failures",
        "fallbacks",
        "latches",
        "mismatches",
        "mismatchSlot",
        "dssInvCyclesLast",
        "dssInvCyclesMax",
        "dssScoreCyclesLast",
        "dssScoreCyclesMax",
    ):
        setattr(c, name, 0xFFFFFFFF)
    c.haveMismatch, c.mismatchBin, c.mismatchField = 1, 63, 5
    assert len(formatted(lib, c, clock_mhz=1)) < 320
