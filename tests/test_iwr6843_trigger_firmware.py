"""Execute the firmware detector with synthetic range-power frames."""

from __future__ import annotations

import ctypes
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843.self_trigger import PHASES, BallLeaveDetector


def _function(source, name):
    start = source.rindex(name)
    brace = source.index("{", start)
    depth = 0
    for end in range(brace, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start : end + 1]
    raise AssertionError(name)


@pytest.fixture(scope="module")
def firmware(tmp_path_factory):
    compiler = shutil.which("cc")
    if compiler is None:
        pytest.skip("C compiler unavailable")
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    globals_ = "\n".join(re.findall(r"^static volatile[^\n]*\bgTrigger\w*[^\n]*;", source, re.M))
    freeze = _function(source, "static int32_t l3_freezeCapture(void)")
    clear = _function(source, "static void l3_clearTriggerMotion(void)")
    consider = _function(source, "static void l3_considerSelfTrigger(void)")
    harness = (
        """
#include <stdint.h>
#include <stddef.h>
#define L3_TRIGGER_APPROACH_BINS 12U
static uint8_t gSelfTriggerLatched, gHwaFreezeRequested, gPostCaptureStarted;
static uint32_t gPreFramesCaptured = 1, gFramePeriodUs = 3000;
static uint32_t gFrameBinCount[1];
static struct { uint32_t preFrames, loops, preBins; } gCapturePlan = {1, 12, 53};
static const float *powers;
static uint8_t gCaptureActive, gCaptureIncomplete;
static void *gHwaFreezeSemaphore = (void *)1;
static int permit, stopCalls;
static int Semaphore_pend(void *semaphore, unsigned timeout) { return permit; }
static void CLI_write(const char *format, ...) { }
static int l3_stopCaptureAtBoundary(void) { stopCalls++; return 0; }
static float l3_verticalPowerAt(uint32_t slot, uint32_t bin) { return powers[bin]; }
"""
        + globals_
        + "\n"
        + clear
        + """
static void l3_noteTrigger(uint8_t phase, float tee, float approach) { gTriggerPhase = phase; }
static void l3_latchSelfTrigger(float tee, float approach) {
    gSelfTriggerLatched = 1;
    l3_clearTriggerMotion();
    l3_noteTrigger(9, tee, approach);
}
"""
        + consider
        + "\n"
        + freeze
        + """
int freeze_capture(int active, int latched, int allow) {
    gCaptureActive = active; gSelfTriggerLatched = latched; permit = allow; stopCalls = 0;
    return l3_freezeCapture();
}
int capture_latched(void) { return gSelfTriggerLatched; }
int freeze_stop_calls(void) { return stopCalls; }
void reset(unsigned period) {
    l3_clearTriggerMotion();
    gTriggerEnabled = 1; gTriggerPower = 1000; gTriggerHits = 2;
    gSelfTriggerLatched = 0; gFramePeriodUs = period;
    gPreFramesCaptured = 1; gCapturePlan.preFrames = 1;
}
void set_history(unsigned count, unsigned required) {
    gPreFramesCaptured = count; gCapturePlan.preFrames = required;
}
int step(float *row, unsigned bin, unsigned count) {
    powers = row; gTriggerBin = bin; gFrameBinCount[0] = count;
    gCapturePlan.preBins = count;
    l3_considerSelfTrigger();
    return gTriggerPhase;
}
"""
    )
    root = tmp_path_factory.mktemp("trigger-c")
    c_file = root / "trigger.c"
    c_file.write_text(harness)
    lib_file = root / "trigger.so"
    subprocess.run(
        [compiler, "-shared", "-fPIC", "-O2", str(c_file), "-o", str(lib_file)], check=True
    )
    lib = ctypes.CDLL(str(lib_file))
    lib.reset.argtypes = [ctypes.c_uint]
    lib.step.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_uint, ctypes.c_uint]
    lib.step.restype = ctypes.c_int
    return lib


@pytest.mark.parametrize("period", [2000, 3000, 4000])
@pytest.mark.parametrize("case", ["reversal", "departure", "gap", "stationary", "timeout"])
def test_firmware_matches_replay_and_rejects_non_shots(firmware, period, case):
    frames = [{14: 1000}, {14: 1000}, {14: 1000, 2: 1500}, {14: 1000, 6: 1500}]
    if case == "reversal":
        frames += [{14: 1000, 3: 1500}]
    elif case == "departure":
        frames += [{16: 1500}, {18: 1500}]
    elif case == "gap":
        frames += [{14: 1000}] * 300 + [{14: 1000, 3: 1500, 16: 1600}, {18: 1600}]
    elif case == "stationary":
        frames += [{16: 1500}] * 5
    else:
        frames += [{14: 1000, 6: 1500}] * 40 + [{16: 1500}, {18: 1500}]
    firmware.reset(period)
    detector = BallLeaveDetector(frame_period_s=period / 1e6)
    phases = []
    for i, peaks in enumerate(frames):
        row = np.zeros(53, dtype=np.float32)
        for bin_index, power in peaks.items():
            row[bin_index] = power
        phase = firmware.step(row.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), 14, 53)
        phases.append(PHASES[phase])
        assert PHASES[phase] == detector.step(i, row, 14, 53).phase
    assert ("fired" in phases) == (case == "departure")


def test_firmware_waits_for_a_full_pretrigger_history(firmware):
    firmware.reset(3000)
    firmware.set_history(2, 9)
    for peaks in (
        {14: 1000},
        {14: 1000},
        {14: 1000, 2: 1500},
        {14: 1000, 6: 1500},
        {16: 1500},
        {18: 1500},
    ):
        row = np.zeros(53, dtype=np.float32)
        for bin_index, power in peaks.items():
            row[bin_index] = power
        phase = firmware.step(row.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), 14, 53)
        assert PHASES[phase] == "no-frame"


@pytest.mark.parametrize(
    "active,latched,allow,result,still_latched,stops",
    [
        (0, 1, 1, 0, 0, 0),
        (1, 1, 1, 0, 0, 0),
        (1, 1, 0, -1, 1, 0),
        (1, 0, 1, 0, 0, 1),
        (0, 0, 1, -1, 0, 0),
    ],
)
def test_freeze_reuses_the_shot_and_preserves_latch_on_timeout(
    firmware, active, latched, allow, result, still_latched, stops
):
    assert firmware.freeze_capture(active, latched, allow) == result
    assert firmware.capture_latched() == still_latched
    assert firmware.freeze_stop_calls() == stops
