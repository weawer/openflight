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
    stop_frozen = _function(source, "static int32_t l3_stopFrozenRing(void)")
    freeze = _function(source, "static int32_t l3_freezeCapture(void)")
    release = _function(source, "static int32_t l3_cli_release(int32_t argc, char *argv[])")
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
static int permit, stopCalls, rfStopCalls, rfStopResult, rearmCalls;
static int Semaphore_pend(void *semaphore, unsigned timeout) { return permit; }
static void CLI_write(const char *format, ...) { }
static int l3_stopCaptureAtBoundary(void) { stopCalls++; return 0; }
static int l3_finishCaptureStop(void) { rfStopCalls++; return rfStopResult; }
static int l3_sparseRearm(void) { rearmCalls++; return 0; }
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
        + stop_frozen
        + "\n"
        + freeze
        + "\n"
        + release
        + """
int freeze_capture_rf(int active, int latched, int allow, int rf_result) {
    gCaptureActive = active; gSelfTriggerLatched = latched; permit = allow; stopCalls = 0;
    rfStopCalls = 0; rfStopResult = rf_result;
    return l3_freezeCapture();
}
int freeze_capture(int active, int latched, int allow) {
    return freeze_capture_rf(active, latched, allow, 0);
}
int freeze_rf_stop_calls(void) { return rfStopCalls; }
int release_ring(int active, int latched, int allow, int rf_result, int incomplete) {
    gCaptureActive = active; gSelfTriggerLatched = latched; permit = allow;
    stopCalls = 0; rfStopCalls = 0; rfStopResult = rf_result; rearmCalls = 0;
    gCaptureIncomplete = incomplete;
    return l3_cli_release(0, NULL);
}
int release_rearm_calls(void) { return rearmCalls; }
int capture_incomplete(void) { return gCaptureIncomplete; }
int freeze_incomplete(int incomplete) {
    gCaptureActive = 1; gSelfTriggerLatched = 0; permit = 1; stopCalls = 0;
    gCaptureIncomplete = incomplete;
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
    "active,latched,allow,result,still_latched,stops,rf_stops",
    [
        # Self-triggered freeze already complete: stop RF, consume the latch.
        (0, 1, 1, 0, 0, 0, 1),
        # Self-triggered freeze still finishing: wait, stop RF, consume.
        (1, 1, 1, 0, 0, 0, 1),
        # Freeze timed out: RF untouched, latch kept for a retry.
        (1, 1, 0, -1, 1, 0, 0),
        # Host-requested dump: the boundary stop owns MMWave_stop.
        (1, 0, 1, 0, 0, 1, 0),
        (0, 0, 1, -1, 0, 0, 0),
    ],
)
def test_freeze_reuses_the_shot_and_preserves_latch_on_timeout(
    firmware, active, latched, allow, result, still_latched, stops, rf_stops
):
    """2026-09-27 hardware: a self-triggered freeze never stopped RF, so the
    rearm's MMWave_start failed with "RF restart failed"."""
    assert firmware.freeze_capture(active, latched, allow) == result
    assert firmware.capture_latched() == still_latched
    assert firmware.freeze_stop_calls() == stops
    assert firmware.freeze_rf_stop_calls() == rf_stops


def test_failed_rf_stop_keeps_the_latch_so_a_retry_stops_rf_again(firmware):
    """Clearing the latch first would send the retry down the unlatched path,
    which returns early once gCaptureActive is 0 and never stops RF."""
    assert firmware.freeze_capture_rf(0, 1, 1, -1) == -1
    assert firmware.capture_latched() == 1
    assert firmware.freeze_capture_rf(0, 1, 1, 0) == 0
    assert firmware.capture_latched() == 0
    assert firmware.freeze_rf_stop_calls() == 1


@pytest.mark.parametrize(
    "active,latched,allow,rf_result,incomplete,result,rearms,rf_stops",
    [
        # Unwanted self-trigger: stop RF, rearm, nothing streamed.
        (0, 1, 1, 0, 0, 0, 1, 1),
        # A discarded capture that overran is still rearmed.
        (0, 1, 1, 0, 1, 0, 1, 1),
        # RF did not stop: rearming would fail with "RF restart failed".
        (0, 1, 1, -1, 0, -1, 0, 1),
        # Freeze timed out: nothing stopped, nothing rearmed, latch kept.
        (1, 1, 0, 0, 0, -1, 0, 0),
        # No self-trigger: stop at the next boundary and rearm.
        (1, 0, 1, 0, 0, 0, 1, 0),
    ],
)
def test_release_stops_rf_then_rearms_and_discards_an_incomplete_capture(
    firmware, active, latched, allow, rf_result, incomplete, result, rearms, rf_stops
):
    """Ported from feat/iwr-calcs (006525f), on this branch's stop step."""
    assert firmware.release_ring(active, latched, allow, rf_result, incomplete) == result
    assert firmware.release_rearm_calls() == rearms
    assert firmware.freeze_rf_stop_calls() == rf_stops
    if result == 0:
        assert firmware.capture_incomplete() == 0
        assert firmware.capture_latched() == 0


def test_a_readback_still_refuses_an_incomplete_capture(firmware):
    assert firmware.freeze_incomplete(0) == 0
    assert firmware.freeze_incomplete(1) == -1


@pytest.fixture(scope="module")
def rf_stop(tmp_path_factory):
    """The real l3_finishCaptureStop, compiled against stubbed SDK calls."""
    compiler = shutil.which("cc")
    if compiler is None:
        pytest.skip("C compiler unavailable")
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    stop = _function(source, "static int32_t l3_finishCaptureStop(void)")
    harness = (
        """
#include <stdint.h>
#include <stddef.h>
#define L3_EDMA_CHANNEL 0
#define EDMA3_CHANNEL_TYPE_DMA 0
static void *gMMWaveHandle, *gEdmaHandle;
static uint8_t gCaptureActive, gFrontEndRunning;
static int stopCalls, stopResult;
static int MMWave_stop(void *handle, int32_t *err) { stopCalls++; *err = -1; return stopResult; }
static void Task_sleep(unsigned ticks) { }
static void CLI_write(const char *format, ...) { }
static int EDMA_disableChannel(void *handle, int channel, int type) { return 0; }
"""
        + stop
        + """
int finish_stop(int running, int result) {
    gCaptureActive = 1; gFrontEndRunning = running; stopCalls = 0; stopResult = result;
    return l3_finishCaptureStop();
}
int mmwave_stop_calls(void) { return stopCalls; }
int front_end_running(void) { return gFrontEndRunning; }
int capture_active(void) { return gCaptureActive; }
"""
    )
    root = tmp_path_factory.mktemp("rf-stop-c")
    c_file = root / "rf_stop.c"
    c_file.write_text(harness)
    lib_file = root / "rf_stop.so"
    subprocess.run(
        [compiler, "-shared", "-fPIC", "-O2", str(c_file), "-o", str(lib_file)], check=True
    )
    return ctypes.CDLL(str(lib_file))


@pytest.mark.parametrize(
    "running,result,status,calls,still_running,active",
    [
        (1, 0, 0, 1, 0, 0),  # RF running: stop it
        (1, -1, -1, 1, 1, 1),  # stop failed: RF still counted as running
        # 2026-09-27 hardware: a self-trigger latched just before sensorStop
        # shut the capture down, the latch outlived the stopped sensor, and
        # the next session's sensorStop called MMWave_stop on it:
        # "MMWave_stop failed (-203227134)" (MMWAVE_EINVAL). Stopping a
        # stopped front end is now a no-op.
        (0, -1, 0, 0, 0, 0),
    ],
)
def test_rf_stop_is_idempotent(rf_stop, running, result, status, calls, still_running, active):
    assert rf_stop.finish_stop(running, result) == status
    assert rf_stop.mmwave_stop_calls() == calls
    assert rf_stop.front_end_running() == still_running
    assert rf_stop.capture_active() == active
