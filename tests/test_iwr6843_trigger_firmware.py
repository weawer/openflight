"""Execute the RF-stop/release paths with a native C harness.

The self-trigger detector itself (firmware/iwr6843/l3_trigger.c, ported from
Cormac131/feat/iwr-calcs) has its own dedicated host-testable unit tests in
test_iwr6843_firmware_trigger.py, which builds and drives it directly. This
file no longer needs to compile it: l3_freezeCapture and l3_cli_release call
gTriggerEnabled/gSelfTriggerLatched only, not detector internals.
"""

from __future__ import annotations

import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest


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
    stop_frozen = _function(source, "static int32_t l3_stopFrozenRing(void)")
    freeze = _function(source, "static int32_t l3_freezeCapture(void)")
    release = _function(source, "static int32_t l3_cli_release(int32_t argc, char *argv[])")
    harness = (
        """
#include <stdint.h>
#include <stddef.h>
static uint8_t gSelfTriggerLatched, gHwaFreezeRequested, gPostCaptureStarted;
static uint8_t gCaptureActive, gCaptureIncomplete;
static void *gHwaFreezeSemaphore = (void *)1;
static int permit, stopCalls, rfStopCalls, rfStopResult, rearmCalls;
static int Semaphore_pend(void *semaphore, unsigned timeout) { return permit; }
static void CLI_write(const char *format, ...) { }
static int l3_stopCaptureAtBoundary(void) { stopCalls++; return 0; }
static int l3_finishCaptureStop(void) { rfStopCalls++; return rfStopResult; }
static int l3_sparseRearm(void) { rearmCalls++; return 0; }
"""
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
"""
    )
    root = tmp_path_factory.mktemp("trigger-c")
    c_file = root / "trigger.c"
    c_file.write_text(harness)
    lib_file = root / "trigger.so"
    subprocess.run(
        [compiler, "-shared", "-fPIC", "-O2", str(c_file), "-o", str(lib_file)], check=True
    )
    return ctypes.CDLL(str(lib_file))


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


def test_all_capture_restart_paths_rearm_the_detector():
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    for signature in (
        "int32_t l3_cli_dump(int32_t argc, char *argv[])",
        "static int32_t l3_sparseRearm(void)",
    ):
        body = _function(source, signature)
        assert "l3_trigRearm();" in body
        assert body.index("l3_trigRearm();") < body.index("l3_startFrontEnd()")


def test_trigger_configuration_disables_reader_before_replacing_state():
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    body = _function(source, "static int32_t l3_cli_triggerCfg(int32_t argc, char *argv[])")
    assert body.index("gTriggerEnabled = 0U") < body.index("gTrigCfg = cfg")


@pytest.fixture(scope="module")
def integrated_detector(tmp_path_factory):
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    root = tmp_path_factory.mktemp("integrated-detector")
    harness = r"""
#include <stdint.h>
#include <string.h>
#include "l3_trigger.h"
#define N_RX 4U
#define L3_MAX_LOOPS 16U
static uint8_t g_ring[200000];
static uint32_t gFrameOffset[1] = {64}, gFrameBinCount[1];
static struct { uint32_t loops, chirpsPerFrame; } gCapturePlan = {12, 36};
static uint32_t gRingFrame, gHwaFreezeRequestFrame, gHwaFreezeTargetFrame;
static uint32_t gPreFramesCaptured, gPostFramesCaptured, gPostFramesObserved;
static uint32_t gPostCaptureStarted, gActiveFrameIsPost, gActiveFrameShouldKeep;
static uint32_t gCaptureActive;
static l3_trig_t gTrig;
static int l3_restartCompletedHwaFrame(void) { return 0; }
static int l3_startFrontEnd(void) { return 0; }
static void CLI_write(const char *format, ...) { }
"""
    for signature in (
        "static void l3_verticalResidual(",
        "static void l3_trigRearm(void)",
        "static int32_t l3_sparseRearm(void)",
    ):
        harness += _function(source, signature) + "\n"
    harness += r"""
void residual(const int16_t *samples, unsigned bins, unsigned bin, l3_trig_obs_t *out) {
    gFrameBinCount[0] = bins;
    memcpy(g_ring + 64, samples, 12 * 3 * 4 * bins * 4);
    l3_verticalResidual(0, bin, out);
}
int rearm_fired(void) {
    gTrig.state = L3_TRIG_STATE_FIRED;
    gTrig.floor = 37.0f;
    if (l3_sparseRearm() != 0 || gTrig.floor != 37.0f) return -1;
    return gTrig.state;
}
"""
    c_file = root / "integration.c"
    c_file.write_text(harness)
    library = root / "integration.so"
    subprocess.run(
        [
            "cc",
            "-shared",
            "-fPIC",
            "-O2",
            "-I",
            "firmware/iwr6843",
            str(c_file),
            "firmware/iwr6843/l3_trigger.c",
            "-lm",
            "-o",
            str(library),
        ],
        check=True,
    )
    return ctypes.CDLL(str(library))


def test_release_resets_fired_detector_without_losing_noise_floor(integrated_detector):
    assert integrated_detector.rearm_fired() == 0
    assert integrated_detector.rearm_fired() == 0


def test_trigger_notification_is_deferred_out_of_capture_work():
    source = Path("firmware/iwr6843/l3_dump.c").read_text()
    update = _function(source, "static void l3_updateSelfTrigger(void)")
    notice = _function(source, "static void l3_triggerNoticeTask(UArg arg0, UArg arg1)")
    assert "CLI_write" not in update
    assert "Semaphore_post(gTriggerNoticeSemaphore)" in update
    assert "Semaphore_pend(gTriggerNoticeSemaphore, BIOS_WAIT_FOREVER)" in notice
    assert 'CLI_write("Triggered\\n")' in notice


@pytest.mark.parametrize("bins,bin_index", [(32, 0), (32, 31), (53, 25)])
def test_compacted_iq16_residual_uses_all_loops_and_only_vertical_tx(
    integrated_detector, bins, bin_index
):
    import numpy as np

    class Observation(ctypes.Structure):
        _fields_ = [(name, ctypes.c_float) for name in ("energy", "peak", "loop0", "r1Re", "r1Im")]

    iq = np.random.default_rng(42).integers(-1000, 1000, (12, 3, 4, bins, 2), dtype=np.int16)
    iq[:, 1] = 30000
    vertical = np.take(iq, [0, 2], axis=1)
    samples = vertical[:, :, :, bin_index, 1].astype(float) + 1j * vertical[:, :, :, bin_index, 0]
    residual = samples - samples.mean(axis=0)
    power = np.sum(abs(residual) ** 2, axis=(1, 2))
    correlation = np.sum(residual[1:] * residual[:-1].conj())
    result = Observation()
    integrated_detector.residual(
        iq.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)), bins, bin_index, ctypes.byref(result)
    )
    assert result.energy == pytest.approx(power.sum(), rel=1e-5)
    assert result.peak == pytest.approx(power.max(), rel=1e-5)
    assert result.loop0 == pytest.approx(power[0], rel=1e-5)
    assert result.r1Re == pytest.approx(correlation.real, rel=1e-5)
    assert result.r1Im == pytest.approx(correlation.imag, rel=1e-5)
