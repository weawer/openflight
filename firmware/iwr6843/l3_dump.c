/* Stage-1c: L3 raw-ADC rolling buffer + dump-on-command  (IWR6843 MSS/R4F).
 *
 * MILESTONE 3 (this build): REAL raw-ADC capture via HARDWARE-triggered EDMA.
 * The DFE's chirp-available hardware event (EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL)
 * directly triggers an EDMA that copies one chirp (CHIRP_BYTES) from the ADCBUF
 * into the L3 rolling buffer -- the CPU is out of the per-chirp timing path, so
 * there is no read-vs-write race (the earlier software-triggered version lost
 * ~23% of chirps to that race). One self-linked SYNC_A param set fills the whole
 * ring continuously (dest auto-advances per chirp, wraps every RING_CHIRPS).
 * `l3dump` disables the channel (freeze), streams the ring, re-arms.
 *
 * HWA_CHAINED_SNAPSHOT_RING replaces that raw path with TI's production data
 * flow: the DFE triggers range FFTs directly in HWA and HWA param completion
 * triggers EDMA copies of selected range bins into the compact L3 ring. The CPU
 * rearms one frame at a time during inter-frame idle time. Completed frames
 * advance through the compact L3 ring so a trigger can preserve deterministic
 * pre-impact history plus a fixed post-impact tail. The leave detector runs on
 * a finished pre-trigger slot while the HWA writes the next one.
 *
 * Chirp order filled == chirp order fired == TDM (chirp c -> tx=c%N_TX,
 * loop=c/N_TX), matching iwr6843_l3dump.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* BIOS/XDC */
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* mmWave SDK drivers + control */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/uart/include/uartsci.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/hwa/hwa.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include "dump_format.h"
#include "detect_queue.h"
#include "capture_plan.h"
#include "track_select.h"
#include "l3_trigger.h"
#include "l3_ball.h"
#include <math.h>

#include "l3_angle.h"
#include "l3_ball_track.h"
#include "l3_club_track.h"
#include "l3_adaptive.h"
#include "l3_impact.h"
#include "l3_iq8.h"
#include "l3_iq16_stats.h"
#include "l3_retain.h"
#include "compact_iq16.h"
#include "l3_profile.h"
#include "l3_result.h"
#include "l3_shot.h"

#if defined(L3_DUMP_IQ8) || defined(L3_RING_IQ8)
#define L3_ANY_IQ8 1
#endif

/* --- task priorities (mirror the mmw demo): ctrl > CLI. -------------------- */
#define L3_INIT_TASK_PRIORITY  2
#define L3_CLI_TASK_PRIORITY   3
/* Above the CLI: a stats or debug write must never delay the next HWA arm
 * past the ~380 us gap a 2 ms frame leaves after its chirps. */
#define L3_HWA_REARM_TASK_PRIORITY (L3_CLI_TASK_PRIORITY + 1U)
/* Keep the live snapshot worker below CLI. SYS/BIOS Task_yield does not allow
 * lower-priority tasks to run, and a priority-4 snapshot loop starved l3dump
 * so the host only saw the echoed 7-byte "l3dump\n" command. */
#define L3_SNAPSHOT_TASK_PRIORITY 1
/* Below rearm, so a slot read runs while the next frame is captured. Kept
 * below the CLI task too: raising it above CLI (tried 2026-09-28, reverted
 * 2026-09-29) stopped scratch_stale during diagnostic output, but the CLI's
 * l3dump readback after a real trigger also runs at this priority, with
 * polled (not interrupt-driven) UART writes -- a detect task that outranks
 * it can starve that polling loop. Hardware evidence: real self-triggered
 * captures on this rig stalled at 18 bytes of a ~450 KB dump with detect
 * above CLI; the l3_iq16_stats fix (l3_dump_hann_fast_3ms_20260929.bin)
 * already cleared scratch_stale at the old, safe ordering. */
#define L3_DETECT_TASK_PRIORITY 1
/* Writes the detect task's CLI lines. At the CLI task's own priority SYS/BIOS
 * never preempts one for the other, so neither can splice a line into the
 * other's; below it, a host command arriving mid-line would let the CLI
 * task cut a "Triggered" notice or a debug line in two. */
#define L3_NOTICE_TASK_PRIORITY L3_CLI_TASK_PRIORITY
#define L3_CTRL_TASK_PRIORITY  5

/* ASCII CAN is reserved as an out-of-band dump cancellation byte. The CLI
 * task is executing l3dump synchronously, so RX interrupts are disabled and
 * the byte remains in the SCI register until checked between frame writes. */
#define L3_DUMP_CANCEL_BYTE 0x18U
#define L3_DUMP_CANCEL_ACK  ((uint8_t *)"ILDCANCEL")
#define L3_DUMP_CANCEL_ACK_BYTES 8U

/* --- capture geometry: MUST match the .cfg the host sends ------------------
 * N_TX / N_SAMPLES / SAVE_SAMPLES / SAVE_OFFSET_SAMPLES / LOOPS /
 * RING_FRAMES are overridable from the make line
 * (variant builds: B = N_TX=2 LOOPS=16 RING_FRAMES=12, TX2 = N_TX=3
 * LOOPS=10 RING_FRAMES=12). sensorStart REJECTS a cfg whose acquired
 * sample/loop/TX geometry mismatches. Keep chirpCfg/frameCfg in sync with
 * N_TX. SAVE_* only changes what raw ADC samples are copied into L3. */
#ifndef N_TX
#define N_TX              2
#endif
#define N_RX              4
#ifndef N_SAMPLES
#define N_SAMPLES         128
#endif
#ifndef SAVE_SAMPLES
#define SAVE_SAMPLES      N_SAMPLES
#endif
#ifndef SAVE_OFFSET_SAMPLES
#define SAVE_OFFSET_SAMPLES 0
#endif
#if SAVE_SAMPLES > N_SAMPLES
#error "SAVE_SAMPLES must be <= N_SAMPLES"
#endif
#if (SAVE_OFFSET_SAMPLES + SAVE_SAMPLES) > N_SAMPLES
#error "SAVE_OFFSET_SAMPLES + SAVE_SAMPLES must be <= N_SAMPLES"
#endif
#ifndef LOOPS
#define LOOPS             32
#endif
#ifndef SNAPSHOT_BINS
#define SNAPSHOT_BINS     16
#endif
#ifndef SNAPSHOT_BIN_START
#define SNAPSHOT_BIN_START 2
#endif
#if SNAPSHOT_BINS > N_SAMPLES
#error "SNAPSHOT_BINS must be <= N_SAMPLES"
#endif
#if (SNAPSHOT_BIN_START + SNAPSHOT_BINS) > N_SAMPLES
#error "SNAPSHOT_BIN_START + SNAPSHOT_BINS must be <= N_SAMPLES"
#endif
#ifdef SNAPSHOT_DYNAMIC_WINDOWS
#ifndef HWA_CHAINED_SNAPSHOT_RING
#error "SNAPSHOT_DYNAMIC_WINDOWS requires HWA_CHAINED_SNAPSHOT_RING"
#endif
#ifndef SNAPSHOT_MIDDLE_BIN_START
#define SNAPSHOT_MIDDLE_BIN_START 32
#endif
#ifndef SNAPSHOT_LATE_BIN_START
#define SNAPSHOT_LATE_BIN_START 47
#endif
#if (SNAPSHOT_MIDDLE_BIN_START + SNAPSHOT_BINS) > N_SAMPLES
#error "middle snapshot window exceeds the range FFT"
#endif
#if (SNAPSHOT_LATE_BIN_START + SNAPSHOT_BINS) > N_SAMPLES
#error "late snapshot window exceeds the range FFT"
#endif
#endif
#ifdef HWA_CHAINED_SNAPSHOT_RING
#ifndef SNAPSHOT_DUMP
#error "HWA_CHAINED_SNAPSHOT_RING requires SNAPSHOT_DUMP"
#endif
#ifndef ENABLE_HWA_SMOKE
#error "HWA_CHAINED_SNAPSHOT_RING requires ENABLE_HWA_SMOKE"
#endif
#endif
#define CHIRPS_PER_FRAME  (N_TX * LOOPS)                        /* 64 */
#define FRAME_COMPLEX     (CHIRPS_PER_FRAME * N_RX * N_SAMPLES)   /* 32768 */
#define FRAME_BYTES       (FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t)) /* 128 KB */
#define SAVED_FRAME_COMPLEX (CHIRPS_PER_FRAME * N_RX * SAVE_SAMPLES)
#define SAVED_FRAME_BYTES (SAVED_FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t))
#define SNAPSHOT_CHIRP_COMPLEX (N_RX * SNAPSHOT_BINS)
#define SNAPSHOT_CHIRP_BYTES (SNAPSHOT_CHIRP_COMPLEX * 2 * (uint32_t)sizeof(int16_t))
#define SNAPSHOT_FRAME_COMPLEX (CHIRPS_PER_FRAME * SNAPSHOT_CHIRP_COMPLEX)
#define SNAPSHOT_FRAME_BYTES (SNAPSHOT_FRAME_COMPLEX * 2 * (uint32_t)sizeof(int16_t))
/* One chirp in ADCBUF (non-interleaved): N_RX x N_SAMPLES x 4 bytes. */
#define CHIRP_BYTES       (N_RX * N_SAMPLES * 2 * (uint32_t)sizeof(int16_t))  /* 2048 */
#define SAVED_CHIRP_BYTES (N_RX * SAVE_SAMPLES * 2 * (uint32_t)sizeof(int16_t))

/* --- rolling buffer (default L3 = 6 banks x 128 KB = 768 KB) ----------------
 * The chained-snapshot HWA path compresses selected FFT range bins from each
 * captured frame into the rolling ring. */
#ifndef RING_FRAMES
#define RING_FRAMES  6
#endif
#define RING_CHIRPS  (RING_FRAMES * CHIRPS_PER_FRAME)
#if defined(HWA_CHAINED_SNAPSHOT_RING)
#define RING_FRAME_COMPLEX SNAPSHOT_FRAME_COMPLEX
#else
#define RING_FRAME_COMPLEX SAVED_FRAME_COMPLEX
#endif

/* Same expression the TI platform linker uses to size the L3_RAM region
 * (ti/platform/xwr68xx/r4f_linker.cmd), computed from the same two --define
 * values the SDK passes to both the compiler and the linker. Deriving it
 * rather than hardcoding means a MMWAVE_L3RAM_NUM_BANK change cannot leave
 * the firmware's idea of the arena disagreeing with the linker's. */
#define L3_TOTAL_BYTES         (MMWAVE_L3RAM_NUM_BANK * MMWAVE_SHMEM_BANK_SIZE)
#define L3_MAX_CAPTURE_FRAMES  64U
#define L3_MAX_LOOPS           16U
#define L3_MIN_LOOPS           2U
#ifdef L3_RING_IQ8
/* The HWA emits complex16 samples. One maximum-size production frame lands in
 * scratch (g_iq16FrameScratch, in DATA_RAM), then the rearm task
 * block-quantizes it into the compact IQ8 ring. Both IQ16 and IQ8 mode use
 * the whole L3 arena directly; only the compressed IQ8 ring's contents
 * differ in size. */
#define L3_IQ8_HWA_SHIFT      4U
#define L3_IQ8_HWA_SCALE      (1U << L3_IQ8_HWA_SHIFT)
#ifdef L3_IQ8_SPARSE_SCALE
/* Preview every eighth complex sample to select a per-frame IQ8 scale before
 * the full packing pass. This keeps the HWA rearm path inside the 2 ms budget. */
#define L3_IQ8_SCALE_COMPLEX_STRIDE 8U
#endif
#define L3_RING_MAX_BINS       64U
#define L3_CAPTURE_FORMAT_IQ16 0U
#define L3_CAPTURE_FORMAT_IQ8  1U
/* Compact IQ16 formats: the HWA writes the wide processing window to the
 * IQ16 scratch, the detect task reads it there at full precision, and the
 * rearm task copies only the retained bins into L3 (compact_iq16.c).
 * compact16 centres the retained window in the processing window;
 * adaptive16 places it where l3_retain.c says the shot is. */
#define L3_CAPTURE_FORMAT_COMPACT16  2U
#define L3_CAPTURE_FORMAT_ADAPTIVE16 3U
#define L3_SCRATCH_NONE 0xFFU
#define L3_IQ16_SCRATCH_FRAME_BYTES  \
    (N_TX * L3_MAX_LOOPS * N_RX * L3_RING_MAX_BINS * 2U * \
     (uint32_t)sizeof(int16_t))
#define L3_IQ16_SCRATCH_WORDS  \
    (L3_IQ16_SCRATCH_FRAME_BYTES / (uint32_t)sizeof(int16_t))
#endif
#ifndef L3_RING_MAX_BINS
/* Only meaningful for the iq8 ring; harmless placeholder for the plan
 * geometry when this build has no iq8 support at all, since l3plan_build
 * only consults it when bytesPerComplex == 2U (iq8). */
#define L3_RING_MAX_BINS       N_SAMPLES
#endif
#define L3_DEFAULT_PRE_START   20U
#define L3_DEFAULT_PRE_BINS    32U
#define L3_DEFAULT_POST_START  32U
#define L3_DEFAULT_LATE_START  47U
#define L3_DEFAULT_POST_BINS   53U
#define L3_DEFAULT_POST_FRAMES 16U
#ifdef HYBRID_CADENCE_CAPTURE
#define L3_DEFAULT_POST_STRIDE 2U
#else
#define L3_DEFAULT_POST_STRIDE 1U
#endif
#define L3_MAX_POST_STRIDE     16U

/* L3CapturePlan (capture_plan.h) mirrors what used to be a private
 * l3_capture_plan_t defined here; it now lives there so l3plan_build can be
 * compiled and tested on the host without any TI headers. */

#pragma DATA_SECTION(g_ring, ".l3ring")
#pragma DATA_ALIGN(g_ring, 8)
static uint8_t g_ring[L3_TOTAL_BYTES];
#ifdef L3_RING_IQ8
static uint8_t gCaptureFormat = L3_CAPTURE_FORMAT_IQ16;
#pragma DATA_SECTION(g_iq16FrameScratch, ".dataScratch")
#pragma DATA_ALIGN(g_iq16FrameScratch, 8)
static int16_t g_iq16FrameScratch[2][L3_IQ16_SCRATCH_WORDS];
/* DATA_RAM is 0x30000 B and is shared with .bss, .data and the stack. If this
 * fires, either the scratch or the rest of the image grew past the region. */
_Static_assert(sizeof(g_iq16FrameScratch) <= 0x18000U,
               "IQ16 scratch exceeds its DATA_RAM allowance");
#endif
static L3CapturePlan gCapturePlan = {
    .preStart = L3_DEFAULT_PRE_START,
    .preBins = L3_DEFAULT_PRE_BINS,
    .postStart = L3_DEFAULT_POST_START,
    .postBins = L3_DEFAULT_POST_BINS,
    .lateStart = L3_DEFAULT_LATE_START,
    .postFrames = L3_DEFAULT_POST_FRAMES,
    .postStride = L3_DEFAULT_POST_STRIDE,
    /* preFrames, totalFrames, loops, chirpsPerFrame, preFrameBytes,
     * postFrameBytes, postBaseOffset, usedBytes, phased,
     * requestedPreFrames, impactStart, impactBins, impactFrames,
     * ballFrames and impactFrameBytes are all zero-initialized and are
     * recomputed by l3plan_build(). */
};
static uint8_t gFrameBinStart[L3_MAX_CAPTURE_FRAMES];
static uint8_t gFrameBinCount[L3_MAX_CAPTURE_FRAMES];
static uint16_t gFrameDeltaUs[L3_MAX_CAPTURE_FRAMES];
static uint32_t gFrameOffset[L3_MAX_CAPTURE_FRAMES];
static uint32_t gFrameBytes[L3_MAX_CAPTURE_FRAMES];
#ifdef L3_ANY_IQ8
static uint16_t gFrameIq8Scale[L3_MAX_CAPTURE_FRAMES];
#endif


/* --- EDMA: the DFE chirp-available hardware event channel + shadow link ----- */
#define L3_EDMA_CHANNEL       EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL
#define L3_EDMA_LINK_CHANNEL  EDMA_NUM_DMA_CHANNELS
#define L3_EDMA_LINK_CHANNEL_PONG (EDMA_NUM_DMA_CHANNELS + 1U)
#ifdef HWA_CHAINED_SNAPSHOT_RING
/* Match TI RangeProc's reserved request lines. HWA param-done events drive the
 * ping/pong output channels; their completion chains a two-hot signature into
 * the dummy HWA paramsets so the next DFE-triggered FFT can run. */
#define L3_HWA_OUT_PING_CHANNEL EDMA_TPCC0_REQ_HWACC_0
#define L3_HWA_OUT_PONG_CHANNEL EDMA_TPCC0_REQ_HWACC_1
#define L3_HWA_SIGNATURE_CHANNEL EDMA_TPCC0_REQ_FREE_7
#ifdef L3_IQ8_EDMA_PACK
#define L3_IQ8_PACK_PING_CHANNEL EDMA_TPCC0_REQ_FREE_8
#define L3_IQ8_PACK_PONG_CHANNEL EDMA_TPCC0_REQ_FREE_9
#endif
#define L3_HWA_OUT_PING_SHADOW EDMA_NUM_DMA_CHANNELS
#define L3_HWA_OUT_PONG_SHADOW (EDMA_NUM_DMA_CHANNELS + 1U)
#define L3_HWA_SIGNATURE_SHADOW (EDMA_NUM_DMA_CHANNELS + 2U)
#define L3_HWA_PARAM_DUMMY_PING 0U
#define L3_HWA_PARAM_FFT_PING   1U
#define L3_HWA_PARAM_DUMMY_PONG 2U
#define L3_HWA_PARAM_FFT_PONG   3U
#endif

/* --- SDK handles ----------------------------------------------------------- */
static SOC_Handle    gSocHandle;
static UART_Handle   gCliUart;    /* MSS UARTA, instance 0, 115200, has RX */
static UART_Handle   gDataUart;   /* MSS UARTB, instance 1, 921600, TX only */
static MMWave_Handle gMMWaveHandle;
static EDMA_Handle   gEdmaHandle;
static ADCBuf_Handle gAdcbufHandle;
#ifdef ENABLE_HWA_SMOKE
static HWA_Handle    gHwaHandle;
static int32_t       gHwaOpenErr;
static uint8_t       gHwaOpened;
static volatile uint8_t gHwaDone;
static int32_t       gHwaTestErr;
static uint16_t      gHwaTestPeakBin;
static uint32_t      gHwaTestPeakPower;
static uint32_t      gHwaTestRuns;
static int32_t       gHwaRealErr;
static uint16_t      gHwaRealPeakBin;
static uint32_t      gHwaRealPeakPower;
static uint32_t      gHwaRealRuns;
#ifdef HWA_CHAINED_SNAPSHOT_RING
static Semaphore_Handle gHwaRearmSemaphore;
static Semaphore_Handle gHwaFreezeSemaphore;
static Semaphore_Handle gDetectSemaphore;
static L3DetectQueue gDetectQueue;
static volatile uint32_t gDetectStale;
#ifdef L3_IQ8_EDMA_PACK
static Semaphore_Handle gIq8EdmaDoneSemaphore;
#endif
#endif
#endif
static uint8_t       gSensorOpened;
static uint32_t      gCpuClock = 200U * 1000000U;

/* --- capture state / diagnostics ------------------------------------------- */
static volatile uint8_t  gCaptureActive;
static uint16_t gFramePeriodUs;         /* from the accepted frameCfg -> header */
static volatile uint32_t gNumFrame;     /* frame-start ISR count (liveness) */
static volatile uint32_t gRingFrame;    /* completed snapshot frames since the
                                         * last dump; next write/oldest slot is
                                         * gRingFrame % RING_FRAMES once full */
static volatile uint32_t gNumWrap;      /* EDMA ring-wrap count */
static volatile uint32_t gCalibStatus;  /* RL_RF_AE_INITCALIBSTATUS payload */
static volatile uint32_t gRfFaults;     /* CPU/ESM/analog fault events */
#ifdef HWA_CHAINED_SNAPSHOT_RING
static volatile uint32_t gHwaFrameDone;
static volatile uint32_t gHwaOutputDone;
static volatile uint32_t gHwaRearms;
static volatile uint32_t gHwaRearmErrors;
static volatile uint32_t gHwaMissedFrameStarts;
/* Frame completion queued -> next HWA arm, in microseconds. */
static volatile uint32_t gHwaRearmQueuedCycles;
static volatile uint8_t  gHwaRearmQueuedValid;
static volatile uint32_t gHwaRearmLastUs;
static volatile uint32_t gHwaRearmMaxUs;
static volatile uint32_t gHwaRearmTimed;
static volatile uint8_t  gHwaArmedForFrame;
static volatile uint8_t  gHwaDoneSeen;
static volatile uint8_t  gHwaOutputSeen;
static volatile uint8_t  gHwaRearmPending;
static volatile uint8_t  gHwaRearmBusy;
static volatile uint8_t  gHwaFreezeRequested;
static volatile uint8_t  gTriggerEnabled;
static volatile uint8_t  gSelfTriggerLatched;
/* Self-trigger detector (l3_trigger.c). The detect task updates it once
 * per completed slot with gTrigBusy raised; the CLI task (triggerCfg,
 * triggerLog) waits for that flag before it resets or reads the state.
 * gTriggerPhase is the readout the host's stats parser already knows. */
static l3_trig_cfg_t     gTrigCfg;
/* How targets read their sub-bin range (L3_OBS_SUBBIN_*): "trackCfg subbin". */
static uint32_t          gObsSubBin = L3_OBS_SUBBIN_PARABOLIC;
/* Range window the detector scores bins through (L3_RANGE_WINDOW_*, see
 * l3_iq16_stats.h): "trackCfg window". Stored samples are never windowed. */
static volatile uint32_t gRangeWindow = L3_RANGE_WINDOW_NONE;
static l3_trig_t         gTrig;
static volatile uint8_t  gTrigBusy;
static float             gTrigLoopPeriodS;
/* Ball-placement detector (l3_ball.c): fed the pre window's static power
 * every other frame by the detect task; "ball cfg" resets it under the
 * same busy handshake as triggerCfg. */
static l3_ball_cfg_t     gBallCfg;
static l3_ball_t         gBall;
static volatile uint8_t  gBallBusy;
/* Where the last scored frame aimed: the locked ball (1) or the configured
 * tee (0). With follow on and no ball locked the trigger falls back to the
 * tee and counts the frames, so the ball detector cannot make a shot
 * uncapturable while it is being proven. */
static volatile uint8_t  gTrigDestBall;
static volatile uint32_t gTrigFallbackFrames;
/* Club track (l3_club_track.c) over the observation layer's targets, fed
 * the same per-bin observations the trigger scores, every frame. Reset with
 * the ring; "triggerLog track" prints it. */
static l3_club_track_t   gClubTrack;
static uint32_t          gClubTrackDest;
/* Radar calibration (attitude, element corrections, baseline zeros, range
 * bias) for angles and golf-frame positions; identity until "trackCfg cal"
 * and "trackCfg elem" set it. gLastAngle is the newest club point's estimate. */
static l3_radar_cal_t    gRadarCal;
static l3_angle_obs_t    gLastAngle;
static uint32_t          gAngleEstimates;
/* The locked ball's own angles from its static return, so the destination
 * the impact test and the ball tracker aim at is a 3D position, not a
 * point on boresight. Valid while the beamformer's peak stands clear. */
static l3_angle_obs_t    gBallAngle;
static uint8_t           gBallAngleValid;
#define L3_BALL_ANGLE_MIN_PEAK_RATIO 3.0F
/* Geometric impact detector over the club delivery and the ball position.
 * It records its verdict every frame; it fires the capture only once armed
 * ("trackCfg impact ... 1"), the range gate being the proven fallback. */
static l3_impact_cfg_t   gImpactCfg;
static l3_impact_t       gImpact;
static uint8_t           gImpactCfgSet;
static uint8_t           gImpactArmed;
static l3_delivery_t     gDelivery;        /* the newest frame's delivery fit */
static l3_vec3_t         gBallPosition;    /* destination in the golf frame */
static uint8_t           gTrigFireSource;  /* bit 0 range gate, bit 1 geometry */
/* The shot state machine and the post-impact ball tracker. Post frames the
 * ring keeps are published to the detect task with L3_DETECT_POST_EPOCH;
 * they are never overwritten before the rearm, so no liveness check. */
#define L3_DETECT_POST_EPOCH 0xFFFFFFFFU
static l3_shot_cfg_t       gShotCfg;
static l3_shot_t           gShot;
static l3_ball_track_cfg_t gBallTrackCfg;
static l3_ball_track_t     gBallTrack;
static l3_launch_t         gLaunch;
static uint32_t            gPostTimestampUs;   /* time of the newest post frame scored */
static uint32_t            gPostFramesScored;
/* The post window's own noise floor: the pre region's floor sits under the
 * golfer's returns and would hide the weak departing ball. */
static float               gBallFloor;
/* The shot result, built once per shot when the machine reaches RESULT;
 * "triggerLog result" prints it with its packet. */
static l3_shot_result_t    gShotResult;
static uint8_t             gShotResultReady;
static uint32_t            gShotId;
/* Per-stage cycle counts ("triggerLog perf") and the adaptive capture
 * windows applied between shots from the locked ball bin. */
static l3_profile_t        gProfile;
static uint8_t             gProfileReady;
static l3_adaptive_cfg_t   gAdaptiveCfg;
static l3_adaptive_windows_t gAdaptiveWindows;
static uint32_t            gAdaptiveApplied;
/* trackCfg's range resolution, defined with the on-chip selector below. */
static double            gTrackRangeResM;
static volatile uint8_t  gTriggerPhase;
static volatile uint32_t gTriggerTeePower;
static volatile uint8_t  gTriggerDebug;
static volatile uint8_t  gTriggerDebugPhase = 0xFFU;
static volatile uint8_t  gHwaShutdownRequested;
static volatile uint32_t gHwaFreezeRequestFrame;
static volatile uint32_t gHwaFreezeRequests;
static volatile uint32_t gHwaFreezeCompletions;
static volatile uint32_t gHwaFreezeTimeouts;
static volatile uint32_t gHwaFreezeRestarts;
static volatile uint32_t gPreFramesCaptured;
static volatile uint32_t gPostFramesCaptured;
static volatile uint32_t gPostFramesObserved;
static volatile uint8_t  gPostCaptureStarted;
static volatile uint8_t  gActiveFrameIsPost;
static volatile uint8_t  gActiveFrameShouldKeep;
#ifdef L3_RING_IQ8
static volatile uint8_t  gIq8Pending;
static volatile uint32_t gIq8PendingSlot;
static volatile uint8_t  gIq8PendingScratch;
static volatile uint8_t  gIq8PendingDetect;
static volatile uint32_t gIq8PendingEpoch;
static volatile uint8_t  gIq8ActiveScratch;
static volatile uint32_t gIq8PackFrames;
static volatile uint32_t gIq8PackOverruns;
static volatile uint32_t gIq8ClippedComponents;
#ifdef L3_IQ8_EDMA_PACK
static volatile uint8_t  gIq8PackDetectArm[2];
static volatile uint16_t gIq8PackDetectSlot[2];
static volatile uint32_t gIq8PackDetectEpoch[2];
static volatile uint8_t  gIq8EdmaBusy[2];
static volatile uint32_t gIq8EdmaDone;
static volatile uint32_t gIq8EdmaErrors;
static volatile uint32_t gIq8EdmaWaits;
static uint16_t gIq8FixedScale = 128U;
static uint8_t  gIq8FixedShift = 7U;
#endif
/* Compact IQ16 formats: which scratch holds each slot's processing frame,
 * the processing window it covered, and whether that scratch is fresh. */
static uint8_t  gFrameScratch[L3_MAX_CAPTURE_FRAMES];
static uint8_t  gFrameProcessStart[L3_MAX_CAPTURE_FRAMES];
static uint8_t  gFrameProcessBins[L3_MAX_CAPTURE_FRAMES];
static volatile uint32_t gScratchFrame[2];   /* HWA output count when the scratch last completed */
static volatile uint8_t  gScratchBusy[2];    /* the HWA is writing (or about to write) it */
static volatile uint8_t  gActiveProcessStart;
static volatile uint8_t  gActiveProcessBins;
static volatile uint32_t gDetectScratchStale; /* detect frames whose scratch was reused first */
static volatile uint32_t gCompactFrames;
static volatile uint32_t gCompactErrors;
static volatile uint32_t gCompactMaxUs;
static l3_retain_cfg_t   gRetainCfg;
static uint8_t           gRetainCfgReady;
static l3_retain_window_t gLastRetain;
static l3_frame_desc_t   gFrameDesc[L3_MAX_CAPTURE_FRAMES];
#endif
#endif

/* Config pulled from the CLI mmWave extension (kept off the stack -- large). */
static MMWave_OpenCfg gOpenCfg;
static MMWave_CtrlCfg gCtrlCfg;

/* Forward declarations. */
int32_t l3_cli_dump(int32_t argc, char *argv[]);
int32_t l3_cli_sparse(int32_t argc, char *argv[]);
int32_t l3_cli_track(int32_t argc, char *argv[]);
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[]);
static int32_t l3_cli_sensorStop(int32_t argc, char *argv[]);
static int32_t l3_cli_stats(int32_t argc, char *argv[]);
static int32_t l3_cli_captureCfg(int32_t argc, char *argv[]);
static int32_t l3_cli_phaseCaptureCfg(int32_t argc, char *argv[]);
#ifdef L3_RING_IQ8
static int32_t l3_cli_captureFormat(int32_t argc, char *argv[]);
#ifdef L3_IQ8_EDMA_PACK
static int32_t l3_cli_iq8Scale(int32_t argc, char *argv[]);
#endif
#endif
#ifdef ENABLE_HWA_SMOKE
static int32_t l3_cli_hwaStats(int32_t argc, char *argv[]);
static int32_t l3_cli_hwaTest(int32_t argc, char *argv[]);
static int32_t l3_cli_hwaReal(int32_t argc, char *argv[]);
#endif
static int32_t l3_armCapture(void);
#ifdef HWA_CHAINED_SNAPSHOT_RING
static void l3_hwaRearmTask(UArg arg0, UArg arg1);
static void l3_considerSelfTrigger(uint32_t slot);
static void l3_considerBall(uint32_t slot);
static void l3_considerBallTrack(uint32_t slot);
static void l3_publishDetectFrame(uint32_t slot, uint32_t epoch);
static void l3_resetDetectQueue(void);
static void l3_detectTask(UArg arg0, UArg arg1);
static void l3_noticeTask(UArg arg0, UArg arg1);
static void l3_trigRearm(void);
#endif

static int32_t l3_parseU8(const char *text, uint8_t *value)
{
    char *end = NULL;
    long parsed = strtol(text, &end, 10);

    if (text == end || end == NULL || *end != '\0' || parsed < 0L || parsed > 255L) {
        return -1;
    }
    *value = (uint8_t)parsed;
    return 0;
}

static uint8_t l3_captureUsesIq8(void)
{
#ifdef L3_RING_IQ8
    return gCaptureFormat == L3_CAPTURE_FORMAT_IQ8;
#else
    return 0U;
#endif
}

/* The HWA writes to the IQ16 scratch (not straight into L3) for IQ8 and the
 * compact IQ16 formats. */
static uint8_t l3_captureUsesScratch(void)
{
#ifdef L3_RING_IQ8
    return (uint8_t)(gCaptureFormat != L3_CAPTURE_FORMAT_IQ16);
#else
    return 0U;
#endif
}

/* The ring holds a retained IQ16 window of each processing frame. */
static uint8_t l3_captureCompactsIq16(void)
{
#ifdef L3_RING_IQ8
    return (uint8_t)(gCaptureFormat == L3_CAPTURE_FORMAT_COMPACT16 ||
                     gCaptureFormat == L3_CAPTURE_FORMAT_ADAPTIVE16);
#else
    return 0U;
#endif
}

static const char *l3_captureFormatName(void)
{
#ifdef L3_RING_IQ8
    switch (gCaptureFormat) {
    case L3_CAPTURE_FORMAT_IQ8:
        return "iq8";
    case L3_CAPTURE_FORMAT_COMPACT16:
        return "compact16";
    case L3_CAPTURE_FORMAT_ADAPTIVE16:
        return "adaptive16";
    default:
        return "iq16";
    }
#else
    return "iq16";
#endif
}

#ifdef L3_RING_IQ8
static void l3_ensureRetainCfg(void)
{
    if (!gRetainCfgReady) {
        l3_retain_cfg_defaults(&gRetainCfg);
        gRetainCfgReady = 1U;
    }
}
#endif

static uint32_t l3_captureCapacityBytes(void)
{
    /* Both IQ8 and IQ16 capture use the entire L3 arena: the IQ16 ping/pong
     * scratch that used to be carved out of the tail of g_ring now lives in
     * DATA_RAM (see g_iq16FrameScratch), so there is no smaller IQ8-only
     * capacity to report. */
    return L3_TOTAL_BYTES;
}

static uint32_t l3_captureBytesPerComplex(void)
{
    return l3_captureUsesIq8()
               ? 2U : 2U * (uint32_t)sizeof(int16_t);
}

#ifdef L3_RING_IQ8
static int32_t l3_cli_captureFormat(int32_t argc, char *argv[])
{
    if (gCaptureActive) {
        CLI_write("Error: stop the sensor before captureFormat\n");
        return -1;
    }
    if (argc != 2) {
        CLI_write("Error: captureFormat needs iq16, iq8, compact16 or adaptive16\n");
        return -1;
    }
    if (strcmp(argv[1], "iq16") == 0) {
        gCaptureFormat = L3_CAPTURE_FORMAT_IQ16;
    } else if (strcmp(argv[1], "iq8") == 0) {
        gCaptureFormat = L3_CAPTURE_FORMAT_IQ8;
    } else if (strcmp(argv[1], "compact16") == 0) {
        gCaptureFormat = L3_CAPTURE_FORMAT_COMPACT16;
    } else if (strcmp(argv[1], "adaptive16") == 0) {
        gCaptureFormat = L3_CAPTURE_FORMAT_ADAPTIVE16;
    } else {
        CLI_write("Error: captureFormat needs iq16, iq8, compact16 or adaptive16\n");
        return -1;
    }
    l3_ensureRetainCfg();
    /* compact16 keeps the policy off (centred windows); adaptive16 turns it
     * on. Retain widths default to 16/24/16 until captureCfg retain says. */
    gRetainCfg.enabled = (uint8_t)(gCaptureFormat == L3_CAPTURE_FORMAT_ADAPTIVE16);
    if (l3_captureCompactsIq16() && gCapturePlan.retainPreBins == 0U) {
        gCapturePlan.retainPreBins = 16U;
        gCapturePlan.retainImpactBins = 24U;
        gCapturePlan.retainPostBins = 16U;
    }
    gCapturePlan.preFrames = 0U;
    gCapturePlan.totalFrames = 0U;
    gCapturePlan.usedBytes = 0U;
    CLI_write("Capture format: %s\n", l3_captureFormatName());
    return 0;
}

#ifdef L3_IQ8_EDMA_PACK
static int32_t l3_cli_iq8Scale(int32_t argc, char *argv[])
{
    char *end = NULL;
    long scale;
    uint8_t shift = 0U;
    uint32_t value;

    if (gCaptureActive) {
        CLI_write("Error: stop the sensor before iq8Scale\n");
        return -1;
    }
    if (argc != 2) {
        CLI_write("Error: iq8Scale needs a power of two from 16 to 256\n");
        return -1;
    }
    scale = strtol(argv[1], &end, 10);
    if (argv[1] == end || end == NULL || *end != '\0' || scale < 16L ||
        scale > 256L || (((uint32_t)scale & ((uint32_t)scale - 1U)) != 0U)) {
        CLI_write("Error: iq8Scale needs a power of two from 16 to 256\n");
        return -1;
    }
    value = (uint32_t)scale;
    while (value > 1U) {
        value >>= 1U;
        shift++;
    }
    gIq8FixedScale = (uint16_t)scale;
    gIq8FixedShift = shift;
    CLI_write("IQ8 fixed scale: %u (HWA shift %u)\n",
              (unsigned)gIq8FixedScale, (unsigned)gIq8FixedShift);
    return 0;
}
#endif
#endif

static int32_t l3_finalizeCapturePlan(uint16_t loops)
{
    static const L3CaptureGeometry geometry = {
        N_TX, N_RX, N_SAMPLES, L3_MAX_CAPTURE_FRAMES,
        L3_MAX_LOOPS, L3_MIN_LOOPS, L3_RING_MAX_BINS, L3_MAX_POST_STRIDE
    };
    L3CaptureTables tables = {
        gFrameBinStart, gFrameBinCount, gFrameDeltaUs, gFrameOffset, gFrameBytes
    };
    uint32_t captureBytes = l3_captureCapacityBytes();
    char err[128];

    gCapturePlan.compact = l3_captureCompactsIq16();
    if (l3_captureUsesScratch() &&
        (gCapturePlan.preBins > L3_RING_MAX_BINS || gCapturePlan.postBins > L3_RING_MAX_BINS ||
         (gCapturePlan.phased && gCapturePlan.impactBins > L3_RING_MAX_BINS))) {
        CLI_write("Error: scratch capture windows cannot exceed %u bins\n",
                  (unsigned)L3_RING_MAX_BINS);
        return -1;
    }
#ifdef L3_RING_IQ8
    if (gCapturePlan.compact && gCapturePlan.phased) {
        /* Spend L3 in priority order (l3_retain.h): every impact frame, then
         * the first ball frames, then the last club frames. A request that
         * does not fit is cut rather than refused, and the cut is printed. */
        l3_retain_request_t request;
        l3_retain_budget_t budget;

        memset(&request, 0, sizeof(request));
        request.bytesPerBin = (uint32_t)N_TX * loops * N_RX * l3_captureBytesPerComplex();
        request.capacityBytes = captureBytes;
        request.maxFrames = L3_MAX_CAPTURE_FRAMES;
        request.preBins = gCapturePlan.retainPreBins;
        request.impactBins = gCapturePlan.retainImpactBins;
        request.ballBins = gCapturePlan.retainPostBins;
        request.preFrames = gCapturePlan.requestedPreFrames;
        request.impactFrames = gCapturePlan.impactFrames;
        request.ballFrames = gCapturePlan.ballFrames;
        if (l3_retain_budget(&request, &budget) != 0) {
            CLI_write("Error: the impact frames do not fit L3 at these retain widths\n");
            return -1;
        }
        if (budget.cutPre || budget.cutBall) {
            static char budgetLine[96];

            (void)l3_retain_format_budget(&budget, budgetLine, sizeof(budgetLine));
            CLI_write("Capture %s\n", budgetLine);
            gCapturePlan.requestedPreFrames = budget.preFrames;
            gCapturePlan.ballFrames = budget.ballFrames;
            gCapturePlan.postFrames = (uint8_t)(budget.impactFrames + budget.ballFrames);
        }
    }
#endif
    if (l3plan_build(&gCapturePlan, &geometry, &tables, loops, gFramePeriodUs,
                      captureBytes, l3_captureBytesPerComplex(),
                      err, (uint32_t)sizeof(err)) != 0) {
        CLI_write("%s", err);
        return -1;
    }
    if (gCapturePlan.compact) {
        CLI_write("Capture retain: %s pre=%u impact=%u post=%u bins of the processing windows\n",
                  l3_captureFormatName(), (unsigned)gCapturePlan.retainPreBins,
                  (unsigned)gCapturePlan.retainImpactBins,
                  (unsigned)gCapturePlan.retainPostBins);
    }

    if (gCapturePlan.phased) {
        CLI_write("Capture plan: loops=%u pre=%ux%u@%uus impact=%ux%u@%uus "
                  "ball=%ux%u@%uus frames=%u bytes=%u/%u starts=%u/%u/%u/%u "
                  "stride=%u\n",
                  (unsigned)gCapturePlan.loops,
                  (unsigned)gCapturePlan.preFrames,
                  (unsigned)gCapturePlan.preBins,
                  (unsigned)gFramePeriodUs,
                  (unsigned)gCapturePlan.impactFrames,
                  (unsigned)gCapturePlan.impactBins,
                  (unsigned)gFramePeriodUs,
                  (unsigned)gCapturePlan.ballFrames,
                  (unsigned)gCapturePlan.postBins,
                  (unsigned)(gFramePeriodUs * gCapturePlan.postStride),
                  (unsigned)gCapturePlan.totalFrames,
                  (unsigned)gCapturePlan.usedBytes,
                  (unsigned)captureBytes,
                  (unsigned)gCapturePlan.preStart,
                  (unsigned)gCapturePlan.impactStart,
                  (unsigned)gCapturePlan.postStart,
                  (unsigned)gCapturePlan.lateStart,
                  (unsigned)gCapturePlan.postStride);
    } else {
        CLI_write("Capture plan: loops=%u pre=%ux%u@%uus post=%ux%u@%uus "
                  "frames=%u bytes=%u/%u starts=%u/%u/%u stride=%u\n",
                  (unsigned)gCapturePlan.loops,
                  (unsigned)gCapturePlan.preFrames,
                  (unsigned)gCapturePlan.preBins,
                  (unsigned)gFramePeriodUs,
                  (unsigned)gCapturePlan.postFrames,
                  (unsigned)gCapturePlan.postBins,
                  (unsigned)(gFramePeriodUs * gCapturePlan.postStride),
                  (unsigned)gCapturePlan.totalFrames,
                  (unsigned)gCapturePlan.usedBytes,
                  (unsigned)captureBytes,
                  (unsigned)gCapturePlan.preStart,
                  (unsigned)gCapturePlan.postStart,
                  (unsigned)gCapturePlan.lateStart,
                  (unsigned)gCapturePlan.postStride);
    }
    return 0;
}

/* captureCfg <preStart> <preBins> <postStart> <postBins> <lateStart>
 *            <postFrames> [postStride]
 *
 * The post reservation is fixed by the requested count. All remaining L3 is
 * converted into pre-trigger ring slots after frameCfg supplies the loop count.
 */
/* "captureCfg adaptive <enabled> <approachBins> <marginBins>": follow the
 * locked ball with the capture windows between shots (l3_adaptive.h). */
static int32_t l3_cli_captureCfgAdaptive(int32_t argc, char *argv[])
{
    uint8_t values[3];
    int32_t i;

    if (argc != 5) {
        CLI_write("Error: captureCfg adaptive <enabled> <approachBins> <marginBins>\n");
        return -1;
    }
    for (i = 0; i < 3; i++) {
        if (l3_parseU8(argv[i + 2], &values[i]) != 0) {
            CLI_write("Error: captureCfg adaptive value\n");
            return -1;
        }
    }
    l3_adaptive_cfg_defaults(&gAdaptiveCfg);
    gAdaptiveCfg.enabled = (values[0] != 0U) ? 1U : 0U;
    gAdaptiveCfg.approachBins = values[1];
    gAdaptiveCfg.marginBins = values[2];
    CLI_write("Done\n");
    return 0;
}

/* "captureCfg retain <preBins> <impactBins> <postBins>": the IQ16 bins each
 * slot stores in the compact formats, inside the processing windows.
 * "captureCfg retainPolicy <approachBins> <marginBins> <impactBiasBins>
 * <ballSearchLeadBins> <ballFollowLeadBins> <spinFrames>": l3_retain_cfg_t. */
static int32_t l3_cli_captureCfgRetain(int32_t argc, char *argv[])
{
    uint8_t values[6];
    int32_t i;

    if (gCaptureActive) {
        CLI_write("Error: stop the sensor before captureCfg retain\n");
        return -1;
    }
    if (strcmp(argv[1], "retain") == 0) {
        if (argc != 5) {
            CLI_write("Error: captureCfg retain <preBins> <impactBins> <postBins>\n");
            return -1;
        }
        for (i = 0; i < 3; i++) {
            if (l3_parseU8(argv[i + 2], &values[i]) != 0 || values[i] == 0U ||
                values[i] > L3_RING_MAX_BINS) {
                CLI_write("Error: captureCfg retain widths are 1..%u bins\n",
                          (unsigned)L3_RING_MAX_BINS);
                return -1;
            }
        }
        gCapturePlan.retainPreBins = values[0];
        gCapturePlan.retainImpactBins = values[1];
        gCapturePlan.retainPostBins = values[2];
        gCapturePlan.preFrames = 0U;
        gCapturePlan.totalFrames = 0U;
        gCapturePlan.usedBytes = 0U;
        CLI_write("Done\n");
        return 0;
    }
#ifdef L3_RING_IQ8
    if (argc != 8) {
        CLI_write("Error: captureCfg retainPolicy <approachBins> <marginBins> <impactBiasBins> "
                  "<ballSearchLeadBins> <ballFollowLeadBins> <spinFrames>\n");
        return -1;
    }
    for (i = 0; i < 6; i++) {
        if (l3_parseU8(argv[i + 2], &values[i]) != 0) {
            CLI_write("Error: captureCfg retainPolicy values must be uint8 integers\n");
            return -1;
        }
    }
    l3_ensureRetainCfg();
    {
        l3_retain_cfg_t cfg = gRetainCfg;

        cfg.approachBins = values[0];
        cfg.approachMarginBins = values[1];
        cfg.impactBiasBins = values[2];
        cfg.ballSearchLeadBins = values[3];
        cfg.ballFollowLeadBins = values[4];
        cfg.spinFrames = values[5];
        if (l3_retain_cfg_check(&cfg) != 0) {
            CLI_write("Error: captureCfg retainPolicy (approach 1..64, others <= 32)\n");
            return -1;
        }
        gRetainCfg = cfg;
    }
    CLI_write("Done\n");
    return 0;
#else
    CLI_write("Error: this build has no compact IQ16 formats\n");
    return -1;
#endif
}

static int32_t l3_cli_captureCfg(int32_t argc, char *argv[])
{
    if (argc >= 2 && strcmp(argv[1], "adaptive") == 0) {
        return l3_cli_captureCfgAdaptive(argc, argv);
    }
    if (argc >= 2 && (strcmp(argv[1], "retain") == 0 || strcmp(argv[1], "retainPolicy") == 0)) {
        return l3_cli_captureCfgRetain(argc, argv);
    }
    uint8_t values[7];
    uint32_t valueCount;
    uint32_t i;

    if (gCaptureActive) {
        CLI_write("Error: stop the sensor before captureCfg\n");
        return -1;
    }
    if (argc != 7 && argc != 8) {
        CLI_write("Error: captureCfg needs 6 or 7 values: "
                  "preStart preBins postStart postBins lateStart postFrames "
                  "[postStride]\n");
        return -1;
    }
    valueCount = (uint32_t)argc - 1U;
    for (i = 0U; i < valueCount; i++) {
        if (l3_parseU8(argv[i + 1U], &values[i]) != 0) {
            CLI_write("Error: captureCfg values must be uint8 integers\n");
            return -1;
        }
    }
    if (valueCount == 6U) {
        values[6] = L3_DEFAULT_POST_STRIDE;
    }
    if (values[1] == 0U || values[3] == 0U || values[5] == 0U ||
        values[6] == 0U || values[6] > L3_MAX_POST_STRIDE ||
        values[5] >= L3_MAX_CAPTURE_FRAMES ||
        ((uint32_t)values[0] + values[1]) > N_SAMPLES ||
        ((uint32_t)values[2] + values[3]) > N_SAMPLES ||
        ((uint32_t)values[4] + values[3]) > N_SAMPLES) {
        CLI_write("Error: captureCfg needs valid %u-bin FFT windows and "
                  "1-%u post frames\n",
                  (unsigned)N_SAMPLES,
                  (unsigned)(L3_MAX_CAPTURE_FRAMES - 1U));
        return -1;
    }
    gCapturePlan.preStart = values[0];
    gCapturePlan.preBins = values[1];
    gCapturePlan.postStart = values[2];
    gCapturePlan.postBins = values[3];
    gCapturePlan.lateStart = values[4];
    gCapturePlan.postFrames = values[5];
    gCapturePlan.postStride = values[6];
    gCapturePlan.phased = 0U;
    gCapturePlan.requestedPreFrames = 0U;
    gCapturePlan.impactStart = 0U;
    gCapturePlan.impactBins = 0U;
    gCapturePlan.impactFrames = 0U;
    gCapturePlan.ballFrames = values[5];
    gCapturePlan.preFrames = 0U;
    gCapturePlan.totalFrames = 0U;
    gCapturePlan.usedBytes = 0U;
    return 0;
}

/* phaseCaptureCfg <preStart> <preBins> <preFrames>
 *                 <impactStart> <impactBins> <impactFrames>
 *                 <postStart> <postBins> <lateStart> <ballFrames>
 *                 <ballStride>
 *
 * The pre and impact phases retain every acquisition. The ball phase retains
 * its first acquisition, then every ballStride acquisition. Its first half
 * uses postStart and its second half uses lateStart.
 */
static int32_t l3_cli_phaseCaptureCfg(int32_t argc, char *argv[])
{
    uint8_t values[11];
    uint32_t i;
    uint32_t totalPost;

    if (gCaptureActive) {
        CLI_write("Error: stop the sensor before phaseCaptureCfg\n");
        return -1;
    }
    if (argc != 12) {
        CLI_write("Error: phaseCaptureCfg needs 11 values: preStart preBins "
                  "preFrames impactStart impactBins impactFrames postStart "
                  "postBins lateStart ballFrames ballStride\n");
        return -1;
    }
    for (i = 0U; i < 11U; i++) {
        if (l3_parseU8(argv[i + 1U], &values[i]) != 0) {
            CLI_write("Error: phaseCaptureCfg values must be uint8 integers\n");
            return -1;
        }
    }
    totalPost = (uint32_t)values[5] + values[9];
    if (values[1] == 0U || values[2] == 0U ||
        values[4] == 0U || values[5] == 0U ||
        values[7] == 0U || values[9] == 0U ||
        values[10] == 0U || values[10] > L3_MAX_POST_STRIDE ||
        totalPost >= L3_MAX_CAPTURE_FRAMES ||
        ((uint32_t)values[2] + totalPost) > L3_MAX_CAPTURE_FRAMES ||
        ((uint32_t)values[0] + values[1]) > N_SAMPLES ||
        ((uint32_t)values[3] + values[4]) > N_SAMPLES ||
        ((uint32_t)values[6] + values[7]) > N_SAMPLES ||
        ((uint32_t)values[8] + values[7]) > N_SAMPLES) {
        CLI_write("Error: phaseCaptureCfg needs valid %u-bin windows and "
                  "1-%u total frames\n",
                  (unsigned)N_SAMPLES,
                  (unsigned)(L3_MAX_CAPTURE_FRAMES - 1U));
        return -1;
    }

    gCapturePlan.preStart = values[0];
    gCapturePlan.preBins = values[1];
    gCapturePlan.requestedPreFrames = values[2];
    gCapturePlan.impactStart = values[3];
    gCapturePlan.impactBins = values[4];
    gCapturePlan.impactFrames = values[5];
    gCapturePlan.postStart = values[6];
    gCapturePlan.postBins = values[7];
    gCapturePlan.lateStart = values[8];
    gCapturePlan.ballFrames = values[9];
    gCapturePlan.postFrames = (uint8_t)totalPost;
    gCapturePlan.postStride = values[10];
    gCapturePlan.phased = 1U;
    gCapturePlan.preFrames = 0U;
    gCapturePlan.totalFrames = 0U;
    gCapturePlan.usedBytes = 0U;
    return 0;
}

#ifdef ENABLE_HWA_SMOKE
#define HWA_FFT_SAMPLES      128U
#define HWA_FFT_RX           4U
#define HWA_FFT_TONE_BIN     9U
#define HWA_COMPLEX16_BYTES  4U
#define HWA_MEM_STRIDE       (16U * 1024U)
#define HWA_TEST_MEM0        SOC_XWR68XX_MSS_HWA_MEM0_BASE_ADDRESS
#define HWA_TEST_MEM2        SOC_XWR68XX_MSS_HWA_MEM2_BASE_ADDRESS

static uint32_t l3_log2_u32(uint32_t x)
{
    uint32_t n = 0U;
    while ((1U << n) < x) {
        n++;
    }
    return n;
}

static void l3_hwaDoneCB(void *arg)
{
    (void)arg;
    gHwaDone = 1U;
}

static int32_t l3_hwaConfigFft(void)
{
    HWA_ParamConfig paramCfg;
    HWA_CommonConfig commonCfg;
    int32_t errCode;

    memset((void *)&paramCfg, 0, sizeof(paramCfg));
    paramCfg.triggerMode = HWA_TRIG_MODE_SOFTWARE;
    paramCfg.accelMode = HWA_ACCELMODE_FFT;
    paramCfg.source.srcAddr = 0U;
    paramCfg.source.srcAcnt = HWA_FFT_SAMPLES - 1U;
    paramCfg.source.srcAIdx = HWA_FFT_RX * HWA_COMPLEX16_BYTES;
    paramCfg.source.srcBcnt = HWA_FFT_RX - 1U;
    paramCfg.source.srcBIdx = HWA_COMPLEX16_BYTES;
    paramCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    paramCfg.source.srcConjugate = HWA_FEATURE_BIT_DISABLE;
    paramCfg.source.srcScale = 0U;
    paramCfg.source.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfg.dest.dstAddr = (uint16_t)(HWA_TEST_MEM2 - HWA_TEST_MEM0);
    paramCfg.dest.dstAcnt = HWA_FFT_SAMPLES - 1U;
    paramCfg.dest.dstAIdx = HWA_FFT_RX * HWA_COMPLEX16_BYTES;
    paramCfg.dest.dstBIdx = HWA_COMPLEX16_BYTES;
    paramCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    paramCfg.dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;
    paramCfg.dest.dstScale = 0U;
    paramCfg.dest.dstSkipInit = 0U;
    paramCfg.accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    paramCfg.accelModeArgs.fftMode.fftSize = (uint8_t)l3_log2_u32(HWA_FFT_SAMPLES);
    paramCfg.accelModeArgs.fftMode.butterflyScaling = 0x7FU;
    paramCfg.accelModeArgs.fftMode.interfZeroOutEn = HWA_FEATURE_BIT_DISABLE;
    paramCfg.accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE;
    paramCfg.accelModeArgs.fftMode.windowStart = 0U;
    paramCfg.accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC;
    paramCfg.accelModeArgs.fftMode.winInterpolateMode = HWA_FFT_WINDOW_INTERPOLATE_MODE_NONE;
    paramCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    paramCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    paramCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    errCode = HWA_configParamSet(gHwaHandle, 0U, &paramCfg, NULL);
    if (errCode != 0) {
        return errCode;
    }

    memset((void *)&commonCfg, 0, sizeof(commonCfg));
    commonCfg.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                           HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                           HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                           HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                           HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
    commonCfg.numLoops = 1U;
    commonCfg.paramStartIdx = 0U;
    commonCfg.paramStopIdx = 0U;
    commonCfg.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    commonCfg.fftConfig.interferenceThreshold = 0xFFFFFFU;
    errCode = HWA_configCommon(gHwaHandle, &commonCfg);
    if (errCode != 0) {
        return errCode;
    }

    return 0;
}

static int32_t l3_hwaRunFft(uint16_t *peakBin, uint32_t *peakPower)
{
    int16_t *dst = (int16_t *)HWA_TEST_MEM2;
    uint32_t i, wait;
    int32_t errCode;

    *peakBin = 0U;
    *peakPower = 0U;

    memset((void *)dst, 0, HWA_MEM_STRIDE);
    errCode = l3_hwaConfigFft();
    if (errCode != 0) {
        return errCode;
    }

    gHwaDone = 0U;
    errCode = HWA_enableDoneInterrupt(gHwaHandle, l3_hwaDoneCB, NULL);
    if (errCode != 0) {
        return errCode;
    }
    errCode = HWA_enable(gHwaHandle, 1U);
    if (errCode != 0) {
        (void)HWA_disableDoneInterrupt(gHwaHandle);
        return errCode;
    }
    errCode = HWA_reset(gHwaHandle);
    if (errCode != 0) {
        (void)HWA_enable(gHwaHandle, 0U);
        (void)HWA_disableDoneInterrupt(gHwaHandle);
        return errCode;
    }
    errCode = HWA_setSoftwareTrigger(gHwaHandle);
    if (errCode != 0) {
        (void)HWA_enable(gHwaHandle, 0U);
        (void)HWA_disableDoneInterrupt(gHwaHandle);
        return errCode;
    }
    for (wait = 0U; wait < 200U && !gHwaDone; wait++) {
        Task_sleep(1);
    }
    (void)HWA_enable(gHwaHandle, 0U);
    (void)HWA_disableDoneInterrupt(gHwaHandle);
    if (!gHwaDone) {
        return -102;
    }

    for (i = 0U; i < HWA_FFT_SAMPLES; i++) {
        int32_t im = (int32_t)dst[((i * HWA_FFT_RX) * 2U) + 0U];
        int32_t re = (int32_t)dst[((i * HWA_FFT_RX) * 2U) + 1U];
        uint32_t p = (uint32_t)((im * im) + (re * re));
        if (p > *peakPower) {
            *peakPower = p;
            *peakBin = (uint16_t)i;
        }
    }
    return 0;
}

#if defined(SNAPSHOT_DUMP) && !defined(HWA_CHAINED_SNAPSHOT_RING)
static int32_t l3_snapshotChirpToBuffer(const int16_t *rawChirp, int16_t *out)
{
    int16_t *src = (int16_t *)HWA_TEST_MEM0;
    int16_t *dst = (int16_t *)HWA_TEST_MEM2;
    uint16_t peakBin;
    uint32_t peakPower;
    uint32_t sample, rx, bin, outWord;
    int32_t errCode;

    memset((void *)src, 0, HWA_MEM_STRIDE);
    for (sample = 0U; sample < HWA_FFT_SAMPLES; sample++) {
        for (rx = 0U; rx < HWA_FFT_RX; rx++) {
            uint32_t rawWord = ((rx * N_SAMPLES) + sample) * 2U;
            uint32_t hwaWord = ((sample * HWA_FFT_RX) + rx) * 2U;
            src[hwaWord + 0U] = rawChirp[rawWord + 0U];
            src[hwaWord + 1U] = rawChirp[rawWord + 1U];
        }
    }

    errCode = l3_hwaRunFft(&peakBin, &peakPower);
    if (errCode != 0) {
        return errCode;
    }

    outWord = 0U;
    for (rx = 0U; rx < N_RX; rx++) {
        for (bin = 0U; bin < SNAPSHOT_BINS; bin++) {
            uint32_t srcBin = SNAPSHOT_BIN_START + bin;
            uint32_t hwaWord = ((srcBin * HWA_FFT_RX) + rx) * 2U;
            out[outWord++] = dst[hwaWord + 0U];
            out[outWord++] = dst[hwaWord + 1U];
        }
    }
    return 0;
}

static int32_t l3_emitSnapshotChirp(const int16_t *rawChirp)
{
    int16_t out[N_RX * SNAPSHOT_BINS * 2U];
    int32_t errCode;

    errCode = l3_snapshotChirpToBuffer(rawChirp, out);
    if (errCode != 0) {
        return errCode;
    }
    UART_writePolling(gDataUart, (uint8_t *)out, sizeof(out));
    return 0;
}
#endif
#endif

#ifdef HWA_CHAINED_SNAPSHOT_RING
/* True when the post-trigger movie is complete and capture may stop.
 * CALLER MUST HOLD THE CRITICAL SECTION: every call site is already inside
 * Hwi_disable(), and the globals it reads are written from the EDMA and HWA
 * completion callbacks. */
static inline uint8_t l3_shouldFreezeNow(void)
{
    return (uint8_t)(gHwaFreezeRequested && gActiveFrameIsPost &&
                     gActiveFrameShouldKeep &&
                     gPostFramesCaptured >= gCapturePlan.postFrames);
}

static void l3_hwaMaybeQueueRearm(void)
{
    uintptr_t key;
    uint8_t queue = 0U;
    uint8_t freeze = 0U;

    key = Hwi_disable();
    if (gCaptureActive && gHwaDoneSeen && gHwaOutputSeen && !gHwaRearmPending) {
        if (gHwaShutdownRequested) {
#if defined(L3_RING_IQ8)
            if (l3_captureUsesScratch()) {
                /* Let the task pack (or compact) the completed scratch frame
                 * before acknowledging the shutdown boundary. */
                gHwaRearmPending = 1U;
                queue = 1U;
            } else
#endif
            {
                gCaptureActive = 0U;
                gHwaShutdownRequested = 0U;
                freeze = 1U;
            }
        } else {
#ifdef L3_RING_IQ8
        if (l3_captureUsesScratch()) {
            /* The completed IQ16 scratch frame must be packed or compacted
             * before scratch can be reused, including the final retained
             * post frame. */
            if (gHwaFreezeRequested && !gActiveFrameIsPost) {
                gPostCaptureStarted = 1U;
            }
            /* This arm queues unconditionally, unlike the IQ16 arm below, which
             * gates queuing on l3_shouldFreezeNow(). That is deliberate: for IQ8
             * the completed scratch frame must still be packed by l3_hwaRearmTask
             * before it can be reused, even on the frame that satisfies the
             * freeze condition, so the freeze decision itself is deferred to that
             * task rather than made here. l3_hwaRearmTask re-evaluates
             * l3_shouldFreezeNow() after packing (l3_dump.c:1944) and freezes
             * there when it holds. Do not collapse these two bodies. */
            gHwaRearmPending = 1U;
            queue = 1U;
        } else if (l3_shouldFreezeNow()) {
            gCaptureActive = 0U;
            gHwaFreezeRequested = 0U;
            gHwaFreezeCompletions++;
            freeze = 1U;
        } else {
            if (gHwaFreezeRequested && !gActiveFrameIsPost) {
                gPostCaptureStarted = 1U;
            }
            gHwaRearmPending = 1U;
            queue = 1U;
        }
#else
        if (l3_shouldFreezeNow()) {
            gCaptureActive = 0U;
            gHwaFreezeRequested = 0U;
            gHwaFreezeCompletions++;
            freeze = 1U;
        } else {
            if (gHwaFreezeRequested && !gActiveFrameIsPost) {
                gPostCaptureStarted = 1U;
            }
            gHwaRearmPending = 1U;
            queue = 1U;
        }
#endif
        }
    }
    if (queue) {
        gHwaRearmQueuedCycles = Cycleprofiler_getTimeStamp();
        gHwaRearmQueuedValid = 1U;
    }
    Hwi_restore(key);
    if (freeze && gHwaFreezeSemaphore != NULL) {
        Semaphore_post(gHwaFreezeSemaphore);
    } else if (queue && gHwaRearmSemaphore != NULL) {
        Semaphore_post(gHwaRearmSemaphore);
    }
}

static void l3_hwaChainDoneCB(void *arg)
{
    (void)arg;
    gHwaFrameDone++;
    gHwaDoneSeen = 1U;
    l3_hwaMaybeQueueRearm();
}

#if defined(HWA_CHAINED_SNAPSHOT_RING)
static void l3_publishDetectFrame(uint32_t slot, uint32_t epoch)
{
    if (gCapturePlan.preFrames == 0U) {
        return;
    }
    if (l3detect_publish(&gDetectQueue, (uint16_t)slot, epoch) != 0) {
        return;
    }
    if (gDetectSemaphore != NULL) {
        Semaphore_post(gDetectSemaphore);
    }
}

static void l3_resetDetectQueue(void)
{
    l3detect_init(&gDetectQueue);
    gDetectStale = 0U;
    if (gDetectSemaphore != NULL) {
        while (Semaphore_pend(gDetectSemaphore, BIOS_NO_WAIT)) {
        }
    }
}
#endif

static void l3_hwaOutputDoneCB(uintptr_t arg, uint8_t tcCode)
{
    (void)arg;
    (void)tcCode;
    gHwaOutputDone++;
    gRingFrame++;
#ifdef L3_RING_IQ8
    if (l3_captureUsesScratch() && gActiveFrameShouldKeep) {
        uint32_t completedSlot = gActiveFrameIsPost
                                     ? gCapturePlan.preFrames + gPostFramesCaptured
                                     : gPreFramesCaptured % gCapturePlan.preFrames;
        if (gIq8Pending) {
            gIq8PackOverruns++;
        }
        gIq8PendingSlot = completedSlot;
        gIq8PendingScratch = gIq8ActiveScratch;
        gIq8Pending = 1U;
        if (l3_captureCompactsIq16()) {
            /* The detect task reads this scratch frame directly, at full
             * precision, until the HWA is aimed at the scratch again; the
             * compaction into L3 happens on the rearm task meanwhile. */
            gFrameScratch[completedSlot] = gIq8ActiveScratch;
            gFrameProcessStart[completedSlot] = gActiveProcessStart;
            gFrameProcessBins[completedSlot] = gActiveProcessBins;
            gScratchFrame[gIq8ActiveScratch] = gHwaOutputDone;
            gScratchBusy[gIq8ActiveScratch] = 0U;
        } else if (gActiveFrameIsPost) {
            /* Kept IQ8 post frames reach the ball tracker once packed. */
            gIq8PendingDetect = 1U;
            gIq8PendingEpoch = L3_DETECT_POST_EPOCH;
        }
    }
#endif
    if (gActiveFrameIsPost) {
        gPostFramesObserved++;
        if (gActiveFrameShouldKeep) {
            uint32_t completedPostSlot = gCapturePlan.preFrames + gPostFramesCaptured;

            gPostFramesCaptured++;
#if defined(L3_RING_IQ8)
            if (!l3_captureUsesIq8())  /* IQ8 frames publish after packing */
#endif
            {
                /* The ball tracker reads kept post frames as they land. */
                l3_publishDetectFrame(completedPostSlot, L3_DETECT_POST_EPOCH);
            }
        }
    } else {
        uint32_t completedPreSlot = gPreFramesCaptured % gCapturePlan.preFrames;

        gPreFramesCaptured++;
        if ((gPreFramesCaptured % gCapturePlan.preFrames) == 0U) {
            gNumWrap++;
        }
#if defined(L3_RING_IQ8)
        if (l3_captureUsesIq8()) {
            if (gActiveFrameShouldKeep) {
                gIq8PendingEpoch = gPreFramesCaptured;
                gIq8PendingDetect = 1U;
            }
        } else
#endif
        {
            l3_publishDetectFrame(completedPreSlot, gPreFramesCaptured);
        }
    }
    gHwaOutputSeen = 1U;
    l3_hwaMaybeQueueRearm();
}

static int32_t l3_hwaStartRing(void)
{
    int32_t errCode;

    errCode = HWA_enable(gHwaHandle, 1U);
    if (errCode != 0) {
        return errCode;
    }
    errCode = HWA_setDMA2ACCManualTrig(gHwaHandle, L3_HWA_PARAM_DUMMY_PING);
    if (errCode != 0) {
        (void)HWA_enable(gHwaHandle, 0U);
        return errCode;
    }
    errCode = HWA_setDMA2ACCManualTrig(gHwaHandle, L3_HWA_PARAM_DUMMY_PONG);
    if (errCode != 0) {
        (void)HWA_enable(gHwaHandle, 0U);
        return errCode;
    }
    gHwaArmedForFrame = 1U;
    return 0;
}

static int32_t l3_configHwaProcessParam(uint8_t paramIdx, uint8_t outChannel,
                                        uint16_t dstAddr)
{
    HWA_ParamConfig paramCfg;
    HWA_InterruptConfig interruptCfg;
    uint8_t dmaChannel;
    int32_t errCode;

    memset((void *)&paramCfg, 0, sizeof(paramCfg));
    paramCfg.triggerMode = HWA_TRIG_MODE_DFE;
    paramCfg.accelMode = HWA_ACCELMODE_FFT;
    paramCfg.source.srcAddr = 0U;
    paramCfg.source.srcAcnt = N_SAMPLES - 1U;
    paramCfg.source.srcAIdx = HWA_COMPLEX16_BYTES;
    paramCfg.source.srcBcnt = N_RX - 1U;
    paramCfg.source.srcBIdx = N_SAMPLES * HWA_COMPLEX16_BYTES;
    paramCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    paramCfg.source.srcConjugate = HWA_FEATURE_BIT_DISABLE;
    paramCfg.source.srcScale = 0U;
    paramCfg.source.bpmEnable = HWA_FEATURE_BIT_DISABLE;

    /* RX-major output keeps each compact frame contiguous. EDMA crops the
     * configured window from each of the four 128-bin RX blocks. */
    paramCfg.dest.dstAddr = dstAddr;
    paramCfg.dest.dstAcnt = N_SAMPLES - 1U;
    paramCfg.dest.dstAIdx = HWA_COMPLEX16_BYTES;
    paramCfg.dest.dstBIdx = N_SAMPLES * HWA_COMPLEX16_BYTES;
    paramCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    paramCfg.dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;
#ifdef L3_RING_IQ8
    /* EDMA compaction copies the low byte of each signed HWA result, so the
     * configurable HWA shift performs the quantization before that copy. */
#ifdef L3_IQ8_EDMA_PACK
    paramCfg.dest.dstScale =
        l3_captureUsesIq8() ? gIq8FixedShift : 0U;
#else
    paramCfg.dest.dstScale =
        l3_captureUsesIq8() ? L3_IQ8_HWA_SHIFT : 0U;
#endif
#else
    paramCfg.dest.dstScale = 0U;
#endif
    paramCfg.dest.dstSkipInit = 0U;
    paramCfg.accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    paramCfg.accelModeArgs.fftMode.fftSize = (uint8_t)l3_log2_u32(N_SAMPLES);
    paramCfg.accelModeArgs.fftMode.butterflyScaling = 0x7FU;
    paramCfg.accelModeArgs.fftMode.interfZeroOutEn = HWA_FEATURE_BIT_DISABLE;
    paramCfg.accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE;
    paramCfg.accelModeArgs.fftMode.windowStart = 0U;
    paramCfg.accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC;
    paramCfg.accelModeArgs.fftMode.winInterpolateMode = HWA_FFT_WINDOW_INTERPOLATE_MODE_NONE;
    paramCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    paramCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    paramCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    errCode = HWA_configParamSet(gHwaHandle, paramIdx, &paramCfg, NULL);
    if (errCode != 0) {
        return errCode;
    }
    errCode = HWA_getDMAChanIndex(gHwaHandle, outChannel, &dmaChannel);
    if (errCode != 0) {
        return errCode;
    }
    memset((void *)&interruptCfg, 0, sizeof(interruptCfg));
    interruptCfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    interruptCfg.dma.dstChannel = dmaChannel;
    return HWA_enableParamSetInterrupt(gHwaHandle, paramIdx, &interruptCfg);
}

static int32_t l3_configHwaOutputEdma(uint8_t channel, uint16_t shadow,
                                      uint32_t source, uint32_t destination,
                                      uintptr_t callbackArg, uint16_t binCount)
{
    EDMA_channelConfig_t channelCfg;
    EDMA_paramConfig_t shadowCfg;
    EDMA_paramSetConfig_t *param;

    memset((void *)&channelCfg, 0, sizeof(channelCfg));
    channelCfg.channelId = channel;
    channelCfg.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    channelCfg.paramId = channel;
    channelCfg.eventQueueId = 0U;
    channelCfg.transferCompletionCallbackFxn =
        (callbackArg != 0U) ? l3_hwaOutputDoneCB : NULL;
    channelCfg.transferCompletionCallbackFxnArg = callbackArg;

    param = &channelCfg.paramSetConfig;
    param->sourceAddress = SOC_translateAddress(source, SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->destinationAddress = SOC_translateAddress(destination, SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->aCount = (uint16_t)(binCount * HWA_COMPLEX16_BYTES);
    param->bCount = (uint16_t)N_RX;
    param->cCount =
        (uint16_t)(gCapturePlan.chirpsPerFrame / 2U);
    param->bCountReload = (uint16_t)N_RX;
    param->sourceBindex = (int16_t)(N_SAMPLES * HWA_COMPLEX16_BYTES);
    param->destinationBindex = (int16_t)(binCount * HWA_COMPLEX16_BYTES);
    param->sourceCindex = 0;
    param->destinationCindex =
        (int16_t)(2U * N_RX * binCount * HWA_COMPLEX16_BYTES);
    param->linkAddress = EDMA_NULL_LINK_ADDRESS;
    param->transferCompletionCode = L3_HWA_SIGNATURE_CHANNEL;
    param->transferType = (uint8_t)EDMA3_SYNC_AB;
    param->sourceAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->destinationAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->fifoWidth = (uint8_t)EDMA3_FIFO_WIDTH_8BIT;
    param->isStaticSet = false;
    param->isEarlyCompletion = false;
    param->isFinalTransferInterruptEnabled = (callbackArg != 0U);
    param->isIntermediateTransferInterruptEnabled = false;
    param->isFinalChainingEnabled = true;
    param->isIntermediateChainingEnabled = true;

    if (EDMA_configChannel(gEdmaHandle, &channelCfg, true) != EDMA_NO_ERROR) {
        return -1;
    }
    memset((void *)&shadowCfg, 0, sizeof(shadowCfg));
    memcpy((void *)&shadowCfg.paramSetConfig, (void *)param, sizeof(*param));
    shadowCfg.transferCompletionCallbackFxn =
        (callbackArg != 0U) ? l3_hwaOutputDoneCB : NULL;
    shadowCfg.transferCompletionCallbackFxnArg = callbackArg;
    if (EDMA_configParamSet(gEdmaHandle, shadow, &shadowCfg) != EDMA_NO_ERROR ||
        EDMA_linkParamSets(gEdmaHandle, channel, shadow) != EDMA_NO_ERROR ||
        EDMA_linkParamSets(gEdmaHandle, shadow, shadow) != EDMA_NO_ERROR) {
        return -1;
    }
    return 0;
}

static int32_t l3_configHwaSignatureEdma(void)
{
    HWA_SrcDMAConfig trigger[2];
    EDMA_channelConfig_t channelCfg;
    EDMA_paramConfig_t shadowCfg;
    EDMA_paramSetConfig_t *param;

    if (HWA_getDMAconfig(gHwaHandle, L3_HWA_PARAM_DUMMY_PING, &trigger[0]) != 0 ||
        HWA_getDMAconfig(gHwaHandle, L3_HWA_PARAM_DUMMY_PONG, &trigger[1]) != 0) {
        return -1;
    }
    memset((void *)&channelCfg, 0, sizeof(channelCfg));
    channelCfg.channelId = L3_HWA_SIGNATURE_CHANNEL;
    channelCfg.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    channelCfg.paramId = L3_HWA_SIGNATURE_CHANNEL;
    channelCfg.eventQueueId = 0U;
    param = &channelCfg.paramSetConfig;
    param->sourceAddress = SOC_translateAddress(trigger[0].srcAddr,
                                                SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->destinationAddress = SOC_translateAddress(trigger[0].destAddr,
                                                     SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->aCount = trigger[0].aCnt;
    param->bCount = (uint16_t)(trigger[0].bCnt * 2U);
    param->cCount = trigger[0].cCnt;
    param->bCountReload = param->bCount;
    param->sourceBindex = (int16_t)(trigger[1].srcAddr - trigger[0].srcAddr);
    param->destinationBindex = 0;
    param->sourceCindex = 0;
    param->destinationCindex = 0;
    param->linkAddress = EDMA_NULL_LINK_ADDRESS;
    param->transferCompletionCode = 0U;
    param->transferType = (uint8_t)EDMA3_SYNC_A;
    param->sourceAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->destinationAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->fifoWidth = (uint8_t)EDMA3_FIFO_WIDTH_8BIT;
    param->isStaticSet = false;
    param->isEarlyCompletion = false;
    if (EDMA_configChannel(gEdmaHandle, &channelCfg, false) != EDMA_NO_ERROR) {
        return -1;
    }
    memset((void *)&shadowCfg, 0, sizeof(shadowCfg));
    memcpy((void *)&shadowCfg.paramSetConfig, (void *)param, sizeof(*param));
    if (EDMA_configParamSet(gEdmaHandle, L3_HWA_SIGNATURE_SHADOW, &shadowCfg) != EDMA_NO_ERROR ||
        EDMA_linkParamSets(gEdmaHandle, L3_HWA_SIGNATURE_CHANNEL,
                          L3_HWA_SIGNATURE_SHADOW) != EDMA_NO_ERROR ||
        EDMA_linkParamSets(gEdmaHandle, L3_HWA_SIGNATURE_SHADOW,
                          L3_HWA_SIGNATURE_SHADOW) != EDMA_NO_ERROR) {
        return -1;
    }
    return 0;
}

static int32_t l3_configHwaCommon(void)
{
    HWA_CommonConfig commonCfg;

    memset((void *)&commonCfg, 0, sizeof(commonCfg));
    commonCfg.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                           HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                           HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                           HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                           HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
    commonCfg.numLoops =
        gCapturePlan.chirpsPerFrame / 2U;
    commonCfg.paramStartIdx = L3_HWA_PARAM_DUMMY_PING;
    commonCfg.paramStopIdx = L3_HWA_PARAM_FFT_PONG;
    commonCfg.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    commonCfg.fftConfig.interferenceThreshold = 0xFFFFFFU;
    return HWA_configCommon(gHwaHandle, &commonCfg);
}

#ifdef L3_DUMP_IQ8
/* Dump-time quantisation: l3_iq8_quantize_scale (l3_iq8.c) is the one
 * definition, shared with the host emulator. */
#endif

#ifdef L3_RING_IQ8
#ifndef L3_IQ8_EDMA_PACK
/* The pack shift and the shift quantiser are l3_iq8_pack_shift and
 * l3_iq8_quantize_shift (l3_iq8.c): the host emulator compiles the same
 * arithmetic, so an offline IQ8 is the board's IQ8. */
static void l3_packIq8CompletedFrame(uint32_t slot, uint8_t scratch)
{
    const int16_t *source = &g_iq16FrameScratch[scratch][0];
    int8_t *destination = (int8_t *)&g_ring[gFrameOffset[slot]];
    uint32_t components = gFrameBytes[slot];
    uint32_t component;
    uint32_t clippedComponents = 0U;
#ifdef L3_IQ8_SPARSE_SCALE
    uint8_t packShift = l3_iq8_pack_shift(source, components, L3_IQ8_SCALE_COMPLEX_STRIDE);
#else
    uint8_t packShift = l3_iq8_pack_shift(source, components, 1U);
#endif

    for (component = 0U; component < components; component++) {
        destination[component] =
            l3_iq8_quantize_shift(source[component], packShift, &clippedComponents);
    }
    gIq8ClippedComponents += clippedComponents;
    gFrameIq8Scale[slot] =
        (uint16_t)(L3_IQ8_HWA_SCALE << packShift);
    gIq8PackFrames++;
}
#else
static void l3_iq8EdmaDoneCB(uintptr_t arg, uint8_t tcCode)
{
    uint8_t scratch = (uint8_t)(arg - 1U);

    (void)tcCode;
    if (scratch < 2U) {
        gIq8EdmaBusy[scratch] = 0U;
        gIq8EdmaDone++;
        gIq8PackFrames++;
        if (gIq8PackDetectArm[scratch] != 0U) {
            uint32_t packedSlot = gIq8PackDetectSlot[scratch];
            uint32_t packedEpoch = gIq8PackDetectEpoch[scratch];

            gIq8PackDetectArm[scratch] = 0U;
            l3_publishDetectFrame(packedSlot, packedEpoch);
        }
    } else {
        gIq8EdmaErrors++;
    }
    if (gIq8EdmaDoneSemaphore != NULL) {
        Semaphore_post(gIq8EdmaDoneSemaphore);
    }
}

static int32_t l3_startIq8EdmaPack(uint32_t slot, uint8_t scratch)
{
    EDMA_channelConfig_t channelCfg;
    EDMA_paramSetConfig_t *param;
    uint8_t channel;
    uint32_t components;
    int32_t errCode;

    if (slot >= gCapturePlan.totalFrames || scratch >= 2U) {
        return -1;
    }
    components = gFrameBytes[slot];
    if (components == 0U || components > 0xFFFFU) {
        return -1;
    }
    channel = scratch == 0U ? L3_IQ8_PACK_PING_CHANNEL
                            : L3_IQ8_PACK_PONG_CHANNEL;

    memset((void *)&channelCfg, 0, sizeof(channelCfg));
    channelCfg.channelId = channel;
    channelCfg.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    channelCfg.paramId = channel;
    channelCfg.eventQueueId = 1U;
    channelCfg.transferCompletionCallbackFxn = l3_iq8EdmaDoneCB;
    channelCfg.transferCompletionCallbackFxnArg = (uintptr_t)(scratch + 1U);

    param = &channelCfg.paramSetConfig;
    param->sourceAddress = SOC_translateAddress(
        (uint32_t)&g_iq16FrameScratch[scratch][0],
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->destinationAddress = SOC_translateAddress(
        (uint32_t)&g_ring[gFrameOffset[slot]],
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    param->aCount = 1U;
    param->bCount = (uint16_t)components;
    param->cCount = 1U;
    param->bCountReload = (uint16_t)components;
    param->sourceBindex = (int16_t)sizeof(int16_t);
    param->destinationBindex = 1;
    param->sourceCindex = 0;
    param->destinationCindex = 0;
    param->linkAddress = EDMA_NULL_LINK_ADDRESS;
    param->transferCompletionCode = channel;
    param->transferType = (uint8_t)EDMA3_SYNC_AB;
    param->sourceAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->destinationAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    param->fifoWidth = (uint8_t)EDMA3_FIFO_WIDTH_8BIT;
    param->isStaticSet = true;
    param->isEarlyCompletion = false;
    param->isFinalTransferInterruptEnabled = true;
    param->isIntermediateTransferInterruptEnabled = false;
    param->isFinalChainingEnabled = false;
    param->isIntermediateChainingEnabled = false;

    (void)EDMA_disableChannel(gEdmaHandle, channel, EDMA3_CHANNEL_TYPE_DMA);
    gFrameIq8Scale[slot] = gIq8FixedScale;
    gIq8EdmaBusy[scratch] = 1U;
    errCode = EDMA_configChannel(gEdmaHandle, &channelCfg, false);
    if (errCode == EDMA_NO_ERROR) {
        errCode = EDMA_startDmaTransfer(gEdmaHandle, channel);
    }
    if (errCode != EDMA_NO_ERROR) {
        gIq8EdmaBusy[scratch] = 0U;
        gIq8PackDetectArm[scratch] = 0U;
        gIq8EdmaErrors++;
        return -1;
    }
    return 0;
}

static void l3_waitForIq8EdmaScratch(uint8_t scratch)
{
    uint8_t waited = 0U;

    while (scratch < 2U && gIq8EdmaBusy[scratch]) {
        if (!waited) {
            gIq8EdmaWaits++;
            waited = 1U;
        }
        Semaphore_pend(gIq8EdmaDoneSemaphore, BIOS_WAIT_FOREVER);
    }
}

static void l3_waitForAllIq8Edma(void)
{
    l3_waitForIq8EdmaScratch(0U);
    l3_waitForIq8EdmaScratch(1U);
}
#endif
#endif

#ifdef L3_RING_IQ8
/* What the retention policy needs to know when a frame completes: the shot
 * machine's state and the trackers' predictions for the coming frame. Read
 * on the rearm task while the detect task may be updating them: a bin off
 * in the window's placement at worst, never a wrong frame. */
static void l3_retainState(uint32_t slot, l3_retain_state_t *state)
{
    uint32_t ballBin = gTrigCfg.teeBin;

    memset(state, 0, sizeof(*state));
    state->shotState = gShot.state;
    if (l3_ball_locked(&gBall, &ballBin)) {
        state->ballLocked = 1U;
    } else {
        ballBin = gTrigCfg.teeBin;
    }
    state->ballBin = (float)ballBin;
    state->clubActive = gClubTrack.active;
    state->clubBin = l3_retain_predict(gClubTrack.lastBin, gClubTrack.velocityBinsPerFrame);
    if (slot >= gCapturePlan.preFrames) {
        state->postFrame = 1U;
        state->postIndex = slot - gCapturePlan.preFrames;
    }
    state->ballTrackConfirmed = gBallTrack.confirmed;
    state->ballTrackBin =
        l3_retain_predict(gBallTrack.core.lastBin, gBallTrack.core.velocityBinsPerFrame);
}

/* Copy the retained IQ16 window of a completed processing frame from its
 * scratch into the slot, and record what the slot now holds. The slot's
 * width is the plan's per-phase retain width; only the start moves. */
static void l3_compactCompletedFrame(uint32_t slot, uint8_t scratch)
{
    l3_retain_state_t state;
    l3_retain_window_t window;
    l3_frame_desc_t *desc;
    uint32_t processStart;
    uint32_t processBins;
    uint32_t ticks = Cycleprofiler_getTimeStamp();
    uint32_t elapsedUs;

    if (slot >= gCapturePlan.totalFrames || scratch >= 2U) {
        gCompactErrors++;
        return;
    }
    processStart = gFrameProcessStart[slot];
    processBins = gFrameProcessBins[slot];
    l3_ensureRetainCfg();
    l3_retainState(slot, &state);
    l3_retain_window(&gRetainCfg, &state, processStart, processBins, gFrameBinCount[slot],
                     &window);
    if (l3_compact_iq16(&g_iq16FrameScratch[scratch][0],
                        (int16_t *)(void *)&g_ring[gFrameOffset[slot]],
                        gCapturePlan.chirpsPerFrame, N_RX, (uint16_t)processBins,
                        (uint16_t)(window.start - processStart), window.bins) != 0) {
        gCompactErrors++;
        return;
    }
    gFrameBinStart[slot] = window.start;
    gFrameBinCount[slot] = window.bins;
    gLastRetain = window;
    desc = &gFrameDesc[slot];
    desc->frame = (uint16_t)(state.postFrame ? gPreFramesCaptured + state.postIndex + 1U
                                              : gPreFramesCaptured);
    desc->timestampUs = (uint32_t)desc->frame * gFramePeriodUs;
    desc->dataOffset = gFrameOffset[slot];
    desc->bytes = gFrameBytes[slot];
    desc->globalBinStart = window.start;
    desc->binCount = window.bins;
    desc->processStart = (uint8_t)processStart;
    desc->processBins = (uint8_t)processBins;
    desc->shotState = state.shotState;
    desc->priority = window.priority;
    desc->why = window.why;
    desc->isPost = state.postFrame;
    gCompactFrames++;
    elapsedUs = (Cycleprofiler_getTimeStamp() - ticks) / (gCpuClock / 1000000U);
    if (elapsedUs > gCompactMaxUs) {
        gCompactMaxUs = elapsedUs;
    }
}

/* A completed scratch frame into L3: IQ8 packs it, the compact formats keep
 * the retained IQ16 window. */
static void l3_storeCompletedFrame(uint32_t slot, uint8_t scratch)
{
    if (l3_captureCompactsIq16()) {
        l3_compactCompletedFrame(slot, scratch);
        return;
    }
#ifdef L3_IQ8_EDMA_PACK
    (void)l3_startIq8EdmaPack(slot, scratch);
#else
    l3_packIq8CompletedFrame(slot, scratch);
#endif
}
#endif

static uint32_t l3_snapshotBinStartForNextFrame(void)
{
    if (!gPostCaptureStarted) {
        return gCapturePlan.preStart;
    }
    if (gCapturePlan.phased &&
        gPostFramesCaptured < gCapturePlan.impactFrames) {
        return gCapturePlan.impactStart;
    }
    if ((!gCapturePlan.phased &&
         gPostFramesCaptured < (gCapturePlan.postFrames / 2U)) ||
        (gCapturePlan.phased &&
         (gPostFramesCaptured - gCapturePlan.impactFrames) <
             (gCapturePlan.ballFrames / 2U))) {
        return gCapturePlan.postStart;
    }
    return gCapturePlan.lateStart;
}

static int32_t l3_configHwaFrameOutput(uint32_t ringSlot)
{
    uint32_t binStart = l3_snapshotBinStartForNextFrame();
    uint16_t binCount;
    uint32_t destination;
    uint32_t pingSource = SOC_XWR68XX_MSS_HWA_MEM2_BASE_ADDRESS +
                          binStart * HWA_COMPLEX16_BYTES;
    uint32_t pongSource = SOC_XWR68XX_MSS_HWA_MEM2_BASE_ADDRESS + HWA_MEM_STRIDE +
                          binStart * HWA_COMPLEX16_BYTES;

    if (gPostCaptureStarted) {
        ringSlot = gCapturePlan.preFrames + gPostFramesCaptured;
        gActiveFrameIsPost = 1U;
        if (gCapturePlan.phased &&
            gPostFramesCaptured < gCapturePlan.impactFrames) {
            binCount = gCapturePlan.impactBins;
            gActiveFrameShouldKeep = 1U;
        } else {
            uint32_t ballObserved = gCapturePlan.phased
                                        ? gPostFramesObserved -
                                              gCapturePlan.impactFrames
                                        : gPostFramesObserved;
            binCount = gCapturePlan.postBins;
            gActiveFrameShouldKeep =
                ((ballObserved % gCapturePlan.postStride) == 0U);
        }
    } else {
        ringSlot = gPreFramesCaptured % gCapturePlan.preFrames;
        binCount = gCapturePlan.preBins;
        gActiveFrameIsPost = 0U;
        gActiveFrameShouldKeep = 1U;
    }
    if (ringSlot >= gCapturePlan.totalFrames) {
        return -1;
    }
#ifdef L3_RING_IQ8
    destination = l3_captureUsesScratch()
                      ? (uint32_t)&g_iq16FrameScratch[gIq8ActiveScratch][0]
                      : (uint32_t)&g_ring[gFrameOffset[ringSlot]];
    if (l3_captureUsesScratch()) {
        /* From here the scratch belongs to the HWA: a detect frame still
         * being read from it is stale (l3_detectFrameStale). */
        gScratchBusy[gIq8ActiveScratch] = 1U;
    }
    gActiveProcessStart = (uint8_t)binStart;
    gActiveProcessBins = (uint8_t)binCount;
#else
    destination = (uint32_t)&g_ring[gFrameOffset[ringSlot]];
#endif

    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PING_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PONG_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    if (l3_configHwaOutputEdma(L3_HWA_OUT_PING_CHANNEL, L3_HWA_OUT_PING_SHADOW,
                              pingSource, destination, 0U, binCount) != 0 ||
        l3_configHwaOutputEdma(L3_HWA_OUT_PONG_CHANNEL, L3_HWA_OUT_PONG_SHADOW,
                              pongSource,
                              destination + N_RX * binCount * HWA_COMPLEX16_BYTES,
                              1U, binCount) != 0) {
        return -1;
    }
    return 0;
}

static void l3_drainHwaRearmSemaphore(void)
{
    if (gHwaRearmSemaphore != NULL) {
        while (Semaphore_pend(gHwaRearmSemaphore, BIOS_NO_WAIT)) {
            /* A completed frame may have posted immediately before capture was
             * frozen. It must not reconfigure the freshly armed chain. */
        }
    }
}

static int32_t l3_restartCompletedHwaFrame(void)
{
    uintptr_t key;
    int32_t errCode;

    (void)HWA_disableDoneInterrupt(gHwaHandle);
    (void)HWA_enable(gHwaHandle, 0U);
    key = Hwi_disable();
    gHwaDoneSeen = 0U;
    gHwaOutputSeen = 0U;
    gHwaRearmPending = 0U;
#ifdef L3_RING_IQ8
    gIq8Pending = 0U;
#endif
    Hwi_restore(key);
    errCode = l3_configHwaCommon();
    if (errCode == 0) {
        errCode =
            l3_configHwaFrameOutput(0U);
    }
    if (errCode == 0) {
        errCode = HWA_enableDoneInterrupt(gHwaHandle, l3_hwaChainDoneCB, NULL);
    }
    if (errCode == 0) {
        errCode = l3_hwaStartRing();
    }
    return errCode;
}

static int32_t l3_freezeHwaAfterPostFrames(void)
{
    uintptr_t key;

    if (gHwaFreezeSemaphore == NULL) {
        return -1;
    }
    while (Semaphore_pend(gHwaFreezeSemaphore, BIOS_NO_WAIT)) {
        /* Discard a stale completion before issuing a new request. */
    }
    key = Hwi_disable();
    gHwaFreezeRequested = 1U;
    gHwaFreezeRequestFrame = gRingFrame;
    gPostCaptureStarted = 0U;
    gPostFramesCaptured = 0U;
    gPostFramesObserved = 0U;
    gActiveFrameShouldKeep = 1U;
    gHwaFreezeRequests++;
    Hwi_restore(key);
    /* Allow the configured post-trigger frames plus scheduling/stop margin. */
    if (!Semaphore_pend(gHwaFreezeSemaphore, 250U)) {
        key = Hwi_disable();
        gHwaFreezeRequested = 0U;
        gHwaFreezeTimeouts++;
        Hwi_restore(key);
        return -1;
    }
    return 0;
}

/* Application shutdown is not a shot trigger. Cancel any pending post-frame
 * plan and stop at the next completed HWA output instead of waiting for a
 * full post-impact movie that may never arrive while the RF chain is idle. */
static int32_t l3_freezeHwaForShutdown(void)
{
    uintptr_t key;

    if (gHwaFreezeSemaphore == NULL) {
        return -1;
    }
    while (Semaphore_pend(gHwaFreezeSemaphore, BIOS_NO_WAIT)) {
        /* Discard a stale completion before issuing a new request. */
    }
    key = Hwi_disable();
    gHwaFreezeRequested = 0U;
    gHwaShutdownRequested = 1U;
    Hwi_restore(key);

    /* Handle a frame that completed just before the shutdown request. */
    l3_hwaMaybeQueueRearm();
    if (!Semaphore_pend(gHwaFreezeSemaphore, 250U)) {
        key = Hwi_disable();
        gHwaShutdownRequested = 0U;
        gHwaFreezeTimeouts++;
        Hwi_restore(key);
        return -1;
    }
    return 0;
}

static int32_t l3_finishCaptureStop(void)
{
    int32_t errCode;

    if (MMWave_stop(gMMWaveHandle, &errCode) < 0) {
        CLI_write("Error: MMWave_stop failed (%d)\n", errCode);
        return -1;
    }
    Task_sleep(10);
#ifdef HWA_CHAINED_SNAPSHOT_RING
    (void)HWA_enable(gHwaHandle, 0U);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PING_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PONG_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_SIGNATURE_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
#else
    (void)EDMA_disableChannel(gEdmaHandle, L3_EDMA_CHANNEL, EDMA3_CHANNEL_TYPE_DMA);
#endif
    gCaptureActive = 0U;
    return 0;
}

/* Shot dumps preserve the configured post-impact movie before stopping. */
static int32_t l3_stopCaptureAtBoundary(void)
{
    if (!gCaptureActive) {
        return 0;
    }
#ifdef HWA_CHAINED_SNAPSHOT_RING
    if (l3_freezeHwaAfterPostFrames() != 0) {
        CLI_write("Error: HWA post-trigger frame freeze timed out\n");
        return -1;
    }
#endif
    return l3_finishCaptureStop();
}

/* sensorStop only needs a clean hardware boundary, not post-impact frames. */
static int32_t l3_stopCaptureForShutdown(void)
{
    if (!gCaptureActive) {
        return 0;
    }
#ifdef HWA_CHAINED_SNAPSHOT_RING
    if (l3_freezeHwaForShutdown() != 0) {
        CLI_write("Error: HWA shutdown boundary timed out\n");
        return -1;
    }
#endif
    return l3_finishCaptureStop();
}

static int32_t l3_armHwaChain(void)
{
    HWA_ParamConfig dummyCfg;
    uint16_t mem2Offset = (uint16_t)(SOC_XWR68XX_MSS_HWA_MEM2_BASE_ADDRESS -
                                     SOC_XWR68XX_MSS_HWA_MEM0_BASE_ADDRESS);
    uint16_t mem3Offset = (uint16_t)(mem2Offset + HWA_MEM_STRIDE);
    int32_t errCode;

    if (!gHwaOpened || gHwaHandle == NULL) {
        return -1;
    }
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PING_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_OUT_PONG_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)EDMA_disableChannel(gEdmaHandle, L3_HWA_SIGNATURE_CHANNEL,
                              EDMA3_CHANNEL_TYPE_DMA);
    (void)HWA_enable(gHwaHandle, 0U);
    gHwaDoneSeen = 0U;
    gHwaOutputSeen = 0U;
    gHwaRearmPending = 0U;
#ifdef SNAPSHOT_DYNAMIC_WINDOWS
    memset((void *)gFrameBinStart, SNAPSHOT_BIN_START, sizeof(gFrameBinStart));
#endif
    (void)HWA_disableDoneInterrupt(gHwaHandle);
    (void)HWA_disableParamSetInterrupt(gHwaHandle, L3_HWA_PARAM_FFT_PING,
                                      HWA_PARAMDONE_INTERRUPT_TYPE_CPU |
                                      HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
    (void)HWA_disableParamSetInterrupt(gHwaHandle, L3_HWA_PARAM_FFT_PONG,
                                      HWA_PARAMDONE_INTERRUPT_TYPE_CPU |
                                      HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
    l3_drainHwaRearmSemaphore();
    /* A dump can freeze the accelerator between ping and pong. Reset the HWA
     * state machine before replacing its paramsets; otherwise the second dump
     * can wedge in MMWave_stop while stale DFE/DMA triggers remain pending. */
    errCode = HWA_reset(gHwaHandle);
    if (errCode != 0) {
        return errCode;
    }

    memset((void *)&dummyCfg, 0, sizeof(dummyCfg));
    dummyCfg.triggerMode = HWA_TRIG_MODE_DMA;
    dummyCfg.dmaTriggerSrc = L3_HWA_PARAM_DUMMY_PING;
    dummyCfg.accelMode = HWA_ACCELMODE_NONE;
    errCode = HWA_configParamSet(gHwaHandle, L3_HWA_PARAM_DUMMY_PING, &dummyCfg, NULL);
    if (errCode != 0) {
        return errCode;
    }
    dummyCfg.dmaTriggerSrc = L3_HWA_PARAM_DUMMY_PONG;
    errCode = HWA_configParamSet(gHwaHandle, L3_HWA_PARAM_DUMMY_PONG, &dummyCfg, NULL);
    if (errCode != 0) {
        return errCode;
    }
    errCode = l3_configHwaProcessParam(L3_HWA_PARAM_FFT_PING,
                                      L3_HWA_OUT_PING_CHANNEL, mem2Offset);
    if (errCode != 0) {
        return errCode;
    }
    errCode = l3_configHwaProcessParam(L3_HWA_PARAM_FFT_PONG,
                                      L3_HWA_OUT_PONG_CHANNEL, mem3Offset);
    if (errCode != 0) {
        return errCode;
    }

    errCode = l3_configHwaCommon();
    if (errCode != 0) {
        return errCode;
    }

    if (l3_configHwaFrameOutput(0U) != 0 || l3_configHwaSignatureEdma() != 0) {
        return -1;
    }
    errCode = HWA_enableDoneInterrupt(gHwaHandle, l3_hwaChainDoneCB, NULL);
    if (errCode != 0) {
        return errCode;
    }
    return l3_hwaStartRing();
}

static void l3_hwaRearmTask(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;
    while (1) {
        Semaphore_pend(gHwaRearmSemaphore, BIOS_WAIT_FOREVER);
        {
            uintptr_t key;
            uint8_t shouldRearm = 0U;
            uint8_t freezeAfterPack = 0U;
#ifdef L3_RING_IQ8
            uint8_t hadPending = 0U;
            uint8_t pendingScratch = 0U;
            uint8_t pendingDetect = 0U;
            uint8_t nextScratch = 0U;
            uint32_t pendingSlot = 0U;
            uint32_t pendingEpoch = 0U;
#endif
            int32_t errCode;
            uint32_t queuedCycles;
            uint8_t timed;

            key = Hwi_disable();
            if (gCaptureActive) {
                gHwaRearmBusy = 1U;
                shouldRearm = 1U;
            }
            Hwi_restore(key);
            if (!shouldRearm) {
                continue;
            }

#if !defined(L3_RING_IQ8)
            key = Hwi_disable();
            if (gHwaShutdownRequested) {
                gCaptureActive = 0U;
                gHwaShutdownRequested = 0U;
                gHwaRearmPending = 0U;
                freezeAfterPack = 1U;
            }
            Hwi_restore(key);
#else
            if (!l3_captureUsesScratch()) {
                key = Hwi_disable();
                if (gHwaShutdownRequested) {
                    gCaptureActive = 0U;
                    gHwaShutdownRequested = 0U;
                    gHwaRearmPending = 0U;
                    freezeAfterPack = 1U;
                }
                Hwi_restore(key);
            }
#endif
#ifdef L3_RING_IQ8
            if (l3_captureUsesScratch()) {
                key = Hwi_disable();
                if (gIq8Pending) {
                    hadPending = 1U;
                    pendingSlot = gIq8PendingSlot;
                    pendingScratch = gIq8PendingScratch;
                    pendingDetect = gIq8PendingDetect;
                    pendingEpoch = gIq8PendingEpoch;
                    gIq8Pending = 0U;
                    gIq8PendingDetect = 0U;
#ifdef L3_IQ8_EDMA_PACK
                    gIq8PackDetectArm[pendingScratch] = pendingDetect;
                    gIq8PackDetectSlot[pendingScratch] = (uint16_t)pendingSlot;
                    gIq8PackDetectEpoch[pendingScratch] = pendingEpoch;
#endif
                }
                if (gHwaShutdownRequested) {
                    gCaptureActive = 0U;
                    gHwaShutdownRequested = 0U;
                    gHwaRearmPending = 0U;
                    freezeAfterPack = 1U;
                } else if (l3_shouldFreezeNow()) {
                    gCaptureActive = 0U;
                    gHwaFreezeRequested = 0U;
                    gHwaFreezeCompletions++;
                    gHwaRearmPending = 0U;
                    freezeAfterPack = 1U;
                } else {
                    nextScratch = gIq8ActiveScratch ^ 1U;
#ifndef L3_IQ8_EDMA_PACK
                    gIq8ActiveScratch = nextScratch;
#endif
                }
                Hwi_restore(key);
            }
#endif
            if (freezeAfterPack) {
#ifdef L3_RING_IQ8
                if (hadPending) {
                    l3_storeCompletedFrame(pendingSlot, pendingScratch);
                }
#ifdef L3_IQ8_EDMA_PACK
                if (l3_captureUsesIq8()) {
                    l3_waitForAllIq8Edma();
                }
#endif
#endif
                if (gHwaFreezeSemaphore != NULL) {
                    Semaphore_post(gHwaFreezeSemaphore);
                }
                key = Hwi_disable();
                gHwaRearmBusy = 0U;
                Hwi_restore(key);
                continue;
            }
#if defined(L3_RING_IQ8) && defined(L3_IQ8_EDMA_PACK)
            if (l3_captureUsesIq8()) {
                l3_waitForIq8EdmaScratch(nextScratch);
                gIq8ActiveScratch = nextScratch;
            } else if (l3_captureCompactsIq16()) {
                /* The compaction is synchronous and happens after the HWA
                 * restart below, so the scratch it reads is the one the HWA
                 * is NOT writing: toggle first, compact from the other. */
                gIq8ActiveScratch = nextScratch;
            }
#endif
            key = Hwi_disable();
            queuedCycles = gHwaRearmQueuedCycles;
            timed = gHwaRearmQueuedValid;
            gHwaRearmQueuedValid = 0U;
            Hwi_restore(key);
            errCode = l3_restartCompletedHwaFrame();
            if (timed) {
                uint32_t elapsedUs = (Cycleprofiler_getTimeStamp() - queuedCycles) /
                                     (gCpuClock / 1000000U);
                gHwaRearmLastUs = elapsedUs;
                if (elapsedUs > gHwaRearmMaxUs) {
                    gHwaRearmMaxUs = elapsedUs;
                }
                gHwaRearmTimed++;
            }
            if (errCode == 0) {
                gHwaRearms++;
            } else {
                gHwaRearmErrors++;
            }
#ifdef L3_RING_IQ8
            if (l3_captureUsesScratch() && hadPending) {
                l3_storeCompletedFrame(pendingSlot, pendingScratch);
#ifndef L3_IQ8_EDMA_PACK
                if (pendingDetect != 0U && l3_captureUsesIq8()) {
                    /* Pre slots to the trigger, kept post slots (post epoch)
                     * to the ball tracker. */
                    l3_publishDetectFrame(pendingSlot, pendingEpoch);
                }
#endif
            }
#endif
            key = Hwi_disable();
            gHwaRearmBusy = 0U;
            Hwi_restore(key);
        }
    }
}
#endif

/* Fill the 20-byte fixed dump header (dump_format.h / iwr6843_l3dump.HEADER). */
static void l3_fill_header(l3_dump_header_t *h, uint16_t n_frames,
                           uint16_t trigger_frame)
{
    memcpy(h->magic, L3_DUMP_MAGIC, 4);
    h->version          = L3_DUMP_VERSION;
    h->n_frames         = n_frames;
    h->chirps_per_frame =
        gCapturePlan.chirpsPerFrame;
    h->n_tx             = N_TX;
    h->n_rx             = N_RX;
#ifdef SNAPSHOT_DUMP
#ifdef HYBRID_CADENCE_CAPTURE
    h->version          = L3_DUMP_VERSION_TIMED;
#ifdef L3_DUMP_IQ8
    h->sample_fmt       = L3_SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED;
#elif defined(L3_RING_IQ8)
    h->sample_fmt       = l3_captureUsesIq8()
                              ? L3_SAMPLE_RANGE_FFT_IQ8_VARIABLE_TIMED
                              : L3_SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED;
#else
    h->sample_fmt       = L3_SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED;
#endif
#else
    h->version          = L3_DUMP_VERSION_VARIABLE;
    h->sample_fmt       = L3_SAMPLE_RANGE_FFT_IQ16_VARIABLE;
#endif
    h->n_samples = gCapturePlan.preBins;
    if (gCapturePlan.impactBins > h->n_samples) {
        h->n_samples = gCapturePlan.impactBins;
    }
    if (gCapturePlan.postBins > h->n_samples) {
        h->n_samples = gCapturePlan.postBins;
    }
    if (gCapturePlan.compact) {
        /* The payload holds the retained windows: their widest is the
         * maximum frame width the decoder sizes for. */
        h->n_samples = gCapturePlan.retainPreBins;
        if (gCapturePlan.phased && gCapturePlan.retainImpactBins > h->n_samples) {
            h->n_samples = gCapturePlan.retainImpactBins;
        }
        if (gCapturePlan.retainPostBins > h->n_samples) {
            h->n_samples = gCapturePlan.retainPostBins;
        }
    }
    h->_pad             = 0U;
#else
    h->n_samples        = SAVE_SAMPLES;
    h->sample_fmt       = L3_SAMPLE_INT16_IQ;
    h->_pad             = 0;
#endif
    h->trigger_frame    = trigger_frame;
    h->frame_period_us  = gFramePeriodUs;
}

static void l3_writeFrameDescriptor(uint32_t slot, uint8_t firstFrame)
{
#ifdef HYBRID_CADENCE_CAPTURE
    uint8_t descriptor[4];
    uint16_t deltaUs = firstFrame ? 0U : gFrameDeltaUs[slot];

    descriptor[0] = gFrameBinStart[slot];
    descriptor[1] = gFrameBinCount[slot];
    descriptor[2] = (uint8_t)(deltaUs & 0xFFU);
    descriptor[3] = (uint8_t)(deltaUs >> 8U);
    UART_writePolling(gDataUart, descriptor, sizeof(descriptor));
#else
    uint8_t descriptor[2];
    (void)firstFrame;

    descriptor[0] = gFrameBinStart[slot];
    descriptor[1] = gFrameBinCount[slot];
    UART_writePolling(gDataUart, descriptor, sizeof(descriptor));
#endif
}

#if defined(L3_DUMP_IQ8)
static uint16_t l3_iq8FrameScale(uint32_t slot)
{
    const int16_t *src = (const int16_t *)&g_ring[gFrameOffset[slot]];
    uint32_t words = gFrameBytes[slot] / (uint32_t)sizeof(int16_t);

    return l3_iq8_dump_scale(l3_iq8_max_abs(src, words));
}
#endif

#if defined(L3_ANY_IQ8)
static void l3_writeU16Le(uint16_t value)
{
    uint8_t out[2];

    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8U);
    UART_writePolling(gDataUart, out, sizeof(out));
}
#endif

#if defined(L3_DUMP_IQ8)
static void l3_writeCompressedIq8Frame(uint32_t slot, uint16_t scale)
{
    const int16_t *src = (const int16_t *)&g_ring[gFrameOffset[slot]];
    uint32_t words = gFrameBytes[slot] / (uint32_t)sizeof(int16_t);
    uint8_t out[256];
    uint32_t pending = 0U;
    uint32_t word;

    for (word = 0U; word < words; word++) {
        out[pending++] = (uint8_t)l3_iq8_quantize_scale(src[word], scale);
        if (pending == sizeof(out)) {
            UART_writePolling(gDataUart, out, pending);
            pending = 0U;
        }
    }
    if (pending > 0U) {
        UART_writePolling(gDataUart, out, pending);
    }
}
#endif

static int32_t l3_readTemperatureReport(l3_temperature_report_t *report)
{
    rlRfTempData_t tempData;
    int32_t errCode;

    memset((void *)&tempData, 0, sizeof(tempData));
    errCode = rlRfGetTemperatureReport(RL_DEVICE_MAP_INTERNAL_BSS, &tempData);
    if (errCode != 0) {
        return errCode;
    }

    report->device_time_ms = tempData.time;
    report->tmpRx0Sens = tempData.tmpRx0Sens;
    report->tmpRx1Sens = tempData.tmpRx1Sens;
    report->tmpRx2Sens = tempData.tmpRx2Sens;
    report->tmpRx3Sens = tempData.tmpRx3Sens;
    report->tmpTx0Sens = tempData.tmpTx0Sens;
    report->tmpTx1Sens = tempData.tmpTx1Sens;
    report->tmpTx2Sens = tempData.tmpTx2Sens;
    report->tmpPmSens = tempData.tmpPmSens;
    report->tmpDig0Sens = tempData.tmpDig0Sens;
    report->tmpDig1Sens = tempData.tmpDig1Sens;
    return 0;
}

#ifndef HWA_CHAINED_SNAPSHOT_RING
/* EDMA ring-wrap completion (fires once per RING_CHIRPS; liveness only). */
static void l3_edmaCB(uintptr_t arg, uint8_t tcCode)
{
    (void)arg; (void)tcCode;
    gNumWrap++;
}
#endif

/* Frame-start ISR: liveness + ring-alignment counters (the EDMA fills the
 * ring autonomously). */
static void l3_frameStartISR(uintptr_t arg)
{
    (void)arg;
    gNumFrame++;
#ifdef HWA_CHAINED_SNAPSHOT_RING
    if (gCaptureActive) {
        if (!gHwaArmedForFrame) {
            gHwaMissedFrameStarts++;
        }
        gHwaArmedForFrame = 0U;
    }
#endif
#if !defined(HWA_CHAINED_SNAPSHOT_RING)
    gRingFrame++;
#endif
}

/* Start the RF front-end (config must already be applied). Shared by
 * sensorStart and the post-dump restart. */
static int32_t l3_startFrontEnd(void)
{
    MMWave_CalibrationCfg calibrationCfg;
    int32_t               errCode;

    memset((void *)&calibrationCfg, 0, sizeof(calibrationCfg));
    calibrationCfg.dfeDataOutputMode                          = gCtrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;
    return MMWave_start(gMMWaveHandle, &calibrationCfg, &errCode);
}

static uint8_t l3_dumpCancelRequested(void)
{
    UART_Config *uartConfig = (UART_Config *)gCliUart;
    UartSci_HwCfg *hwCfg;
    uint8_t value;

    if (uartConfig == NULL || uartConfig->hwAttrs == NULL) {
        return 0U;
    }
    hwCfg = (UartSci_HwCfg *)uartConfig->hwAttrs;
    if (CSL_FEXTR(hwCfg->ptrSCIRegs->SCIFLR, 9U, 9U) == 0U) {
        return 0U;
    }
    value = (uint8_t)CSL_FEXTR(hwCfg->ptrSCIRegs->SCIRD, 7U, 0U);
    return (value == L3_DUMP_CANCEL_BYTE) ? 1U : 0U;
}

/* CLI "l3dump": record the current circular position, retain the configured
 * post-trigger frames, stop at that completed frame boundary, stream the ring,
 * then restart from slot zero. */
int32_t l3_cli_dump(int32_t argc, char *argv[])
{
    l3_dump_header_t h;
    uint32_t         i;
    uint8_t          dumpCancelled = 0U;
    uint32_t actualPre;
    uint32_t actualPost;
    uint32_t oldestPre;
    (void)argc; (void)argv;

    if (!gCaptureActive) {
        return -1;
    }

    /* Halt chirping only after HWA and both output EDMAs completed naturally. */
    if (l3_stopCaptureAtBoundary() != 0) {
        return -1;
    }

    /* Oldest slot = time-order start (best-effort: a frame-start ISR racing
     * the stop can skew this by one; the host cross-checks with its own
     * rotation solve). Before the first wrap the oldest data is slot 0. */
    actualPre = (gPreFramesCaptured < gCapturePlan.preFrames)
                    ? gPreFramesCaptured : gCapturePlan.preFrames;
    actualPost = (gPostFramesCaptured < gCapturePlan.postFrames)
                     ? gPostFramesCaptured : gCapturePlan.postFrames;
    oldestPre = (gPreFramesCaptured >= gCapturePlan.preFrames)
                    ? (gPreFramesCaptured % gCapturePlan.preFrames) : 0U;
    l3_fill_header(&h, (uint16_t)(actualPre + actualPost), 0U);
    {
        l3_temperature_report_t tempReport;
        int32_t tempStatus;

        memset((void *)&tempReport, 0, sizeof(tempReport));
        tempStatus = l3_readTemperatureReport(&tempReport);
        if (tempStatus == 0) {
            h.version = L3_DUMP_VERSION_CAPTURE_TEMPERATURE;
        }
        UART_writePolling(gDataUart, (uint8_t *)&h, sizeof(h));
        if (tempStatus == 0) {
            UART_writePolling(gDataUart, (uint8_t *)&tempReport, sizeof(tempReport));
        }
    }
    for (i = 0U; i < actualPre; i++) {
        uint32_t slot = (oldestPre + i) % gCapturePlan.preFrames;
        l3_writeFrameDescriptor(slot, (i == 0U));
    }
    for (i = 0U; i < actualPost; i++) {
        uint32_t slot = gCapturePlan.preFrames + i;
        l3_writeFrameDescriptor(slot, (actualPre == 0U && i == 0U));
    }
#ifdef L3_ANY_IQ8
    if (
#ifdef L3_DUMP_IQ8
        1U
#else
        l3_captureUsesIq8()
#endif
    )
    {
        for (i = 0U; i < actualPre; i++) {
            uint32_t slot = (oldestPre + i) % gCapturePlan.preFrames;
#ifdef L3_RING_IQ8
            l3_writeU16Le(gFrameIq8Scale[slot]);
#else
            uint32_t outputIndex = i;
            gFrameIq8Scale[outputIndex] = l3_iq8FrameScale(slot);
            l3_writeU16Le(gFrameIq8Scale[outputIndex]);
#endif
        }
        for (i = 0U; i < actualPost; i++) {
            uint32_t slot = gCapturePlan.preFrames + i;
#ifdef L3_RING_IQ8
            l3_writeU16Le(gFrameIq8Scale[slot]);
#else
            uint32_t outputIndex = actualPre + i;
            gFrameIq8Scale[outputIndex] = l3_iq8FrameScale(slot);
            l3_writeU16Le(gFrameIq8Scale[outputIndex]);
#endif
        }
    }
#endif
#ifdef SNAPSHOT_DUMP
#if defined(HWA_CHAINED_SNAPSHOT_RING)
    for (i = 0U; i < actualPre; i++) {
        uint32_t slot = (oldestPre + i) % gCapturePlan.preFrames;
#ifdef L3_RING_IQ8
        UART_writePolling(gDataUart, &g_ring[gFrameOffset[slot]],
                          gFrameBytes[slot]);
#elif defined(L3_DUMP_IQ8)
        l3_writeCompressedIq8Frame(slot, gFrameIq8Scale[i]);
#else
        UART_writePolling(gDataUart, &g_ring[gFrameOffset[slot]],
                          gFrameBytes[slot]);
#endif
        if (l3_dumpCancelRequested()) {
            dumpCancelled = 1U;
            break;
        }
    }
    for (i = 0U; !dumpCancelled && i < actualPost; i++) {
        uint32_t slot = gCapturePlan.preFrames + i;
#ifdef L3_RING_IQ8
        UART_writePolling(gDataUart, &g_ring[gFrameOffset[slot]],
                          gFrameBytes[slot]);
#elif defined(L3_DUMP_IQ8)
        l3_writeCompressedIq8Frame(slot, gFrameIq8Scale[actualPre + i]);
#else
        UART_writePolling(gDataUart, &g_ring[gFrameOffset[slot]],
                          gFrameBytes[slot]);
#endif
        if (l3_dumpCancelRequested()) {
            dumpCancelled = 1U;
            break;
        }
    }
#else
    if (!gHwaOpened || gHwaHandle == NULL) {
        CLI_write("Error: HWA unavailable for snapshot dump\n");
        gCaptureActive = 0U;
        return -1;
    }
    for (i = 0; i < RING_FRAMES; i++) {
        uint32_t chirp;
        for (chirp = 0U; chirp < CHIRPS_PER_FRAME; chirp++) {
            uint32_t rawWord = chirp * N_RX * N_SAMPLES * 2U;
            if (l3_emitSnapshotChirp(&g_ring[i][rawWord]) != 0) {
                CLI_write("Error: HWA snapshot emit failed\n");
                gCaptureActive = 0U;
                return -1;
            }
        }
    }
#endif
#else
    for (i = 0; i < RING_FRAMES; i++) {
        UART_writePolling(gDataUart, (uint8_t *)g_ring[i], sizeof(g_ring[i]));
    }
#endif

#ifdef HWA_CHAINED_SNAPSHOT_RING
    gRingFrame = 0U;
    gHwaFreezeRequestFrame = 0U;
    gPreFramesCaptured = 0U;
    l3_trigRearm();
    gPostFramesCaptured = 0U;
    gPostFramesObserved = 0U;
    gPostCaptureStarted = 0U;
    gActiveFrameIsPost = 0U;
    gActiveFrameShouldKeep = 1U;
    l3_resetDetectQueue();
    if (l3_restartCompletedHwaFrame() < 0) {
        CLI_write("Error: completed HWA frame restart failed\n");
        gCaptureActive = 0U;
        return -1;
    }
    gHwaFreezeRestarts++;
#else
    if (l3_armCapture() < 0) {
        CLI_write("Error: EDMA re-arm failed\n");
        gCaptureActive = 0U;
        return -1;
    }
#endif
#ifndef HWA_CHAINED_SNAPSHOT_RING
    gRingFrame = 0U;
#endif
#ifdef HWA_CHAINED_SNAPSHOT_RING
    gCaptureActive = 1U;
#endif
    if (l3_startFrontEnd() < 0) {
        CLI_write("Error: RF restart failed\n");
        gCaptureActive = 0U;
        return -1;
    }
    if (dumpCancelled) {
        UART_writePolling(gDataUart, L3_DUMP_CANCEL_ACK, L3_DUMP_CANCEL_ACK_BYTES);
    }
    return 0;
}

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
/* CLI "l3sparse": freeze, stream vertical residual power, then the complex
 * cells the host names. The host falls back to l3dump when this command
 * returns an error before the freeze. */
static void l3_writeU16(uint16_t value)
{
    uint8_t bytes[2];

    bytes[0] = (uint8_t)(value & 0xFFU);
    bytes[1] = (uint8_t)(value >> 8U);
    UART_writePolling(gDataUart, bytes, sizeof(bytes));
}
#endif

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
static void l3_writeF32(float value)
{
    uint32_t bits;
    uint8_t bytes[4];

    memcpy(&bits, &value, sizeof(bits));
    bytes[0] = (uint8_t)(bits & 0xFFU);
    bytes[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    bytes[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    bytes[3] = (uint8_t)((bits >> 24U) & 0xFFU);
    UART_writePolling(gDataUart, bytes, sizeof(bytes));
}
#endif

/* Read one CLI line into buf through the UART driver's interrupt receive.
 * Returns 0 on a line, -1 on timeout, -3 on an empty line, and -2 when the
 * line does not fit: the rest of it is then read and discarded so none of it
 * reaches the CLI parser as a command.
 *
 * The SCI receiver holds one byte. Polling it from this task lost bytes
 * whenever the HWA rearm task (above the CLI) ran mid-line, and a host cell
 * line spans several frames; the driver's RX interrupt captures each byte
 * regardless of which task is running. The CLI handle reads TEXT with newline
 * return, so UART_read completes at '\n' (a CR is folded into one). */
#define L3_READLINE_OVERFLOW (-2)
#define L3_READLINE_EMPTY (-3)
/* Bytes of an overlong line drained before giving up: 48 KB, about 4 s at
 * the CLI baud, far past any real request. */
#define L3_READLINE_DRAIN_MAX (64U * L3_SPARSE_REQUEST_MAX)

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
static int32_t l3_readLine(char *buf, uint32_t cap)
{
    UART_Config *uartConfig = (UART_Config *)gCliUart;
    UartSci_Driver *driver;
    uint32_t savedTimeout;
    uint32_t drained = 0U;
    int32_t count;
    int32_t status;

    if (uartConfig == NULL || uartConfig->object == NULL || cap < 2U) {
        return -1;
    }
    /* The handle's read timeout is the CLI's (forever). A missing request
     * must not freeze the ring for good, so bound this one read. */
    driver = (UartSci_Driver *)uartConfig->object;
    savedTimeout = driver->params.readTimeout;
    driver->params.readTimeout = L3_SPARSE_REQUEST_TIMEOUT_MS;

    count = UART_read(gCliUart, (uint8_t *)buf, cap - 1U);
    if (count <= 0) {
        buf[0] = '\0';
        status = -1;
    } else if (buf[count - 1] == '\n') {
        buf[count - 1] = '\0';
        status = (count == 1) ? L3_READLINE_EMPTY : 0;
    } else {
        /* The buffer filled before a newline: the line does not fit. Drain
         * the rest of it so none of it reaches the CLI parser as commands.
         * Each read is bounded by the request timeout; the byte bound only
         * stops a stream that never ends, and must cover any request a host
         * could plausibly send (a 4x oversized cell line is ~4 KB, which the
         * old 4 x cap bound left half of in the FIFO). */
        buf[cap - 1U] = '\0';
        while (drained < L3_READLINE_DRAIN_MAX) {
            uint8_t scratch[32];

            count = UART_read(gCliUart, scratch, sizeof(scratch));
            if (count <= 0) {
                break;
            }
            drained += (uint32_t)count;
            if (scratch[count - 1] == '\n') {
                break;
            }
        }
        status = L3_READLINE_OVERFLOW;
    }

    driver->params.readTimeout = savedTimeout;
    return status;
}
#endif

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
static const int16_t *l3_iq16Sample(
    uint32_t slot, uint32_t chirp, uint32_t rx, uint32_t localBin)
{
    const int16_t *frame = (const int16_t *)&g_ring[gFrameOffset[slot]];
    uint32_t binCount = gFrameBinCount[slot];
    uint32_t index = ((chirp * N_RX) + rx) * binCount + localBin;

    return &frame[index * 2U];
}
#endif

/* Samples for the detect path: int16 (Im, Re) pairs in IQ16, int8 pairs
 * times the frame's scale in IQ8 (see gFrameIq8Scale), so the trigger, the
 * trackers and the ball detector read either ring. Each component is
 * l3_ringComponentBytes() wide; a complex sample is two of them. In the
 * compact IQ16 formats a detect frame is the wide processing window in the
 * IQ16 scratch, not the retained window in the ring: l3_detectFrameOf says
 * where a slot's frame is, l3_detectFrameStale whether it is still there. */
typedef struct {
    const uint8_t *base;
    uint32_t binStart;   /* global bin of the first sample */
    uint32_t binCount;
    uint32_t cb;         /* bytes per component */
    float    scale;      /* physical amplitude per stored unit */
    uint8_t  scratch;    /* L3_SCRATCH_NONE when the frame is in the ring */
    uint32_t epoch;      /* the scratch's completion count when this was taken */
} l3_detect_frame_t;

static uint32_t l3_ringComponentBytes(void)
{
    return l3_captureUsesIq8() ? 1U : 2U;
}

static float l3_ringScale(uint32_t slot)
{
#ifdef L3_RING_IQ8
    if (l3_captureUsesIq8()) {
        return (float)gFrameIq8Scale[slot];
    }
#else
    (void)slot;
#endif
    return 1.0F;
}

static float l3_ringComponent(const uint8_t *component, uint32_t bytes)
{
    if (bytes == 1U) {
        return (float)*(const int8_t *)component;
    }
    return (float)*(const int16_t *)(const void *)component;
}

/* One component of a bin through the range window: the same component of the
 * adjacent complex samples sits 2 * bytes either side. */
static float l3_ringComponentWindowed(const uint8_t *component, uint32_t bytes, uint32_t window)
{
    float value = l3_ringComponent(component, bytes);

    if (window == L3_RANGE_WINDOW_HANN) {
        value -= 0.5F * (l3_ringComponent(component - 2U * bytes, bytes) +
                         l3_ringComponent(component + 2U * bytes, bytes));
    }
    return value;
}

/* The slot's frame as stored in the ring (the retained window). */
static l3_detect_frame_t l3_ringFrameOf(uint32_t slot)
{
    l3_detect_frame_t frame;

    frame.base = &g_ring[gFrameOffset[slot]];
    frame.binStart = gFrameBinStart[slot];
    frame.binCount = gFrameBinCount[slot];
    frame.cb = l3_ringComponentBytes();
    frame.scale = l3_ringScale(slot);
    frame.scratch = L3_SCRATCH_NONE;
    frame.epoch = 0U;
    return frame;
}

/* The slot's frame as the detect task should read it: the wide IQ16
 * processing window in the scratch for the compact formats, else the ring. */
static l3_detect_frame_t l3_detectFrameOf(uint32_t slot)
{
#ifdef L3_RING_IQ8
    if (l3_captureCompactsIq16() && slot < L3_MAX_CAPTURE_FRAMES &&
        gFrameScratch[slot] < 2U) {
        l3_detect_frame_t frame;

        frame.scratch = gFrameScratch[slot];
        frame.base = (const uint8_t *)&g_iq16FrameScratch[frame.scratch][0];
        frame.binStart = gFrameProcessStart[slot];
        frame.binCount = gFrameProcessBins[slot];
        frame.cb = 2U;
        frame.scale = 1.0F;
        frame.epoch = gScratchFrame[frame.scratch];
        return frame;
    }
#endif
    return l3_ringFrameOf(slot);
}

/* 1 when the scratch a detect frame was read from has been handed back to
 * the HWA (or completed another frame) since: the observations just
 * computed from it may mix two frames and must not drive a decision. */
static int32_t l3_detectFrameStale(const l3_detect_frame_t *frame)
{
#ifdef L3_RING_IQ8
    if (frame->scratch != L3_SCRATCH_NONE) {
        return (gScratchBusy[frame->scratch] || gScratchFrame[frame->scratch] != frame->epoch)
                   ? 1
                   : 0;
    }
#else
    (void)frame;
#endif
    return 0;
}

/* Burst-MTI residual of one bin over every loop of a frame, summed over the
 * vertical TX pair (TX0 and TX2 of three) and all RX. Each (tx, rx) loop
 * mean is computed once, so a bin costs O(loops), not O(loops^2). perLoop[]
 * (gCapturePlan.loops values) receives the residual power of each loop; obs
 * receives the residual energy integrated over every loop and the lag-1
 * loop autocorrelation the trigger reads Doppler from. Either may be NULL. */
static void l3_verticalResidual(const l3_detect_frame_t *source, uint32_t localBin,
                                float *perLoop, l3_trig_obs_t *obs)
{
    const uint8_t *frame = source->base;
    uint32_t binCount = source->binCount;
    uint32_t ntx = gCapturePlan.chirpsPerFrame / gCapturePlan.loops;
    uint32_t loops = gCapturePlan.loops;
    uint32_t cb = source->cb;
    float scale = source->scale;
    /* The same (tx, rx) one loop later is ntx chirps on: N_RX * binCount
     * complex samples per chirp, two components each. */
    uint32_t loopStride = ntx * N_RX * binCount * 2U * cb;
    /* Per-loop power summed over channels, for the rows and the peak. */
    float loopPower[L3_MAX_LOOPS];
    float energy = 0.0F;
    float peak = 0.0F;
    float r1Re = 0.0F;
    float r1Im = 0.0F;
    /* A bin on the edge of the frame's window has one neighbour: unwindowed. */
    uint32_t window = (localBin > 0U && localBin + 1U < binCount) ? gRangeWindow
                                                                  : L3_RANGE_WINDOW_NONE;
    uint32_t tx;
    uint32_t loop;

    for (loop = 0U; loop < loops; loop++) {
        loopPower[loop] = 0.0F;
    }
    if (cb == 2U && loops <= L3_IQ16_MAX_LOOPS) {
        /* IQ16: exact integer statistics (l3_iq16_stats.c), converted to
         * float once at the end, so the IQ16 precision the scratch or ring
         * holds is not spent in float rounding on the way to the detector. */
        l3_iq16_bin_stats_t bin;
        l3_iq16_channel_stats_t channelStats;

        l3_iq16_bin_stats_init(&bin, loops);
        for (tx = 0U; tx < ntx; tx++) {
            uint32_t rx;
            if (ntx == 3U && tx == 1U) {
                continue;
            }
            for (rx = 0U; rx < N_RX; rx++) {
                const int16_t *words =
                    (const int16_t *)(const void *)&frame[((tx * N_RX + rx) * binCount + localBin) * 4U];

                if (l3_iq16_channel_stats_windowed(words, loops, loopStride / 2U, window,
                                                   &channelStats) == 0) {
                    l3_iq16_bin_stats_add(&bin, &channelStats);
                }
            }
        }
        l3_iq16_bin_stats_finish(&bin, &energy, &peak, &loopPower[0], &r1Re, &r1Im, perLoop);
        if (obs != NULL) {
            obs->energy = energy;
            obs->peak = peak;
            obs->loop0 = loopPower[0];
            obs->r1Re = r1Re;
            obs->r1Im = r1Im;
        }
        return;
    }
    for (tx = 0U; tx < ntx; tx++) {
        uint32_t rx;
        if (ntx == 3U && tx == 1U) {
            continue;
        }
        for (rx = 0U; rx < N_RX; rx++) {
            const uint8_t *base = &frame[((tx * N_RX + rx) * binCount + localBin) * 2U * cb];
            const uint8_t *sample = base;
            float meanIm = 0.0F;
            float meanRe = 0.0F;
            float prevIm = 0.0F;
            float prevRe = 0.0F;
            /* Read each loop's windowed component once and reuse it below,
             * as l3_iq16_channel_stats_windowed does: the two-pass algorithm
             * would otherwise re-run the windowed read for every sample. */
            float valueIm[L3_MAX_LOOPS];
            float valueRe[L3_MAX_LOOPS];

            for (loop = 0U; loop < loops; loop++) {
                valueIm[loop] = l3_ringComponentWindowed(sample, cb, window);
                valueRe[loop] = l3_ringComponentWindowed(sample + cb, cb, window);
                meanIm += valueIm[loop];
                meanRe += valueRe[loop];
                sample += loopStride;
            }
            meanIm /= (float)loops;
            meanRe /= (float)loops;
            for (loop = 0U; loop < loops; loop++) {
                float im = (valueIm[loop] - meanIm) * scale;
                float re = (valueRe[loop] - meanRe) * scale;
                float power = im * im + re * re;

                energy += power;
                loopPower[loop] += power;
                if (loop > 0U) {
                    /* residual[loop] * conj(residual[loop - 1]) */
                    r1Re += re * prevRe + im * prevIm;
                    r1Im += im * prevRe - re * prevIm;
                }
                prevIm = im;
                prevRe = re;
            }
        }
    }
    for (loop = 0U; loop < loops; loop++) {
        if (perLoop != NULL) {
            perLoop[loop] = loopPower[loop];
        }
        if (loopPower[loop] > peak) {
            peak = loopPower[loop];
        }
    }
    if (obs != NULL) {
        obs->energy = energy;
        obs->peak = peak;
        obs->loop0 = loopPower[0];
        obs->r1Re = r1Re;
        obs->r1Im = r1Im;
    }
}

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
/* l3sparse's per-loop residual power rows. */
/* Per-loop residual power of a FROZEN ring slot, for the l3sparse power map
 * and the l3track cell selection: always the retained window in the ring,
 * never the scratch, which the HWA has long since reused. */
static void l3_verticalPowerLoops(uint32_t slot, uint32_t localBin, float *out)
{
    l3_detect_frame_t frame = l3_ringFrameOf(slot);

    l3_verticalResidual(&frame, localBin, out, NULL);
}
#endif

/* Static (non-MTI) power of one bin: mean |I + jQ|^2 per complex sample over
 * every loopStep-th loop of the vertical TX pair and all RX. The residual above removes
 * exactly this, so it is the view of a stationary ball on the tee that the
 * trigger never sees; teeScan reports it so a ball's presence and range bin
 * can be proved before any swing is judged. Diagnostic only. */
static float l3_verticalStaticPower(const l3_detect_frame_t *source, uint32_t localBin,
                                    uint32_t loopStep)
{
    const uint8_t *frame = source->base;
    uint32_t binCount = source->binCount;
    uint32_t ntx = gCapturePlan.chirpsPerFrame / gCapturePlan.loops;
    uint32_t loops = gCapturePlan.loops;
    uint32_t cb = source->cb;
    float scale = source->scale;
    uint32_t loopStride = ntx * N_RX * binCount * 2U * cb * loopStep;
    float total = 0.0F;
    uint32_t samples = 0U;
    uint32_t tx;

    for (tx = 0U; tx < ntx; tx++) {
        uint32_t rx;
        if (ntx == 3U && tx == 1U) {
            continue;
        }
        for (rx = 0U; rx < N_RX; rx++) {
            const uint8_t *sample = &frame[((tx * N_RX + rx) * binCount + localBin) * 2U * cb];
            uint32_t loop;
            /* loopStep > 1 subsamples the loops: a static target does not
             * change between them, and the ball detector runs every frame. */
            for (loop = 0U; loop < loops; loop += loopStep) {
                float im = l3_ringComponent(sample, cb) * scale;
                float re = l3_ringComponent(sample + cb, cb) * scale;
                total += im * im + re * re;
                sample += loopStride;
                samples++;
            }
        }
    }
    return (samples > 0U) ? (total / (float)samples) : 0.0F;
}

/* One target's antenna channels at localBin of slot, for angle estimation:
 * each (tx, rx) is its burst-MTI residual summed coherently over the loops
 * with the target's per-loop Doppler phase (the lag-1 phase the observation
 * layer measured) unwound, so the loops add in phase and only the TDM chirp
 * offsets between the TX blocks remain for l3_angle_estimate to remove. */
static void l3_channelSnapshot(const l3_detect_frame_t *source, uint32_t localBin, float lag1PhaseRad,
                               float radialVelocityMps, l3_angle_snapshot_t *out)
{
    const uint8_t *frame = source->base;
    uint32_t binCount = source->binCount;
    uint32_t ntx = gCapturePlan.chirpsPerFrame / gCapturePlan.loops;
    uint32_t loops = gCapturePlan.loops;
    uint32_t cb = source->cb;
    float scale = source->scale;
    uint32_t loopStride = ntx * N_RX * binCount * 2U * cb;
    float stepRe = cosf(lag1PhaseRad);
    float stepIm = -sinf(lag1PhaseRad);
    uint32_t tx;

    l3_angle_snapshot_init(out, ntx, N_RX);
    out->lag1PhaseRad = lag1PhaseRad;
    out->radialVelocityMps = radialVelocityMps;
    out->chirpPeriodS = (ntx > 0U) ? (gTrigLoopPeriodS / (float)ntx) : 45.0e-6F;
    for (tx = 0U; tx < out->ntx; tx++) {
        uint32_t rx;
        for (rx = 0U; rx < out->nrx; rx++) {
            const uint8_t *base = &frame[((tx * N_RX + rx) * binCount + localBin) * 2U * cb];
            const uint8_t *sample = base;
            float meanIm = 0.0F;
            float meanRe = 0.0F;
            float sumRe = 0.0F;
            float sumIm = 0.0F;
            float rotRe = 1.0F;   /* exp(-j * loop * lag1) */
            float rotIm = 0.0F;
            uint32_t loop;

            for (loop = 0U; loop < loops; loop++) {
                meanIm += l3_ringComponent(sample, cb);
                meanRe += l3_ringComponent(sample + cb, cb);
                sample += loopStride;
            }
            meanIm /= (float)loops;
            meanRe /= (float)loops;
            sample = base;
            for (loop = 0U; loop < loops; loop++) {
                float im = (l3_ringComponent(sample, cb) - meanIm) * scale;
                float re = (l3_ringComponent(sample + cb, cb) - meanRe) * scale;
                float nextRe = rotRe * stepRe - rotIm * stepIm;
                float nextIm = rotRe * stepIm + rotIm * stepRe;

                sumRe += re * rotRe - im * rotIm;
                sumIm += re * rotIm + im * rotRe;
                rotRe = nextRe;
                rotIm = nextIm;
                sample += loopStride;
            }
            out->channel[tx * out->nrx + rx].re = sumRe;
            out->channel[tx * out->nrx + rx].im = sumIm;
        }
    }
}

/* The same channels for a STATIC target (the ball on its tee): the raw
 * samples summed over the loops, no mean removed and no Doppler to unwind,
 * so the TX blocks differ only by their fixed phases and the beamformer
 * reads the target's direction. */
static void l3_channelSnapshotStatic(const l3_detect_frame_t *source, uint32_t localBin, l3_angle_snapshot_t *out)
{
    const uint8_t *frame = source->base;
    uint32_t binCount = source->binCount;
    uint32_t ntx = gCapturePlan.chirpsPerFrame / gCapturePlan.loops;
    uint32_t loops = gCapturePlan.loops;
    uint32_t cb = source->cb;
    float scale = source->scale;
    uint32_t loopStride = ntx * N_RX * binCount * 2U * cb;
    uint32_t tx;

    l3_angle_snapshot_init(out, ntx, N_RX);
    out->lag1PhaseRad = 0.0F;
    out->radialVelocityMps = 0.0F;
    out->chirpPeriodS = (ntx > 0U) ? (gTrigLoopPeriodS / (float)ntx) : 45.0e-6F;
    for (tx = 0U; tx < out->ntx; tx++) {
        uint32_t rx;
        for (rx = 0U; rx < out->nrx; rx++) {
            const uint8_t *sample = &frame[((tx * N_RX + rx) * binCount + localBin) * 2U * cb];
            float sumRe = 0.0F;
            float sumIm = 0.0F;
            uint32_t loop;

            for (loop = 0U; loop < loops; loop++) {
                sumIm += l3_ringComponent(sample, cb) * scale;
                sumRe += l3_ringComponent(sample + cb, cb) * scale;
                sample += loopStride;
            }
            out->channel[tx * out->nrx + rx].re = sumRe;
            out->channel[tx * out->nrx + rx].im = sumIm;
        }
    }
}

/* --- CLI notices from the detect task ----------------------------------------
 * The detect task runs below the CLI task, so it must not write the CLI UART
 * itself: a host command arriving mid-line preempts it and the CLI task's
 * reply lands inside the notice (the host then sees half a debug line, or
 * misses "Triggered" altogether). Lines are queued here and written by
 * l3_noticeTask at the CLI task's priority, where each line goes out whole.
 * A full queue drops the line and counts it; "Triggered" is queued ahead of
 * the debug line that describes it. */
#define L3_NOTICE_LINES      4U
#define L3_NOTICE_LINE_BYTES 160U
static char gNoticeLines[L3_NOTICE_LINES][L3_NOTICE_LINE_BYTES];
static volatile uint32_t gNoticeHead;     /* next line the notice task writes */
static volatile uint32_t gNoticeTail;     /* next slot to fill */
static volatile uint32_t gNoticeDropped;
static Semaphore_Handle  gNoticeSemaphore;

static void l3_queueNotice(const char *text)
{
    uintptr_t key = Hwi_disable();
    uint32_t next = (gNoticeTail + 1U) % L3_NOTICE_LINES;

    if (gNoticeSemaphore == NULL || next == gNoticeHead) {
        gNoticeDropped++;
        Hwi_restore(key);
        return;
    }
    strncpy(gNoticeLines[gNoticeTail], text, L3_NOTICE_LINE_BYTES - 1U);
    gNoticeLines[gNoticeTail][L3_NOTICE_LINE_BYTES - 1U] = '\0';
    gNoticeTail = next;
    Hwi_restore(key);
    Semaphore_post(gNoticeSemaphore);
}

static void l3_noticeTask(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;
    while (1) {
        Semaphore_pend(gNoticeSemaphore, BIOS_WAIT_FOREVER);
        while (gNoticeHead != gNoticeTail) {
            CLI_write("%s", gNoticeLines[gNoticeHead]);
            gNoticeHead = (gNoticeHead + 1U) % L3_NOTICE_LINES;
        }
    }
}

/* Stats and debugCfg still speak the phase names the host already parses.
 * The detector itself is l3_trigger.c; these names are only a readout. */
static const char *l3_triggerPhaseName(uint8_t phase)
{
    static const char *const names[] = {
        "off", "no-frame", "bin-outside", "tee-low", "occupying",
        "watching", "no-approach", "toward", "away", "fired", "no-ball"
    };

    if (phase >= (uint8_t)(sizeof(names) / sizeof(names[0]))) {
        return "unknown";
    }
    return names[phase];
}

/* The debug line for a phase, or 0 when debug is off or the phase is
 * unchanged. The CLI task writes it directly (debugCfg 1 answers with the
 * current line before Done); the detect task queues it. */
static int32_t l3_formatTriggerDebug(uint8_t phase, char *out, uint32_t cap)
{
    if (!gTriggerDebug) {
        return 0;
    }
    if (phase == gTriggerDebugPhase) {
        return 0;
    }
    gTriggerDebugPhase = phase;
    (void)snprintf(
        out, cap,
        "trig phase=%s tee=%u approach=%u ready=%u toward=%u away=%u "
        "run=%u peak=%u have=%u bin=%u level=%u latched=%u\n",
        l3_triggerPhaseName(phase),
        (unsigned)gTriggerTeePower,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U,
        0U,
        (unsigned)gTrigCfg.teeBin,
        (unsigned)gTrigCfg.snr,
        (unsigned)gSelfTriggerLatched);
    return 1;
}

/* Detect task: record the phase and queue its debug line for the notice
 * task. Static line buffer: the detect task's stack is small and only it
 * runs this. */
static void l3_noteTrigger(uint8_t phase, float tee)
{
    static char line[L3_NOTICE_LINE_BYTES];

    gTriggerPhase = phase;
    gTriggerTeePower = (uint32_t)tee;
    if (l3_formatTriggerDebug(phase, line, sizeof(line))) {
        l3_queueNotice(line);
    }
}

/* The ring was re-armed for the next shot: forget the track and any fired
 * state, keep the noise floor, counters and log. */
/* Between shots: if the ball detector holds a lock and adaptive windows are
 * enabled, move the plan's windows to the ball and rebuild the frame tables.
 * Runs on the CLI task with the capture stopped, before the HWA restarts. */
static void l3_applyAdaptiveWindows(void)
{
    uint32_t ballBin;
    l3_adaptive_windows_t windows;

    if (!gAdaptiveCfg.enabled || gCaptureActive || !l3_ball_locked(&gBall, &ballBin)) {
        return;
    }
    if (!l3_adaptive_windows(&gAdaptiveCfg, ballBin, N_SAMPLES, gCapturePlan.preBins,
                             gCapturePlan.impactBins, gCapturePlan.postBins, &windows)) {
        return;
    }
    gAdaptiveWindows = windows;
    if (!l3_adaptive_differs(&windows, gCapturePlan.preStart, gCapturePlan.impactStart,
                             gCapturePlan.postStart, gCapturePlan.lateStart)) {
        return;
    }
    gCapturePlan.preStart = windows.preStart;
    gCapturePlan.impactStart = windows.impactStart;
    gCapturePlan.postStart = windows.postStart;
    gCapturePlan.lateStart = windows.lateStart;
    if (l3_finalizeCapturePlan(gCapturePlan.loops) == 0) {
        gAdaptiveApplied++;
    }
}

static void l3_trigRearm(void)
{
    l3_applyAdaptiveWindows();
    l3_trig_rearm(&gTrig);
    l3_track_reset(&gClubTrack);
    l3_impact_rearm(&gImpact);
    l3_shot_rearm(&gShot);
    l3_ball_track_reset(&gBallTrack);
    memset(&gLaunch, 0, sizeof(gLaunch));
    gTrigFireSource = 0U;
    gPostTimestampUs = 0U;
    gPostFramesScored = 0U;
    gBallFloor = 0.0F;
    gShotResultReady = 0U;
}

/* Stage timing: Cycleprofiler ticks at the CPU clock; one call per stage. */
static void l3_profileStage(uint32_t stage, uint32_t startTicks)
{
    if (!gProfileReady) {
        l3_profile_init(&gProfile, gCpuClock / 1000000U);
        gProfileReady = 1U;
    }
    l3_profile_add(&gProfile, stage, Cycleprofiler_getTimeStamp() - startTicks);
}

/* The calibration starts as identity; the CLI refines it. */
static void l3_ensureRadarCal(void)
{
    if (gRadarCal.virtualElements == 0U) {
        l3_cal_identity(&gRadarCal, L3_CAL_MAX_VIRTUAL);
    }
    if (!gImpactCfgSet) {
        l3_impact_cfg_defaults(&gImpactCfg);
        gImpactCfgSet = 1U;
    }
}

/* The club track's configuration follows the trigger's arm: statistic and
 * snr for target extraction, the range resolution trackCfg supplied (else the
 * 6 m / 128 default) for metres and m/s. */
static void l3_clubTrackConfigure(void)
{
    l3_track_cfg_t cfg;

    l3_track_cfg_defaults(&cfg);
    if (gTrackRangeResM > 0.0) {
        cfg.binWidthM = (float)gTrackRangeResM;
    }
    if (gTrigLoopPeriodS > 0.0F) {
        cfg.velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * gTrigLoopPeriodS);
    }
    l3_ensureRadarCal();
    cfg.cal = gRadarCal;
    l3_track_init(&gClubTrack, &cfg);
    l3_impact_init(&gImpact, &gImpactCfg);
    /* The ball track shares the club track's geometry; the shot machine
     * gives the ball tracker the whole post movie. */
    l3_ball_track_cfg_defaults(&gBallTrackCfg);
    gBallTrackCfg.core.binWidthM = cfg.binWidthM;
    gBallTrackCfg.core.velocitySpanMps = cfg.velocitySpanMps;
    gBallTrackCfg.core.cal = gRadarCal;
    l3_ball_track_init(&gBallTrack, &gBallTrackCfg);
    l3_shot_cfg_defaults(&gShotCfg);
    gShotCfg.requireBall = gBallCfg.follow;
    if (gCapturePlan.postFrames > 0U) {
        gShotCfg.ballTrackFrames = gCapturePlan.postFrames;
    }
    l3_shot_init(&gShot, &gShotCfg);
    memset(&gLaunch, 0, sizeof(gLaunch));
}

/* The shot machine's view of one pre-impact frame. Entering IMPACT arms the
 * ball tracker at the destination with the impact time: the geometric
 * detector's interpolated one when it fired, else this frame's. */
static void l3_shotObserve(uint32_t teeBin, int32_t gateFired, int32_t geometric)
{
    l3_shot_input_t in;
    uint32_t frameUs = gPreFramesCaptured * (uint32_t)gFramePeriodUs;

    memset(&in, 0, sizeof(in));
    in.ballLocked = gTrigDestBall;
    in.ballPosition = gBallPosition;
    in.clubActive = gClubTrack.active;
    in.clubPoints = gClubTrack.count;
    in.gateFired = (uint8_t)(gateFired ? 1U : 0U);
    in.geometricFired = (uint8_t)(geometric ? 1U : 0U);
    in.impactTimestampUs = (geometric && gImpact.fired) ? gImpact.impactTimestampUs : frameUs;
    in.delivery = &gDelivery;
    in.club = &gClubTrack;
    if (l3_shot_update(&gShot, &in, gPreFramesCaptured) == L3_SHOT_IMPACT &&
        gShot.impactFrame == gPreFramesCaptured) {
        l3_ball_track_arm(&gBallTrack, (float)teeBin, &gBallPosition, in.impactTimestampUs);
        gPostTimestampUs = frameUs;
    }
}

/* Every other completed pre-trigger slot: the static power of the whole
 * window, loops subsampled by four, into the ball detector. About a fifth
 * of the trigger's own cost per frame. */
static void l3_considerBall(uint32_t slot)
{
    l3_detect_frame_t frame = l3_detectFrameOf(slot);
    static float power[L3_BALL_MAX_BINS];
    uint32_t count = frame.binCount;
    uint32_t bin;
    uint32_t ticks;

    if (gBall.state == L3_BALL_STATE_OFF || gCapturePlan.loops == 0U ||
        (gPreFramesCaptured & 1U) != 0U) {
        return;
    }
    if (count > L3_BALL_MAX_BINS) {
        count = L3_BALL_MAX_BINS;
    }
    gBallBusy = 1U;
    ticks = Cycleprofiler_getTimeStamp();
    for (bin = 0U; bin < count; bin++) {
        power[bin] = l3_verticalStaticPower(&frame, bin, 4U);
    }
    if (l3_detectFrameStale(&frame)) {
        gDetectScratchStale++;
        gBallBusy = 0U;
        return;
    }
    (void)l3_ball_update(&gBall, frame.binStart, power, count);
    {
        /* The locked ball's direction from its static return. */
        uint32_t ballBin;

        if (l3_ball_locked(&gBall, &ballBin) && ballBin >= frame.binStart &&
            ballBin - frame.binStart < count) {
            static l3_angle_snapshot_t snapshot;

            l3_channelSnapshotStatic(&frame, ballBin - frame.binStart, &snapshot);
            gBallAngleValid = (uint8_t)(l3_angle_estimate(&gRadarCal, &snapshot, &gBallAngle) &&
                                        gBallAngle.elevationValid &&
                                        gBallAngle.elevationPeakRatio >=
                                            L3_BALL_ANGLE_MIN_PEAK_RATIO);
        } else {
            gBallAngleValid = 0U;
        }
    }
    l3_profileStage(L3_PROF_BALL_DETECT, ticks);
    gBallBusy = 0U;
}

/* Per kept post-impact slot: the whole window as observations, ranked
 * targets against the trigger's floor, into the ball tracker (which only
 * looks at or beyond the origin), angles for the appended point, the launch
 * fit, and the shot machine's post-impact transitions. Runs on the detect
 * task after the freeze was requested, while the post movie is filling. */
static void l3_considerBallTrack(uint32_t slot)
{
    l3_detect_frame_t frame = l3_detectFrameOf(slot);
    static l3_trig_obs_t obs[L3_TRIG_MAX_BINS];
    static l3_target_obs_t targets[L3_OBS_MAX_TARGETS];
    static l3_angle_snapshot_t snapshot;
    l3_obs_params_t params;
    l3_shot_input_t in;
    l3_track_point_t newest;
    uint32_t count = frame.binCount;
    uint32_t found;
    uint32_t bin;
    uint32_t frameIndex;
    uint32_t ticks;

    if (!gBallTrack.armed || gCapturePlan.loops == 0U) {
        return;
    }
    if (count > L3_TRIG_MAX_BINS) {
        count = L3_TRIG_MAX_BINS;
    }
    gPostTimestampUs += gFrameDeltaUs[slot];
    gPostFramesScored++;
    frameIndex = gPreFramesCaptured + gPostFramesScored;
    gTrigBusy = 1U;
    ticks = Cycleprofiler_getTimeStamp();
    for (bin = 0U; bin < count; bin++) {
        l3_verticalResidual(&frame, bin, NULL, &obs[bin]);
    }
    if (l3_detectFrameStale(&frame)) {
        gDetectScratchStale++;
        gTrigBusy = 0U;
        return;
    }
    params.stat = gTrigCfg.stat;
    params.snr = gBallTrackCfg.snr;   /* a departing ball is a weaker return than a club */
    params.loopPeriodS = gTrigLoopPeriodS;
    params.subBin = gObsSubBin;
    l3_obs_floor_update(&gBallFloor, gTrigCfg.stat, obs, count, L3_TRIG_FLOOR_SHIFT);
    found = l3_obs_extract(&params, frameIndex, gPostTimestampUs, frame.binStart, obs, count,
                           gBallFloor, targets, L3_OBS_MAX_TARGETS);
    /* After impact two tracks are visible: the club carries on (followed by
     * association only, the stronger return) beside the departing ball.
     * gDelivery was read at impact and stays the approach's. The ball tracker
     * does not use the club's claim yet: on the 2026-09-27 captures that lost
     * more balls than it saved. */
    (void)l3_track_follow(&gClubTrack, targets, found, frameIndex, gPostTimestampUs);
    if (l3_ball_track_update_joint(&gBallTrack, targets, found, frameIndex, gPostTimestampUs,
                                   gClubTrack.lastTargetIndex) &&
        gBallTrack.lastTargetIndex < found && gBallTrack.core.count > 1U &&
        l3_track_point(&gBallTrack.core, gBallTrack.core.count - 1U, &newest)) {
        const l3_target_obs_t *hit = &targets[gBallTrack.lastTargetIndex];

        {
            l3_angle_obs_t angle;

            l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart,
                               hit->dopplerPhaseRad, newest.radialVelocityMps, &snapshot);
            if (l3_angle_estimate(&gRadarCal, &snapshot, &angle)) {
                uint8_t flags = 0U;

                if (angle.azimuthValid) {
                    flags |= L3_OBS_ANGLE_AZIMUTH;
                }
                if (angle.elevationValid) {
                    flags |= L3_OBS_ANGLE_ELEVATION;
                }
                (void)l3_ball_track_set_angles(&gBallTrack, angle.azimuthRad,
                                               angle.elevationRad, flags);
                gAngleEstimates++;
            }
        }
    }
#if L3_BALL_HYPOTHESES
    {
        /* Angles for every ball-hypothesis point this frame appended: once one
         * is chosen, its early points must still carry them. */
        uint32_t index;

        for (index = 0U; index < L3_BALL_HYP_MAX; index++) {
            const l3_ball_hyp_t *hyp = &gBallTrack.hyps.hyp[index];
            const l3_target_obs_t *hit;
            l3_angle_obs_t angle;
            float rate;
            float at;
            float residual;
            float radial = 0.0F;

            if (!hyp->active || hyp->lastTargetIndex >= found) {
                continue;
            }
            hit = &targets[hyp->lastTargetIndex];
            if (l3_ball_hyp_fit(hyp, hyp->points[hyp->count - 1U].timestampUs, &rate, &at,
                                &residual)) {
                radial = rate * gBallTrack.hyps.cfg.binWidthM;
            }
            l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart,
                               hit->dopplerPhaseRad, radial, &snapshot);
            if (l3_angle_estimate(&gRadarCal, &snapshot, &angle)) {
                uint8_t flags = 0U;

                if (angle.azimuthValid) {
                    flags |= L3_OBS_ANGLE_AZIMUTH;
                }
                if (angle.elevationValid) {
                    flags |= L3_OBS_ANGLE_ELEVATION;
                }
                (void)l3_ball_hyps_set_angles(&gBallTrack.hyps, index, angle.azimuthRad,
                                              angle.elevationRad, flags);
                gAngleEstimates++;
            }
        }
    }
#endif /* L3_BALL_HYPOTHESES */
    (void)l3_ball_track_launch(&gBallTrack, &gLaunch);
    l3_profileStage(L3_PROF_BALL_TRACK, ticks);
    gTrigBusy = 0U;
    /* IMPACT -> BALL_TRACK -> SOLVE on the frames; SOLVE -> RESULT at once,
     * the launch fit being the solver. */
    memset(&in, 0, sizeof(in));
    in.ballLocked = gTrigDestBall;
    in.ballPosition = gBallPosition;
    in.postFrame = 1U;
    in.ballTrackDone = gBallTrack.done;
    in.delivery = &gDelivery;
    in.club = &gClubTrack;
    if (l3_shot_update(&gShot, &in, frameIndex) == L3_SHOT_SOLVE) {
        in.solved = 1U;
        (void)l3_shot_update(&gShot, &in, frameIndex);
    }
    if (gShot.state == L3_SHOT_RESULT && !gShotResultReady) {
        l3_result_build(&gShot, &gBallTrack, &gLaunch, ++gShotId, gTrigDestBall, &gShotResult);
        gShotResultReady = 1U;
    }
}

/* Per completed pre-trigger slot: reduce the watch region to one observation
 * per bin and hand it to the detector. The detect task passes the slot it
 * popped, so a slow read does not score a frame the ring has reused. */
static void l3_considerSelfTrigger(uint32_t slot)
{
    l3_detect_frame_t frame = l3_detectFrameOf(slot);
    /* Static: this runs on the detect task, whose stack is small. */
    static l3_trig_obs_t obs[L3_TRIG_MAX_BINS];
    uint32_t first;
    uint32_t count;
    uint32_t bin;
    int32_t fired;
    int32_t geometric = 0;
    l3_track_point_t newest;
    uint32_t ticks;
    uintptr_t key;

    uint32_t teeBin = gTrigCfg.teeBin;

    if (!gTriggerEnabled) {
        l3_noteTrigger(0U, 0.0F);
        return;
    }
    if (gSelfTriggerLatched || gHwaFreezeRequested || gPostCaptureStarted) {
        l3_noteTrigger(9U, (float)gTriggerTeePower);
        return;
    }
    /* Freezing before the ring has wrapped would hand the host pre-trigger
     * slots this session never wrote. */
    if (gCapturePlan.preFrames == 0U || gCapturePlan.loops == 0U ||
        gPreFramesCaptured < gCapturePlan.preFrames) {
        l3_noteTrigger(1U, 0.0F);
        return;
    }
    /* Following the ball detector, the destination is where the ball was
     * placed, in global bins (ten captures put the club at 2.2-2.4 m while
     * a hand-measured tee said 1.575 m). Without a locked ball the
     * configured tee stands in, motion only, and the fallback is counted. */
    gTrigDestBall = 0U;
    if (gBallCfg.follow) {
        if (l3_ball_locked(&gBall, &teeBin)) {
            gTrigDestBall = 1U;
        } else {
            teeBin = gTrigCfg.teeBin;
            gTrigFallbackFrames++;
        }
    }
    if (!l3_trig_region(&gTrigCfg, teeBin, frame.binStart, frame.binCount,
                        &first, &count)) {
        l3_noteTrigger(2U, 0.0F);
        return;
    }
    gTrigBusy = 1U;
    ticks = Cycleprofiler_getTimeStamp();
    for (bin = 0U; bin < count; bin++) {
        l3_verticalResidual(&frame, first + bin, NULL, &obs[bin]);
    }
    l3_profileStage(L3_PROF_RESIDUAL, ticks);
    if (l3_detectFrameStale(&frame)) {
        /* The scratch went back to the HWA before this read finished. */
        gDetectScratchStale++;
        gTrigBusy = 0U;
        l3_noteTrigger(1U, 0.0F);
        return;
    }
    gTrig.loopPeriodS = gTrigLoopPeriodS;
    ticks = Cycleprofiler_getTimeStamp();
    fired = l3_trig_update(&gTrig, gPreFramesCaptured, teeBin, frame.binStart + first,
                           obs, count);
    l3_profileStage(L3_PROF_TRIGGER, ticks);
    {
        /* The same observations, as ranked targets, into the club track. */
        static l3_target_obs_t targets[L3_OBS_MAX_TARGETS];
        l3_obs_params_t params;
        uint32_t found;
        int32_t appended;

        params.stat = gTrigCfg.stat;
        params.snr = gTrigCfg.snr;
        params.loopPeriodS = gTrigLoopPeriodS;
        params.subBin = gObsSubBin;
        ticks = Cycleprofiler_getTimeStamp();
        found = l3_obs_extract(&params, gPreFramesCaptured,
                               gPreFramesCaptured * (uint32_t)gFramePeriodUs,
                               frame.binStart + first, obs, count, gTrig.floor,
                               targets, L3_OBS_MAX_TARGETS);
        l3_profileStage(L3_PROF_EXTRACT, ticks);
        gClubTrackDest = teeBin;
        ticks = Cycleprofiler_getTimeStamp();
        appended = l3_track_update(&gClubTrack, targets, found, gPreFramesCaptured,
                                   gPreFramesCaptured * (uint32_t)gFramePeriodUs);
        l3_profileStage(L3_PROF_CLUB_TRACK, ticks);
        if (appended && gClubTrack.lastTargetIndex < found && gClubTrack.count > 1U &&
            l3_track_point(&gClubTrack, gClubTrack.count - 1U, &newest)) {
            /* Angles for the associated target only: one estimate per frame.
             * The track's range-rate velocity resolves the TDM alias, so the
             * first point of a track (no range rate yet) stays range-only. */
            static l3_angle_snapshot_t snapshot;
            const l3_target_obs_t *hit = &targets[gClubTrack.lastTargetIndex];

            ticks = Cycleprofiler_getTimeStamp();
            l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart,
                               hit->dopplerPhaseRad, newest.radialVelocityMps, &snapshot);
            if (l3_angle_estimate(&gRadarCal, &snapshot, &gLastAngle)) {
                uint8_t flags = 0U;

                if (gLastAngle.azimuthValid) {
                    flags |= L3_OBS_ANGLE_AZIMUTH;
                }
                if (gLastAngle.elevationValid) {
                    flags |= L3_OBS_ANGLE_ELEVATION;
                }
                (void)l3_track_set_angles(&gClubTrack, gLastAngle.azimuthRad,
                                          gLastAngle.elevationRad, flags);
                gAngleEstimates++;
            }
            l3_profileStage(L3_PROF_ANGLE, ticks);
        }
        /* The delivery and the geometric impact verdict, every frame. The
         * destination bin (locked ball, else the tee) on boresight is the
         * ball position until the ball detector measures its angles. */
        ticks = Cycleprofiler_getTimeStamp();
        (void)l3_track_delivery(&gClubTrack, 8U, &gDelivery);
        if (gTrigDestBall && gBallAngleValid) {
            l3_frames_observe(&gRadarCal, (float)teeBin * gClubTrack.cfg.binWidthM,
                              gBallAngle.azimuthValid ? gBallAngle.azimuthRad : 0.0F,
                              gBallAngle.elevationRad, &gBallPosition);
        } else {
            l3_frames_observe(&gRadarCal, (float)teeBin * gClubTrack.cfg.binWidthM, 0.0F, 0.0F,
                              &gBallPosition);
        }
        geometric = l3_impact_update(&gImpact, &gDelivery, &gBallPosition, 1U);
        l3_profileStage(L3_PROF_IMPACT, ticks);
    }
    if (gProfileReady) {
        l3_profile_frame(&gProfile);
    }
    gTrigBusy = 0U;
    gTrigFireSource = (uint8_t)((fired ? 1U : 0U) | (geometric ? 2U : 0U));
    l3_shotObserve(teeBin, fired, geometric && gImpactArmed);
    if (geometric && gImpactArmed) {
        fired = 1;
    }
    if (!fired) {
        l3_noteTrigger(gTrig.state == L3_TRIG_STATE_TRACKING ? 7U : 5U, gTrig.floor);
        return;
    }
    key = Hwi_disable();
    gHwaFreezeRequested = 1U;
    gPostCaptureStarted = 0U;
    gPostFramesCaptured = 0U;
    gPostFramesObserved = 0U;
    gActiveFrameShouldKeep = 1U;
    gSelfTriggerLatched = 1U;
    gHwaFreezeRequests++;
    Hwi_restore(key);
    /* The notice first: the host's S! waits on it, the debug line does not. */
    l3_queueNotice("Triggered\n");
    l3_noteTrigger(9U, gTrig.floor);
}

#ifdef HWA_CHAINED_SNAPSHOT_RING
static void l3_detectTask(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;
    while (1) {
        uint16_t queuedSlot = 0U;
        uint32_t epoch = 0U;

        Semaphore_pend(gDetectSemaphore, BIOS_WAIT_FOREVER);
        if (!l3detect_pop(&gDetectQueue, &queuedSlot, &epoch)) {
            continue;
        }
        if (epoch == L3_DETECT_POST_EPOCH && queuedSlot >= gCapturePlan.preFrames) {
            l3_considerBallTrack(queuedSlot);
            continue;
        }
        if (!l3detect_slot_live(epoch, gPreFramesCaptured, gCapturePlan.preFrames)) {
            gDetectStale++;
            continue;
        }
        l3_considerBall(queuedSlot);
        l3_considerSelfTrigger(queuedSlot);
    }
}
#endif

/* The frozen frames a sparse command streams, oldest first. */
typedef struct {
    uint32_t slots[L3_MAX_CAPTURE_FRAMES];
    uint8_t  starts[L3_MAX_CAPTURE_FRAMES];
    uint8_t  counts[L3_MAX_CAPTURE_FRAMES];
    uint32_t nFrames;
    uint32_t maxBins;
} l3_sparse_window_t;

/* Wait until a self-trigger freeze has landed, or stop at the next frame
 * boundary. Does not stream and does not read a follow-up CLI line.
 * The HWA freeze only stops re-arm; the BSS keeps chirping until
 * l3_finishCaptureStop, and MMWave_start refuses a sensor that is still running. */
static int32_t l3_awaitFrozenRing(void)
{
    if (!gCaptureActive && !gSelfTriggerLatched) {
        return -1;
    }
    if (gSelfTriggerLatched) {
        if (gCaptureActive && gHwaFreezeSemaphore != NULL &&
            !Semaphore_pend(gHwaFreezeSemaphore, 250U)) {
            CLI_write("Error: self-trigger freeze timed out\n");
            gSelfTriggerLatched = 0U;
            return -1;
        }
        gSelfTriggerLatched = 0U;
        return l3_finishCaptureStop();
    }
    if (l3_stopCaptureAtBoundary() != 0) {
        return -1;
    }
    return 0;
}

/* Freeze the ring for l3sparse or l3track. A self-trigger has already
 * requested the freeze; otherwise stop at the next frame boundary. */
static int32_t l3_sparseFreeze(void)
{
#ifdef L3_RING_IQ8
    if (l3_captureUsesIq8()) {
        CLI_write("Error: sparse dump requires IQ16 storage\n");
        return -1;
    }
#endif
    return l3_awaitFrozenRing();
}

static void l3_sparseWindow(l3_sparse_window_t *window)
{
    uint32_t frame;
    uint32_t actualPre;
    uint32_t actualPost;
    uint32_t oldestPre;

    window->nFrames = 0U;
    window->maxBins = 0U;
    actualPre = (gPreFramesCaptured < gCapturePlan.preFrames)
                    ? gPreFramesCaptured : gCapturePlan.preFrames;
    actualPost = (gPostFramesCaptured < gCapturePlan.postFrames)
                     ? gPostFramesCaptured : gCapturePlan.postFrames;
    oldestPre = (gPreFramesCaptured >= gCapturePlan.preFrames)
                    ? (gPreFramesCaptured % gCapturePlan.preFrames) : 0U;
    for (frame = 0U; frame < actualPre && window->nFrames < L3_MAX_CAPTURE_FRAMES; frame++) {
        window->slots[window->nFrames] = (oldestPre + frame) % gCapturePlan.preFrames;
        window->nFrames++;
    }
    for (frame = 0U; frame < actualPost && window->nFrames < L3_MAX_CAPTURE_FRAMES; frame++) {
        window->slots[window->nFrames] = gCapturePlan.preFrames + frame;
        window->nFrames++;
    }
    for (frame = 0U; frame < window->nFrames; frame++) {
        window->starts[frame] = gFrameBinStart[window->slots[frame]];
        window->counts[frame] = gFrameBinCount[window->slots[frame]];
        if (window->counts[frame] > window->maxBins) {
            window->maxBins = window->counts[frame];
        }
    }
}

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
/* ILP1/ILT1 header and per-frame window table (sparse.CaptureLayout). */
static void l3_sparseWriteHeader(const char *magic, const l3_sparse_window_t *window)
{
    uint32_t frame;

    UART_writePolling(gDataUart, (uint8_t *)magic, 4U);
    l3_writeU16((uint16_t)window->nFrames);
    l3_writeU16((uint16_t)gCapturePlan.loops);
    l3_writeU16((uint16_t)window->maxBins);
    l3_writeU16((uint16_t)(gCapturePlan.chirpsPerFrame / gCapturePlan.loops));
    l3_writeU16((uint16_t)N_RX);
    l3_writeU16(window->nFrames ? window->starts[0] : 0U);
    l3_writeU16(gFramePeriodUs);
    /* Zero tells the host to keep its own noise floor. A summed-power median
     * is not the per-sample floor LCMF divides by. */
    l3_writeF32(0.0F);
    for (frame = 0U; frame < window->nFrames; frame++) {
        uint8_t pair[2];
        pair[0] = window->starts[frame];
        pair[1] = window->counts[frame];
        UART_writePolling(gDataUart, pair, sizeof(pair));
    }
}
#endif

#ifdef L3_SPARSE_READBACK /* only l3sparse / l3track use this */
/* One ILS1 cell: (frame, local bin) then the vertical TX pair's samples. */
static void l3_sparseWriteCell(const l3_sparse_window_t *window,
                               uint32_t frameIndex, uint32_t localBin)
{
    uint32_t ntx = gCapturePlan.chirpsPerFrame / gCapturePlan.loops;
    uint32_t verticalChirps = gCapturePlan.loops * ((ntx == 3U) ? 2U : ntx);
    uint32_t vertical;

    l3_writeU16((uint16_t)frameIndex);
    l3_writeU16((uint16_t)localBin);
    for (vertical = 0U; vertical < verticalChirps; vertical++) {
        uint32_t loop = vertical / ((ntx == 3U) ? 2U : ntx);
        uint32_t which = vertical % ((ntx == 3U) ? 2U : ntx);
        uint32_t tx = (ntx == 3U && which == 1U) ? 2U : which;
        uint32_t chirp = loop * ntx + tx;
        uint32_t rx;
        for (rx = 0U; rx < N_RX; rx++) {
            if (frameIndex >= window->nFrames || localBin >= window->counts[frameIndex]) {
                l3_writeU16(0U);
                l3_writeU16(0U);
            } else {
                const int16_t *sample = l3_iq16Sample(
                    window->slots[frameIndex], chirp, rx, localBin);
                l3_writeU16((uint16_t)sample[0]);
                l3_writeU16((uint16_t)sample[1]);
            }
        }
    }
}
#endif

/* Clear the capture state and restart the ring after a sparse command. */
static int32_t l3_sparseRearm(void)
{
    gRingFrame = 0U;
    gHwaFreezeRequestFrame = 0U;
    gPreFramesCaptured = 0U;
    l3_trigRearm();
    gPostFramesCaptured = 0U;
    gPostFramesObserved = 0U;
    gPostCaptureStarted = 0U;
    gActiveFrameIsPost = 0U;
    gActiveFrameShouldKeep = 1U;
    l3_resetDetectQueue();
    if (l3_restartCompletedHwaFrame() < 0) {
        CLI_write("Error: completed HWA frame restart failed\n");
        gCaptureActive = 0U;
        return -1;
    }
    gCaptureActive = 1U;
    if (l3_startFrontEnd() < 0) {
        CLI_write("Error: RF restart failed\n");
        gCaptureActive = 0U;
        return -1;
    }
    return 0;
}

/* CLI "l3release": rearm after a self-trigger without streaming the power
 * map. The SCI receiver holds one byte, so a cell line written while that
 * map is going out is lost before l3_readLine runs. */
static int32_t l3_cli_release(int32_t argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (l3_awaitFrozenRing() != 0) {
        return -1;
    }
    return l3_sparseRearm();
}

/* Sparse readback (l3sparse, l3track): freeze the ring and stream chosen
 * cells instead of the whole capture. Off by default: its buffers cost
 * ~22 KB of DATA_RAM (gTrackWorkspace alone is 17 KB), and adaptive16
 * captures are read whole with l3dump. The stubs answer "Error:", which
 * the host treats as a refusal before freezing and falls back to l3dump.
 * Build with --define=L3_SPARSE_READBACK=1 to restore them. */
#ifndef L3_SPARSE_READBACK
int32_t l3_cli_sparse(int32_t argc, char *argv[])
{
    (void)argc;
    (void)argv;
    CLI_write("Error: l3sparse is not in this build (L3_SPARSE_READBACK)\n");
    return -1;
}
#else
int32_t l3_cli_sparse(int32_t argc, char *argv[])
{
    l3_sparse_window_t window;
    uint32_t frame;
    char request[L3_SPARSE_REQUEST_MAX];
    static uint16_t cellFrames[L3_SPARSE_REQUEST_MAX / 4U];
    static uint16_t cellBins[L3_SPARSE_REQUEST_MAX / 4U];
    static float powerRow[L3_MAX_LOOPS * L3_RING_MAX_BINS];
    char *cursor;
    char *next;
    int32_t lineStatus;
    int32_t cellCount;
    int32_t cell;
    (void)argc;
    (void)argv;
    if (l3_sparseFreeze() != 0) {
        return -1;
    }
    l3_sparseWindow(&window);
    l3_sparseWriteHeader("ILP1", &window);
    for (frame = 0U; frame < window.nFrames; frame++) {
        uint32_t loop;
        uint32_t bin;
        uint32_t maxBins = window.maxBins;
        float perLoop[L3_MAX_LOOPS];
        for (bin = 0U; bin < maxBins; bin++) {
            if (bin < window.counts[frame]) {
                l3_verticalPowerLoops(window.slots[frame], bin, perLoop);
            }
            for (loop = 0U; loop < gCapturePlan.loops; loop++) {
                powerRow[loop * maxBins + bin] =
                    (bin < window.counts[frame]) ? perLoop[loop] : 0.0F;
            }
        }
        for (loop = 0U; loop < gCapturePlan.loops; loop++) {
            UART_writePolling(gDataUart, (uint8_t *)&powerRow[loop * maxBins],
                              maxBins * sizeof(float));
        }
    }
    lineStatus = l3_readLine(request, sizeof(request));
    /* A stray CR/LF in the FIFO is not the cell request. "\r\n" is two empty
     * reads; skip those and take the line that follows. A timeout is not
     * retried, so a missing request still fails in one 5s wait. */
    if (lineStatus == L3_READLINE_EMPTY) {
        uint32_t emptyReads = 1U;
        while (lineStatus == L3_READLINE_EMPTY && emptyReads < 3U) {
            emptyReads++;
            lineStatus = l3_readLine(request, sizeof(request));
        }
    }
    if (lineStatus == L3_READLINE_OVERFLOW) {
        CLI_write("Error: sparse cell request longer than L3_SPARSE_REQUEST_MAX\n");
        return l3_sparseRearm();
    }
    if (lineStatus != 0) {
        CLI_write("Error: sparse cell request missing\n");
        return l3_sparseRearm();
    }
    cursor = request;
    while (*cursor != '\0' && *cursor != ' ') {
        cursor++;
    }
    cellCount = 0;
    if (*cursor == ' ') {
        long claimed = strtol(cursor, &cursor, 10);
        while (claimed > 0 && cellCount < (int32_t)(sizeof(cellFrames) / sizeof(cellFrames[0]))) {
            long frameValue = strtol(cursor, &next, 10);
            long binValue;
            if (next == cursor) {
                break;
            }
            cursor = next;
            binValue = strtol(cursor, &next, 10);
            if (next == cursor) {
                break;
            }
            cursor = next;
            cellFrames[cellCount] = (uint16_t)((frameValue < 0) ? 0 : frameValue);
            cellBins[cellCount] = (uint16_t)((binValue < 0) ? 0 : binValue);
            cellCount++;
            claimed--;
        }
    }
    UART_writePolling(gDataUart, (uint8_t *)"ILS1", 4U);
    l3_writeU16((uint16_t)cellCount);
    for (cell = 0; cell < cellCount; cell++) {
        l3_sparseWriteCell(&window, cellFrames[cell], cellBins[cell]);
    }
    return l3_sparseRearm();
}
#endif

#ifndef L3_SPARSE_READBACK
int32_t l3_cli_track(int32_t argc, char *argv[])
{
    (void)argc;
    (void)argv;
    CLI_write("Error: l3track is not in this build (L3_SPARSE_READBACK)\n");
    return -1;
}
#else
/* Firmware ball tracker (track_select.c). trackCfg supplies the rig limits;
 * the algorithm constants live in l3track_default_params. */
static L3TrackWorkspace gTrackWorkspace;
static L3TrackParams gTrackParams;
static double gTrackLoopPeriodS;
static double gTrackRangeResM;
static uint8_t gTrackConfigured;

static void l3_trackRow(void *ctx, uint32_t frame, uint32_t loop,
                        float *out, uint32_t count)
{
    const l3_sparse_window_t *window = (const l3_sparse_window_t *)ctx;
    uint32_t bin;

    for (bin = 0U; bin < count; bin++) {
        float perLoop[L3_MAX_LOOPS];

        l3_verticalPowerLoops(window->slots[frame], bin, perLoop);
        out[bin] = perLoop[loop];
    }
}

/* CLI "l3track": freeze, find the ball and club cells on-chip, then stream
 * an ILT1 layout + track record and the ILS1 cells. No host round trip. */
int32_t l3_cli_track(int32_t argc, char *argv[])
{
    l3_sparse_window_t window;
    L3TrackLayout layout;
    L3TrackResult result;
    L3TrackRng rng;
    int32_t cellCount;
    uint32_t frame;
    (void)argc;
    (void)argv;
    if (!gTrackConfigured) {
        CLI_write("Error: l3track needs trackCfg\n");
        return -1;
    }
    if (l3_sparseFreeze() != 0) {
        return -1;
    }
    l3_sparseWindow(&window);
    layout.nFrames = window.nFrames;
    layout.nLoops = gCapturePlan.loops;
    layout.maxBins = window.maxBins;
    layout.binStarts = window.starts;
    layout.binCounts = window.counts;
    /* sparse._parse_layout: a zero period reads as 3 ms. */
    layout.framePeriodS = (gFramePeriodUs != 0U) ? ((double)gFramePeriodUs / 1e6) : 0.003;
    layout.loopPeriodS = gTrackLoopPeriodS;
    layout.rangeResM = gTrackRangeResM;
    l3track_rng_seed(&rng, 1U);
    cellCount = l3track_select(&layout, &gTrackParams, l3_trackRow, &window,
                               l3track_rng_pair, &rng, &gTrackWorkspace, &result);
    if (cellCount < 0) {
        CLI_write("Error: track layout exceeds firmware limits\n");
        (void)l3_sparseRearm();
        return -1;
    }
    l3_sparseWriteHeader("ILT1", &window);
    l3_writeU16(result.found ? 1U : 0U);
    l3_writeU16((uint16_t)result.nInliers);
    l3_writeF32((float)result.slopeBins);
    l3_writeF32((float)result.interceptBins);
    l3_writeF32((float)result.rmsBins);
    l3_writeF32((float)result.tFirstS);
    l3_writeF32((float)result.tLastS);
    UART_writePolling(gDataUart, (uint8_t *)"ILS1", 4U);
    l3_writeU16((uint16_t)cellCount);
    for (frame = 0U; frame < window.nFrames; frame++) {
        uint32_t bin;
        for (bin = 0U; bin < L3T_MAX_BINS; bin++) {
            if (((gTrackWorkspace.cellMask[frame] >> bin) & 1U) != 0U) {
                l3_sparseWriteCell(&window, frame, bin);
            }
        }
    }
    return l3_sparseRearm();
}
#endif

/* Parse count floats from argv[first..], any value; 0 on success. */
static int32_t l3_parseFloats(int32_t argc, char *argv[], int32_t first, uint32_t count,
                              float *values)
{
    uint32_t i;

    if (argc != first + (int32_t)count) {
        return -1;
    }
    for (i = 0U; i < count; i++) {
        char *end;
        double value = strtod(argv[first + (int32_t)i], &end);

        if (*end != '\0' || end == argv[first + (int32_t)i]) {
            return -1;
        }
        values[i] = (float)value;
    }
    return 0;
}

/* "trackCfg cal <pitchDeg> <yawDeg> <rollDeg> <azOffsetRad> <elOffsetDeg>
 * <rangeBiasM>": the enclosure attitude and the baseline zeros of
 * l3_frames.h. Takes effect for the club track on the next triggerCfg. */
static int32_t l3_cli_trackCfgCal(int32_t argc, char *argv[])
{
    float values[6];

    if (l3_parseFloats(argc, argv, 2, 6U, values) != 0) {
        CLI_write("Error: trackCfg cal <pitchDeg> <yawDeg> <rollDeg> <azOffsetRad> "
                  "<elOffsetDeg> <rangeBiasM>\n");
        return -1;
    }
    l3_ensureRadarCal();
    gRadarCal.radarPitchRad = values[0] * (L3_FRAMES_PI / 180.0F);
    gRadarCal.radarYawRad = values[1] * (L3_FRAMES_PI / 180.0F);
    gRadarCal.radarRollRad = values[2] * (L3_FRAMES_PI / 180.0F);
    gRadarCal.azimuthOffsetRad = values[3];
    gRadarCal.elevationOffsetRad = values[4] * (L3_FRAMES_PI / 180.0F);
    gRadarCal.rangeBiasM = values[5];
    CLI_write("Done\n");
    return 0;
}

/* "trackCfg elem <index> <phaseRad> <gain>": one virtual element's
 * correction in PHYSICAL order, as the calibration file stores it:
 * correction = exp(-j phase) / gain. */
static int32_t l3_cli_trackCfgElem(int32_t argc, char *argv[])
{
    float values[3];
    uint32_t index;

    if (l3_parseFloats(argc, argv, 2, 3U, values) != 0 || values[0] < 0.0F ||
        values[0] >= (float)L3_CAL_MAX_VIRTUAL || values[2] <= 0.0F) {
        CLI_write("Error: trackCfg elem <index 0..7> <phaseRad> <gain>\n");
        return -1;
    }
    l3_ensureRadarCal();
    index = (uint32_t)values[0];
    if (l3_cal_set_element(&gRadarCal, index, values[2], values[1]) != 0) {
        CLI_write("Error: trackCfg elem <index 0..7> <phaseRad> <gain>\n");
        return -1;
    }
    CLI_write("Done\n");
    return 0;
}

/* "trackCfg impact <toleranceM> <horizonS> <minSpeedMps> <minConfidence>
 * <armed>": the geometric impact detector. armed 0 records its verdicts
 * beside the range gate without firing; 1 lets it fire the capture. */
static int32_t l3_cli_trackCfgImpact(int32_t argc, char *argv[])
{
    float values[5];

    if (l3_parseFloats(argc, argv, 2, 5U, values) != 0 || values[0] <= 0.0F ||
        values[1] <= 0.0F) {
        CLI_write("Error: trackCfg impact <toleranceM> <horizonS> <minSpeedMps> "
                  "<minConfidence> <armed>\n");
        return -1;
    }
    l3_ensureRadarCal();
    gImpactCfg.toleranceM = values[0];
    gImpactCfg.horizonS = values[1];
    gImpactCfg.minSpeedMps = values[2];
    gImpactCfg.minConfidence = values[3];
    gImpactArmed = (values[4] != 0.0F) ? 1U : 0U;
    l3_impact_init(&gImpact, &gImpactCfg);
    CLI_write("Done\n");
    return 0;
}

/* CLI "trackCfg <loopPeriodS> <rangeResM> <maxRangeM> <clubLoM> <clubHiM>":
 * the rig limits from IWR6843Runtime.track_config_command. maxRangeM of 0
 * disables the net clamp; clubHiM <= clubLoM disables the club cells.
 * Sub-modes cal, elem and impact configure the geometry stack above. */
static int32_t l3_cli_trackCfg(int32_t argc, char *argv[])
{
    double values[5];
    char *end;
    int32_t i;

    if (argc >= 2 && strcmp(argv[1], "cal") == 0) {
        return l3_cli_trackCfgCal(argc, argv);
    }
    if (argc >= 2 && strcmp(argv[1], "elem") == 0) {
        return l3_cli_trackCfgElem(argc, argv);
    }
    if (argc >= 2 && strcmp(argv[1], "impact") == 0) {
        return l3_cli_trackCfgImpact(argc, argv);
    }
    if (argc == 3 && strcmp(argv[1], "subbin") == 0) {
        /* "trackCfg subbin centroid|parabolic": how targets read their
         * sub-bin range (l3_observation.h). */
        if (strcmp(argv[2], "centroid") == 0) {
            gObsSubBin = L3_OBS_SUBBIN_CENTROID;
        } else if (strcmp(argv[2], "parabolic") == 0) {
            gObsSubBin = L3_OBS_SUBBIN_PARABOLIC;
        } else {
            CLI_write("Error: trackCfg subbin centroid|parabolic\n");
            return -1;
        }
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 3 && strcmp(argv[1], "window") == 0) {
        /* "trackCfg window none|hann": the range window the detector scores
         * bins through (l3_iq16_stats.h). Send it before triggerCfg, which
         * restarts the noise floor the old window's statistics built. */
        if (strcmp(argv[2], "none") == 0) {
            gRangeWindow = L3_RANGE_WINDOW_NONE;
        } else if (strcmp(argv[2], "hann") == 0) {
            gRangeWindow = L3_RANGE_WINDOW_HANN;
        } else {
            CLI_write("Error: trackCfg window none|hann\n");
            return -1;
        }
        CLI_write("Done\n");
        return 0;
    }
    if (argc != 6) {
        CLI_write("Error: trackCfg <loopPeriodS> <rangeResM> <maxRangeM> <clubLoM> <clubHiM> "
                  "| cal ... | elem ... | impact ... | subbin ... | window ...\n");
        return -1;
    }
    for (i = 0; i < 5; i++) {
        values[i] = strtod(argv[i + 1], &end);
        if (*end != '\0' || !(values[i] >= 0.0)) {
            CLI_write("Error: trackCfg value\n");
            return -1;
        }
    }
    if (values[0] <= 0.0 || values[1] <= 0.0) {
        CLI_write("Error: trackCfg period and resolution must be positive\n");
        return -1;
    }
    gTrackRangeResM = values[1];
#ifdef L3_SPARSE_READBACK
    l3track_default_params(&gTrackParams);
    gTrackLoopPeriodS = values[0];
    gTrackParams.maxRangeM = values[2];
    gTrackParams.clubGate.loM = values[3];
    gTrackParams.clubGate.hiM = values[4];
    gTrackConfigured = 1U;
#endif
    CLI_write("Done\n");
    return 0;
}

/* Longest triggerCfg waits for the detect task to finish scoring a frame. */
#define L3_TRIGGER_CFG_WAIT_MS 50U

/* CLI "triggerCfg <bin> <snr> <frames> [approach gate minCoh minStep stat
 * minSpeed]": arm the approaching-clubhead detector around the tee bin, a
 * GLOBAL range-FFT bin (the host converts the tee range; bin 34 is 1.59 m
 * on a 128-point FFT over 6 m). A candidate needs a residual statistic of at least <snr> times the
 * running noise floor; its track needs <frames> observations before
 * entering the impact gate fires the capture. frames of 0 disables the
 * trigger. The optional values are the bins watched short of the tee, the
 * gate half-width in bins, the minimum Doppler coherence (0..1, 0 = off),
 * the minimum mean approach rate in bins per frame, the statistic (0 =
 * energy over all loops, 1 = strongest loop) and the minimum apparent
 * Doppler speed of a candidate in m/s (0 = off). Re-arming clears the log. */
static int32_t l3_cli_triggerCfg(int32_t argc, char *argv[])
{
    l3_trig_cfg_t cfg;
    unsigned long value;
    char *end;

    if (argc < 4 || argc > 11) {
        CLI_write("Error: triggerCfg <globalBin> <snr> <frames> "
                  "[approach gate minCoh minStep stat minSpeed minApproach]\n");
        return -1;
    }
    l3_trig_cfg_defaults(&cfg);
    value = strtoul(argv[1], &end, 10);
    if (*end != '\0') {
        CLI_write("Error: trigger bin\n");
        return -1;
    }
    cfg.teeBin = (uint32_t)value;
    cfg.snr = strtof(argv[2], &end);
    if (*end != '\0') {
        CLI_write("Error: trigger snr\n");
        return -1;
    }
    value = strtoul(argv[3], &end, 10);
    if (*end != '\0') {
        CLI_write("Error: trigger frames\n");
        return -1;
    }
    cfg.trackFrames = (uint32_t)value;
    if (argc > 4) {
        value = strtoul(argv[4], &end, 10);
        if (*end != '\0') {
            CLI_write("Error: trigger approach bins\n");
            return -1;
        }
        cfg.approachBins = (uint32_t)value;
    }
    if (argc > 5) {
        value = strtoul(argv[5], &end, 10);
        if (*end != '\0') {
            CLI_write("Error: trigger gate bins\n");
            return -1;
        }
        cfg.gateBins = (uint32_t)value;
    }
    if (argc > 6) {
        cfg.minCoherence = strtof(argv[6], &end);
        if (*end != '\0') {
            CLI_write("Error: trigger min coherence\n");
            return -1;
        }
    }
    if (argc > 7) {
        cfg.minStepBins = strtof(argv[7], &end);
        if (*end != '\0') {
            CLI_write("Error: trigger min step\n");
            return -1;
        }
    }
    if (argc > 8) {
        value = strtoul(argv[8], &end, 10);
        if (*end != '\0') {
            CLI_write("Error: trigger stat\n");
            return -1;
        }
        cfg.stat = (uint32_t)value;
    }
    if (argc > 9) {
        cfg.minSpeedMps = strtof(argv[9], &end);
        if (*end != '\0') {
            CLI_write("Error: trigger min speed\n");
            return -1;
        }
    }
    if (argc > 10) {
        value = strtoul(argv[10], &end, 10);
        if (*end != '\0') {
            CLI_write("Error: trigger min approach bins\n");
            return -1;
        }
        cfg.minApproachBins = (uint32_t)value;
    }
    if (cfg.trackFrames != 0U && l3_trig_cfg_check(&cfg) != 0) {
        CLI_write("Error: trigger config (snr >= 1, gate < approach <= %u, "
                  "minApproach <= approach)\n",
                  (unsigned)L3_TRIG_MAX_BINS);
        return -1;
    }
    /* Stop the detector, let a frame already being scored finish, then reset.
     * Scoring takes well under a frame; the bound only guards a stalled
     * detect task from wedging the CLI. */
    gTriggerEnabled = 0U;
    {
        uint32_t waited = 0U;
        while (gTrigBusy && waited < L3_TRIGGER_CFG_WAIT_MS) {
            Task_sleep(1);
            waited++;
        }
    }
    gTrigCfg = cfg;
    l3_trig_init(&gTrig, &gTrigCfg, gTrigLoopPeriodS);
    l3_clubTrackConfigure();
    gTriggerEnabled = (cfg.trackFrames != 0U) ? 1U : 0U;
    CLI_write("Done\n");
    return 0;
}

/* "triggerLog trace": what the detector was offered. A header, the per-bin
 * maximum of the detection statistic since arming (eight bins per line, each
 * "bin:max@frame"), then one line per traced frame, oldest first: the
 * region's strongest bin whenever it reached the trace bar, with its
 * all-loop energy, strongest-loop power, loop-0 power and the floor. A swing
 * that never becomes a candidate still shows up here, or shows up nowhere,
 * which says whether the club is invisible or merely rejected. */
static void l3_writeTriggerTrace(char *line, uint32_t cap)
{
    l3_trig_trace_t entry;
    uint32_t count;
    uint32_t index;

    (void)l3_trig_format_trace_header(&gTrig, line, cap);
    CLI_write("%s\n", line);
    for (index = 0U; index < gTrig.maxBins; index += 8U) {
        (void)l3_trig_format_maxhold(&gTrig, index, 8U, line, cap);
        CLI_write("%s\n", line);
    }
    count = l3_trig_trace_count(&gTrig);
    for (index = 0U; index < count; index++) {
        if (!l3_trig_trace_get(&gTrig, index, &entry)) {
            break;
        }
        (void)l3_trig_format_trace(&entry, line, cap);
        CLI_write("%s\n", line);
    }
}

/* CLI "triggerLog [trace|track|shot|result|perf|clear]". Bare: the detector's state and counters,
 * its configuration, then one line per logged frame, oldest first. Only
 * frames with a candidate or an active track are logged; gap= counts the
 * quiet frames before each. "trace" prints the raw-input trace instead (see
 * above); "clear" empties the trace and its maxima without touching the log
 * or the arm. Records keep accruing while this prints, so a frame logged
 * mid-print can show twice or not at all. */
static int32_t l3_cli_triggerLog(int32_t argc, char *argv[])
{
    /* Static, not on the CLI task's small stack. */
    static char line[160];
    l3_trig_record_t record;
    uint32_t count;
    uint32_t index;

    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        l3_trig_trace_clear(&gTrig);
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "trace") == 0) {
        l3_writeTriggerTrace(line, sizeof(line));
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "cal") == 0) {
        /* The calibration in force: offsets, attitude, range bias, then
         * every element's gain and phase (physical order). */
        l3_ensureRadarCal();
        (void)l3_cal_format(&gRadarCal, line, sizeof(line));
        CLI_write("%s\n", line);
        for (index = 0U; index < gRadarCal.virtualElements && index < L3_CAL_MAX_VIRTUAL; index++) {
            (void)l3_cal_format_element(&gRadarCal, index, line, sizeof(line));
            CLI_write("%s\n", line);
        }
        CLI_write("Done\n");
        return 0;
    }
#ifdef L3_RING_IQ8
    if (argc == 2 && strcmp(argv[1], "frames") == 0) {
        /* What each stored slot of the compact ring holds: pre slots oldest
         * first, then the post slots. */
        uint32_t actualPre = (gPreFramesCaptured < gCapturePlan.preFrames)
                                 ? gPreFramesCaptured : gCapturePlan.preFrames;
        uint32_t oldestPre = (gPreFramesCaptured >= gCapturePlan.preFrames)
                                 ? (gPreFramesCaptured % gCapturePlan.preFrames) : 0U;

        if (!l3_captureCompactsIq16()) {
            CLI_write("frames: not a compact format\n");
            CLI_write("Done\n");
            return 0;
        }
        for (index = 0U; index < actualPre; index++) {
            uint32_t slot = (oldestPre + index) % gCapturePlan.preFrames;

            (void)l3_frame_desc_format(&gFrameDesc[slot], line, sizeof(line));
            CLI_write("slot %u %s\n", (unsigned)slot, line);
        }
        for (index = 0U; index < gPostFramesCaptured && index < gCapturePlan.postFrames; index++) {
            uint32_t slot = gCapturePlan.preFrames + index;

            (void)l3_frame_desc_format(&gFrameDesc[slot], line, sizeof(line));
            CLI_write("slot %u %s\n", (unsigned)slot, line);
        }
        CLI_write("Done\n");
        return 0;
    }
#endif
    if (argc == 2 && strcmp(argv[1], "perf") == 0) {
        /* Per-stage cost in microseconds, then the adaptive window state. */
        if (gProfileReady) {
            (void)l3_profile_format_summary(&gProfile, line, sizeof(line));
            CLI_write("%s\n", line);
            for (index = 0U; index < L3_PROF_STAGE_COUNT; index++) {
                (void)l3_profile_format(&gProfile, index, line, sizeof(line));
                CLI_write("%s\n", line);
            }
        } else {
            CLI_write("perf frames=0 (no frame scored yet)\n");
        }
        (void)l3_adaptive_format(&gAdaptiveCfg, &gAdaptiveWindows, line, sizeof(line));
        CLI_write("%s applied=%u\n", line, (unsigned)gAdaptiveApplied);
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "result") == 0) {
        /* The shot result: verdict and quality, every metric with its
         * confidence and flags, then the packet as hex. Readable before
         * RESULT too: it is then the last shot's, or all invalid. */
        (void)l3_result_format(&gShotResult, line, sizeof(line));
        CLI_write("%s ready=%u\n", line, (unsigned)gShotResultReady);
        for (index = 0U; index < L3_RESULT_METRICS; index++) {
            (void)l3_result_format_metric(&gShotResult, index, line, sizeof(line));
            CLI_write("%s\n", line);
        }
        {
            /* 200 hex characters is longer than a CLI line; two halves. */
            static char hex[L3_RESULT_PACKET_BYTES * 2U + 1U];

            (void)l3_result_format_hex(&gShotResult, hex, sizeof(hex));
            memcpy(line, hex, L3_RESULT_PACKET_BYTES);
            line[L3_RESULT_PACKET_BYTES] = '\0';
            CLI_write("packet %s\n", line);
            CLI_write("packet+ %s\n", &hex[L3_RESULT_PACKET_BYTES]);
        }
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "shot") == 0) {
        /* The shot machine, the ball tracker, the launch and its points. */
        l3_track_point_t point;

        (void)l3_shot_format(&gShot, line, sizeof(line));
        CLI_write("%s\n", line);
        (void)l3_ball_track_format_status(&gBallTrack, line, sizeof(line));
        CLI_write("%s post=%u\n", line, (unsigned)gPostFramesScored);
        (void)l3_launch_format(&gLaunch, line, sizeof(line));
        CLI_write("%s\n", line);
        for (index = 0U; l3_track_point(&gBallTrack.core, index, &point); index++) {
            (void)l3_track_format_point(&point, (uint32_t)gBallTrack.originBin, line,
                                        sizeof(line));
            CLI_write("%s\n", line);
        }
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "track") == 0) {
        /* The club track: status with the fitted speed, then every held
         * point oldest first, distances in bins short of the destination. */
        l3_track_point_t point;
        (void)l3_track_format_status(&gClubTrack, gClubTrackDest, line, sizeof(line));
        CLI_write("%s\n", line);
        (void)l3_track_format_delivery(&gDelivery, line, sizeof(line));
        CLI_write("%s\n", line);
        (void)l3_angle_format(&gLastAngle, line, sizeof(line));
        CLI_write(" %s estimates=%u\n", line, (unsigned)gAngleEstimates);
        (void)l3_impact_format(&gImpact, line, sizeof(line));
        CLI_write("%s armed=%u source=%u\n", line, (unsigned)gImpactArmed,
                  (unsigned)gTrigFireSource);
        for (index = 0U; l3_track_point(&gClubTrack, index, &point); index++) {
            (void)l3_track_format_point(&point, gClubTrackDest, line, sizeof(line));
            CLI_write("%s\n", line);
        }
        CLI_write("Done\n");
        return 0;
    }
    if (argc != 1) {
        CLI_write("Error: triggerLog [trace|track|shot|result|perf|frames|cal|clear]\n");
        return -1;
    }
    (void)l3_trig_format_summary(&gTrig, line, sizeof(line));
    CLI_write("%s\n", line);
    (void)l3_trig_format_config(&gTrig, line, sizeof(line));
    CLI_write("%s\n", line);
    count = l3_trig_log_count(&gTrig);
    for (index = 0U; index < count; index++) {
        if (!l3_trig_log_get(&gTrig, index, &record)) {
            break;
        }
        (void)l3_trig_format_record(&record, line, sizeof(line));
        CLI_write("%s\n", line);
    }
    CLI_write("Done\n");
    return 0;
}

/* CLI "debugCfg <0|1>": stream one trig line per detection frame. */
static int32_t l3_cli_debugCfg(int32_t argc, char *argv[])
{
    unsigned long enabled;
    char *end;

    if (argc != 2) {
        CLI_write("Error: debugCfg <0|1>\n");
        return -1;
    }
    enabled = strtoul(argv[1], &end, 10);
    if (*end != '\0' || enabled > 1UL) {
        CLI_write("Error: debugCfg <0|1>\n");
        return -1;
    }
    gTriggerDebug = (uint8_t)enabled;
    if (!gTriggerDebug) {
        gTriggerDebugPhase = 0xFFU;
    } else {
        /* Static, not on the CLI task's small stack. */
        static char line[L3_NOTICE_LINE_BYTES];
        if (l3_formatTriggerDebug(gTriggerPhase, line, sizeof(line))) {
            CLI_write("%s", line);
        }
    }
    CLI_write("Done\n");
    return 0;
}

/* "ball scan <firstBin> <count>": freeze at the next frame boundary, report
 * the static power of <count> GLOBAL bins from <firstBin>, averaged over
 * every pre-trigger frame in the ring, then rearm. Values are the mean
 * |I + jQ|^2 per complex sample (vertical TX pair, all RX, all loops), so
 * scans of different depths compare directly; the host averages repeated
 * scans for a longer baseline. Bins outside a frame's window read 0. */
static int32_t l3_ballScan(uint32_t first, uint32_t count)
{
    static float power[L3_RING_MAX_BINS];
    l3_sparse_window_t window;
    uint32_t bin;
    uint32_t frame;
    uint32_t preFrames = 0U;

    if (l3_sparseFreeze() != 0) {
        return -1;
    }
    l3_sparseWindow(&window);
    for (bin = 0U; bin < count; bin++) {
        power[bin] = 0.0F;
    }
    /* Pre-trigger slots only: post slots may hold another window. */
    for (frame = 0U; frame < window.nFrames; frame++) {
        uint32_t windowStart = window.starts[frame];
        if (window.slots[frame] >= gCapturePlan.preFrames) {
            continue;
        }
        preFrames++;
        for (bin = 0U; bin < count; bin++) {
            uint32_t global = first + bin;
            if (global >= windowStart && global - windowStart < window.counts[frame]) {
                l3_detect_frame_t ringFrame = l3_ringFrameOf(window.slots[frame]);

                power[bin] += l3_verticalStaticPower(&ringFrame, global - windowStart, 1U);
            }
        }
    }
    CLI_write("teescan frames=%u loops=%u first=%u count=%u start=%u\n",
              (unsigned)preFrames, (unsigned)gCapturePlan.loops,
              (unsigned)first, (unsigned)count,
              (unsigned)(window.nFrames ? window.starts[0] : 0U));
    for (bin = 0U; bin < count; bin++) {
        float mean = (preFrames > 0U) ? (power[bin] / (float)preFrames) : 0.0F;
        if (mean > 4.0e9F) {
            mean = 4.0e9F;
        }
        CLI_write("bin=%u power=%u\n", (unsigned)(first + bin), (unsigned)mean);
    }
    return l3_sparseRearm();
}

/* CLI "ball [status] | ball scan <firstBin> <count> | ball cfg <enable>
 * <follow> [minRatio stableUpdates buildUpdates]": the ball-placement
 * detector. "cfg" (re)starts it: enable 0 turns it off; follow 1 makes the
 * self-trigger track the club toward the locked ball's bin instead of the
 * configured tee, falling back to the tee (counted in stats) while no ball
 * is locked. "status" (or no argument) prints the state line that stats
 * also carries, then a balldbg line with centroid, width, persistence and
 * the reasons acquisition did not lock. */
static int32_t l3_cli_ball(int32_t argc, char *argv[])
{
    static char line[192];

    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        (void)l3_ball_format_status(&gBall, line, sizeof(line));
        CLI_write("%s\n", line);
        (void)l3_ball_format_debug(&gBall, line, sizeof(line));
        CLI_write("%s\n", line);
        (void)l3_angle_format(&gBallAngle, line, sizeof(line));
        CLI_write("ball%s valid=%u\n", line + 5, (unsigned)gBallAngleValid); /* "ballangle ..." */
        CLI_write("Done\n");
        return 0;
    }
    if (argc == 4 && strcmp(argv[1], "scan") == 0) {
        unsigned long first;
        unsigned long count;
        char *end;

        first = strtoul(argv[2], &end, 10);
        if (*end != '\0' || first >= 2U * L3_RING_MAX_BINS) {
            CLI_write("Error: ball scan first bin\n");
            return -1;
        }
        count = strtoul(argv[3], &end, 10);
        if (*end != '\0' || count == 0UL || count > L3_RING_MAX_BINS) {
            CLI_write("Error: ball scan count\n");
            return -1;
        }
        return l3_ballScan((uint32_t)first, (uint32_t)count);
    }
    if (argc >= 4 && argc <= 7 && strcmp(argv[1], "cfg") == 0) {
        l3_ball_cfg_t cfg;
        unsigned long value;
        char *end;
        uint32_t waited = 0U;

        l3_ball_cfg_defaults(&cfg);
        value = strtoul(argv[2], &end, 10);
        if (*end != '\0' || value > 1UL) {
            CLI_write("Error: ball cfg enable\n");
            return -1;
        }
        cfg.enabled = (uint8_t)value;
        value = strtoul(argv[3], &end, 10);
        if (*end != '\0' || value > 1UL) {
            CLI_write("Error: ball cfg follow\n");
            return -1;
        }
        cfg.follow = (uint8_t)value;
        if (argc > 4) {
            cfg.minRatio = strtof(argv[4], &end);
            if (*end != '\0') {
                CLI_write("Error: ball cfg min ratio\n");
                return -1;
            }
        }
        if (argc > 5) {
            value = strtoul(argv[5], &end, 10);
            if (*end != '\0') {
                CLI_write("Error: ball cfg stable updates\n");
                return -1;
            }
            cfg.stableUpdates = (uint32_t)value;
        }
        if (argc > 6) {
            value = strtoul(argv[6], &end, 10);
            if (*end != '\0') {
                CLI_write("Error: ball cfg build updates\n");
                return -1;
            }
            cfg.buildUpdates = (uint32_t)value;
        }
        if (cfg.enabled && l3_ball_cfg_check(&cfg) != 0) {
            CLI_write("Error: ball cfg (minRatio > 0, counts > 0)\n");
            return -1;
        }
        if (cfg.follow && !cfg.enabled) {
            CLI_write("Error: ball cfg follow needs enable\n");
            return -1;
        }
        /* Let a frame being scored finish, as triggerCfg does. */
        while (gBallBusy && waited < L3_TRIGGER_CFG_WAIT_MS) {
            Task_sleep(1);
            waited++;
        }
        gBallCfg = cfg;
        l3_ball_init(&gBall, &gBallCfg);
        gTrigDestBall = 0U;
        gTrigFallbackFrames = 0U;
        CLI_write("Done\n");
        return 0;
    }
    CLI_write("Error: ball [status] | ball scan <firstBin> <count> | "
              "ball cfg <enable> <follow> [minRatio stable build]\n");
    return -1;
}

/* CLI "stats": report capture counters (diagnostic). */
static int32_t l3_cli_stats(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
#ifdef HWA_CHAINED_SNAPSHOT_RING
#ifdef L3_RING_IQ8
    CLI_write("frames=%u wraps=%u active=%d calib=0x%x rf_faults=%u "
              "hwa_frames=%u hwa_out=%u hwa_rearms=%u hwa_rearm_err=%u "
              "hwa_missed=%u freeze_req=%u freeze_done=%u freeze_to=%u "
              "format=%s plan=%upre/%upost loops=%u used=%u/%u\n",
              (unsigned)gNumFrame, (unsigned)gNumWrap, (int)gCaptureActive,
              (unsigned)gCalibStatus, (unsigned)gRfFaults,
              (unsigned)gHwaFrameDone, (unsigned)gHwaOutputDone,
              (unsigned)gHwaRearms, (unsigned)gHwaRearmErrors,
              (unsigned)gHwaMissedFrameStarts,
              (unsigned)gHwaFreezeRequests, (unsigned)gHwaFreezeCompletions,
              (unsigned)gHwaFreezeTimeouts,
              l3_captureFormatName(),
              (unsigned)gCapturePlan.preFrames,
              (unsigned)gCapturePlan.postFrames,
              (unsigned)gCapturePlan.loops,
              (unsigned)gCapturePlan.usedBytes,
              (unsigned)l3_captureCapacityBytes());
    CLI_write("iq8_packed=%u iq8_overrun=%u iq8_clipped=%u pending=%u pre_seen=%u "
              "post_kept=%u post_seen=%u stride=%u"
#ifdef L3_IQ8_EDMA_PACK
              " iq8_edma_done=%u iq8_edma_err=%u iq8_edma_wait=%u "
              "iq8_busy=%u/%u iq8_scale=%u"
#endif
              "\n",
              (unsigned)gIq8PackFrames,
              (unsigned)gIq8PackOverruns,
              (unsigned)gIq8ClippedComponents,
              (unsigned)gIq8Pending,
              (unsigned)gPreFramesCaptured,
              (unsigned)gPostFramesCaptured,
              (unsigned)gPostFramesObserved,
              (unsigned)gCapturePlan.postStride
#ifdef L3_IQ8_EDMA_PACK
              , (unsigned)gIq8EdmaDone,
              (unsigned)gIq8EdmaErrors,
              (unsigned)gIq8EdmaWaits,
              (unsigned)gIq8EdmaBusy[0],
              (unsigned)gIq8EdmaBusy[1],
              (unsigned)gIq8FixedScale
#endif
              );
#else
    CLI_write("frames=%u wraps=%u active=%d calib=0x%x rf_faults=%u "
              "hwa_frames=%u hwa_out=%u hwa_rearms=%u hwa_rearm_err=%u "
              "hwa_missed=%u hwa_wait=0x%x freeze_req=%u freeze_done=%u freeze_to=%u "
              "freeze_restart=%u plan=%upre/%upost bins=%u/%u loops=%u "
              "used=%u pre_seen=%u post_kept=%u post_seen=%u stride=%u\n",
              (unsigned)gNumFrame, (unsigned)gNumWrap, (int)gCaptureActive,
              (unsigned)gCalibStatus, (unsigned)gRfFaults,
              (unsigned)gHwaFrameDone, (unsigned)gHwaOutputDone,
              (unsigned)gHwaRearms, (unsigned)gHwaRearmErrors,
              (unsigned)gHwaMissedFrameStarts,
              (unsigned)((gHwaDoneSeen ? 1U : 0U) |
                         (gHwaOutputSeen ? 2U : 0U)),
              (unsigned)gHwaFreezeRequests, (unsigned)gHwaFreezeCompletions,
              (unsigned)gHwaFreezeTimeouts, (unsigned)gHwaFreezeRestarts,
              (unsigned)gCapturePlan.preFrames,
              (unsigned)gCapturePlan.postFrames,
              (unsigned)gCapturePlan.preBins,
              (unsigned)gCapturePlan.postBins,
              (unsigned)gCapturePlan.loops,
              (unsigned)gCapturePlan.usedBytes,
              (unsigned)gPreFramesCaptured,
              (unsigned)gPostFramesCaptured,
              (unsigned)gPostFramesObserved,
              (unsigned)gCapturePlan.postStride);
#endif
#else
    CLI_write("frames=%u wraps=%u active=%d calib=0x%x rf_faults=%u\n",
              (unsigned)gNumFrame, (unsigned)gNumWrap, (int)gCaptureActive,
              (unsigned)gCalibStatus, (unsigned)gRfFaults);
#endif
    {
        static char ballLine[192];
        (void)l3_ball_format_status(&gBall, ballLine, sizeof(ballLine));
        CLI_write("%s\n", ballLine);
    }
    CLI_write("trig dest=%u source=%s fallback=%u\n",
              (unsigned)(gTrigDestBall ? gBall.ballBin : gTrigCfg.teeBin),
              gTrigDestBall ? "ball" : "tee", (unsigned)gTrigFallbackFrames);
    CLI_write("trig phase=%s tee=%u latched=%u enabled=%u\n",
              l3_triggerPhaseName(gTriggerPhase),
              (unsigned)gTriggerTeePower,
              (unsigned)gSelfTriggerLatched,
              (unsigned)gTriggerEnabled);
#ifdef HWA_CHAINED_SNAPSHOT_RING
    CLI_write("detect dropped=%u stale=%u notice_dropped=%u\n",
              (unsigned)gDetectQueue.dropped,
              (unsigned)gDetectStale,
              (unsigned)gNoticeDropped);
#ifdef L3_RING_IQ8
    if (l3_captureCompactsIq16()) {
        static char retainLine[96];

        (void)l3_retain_format(&gLastRetain, retainLine, sizeof(retainLine));
        CLI_write("compact frames=%u errors=%u max_us=%u scratch_stale=%u retain=%u/%u/%u %s\n",
                  (unsigned)gCompactFrames, (unsigned)gCompactErrors,
                  (unsigned)gCompactMaxUs, (unsigned)gDetectScratchStale,
                  (unsigned)gCapturePlan.retainPreBins,
                  (unsigned)gCapturePlan.retainImpactBins,
                  (unsigned)gCapturePlan.retainPostBins, retainLine);
    }
#endif
    CLI_write("rearm_last_us=%u rearm_max_us=%u rearm_timed=%u\n",
              (unsigned)gHwaRearmLastUs,
              (unsigned)gHwaRearmMaxUs,
              (unsigned)gHwaRearmTimed);
#endif
    return 0;
}

#ifdef ENABLE_HWA_SMOKE
/* CLI "hwastats": smoke-test visibility for the HWA driver/link path. */
static int32_t l3_cli_hwaStats(int32_t argc, char *argv[])
{
    (void)argc; (void)argv;
    CLI_write("hwa_opened=%u hwa_handle=0x%x hwa_open_err=%d "
              "test_err=%d peak_bin=%u peak_power=%u runs=%u "
              "real_err=%d real_peak_bin=%u real_peak_power=%u real_runs=%u\n",
              (unsigned)gHwaOpened, (unsigned)gHwaHandle, (int)gHwaOpenErr,
              (int)gHwaTestErr, (unsigned)gHwaTestPeakBin,
              (unsigned)gHwaTestPeakPower, (unsigned)gHwaTestRuns,
              (int)gHwaRealErr, (unsigned)gHwaRealPeakBin,
              (unsigned)gHwaRealPeakPower, (unsigned)gHwaRealRuns);
    return 0;
}

/* CLI "hwatest": software-triggered 128-point FFT on a synthetic tone.
 * This proves HWA param/common config and execution before live chirp plumbing. */
static int32_t l3_cli_hwaTest(int32_t argc, char *argv[])
{
    int16_t *src = (int16_t *)HWA_TEST_MEM0;
    uint32_t rx;
    int32_t errCode;

    (void)argc; (void)argv;
    gHwaTestRuns++;
    gHwaTestErr = 0;
    gHwaTestPeakBin = 0;
    gHwaTestPeakPower = 0;

    if (!gHwaOpened || gHwaHandle == NULL) {
        gHwaTestErr = -100;
        CLI_write("hwatest failed: HWA not open\n");
        return -1;
    }
    if (gCaptureActive) {
        gHwaTestErr = -101;
        CLI_write("hwatest failed: stop sensor first\n");
        return -1;
    }

    /* Build a sparse impulse without pulling in math libraries. This does not
     * validate frequency-bin placement yet; it proves the HWA FFT param set can
     * execute and produce non-zero output without touching live chirp timing. */
    memset((void *)src, 0, HWA_MEM_STRIDE);
    for (rx = 0; rx < HWA_FFT_RX; rx++) {
        uint32_t word = ((HWA_FFT_TONE_BIN * HWA_FFT_RX) + rx) * 2U;
        src[word + 0U] = 0;       /* imag */
        src[word + 1U] = 4096;    /* real */
    }

    errCode = l3_hwaRunFft(&gHwaTestPeakBin, &gHwaTestPeakPower);
    if (errCode != 0) {
        gHwaTestErr = errCode;
        CLI_write("hwatest failed: err=%d\n", errCode);
        return -1;
    }
    CLI_write("hwatest ok: peak_bin=%u peak_power=%u impulse_at_sample=%u runs=%u\n",
              (unsigned)gHwaTestPeakBin, (unsigned)gHwaTestPeakPower,
              (unsigned)HWA_FFT_TONE_BIN, (unsigned)gHwaTestRuns);
    return 0;
}

/* CLI "hwareal": copy the current ADCBUF chirp into HWA memory and FFT it.
 * This is not the final rolling path yet; it validates real chirp layout,
 * ADCBUF visibility, and HWA FFT together before event-driven snapshotting. */
static int32_t l3_cli_hwaReal(int32_t argc, char *argv[])
{
    volatile int16_t *adc = (volatile int16_t *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
    int16_t *src = (int16_t *)HWA_TEST_MEM0;
    uint32_t sample, rx;
    int32_t errCode;

    (void)argc; (void)argv;
    gHwaRealRuns++;
    gHwaRealErr = 0;
    gHwaRealPeakBin = 0;
    gHwaRealPeakPower = 0;

    if (!gHwaOpened || gHwaHandle == NULL) {
        gHwaRealErr = -100;
        CLI_write("hwareal failed: HWA not open\n");
        return -1;
    }

    memset((void *)src, 0, HWA_MEM_STRIDE);
    for (sample = 0U; sample < HWA_FFT_SAMPLES; sample++) {
        for (rx = 0U; rx < HWA_FFT_RX; rx++) {
            uint32_t adcWord = ((rx * N_SAMPLES) + sample) * 2U;
            uint32_t hwaWord = ((sample * HWA_FFT_RX) + rx) * 2U;
            src[hwaWord + 0U] = adc[adcWord + 0U];
            src[hwaWord + 1U] = adc[adcWord + 1U];
        }
    }

    errCode = l3_hwaRunFft(&gHwaRealPeakBin, &gHwaRealPeakPower);
    if (errCode != 0) {
        gHwaRealErr = errCode;
        CLI_write("hwareal failed: err=%d\n", errCode);
        return -1;
    }
    CLI_write("hwareal ok: peak_bin=%u peak_power=%u active=%u frames=%u runs=%u\n",
              (unsigned)gHwaRealPeakBin, (unsigned)gHwaRealPeakPower,
              (unsigned)gCaptureActive, (unsigned)gNumFrame,
              (unsigned)gHwaRealRuns);
    return 0;
}
#endif

/* Configure the ADCBUF for our chirp format (complex, non-interleaved, 4 RX). */
static int32_t l3_configAdcBuf(void)
{
    ADCBuf_dataFormat dataFormat;
    ADCBuf_RxChanConf rxChanCfg;
    uint32_t          rxChanMask = 0xFU;
    uint32_t          chirpThreshold = 1U;
    uint8_t           ch;

    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE,
                       (void *)&rxChanMask) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.adcOutFormat      = 0;   /* complex */
    dataFormat.sampleInterleave  = 1;
    dataFormat.channelInterleave = 1;   /* non-interleaved: RX in contiguous blocks */
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT,
                       (void *)&dataFormat) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    memset((void *)&rxChanCfg, 0, sizeof(rxChanCfg));
    for (ch = 0; ch < N_RX; ch++) {
        rxChanCfg.channel = ch;
        if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE,
                           (void *)&rxChanCfg) != ADCBuf_STATUS_SUCCESS) {
            return -1;
        }
        rxChanCfg.offset += N_SAMPLES * 2 * (uint32_t)sizeof(int16_t);
    }
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                       (void *)&chirpThreshold) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    if (ADCBuf_control(gAdcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                       (void *)&chirpThreshold) != ADCBuf_STATUS_SUCCESS) {
        return -1;
    }
    return 0;
}

/* (Re)configure + enable the hardware-triggered capture EDMA. Exact clone of
 * the shipping demo's rangeproc dataIn (mmw_res.h + rangeProcHWA_ConfigEDMA_
 * DataIn, non-interleaved): DFE_CHIRP_AVAIL event, queue 0, SYNC_AB with
 * aCount=one RX chan (512 B), bCount=N_RX -- one event moves one whole chirp.
 * Source stays at the ADCBUF base every event (srcCIdx=0, hardware presents the
 * completed chirp there); dest advances one chirp per event through the ring
 * (their dstCIdx toggled between 2 HWA banks; ours walks RING_CHIRPS slots).
 * After RING_CHIRPS events the self-linked shadow reloads (dest back to ring
 * base) -> continuous rolling capture. */
static int32_t l3_armCapture(void)
{
#ifdef HWA_CHAINED_SNAPSHOT_RING
    return l3_armHwaChain();
#else
    EDMA_channelConfig_t   ch;
    EDMA_paramSetConfig_t *ps;
    EDMA_paramConfig_t     linkCfg;
    uint32_t               srcAddr, dstAddr;

    srcAddr = SOC_translateAddress(
        SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS +
            (SAVE_OFFSET_SAMPLES * 2U * (uint32_t)sizeof(int16_t)),
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    dstAddr = SOC_translateAddress((uint32_t)&g_ring[0][0],
                                   SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    (void)EDMA_disableChannel(gEdmaHandle, L3_EDMA_CHANNEL, EDMA3_CHANNEL_TYPE_DMA);

    memset((void *)&ch, 0, sizeof(ch));
    ch.channelId    = L3_EDMA_CHANNEL;
    ch.channelType  = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    ch.paramId      = L3_EDMA_CHANNEL;
    ch.eventQueueId = 0U;                 /* demo EDMAIN_EVENT_QUE = 0 */
    ch.transferCompletionCallbackFxn    = l3_edmaCB;
    ch.transferCompletionCallbackFxnArg = (uintptr_t)0U;

    ps = &ch.paramSetConfig;
    ps->sourceAddress      = srcAddr;
    ps->destinationAddress = dstAddr;
    ps->aCount             = (uint16_t)(SAVE_SAMPLES * 2U * sizeof(int16_t));
    ps->bCount             = (uint16_t)N_RX;          /* 4 arrays = one chirp per event */
    ps->cCount             = (uint16_t)RING_CHIRPS;   /* events before wrap */
    ps->bCountReload       = (uint16_t)N_RX;
    ps->sourceBindex       = (int16_t)(N_SAMPLES * 2U * sizeof(int16_t)); /* step RX chans */
    ps->destinationBindex  = (int16_t)(SAVE_SAMPLES * 2U * sizeof(int16_t));
    ps->sourceCindex       = 0;                       /* re-read ADCBUF base every event */
    ps->destinationCindex  = (int16_t)SAVED_CHIRP_BYTES; /* advance dest one chirp per event */
    ps->transferCompletionCode = (uint8_t)L3_EDMA_CHANNEL;
    ps->linkAddress        = EDMA_NULL_LINK_ADDRESS;
    ps->transferType       = (uint8_t)EDMA3_SYNC_AB;
    ps->sourceAddressingMode      = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    ps->destinationAddressingMode = (uint8_t)EDMA3_ADDRESSING_MODE_LINEAR;
    ps->fifoWidth          = (uint8_t)EDMA3_FIFO_WIDTH_8BIT;
    ps->isStaticSet        = false;
    ps->isEarlyCompletion  = false;
    ps->isFinalTransferInterruptEnabled        = true;    /* per wrap */
    ps->isIntermediateTransferInterruptEnabled = false;
    ps->isFinalChainingEnabled        = false;
    ps->isIntermediateChainingEnabled = false;

    /* isEventTriggered = true: the DFE chirp event drives the channel. */
    if (EDMA_configChannel(gEdmaHandle, &ch, true) != EDMA_NO_ERROR) {
        return -1;
    }
    memcpy((void *)&linkCfg.paramSetConfig, (void *)ps, sizeof(EDMA_paramSetConfig_t));
    linkCfg.transferCompletionCallbackFxn    = l3_edmaCB;
    linkCfg.transferCompletionCallbackFxnArg = (uintptr_t)0U;
    if (EDMA_configParamSet(gEdmaHandle, L3_EDMA_LINK_CHANNEL, &linkCfg) != EDMA_NO_ERROR) {
        return -1;
    }
    if (EDMA_linkParamSets(gEdmaHandle, L3_EDMA_CHANNEL, L3_EDMA_LINK_CHANNEL) != EDMA_NO_ERROR) {
        return -1;
    }
    if (EDMA_linkParamSets(gEdmaHandle, L3_EDMA_LINK_CHANNEL, L3_EDMA_LINK_CHANNEL) != EDMA_NO_ERROR) {
        return -1;
    }
    /* Arm the channel to respond to the hardware event. */
    if (EDMA_enableChannel(gEdmaHandle, L3_EDMA_CHANNEL, EDMA3_CHANNEL_TYPE_DMA) != EDMA_NO_ERROR) {
        return -1;
    }
    return 0;
#endif
}

/* mmWave async-event callback: record RF health (calibration status, faults). */
static int32_t l3_mmwaveEvent(uint16_t msgId, uint16_t sbId, uint16_t sbLen,
                              uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);
    (void)sbLen;

    if (msgId == RL_RF_ASYNC_EVENT_MSG) {
        switch (asyncSB) {
            case RL_RF_AE_INITCALIBSTATUS_SB:
                gCalibStatus = ((rlRfInitComplete_t *)payload)->calibStatus & 0x1FFFU;
                break;
            case RL_RF_AE_CPUFAULT_SB:
            case RL_RF_AE_ESMFAULT_SB:
            case RL_RF_AE_ANALOG_FAULT_SB:
                gRfFaults++;
                break;
            default:
                break;
        }
    }
    return 0;
}


/* mmWave control execution context (must outrank the CLI task). */
static void l3_mmwaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;
    (void)arg0; (void)arg1;
    while (1) {
        MMWave_execute(gMMWaveHandle, &errCode);
    }
}

/* CLI "sensorStart": open -> config -> ADCBUF + arm capture EDMA -> start. */
static int32_t l3_cli_sensorStart(int32_t argc, char *argv[])
{
    MMWave_ErrorLevel     errorLevel;
    int16_t               mmwErr, subErr;
    int32_t               errCode;
    (void)argc; (void)argv;

    if (!gSensorOpened) {
        /* Demo sends this before the first MMWave_open (board PA supply cfg);
         * values mirror the mmw demo's gRFLdoBypassCfg for this EVM class. */
        rlRfLdoBypassCfg_t ldoCfg;
        memset((void *)&ldoCfg, 0, sizeof(ldoCfg));
        if (rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, &ldoCfg) != 0) {
            CLI_write("Error: rlRfSetLdoBypassConfig failed\n");
            return -1;
        }
        CLI_getMMWaveExtensionOpenConfig(&gOpenCfg);
        gOpenCfg.freqLimitLow                = 600U;
        gOpenCfg.freqLimitHigh               = 640U;
        gOpenCfg.calibMonTimeUnit            = 1;
        gOpenCfg.useCustomCalibration        = false;
        gOpenCfg.customCalibrationEnableMask = 0x0U;
        gOpenCfg.disableFrameStartAsyncEvent = false;
        gOpenCfg.disableFrameStopAsyncEvent  = false;
        if (MMWave_open(gMMWaveHandle, &gOpenCfg, NULL, &errCode) < 0) {
            MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
            CLI_write("Error: MMWave_open failed [mmwave %d subsys %d]\n", mmwErr, subErr);
            return -1;
        }
        gSensorOpened = 1U;
    }

    CLI_getMMWaveExtensionConfig(&gCtrlCfg);
    /* Geometry guard: the ring/EDMA are compile-time sized, so a cfg with a
     * different loop count or sample count would capture garbage silently.
     * Refuse it loudly instead (three variant builds are in circulation). */
    {
        rlProfileCfg_t profCfg;
        memset((void *)&profCfg, 0, sizeof(profCfg));
        uint16_t chirpCount =
            (uint16_t)(gCtrlCfg.u.frameCfg.frameCfg.chirpEndIdx -
                       gCtrlCfg.u.frameCfg.frameCfg.chirpStartIdx + 1U);
        if ((gCtrlCfg.u.frameCfg.profileHandle[0] == NULL) ||
            (MMWave_getProfileCfg(gCtrlCfg.u.frameCfg.profileHandle[0],
                                  &profCfg, &errCode) < 0) ||
            (profCfg.numAdcSamples != N_SAMPLES) ||
            (chirpCount != N_TX)) {
            CLI_write("Error: cfg geometry mismatch -- this firmware needs "
                      "%d TX x %d samples\n", N_TX, N_SAMPLES);
            return -1;
        }
        /* framePeriodicity LSB = 5 ns -> microseconds. */
        gFramePeriodUs =
            (uint16_t)(gCtrlCfg.u.frameCfg.frameCfg.framePeriodicity / 200U);
        /* One loop is one chirp per TX; idle and ramp are in 10 ns units.
         * The trigger's Doppler readout is aliased at +/- lambda / (4 T). */
        gTrigLoopPeriodS = (float)(profCfg.idleTimeConst + profCfg.rampEndTime) *
                           1.0e-8F * (float)chirpCount;
        if (l3_finalizeCapturePlan(gCtrlCfg.u.frameCfg.frameCfg.numLoops) != 0) {
            return -1;
        }
    }
    if (MMWave_config(gMMWaveHandle, &gCtrlCfg, &errCode) < 0) {
        MMWave_decodeError(errCode, &errorLevel, &mmwErr, &subErr);
        CLI_write("Error: MMWave_config failed [mmwave %d subsys %d]\n", mmwErr, subErr);
        return -1;
    }

    if (l3_configAdcBuf() < 0) {
        CLI_write("Error: ADCBUF config failed\n");
        return -1;
    }
    gNumFrame  = 0U;
    gRingFrame = 0U;
    gNumWrap   = 0U;
    if (gProfileReady) {
        l3_profile_reset(&gProfile);
    }
#ifdef HWA_CHAINED_SNAPSHOT_RING
    gHwaFrameDone      = 0U;
    gHwaOutputDone     = 0U;
    gHwaRearms         = 0U;
    gHwaRearmErrors    = 0U;
    gHwaMissedFrameStarts = 0U;
    gHwaRearmQueuedValid = 0U;
    gHwaRearmLastUs    = 0U;
    gHwaRearmMaxUs     = 0U;
    gHwaRearmTimed     = 0U;
    gHwaArmedForFrame  = 0U;
    gHwaDoneSeen       = 0U;
    gHwaOutputSeen     = 0U;
    gHwaRearmPending   = 0U;
    gHwaRearmBusy      = 0U;
    gHwaFreezeRequested = 0U;
    gHwaShutdownRequested = 0U;
    gHwaFreezeRequestFrame = 0U;
    gHwaFreezeRequests = 0U;
    gHwaFreezeCompletions = 0U;
    gHwaFreezeTimeouts = 0U;
    gHwaFreezeRestarts = 0U;
    gPreFramesCaptured = 0U;
    l3_trigRearm();
    gPostFramesCaptured = 0U;
    gPostFramesObserved = 0U;
    gPostCaptureStarted = 0U;
    gActiveFrameIsPost = 0U;
    gActiveFrameShouldKeep = 1U;
    l3_resetDetectQueue();
    /* A new session starts untriggered: a latch or enable left by a host that
     * died mid-shot must not freeze or self-trigger this one. triggerCfg
     * re-enables it. */
    gSelfTriggerLatched = 0U;
    gTriggerEnabled = 0U;
    gTriggerPhase = 0U;
    gTriggerTeePower = 0U;
    gTriggerDebugPhase = 0xFFU;
#ifdef L3_RING_IQ8
    gIq8Pending = 0U;
    gIq8PendingDetect = 0U;
    gIq8PendingEpoch = 0U;
    gIq8PendingSlot = 0U;
    gIq8PendingScratch = 0U;
    gIq8ActiveScratch = 0U;
    gIq8PackFrames = 0U;
    gIq8PackOverruns = 0U;
    gIq8ClippedComponents = 0U;
    gScratchFrame[0] = 0U;
    gScratchFrame[1] = 0U;
    gScratchBusy[0] = 0U;
    gScratchBusy[1] = 0U;
    gDetectScratchStale = 0U;
    gCompactFrames = 0U;
    gCompactErrors = 0U;
    gCompactMaxUs = 0U;
    memset(gFrameScratch, L3_SCRATCH_NONE, sizeof(gFrameScratch));
    memset(&gLastRetain, 0, sizeof(gLastRetain));
    memset(gFrameDesc, 0, sizeof(gFrameDesc));
#ifdef L3_IQ8_EDMA_PACK
    gIq8PackDetectArm[0] = 0U;
    gIq8PackDetectArm[1] = 0U;
    gIq8EdmaBusy[0] = 0U;
    gIq8EdmaBusy[1] = 0U;
    gIq8EdmaDone = 0U;
    gIq8EdmaErrors = 0U;
    gIq8EdmaWaits = 0U;
    while (Semaphore_pend(gIq8EdmaDoneSemaphore, BIOS_NO_WAIT)) {
        /* Discard completion signals from the previous capture. */
    }
#endif
    memset((void *)gFrameIq8Scale, 0, sizeof(gFrameIq8Scale));
#endif
#endif
    if (l3_armCapture() < 0) {
        CLI_write("Error: capture arm failed\n");
        return -1;
    }
    gCaptureActive = 1U;

    if (l3_startFrontEnd() < 0) {
        gCaptureActive = 0U;
        CLI_write("Error: MMWave_start failed\n");
        return -1;
    }
    return 0;
}

static int32_t l3_cli_sensorStop(int32_t argc, char *argv[])
{
    int32_t status = 0;
    int32_t errCode;
    (void)argc; (void)argv;

    if (gSelfTriggerLatched) {
        /* A self-trigger froze the ring and nobody read it. The freeze
         * already cleared gCaptureActive, but the BSS is still chirping;
         * closing over it wedged the CLI ("did not acknowledge sensorStop").
         * Take the freeze like l3release does, stopping the front end. */
        status = l3_awaitFrozenRing();
    } else if (gCaptureActive) {
        status = l3_stopCaptureForShutdown();
    }
    /* MMWave_config is refused while the BSS still holds the last profile
     * (-3110 subsys 83). Closing here lets the next sensorStart reopen and
     * configure again without a power cycle. */
    if (gSensorOpened) {
        if (MMWave_close(gMMWaveHandle, &errCode) < 0) {
            CLI_write("Error: MMWave_close failed (%d)\n", errCode);
            return -1;
        }
        gSensorOpened = 0U;
    }
    /* Every configuration starts with sensorStop (driver.send_config): a cfg
     * that wants the range window sets it again, and one without it must not
     * inherit the last run's. */
    gRangeWindow = L3_RANGE_WINDOW_NONE;
    return status;
}

/* System init task: UART, mmWave control, EDMA + ADCBUF + frame-start ISR, CLI. */
static void l3_initTask(UArg arg0, UArg arg1)
{
    MMWave_InitCfg        initCfg;
    UART_Params           uartParams;
    Task_Params           taskParams;
    ADCBuf_Params         adcbufParams;
    SOC_SysIntListenerCfg socIntCfg;
    EDMA_instanceInfo_t   edmaInstanceInfo;
#ifdef HWA_CHAINED_SNAPSHOT_RING
    Semaphore_Params      semaphoreParams;
#endif
    static CLI_Cfg        cliCfg;
    int32_t               errCode;

    (void)arg0; (void)arg1;

    /* Starts the R4F PMU cycle counter behind Cycleprofiler_getTimeStamp. */
    Cycleprofiler_init();
    UART_init();
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ,
                            PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    /* v3 UART-SPEED TEST: single-port operation. UARTA routes to the CP2105
     * ENHANCED interface (2 Mbps capable; the Standard iface that carried the
     * dump in v1/v2 is capped at 921600 AND threw sporadic -110 control
     * timeouts on the Pi). 1,041,667 divides EXACTLY on our side
     * (200 MHz/16/12) and to +0.17% in the CP2105 (24 MHz/23). CLI commands
     * and the dump share this port; the host syncs on the "ILD1" magic.
     * BINARY write mode is ESSENTIAL for the dump path (TEXT inserts \r
     * before 0x0A bytes); read mode stays TEXT for CLI line handling. */
    uartParams.writeDataMode  = UART_DATA_BINARY;
    uartParams.baudRate       = 1041667;
    uartParams.isPinMuxDone    = 1;
    /* No echo: the driver echoes from inside its RX interrupt, spinning on
     * TX-free between bytes. A host cell line arrives back to back at wire
     * rate, and the one-byte SCI receiver overruns while the ISR waits to
     * echo. Nothing on the host reads the echo; it syncs on Done/Error and
     * the packet magics. */
    uartParams.readEcho       = UART_ECHO_OFF;
    gCliUart = UART_open(0, &uartParams);

    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gCpuClock;
    /* BINARY write mode is ESSENTIAL: the driver default (UART_DATA_TEXT)
     * inserts \r before every 0x0A byte, corrupting a raw binary stream with
     * one-byte shifts (~1 insertion per 256 bytes). This -- not the EDMA, not
     * the baud -- was the capture-corruption root cause. */
    uartParams.writeDataMode  = UART_DATA_BINARY;
    uartParams.readDataMode   = UART_DATA_BINARY;
    /* 460800: divides cleanly from the 200 MHz UART clock (921600 does not:
     * divisor 13.56 -> 3-4% baud error). Keep the safe rate. */
    uartParams.baudRate       = 460800;
    uartParams.isPinMuxDone    = 1;
    gDataUart = UART_open(1, &uartParams);
    /* v3: dump goes out the CLI port (Enhanced iface) at 1.04 M. UARTB stays
     * open as a debug spare; flip this assignment back for v2 behavior. */
    gDataUart = gCliUart;

    /* EDMA + ADCBUF drivers. */
    EDMA_init(0);
    gEdmaHandle = EDMA_open(0, &errCode, &edmaInstanceInfo);
    if (gEdmaHandle == NULL) {
        return;
    }
    ADCBuf_init();
    ADCBuf_Params_init(&adcbufParams);
    adcbufParams.chirpThresholdPing = 1U;
    adcbufParams.chirpThresholdPong = 1U;
    adcbufParams.continousMode      = 0U;
    adcbufParams.socHandle          = gSocHandle;
    gAdcbufHandle = ADCBuf_open(0, &adcbufParams);
    if (gAdcbufHandle == NULL) {
        return;
    }

#ifdef ENABLE_HWA_SMOKE
    /* Incremental HWA bring-up: prove the driver/lib opens on MSS before we
     * put HWA in the chirp timing path. No HWA params or DMA triggers yet. */
    HWA_init();
    gHwaHandle = HWA_open(0, gSocHandle, &gHwaOpenErr);
    gHwaOpened = (gHwaHandle != NULL) ? 1U : 0U;
#endif

    /* Frame-start liveness ISR (per-chirp capture is now the hardware EDMA). */
    memset((void *)&socIntCfg, 0, sizeof(socIntCfg));
    socIntCfg.systemInterrupt = SOC_XWR68XX_MSS_FRAME_START_INT;
    socIntCfg.listenerFxn     = l3_frameStartISR;
    socIntCfg.arg             = (uintptr_t)0U;
    if (SOC_registerSysIntListener(gSocHandle, &socIntCfg, &errCode) == NULL) {
        return;
    }

    /* mmWave control (FULL, ISOLATION). */
    Mailbox_init(MAILBOX_TYPE_MSS);
    memset((void *)&initCfg, 0, sizeof(MMWave_InitCfg));
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gSocHandle;
    initCfg.eventFxn                = l3_mmwaveEvent;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode           = MMWave_ExecutionMode_ISOLATION;
    gMMWaveHandle = MMWave_init(&initCfg, &errCode);
    if (gMMWaveHandle == NULL) {
        return;
    }
    while (1) {
        int32_t syncStatus = MMWave_sync(gMMWaveHandle, &errCode);
        if (syncStatus < 0) {
            return;
        }
        if (syncStatus == 1) {
            break;
        }
        Task_sleep(1);
    }
    Task_Params_init(&taskParams);
    taskParams.priority  = L3_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3 * 1024;
    Task_create(l3_mmwaveCtrlTask, &taskParams, NULL);

#ifdef HWA_CHAINED_SNAPSHOT_RING
    Semaphore_Params_init(&semaphoreParams);
    semaphoreParams.mode = Semaphore_Mode_BINARY;
    gHwaRearmSemaphore = Semaphore_create(0, &semaphoreParams, NULL);
    if (gHwaRearmSemaphore == NULL) {
        return;
    }
    gHwaFreezeSemaphore = Semaphore_create(0, &semaphoreParams, NULL);
    if (gHwaFreezeSemaphore == NULL) {
        return;
    }
#if defined(L3_RING_IQ8) && \
    defined(L3_IQ8_EDMA_PACK)
    gIq8EdmaDoneSemaphore = Semaphore_create(0, &semaphoreParams, NULL);
    if (gIq8EdmaDoneSemaphore == NULL) {
        return;
    }
#endif
    Task_Params_init(&taskParams);
    taskParams.priority = L3_HWA_REARM_TASK_PRIORITY;
    taskParams.stackSize = 2U * 1024U;
    Task_create(l3_hwaRearmTask, &taskParams, NULL);
    semaphoreParams.mode = Semaphore_Mode_COUNTING;
    gDetectSemaphore = Semaphore_create(0, &semaphoreParams, NULL);
    if (gDetectSemaphore == NULL) {
        return;
    }
    l3detect_init(&gDetectQueue);
    Task_Params_init(&taskParams);
    taskParams.priority = L3_DETECT_TASK_PRIORITY;
    taskParams.stackSize = 3U * 1024U;
    Task_create(l3_detectTask, &taskParams, NULL);
    gNoticeSemaphore = Semaphore_create(0, &semaphoreParams, NULL);
    if (gNoticeSemaphore == NULL) {
        return;
    }
    Task_Params_init(&taskParams);
    taskParams.priority = L3_NOTICE_TASK_PRIORITY;
    taskParams.stackSize = 2U * 1024U;
    Task_create(l3_noticeTask, &taskParams, NULL);
#endif

    /* CLI with the mmWave extension. */
    cliCfg.cliPrompt             = "l3dump:/>";
#ifdef HYBRID_CADENCE_CAPTURE
    cliCfg.cliBanner             = "OpenFlight hybrid-cadence L3 firmware (v6)\n";
#else
    cliCfg.cliBanner             = "OpenFlight configurable L3 snapshot firmware (v5)\n";
#endif
    cliCfg.cliUartHandle         = gCliUart;
    cliCfg.socHandle             = gSocHandle;
    cliCfg.mmWaveHandle          = gMMWaveHandle;
    cliCfg.enableMMWaveExtension = 1U;
    cliCfg.taskPriority          = L3_CLI_TASK_PRIORITY;
    cliCfg.usePolledMode         = true;
    cliCfg.tableEntry[0].cmd           = "sensorStart";
    cliCfg.tableEntry[0].helpString    = "Configure + start capture (no args)";
    cliCfg.tableEntry[0].cmdHandlerFxn = l3_cli_sensorStart;
    cliCfg.tableEntry[1].cmd           = "sensorStop";
    cliCfg.tableEntry[1].helpString    = "Stop the front-end";
    cliCfg.tableEntry[1].cmdHandlerFxn = l3_cli_sensorStop;
    cliCfg.tableEntry[2].cmd           = "l3dump";
    cliCfg.tableEntry[2].helpString    = "Freeze and stream one L3 snapshot dump";
    cliCfg.tableEntry[2].cmdHandlerFxn = l3_cli_dump;
    cliCfg.tableEntry[3].cmd           = "stats";
    cliCfg.tableEntry[3].helpString    = "Report capture counters";
    cliCfg.tableEntry[3].cmdHandlerFxn = l3_cli_stats;
#ifdef ENABLE_HWA_SMOKE
    cliCfg.tableEntry[4].cmd           = "hwastats";
    cliCfg.tableEntry[4].helpString    = "Report HWA smoke-test status";
    cliCfg.tableEntry[4].cmdHandlerFxn = l3_cli_hwaStats;
    cliCfg.tableEntry[5].cmd           = "hwatest";
    cliCfg.tableEntry[5].helpString    = "Run HWA FFT self-test";
    cliCfg.tableEntry[5].cmdHandlerFxn = l3_cli_hwaTest;
    cliCfg.tableEntry[6].cmd           = "hwareal";
    cliCfg.tableEntry[6].helpString    = "Run HWA FFT on current ADCBUF chirp";
    cliCfg.tableEntry[6].cmdHandlerFxn = l3_cli_hwaReal;
#endif
    cliCfg.tableEntry[7].cmd           = "captureCfg";
    cliCfg.tableEntry[7].helpString    =
        "captureCfg preStart preBins postStart postBins lateStart postFrames [postStride]";
    cliCfg.tableEntry[7].cmdHandlerFxn = l3_cli_captureCfg;
    cliCfg.tableEntry[8].cmd           = "phaseCaptureCfg";
    cliCfg.tableEntry[8].helpString    =
        "phaseCaptureCfg preStart preBins preFrames impactStart impactBins "
        "impactFrames postStart postBins lateStart ballFrames ballStride";
    cliCfg.tableEntry[8].cmdHandlerFxn = l3_cli_phaseCaptureCfg;
#ifdef L3_RING_IQ8
    cliCfg.tableEntry[9].cmd           = "captureFormat";
    cliCfg.tableEntry[9].helpString    = "captureFormat iq16|iq8|compact16|adaptive16";
    cliCfg.tableEntry[9].cmdHandlerFxn = l3_cli_captureFormat;
#ifdef L3_IQ8_EDMA_PACK
    cliCfg.tableEntry[10].cmd           = "iq8Scale";
    cliCfg.tableEntry[10].helpString    = "iq8Scale 16|32|64|128|256";
    cliCfg.tableEntry[10].cmdHandlerFxn = l3_cli_iq8Scale;
#endif
#endif
    cliCfg.tableEntry[11].cmd           = "l3sparse";
    cliCfg.tableEntry[11].helpString    = "Freeze, send residual power, then requested cells";
    cliCfg.tableEntry[11].cmdHandlerFxn = l3_cli_sparse;
    cliCfg.tableEntry[12].cmd           = "triggerCfg";
    cliCfg.tableEntry[12].helpString    =
        "triggerCfg <localBin> <snr> <frames> [approach gate minCoh minStep stat minSpeed]";
    cliCfg.tableEntry[12].cmdHandlerFxn = l3_cli_triggerCfg;
    cliCfg.tableEntry[13].cmd           = "l3track";
    cliCfg.tableEntry[13].helpString    = "Freeze, pick ball and club cells on-chip, send them";
    cliCfg.tableEntry[13].cmdHandlerFxn = l3_cli_track;
    cliCfg.tableEntry[14].cmd           = "trackCfg";
    cliCfg.tableEntry[14].helpString    = "trackCfg <loopPeriodS> <rangeResM> <maxRangeM> <clubLoM> <clubHiM> or cal/elem/impact ...";
    cliCfg.tableEntry[14].cmdHandlerFxn = l3_cli_trackCfg;
    cliCfg.tableEntry[15].cmd           = "debugCfg";
    cliCfg.tableEntry[15].helpString    = "debugCfg <0|1> stream trigger decisions";
    cliCfg.tableEntry[15].cmdHandlerFxn = l3_cli_debugCfg;
    cliCfg.tableEntry[16].cmd           = "l3release";
    cliCfg.tableEntry[16].helpString    = "Rearm a self-trigger freeze without streaming";
    cliCfg.tableEntry[16].cmdHandlerFxn = l3_cli_release;
    /* Entries 0..18 plus the mmWave extension's commands must fit the SDK's
     * CLI_MAX_CMD; keep new diagnostics as sub-modes of existing commands. */
    cliCfg.tableEntry[18].cmd           = "ball";
    cliCfg.tableEntry[18].helpString    =
        "ball [status] | ball scan <firstBin> <count> | ball cfg <enable> <follow> [...]";
    cliCfg.tableEntry[18].cmdHandlerFxn = l3_cli_ball;
    cliCfg.tableEntry[17].cmd           = "triggerLog";
    cliCfg.tableEntry[17].helpString    = "triggerLog [trace|track|shot|result|perf|frames|cal|clear]: log, trace, club, shot, result, perf, stored frames, calibration";
    cliCfg.tableEntry[17].cmdHandlerFxn = l3_cli_triggerLog;
    CLI_open(&cliCfg);
}

int32_t main(void)
{
    Task_Params taskParams;
    SOC_Cfg     socCfg;
    int32_t     errCode;

    ESM_init(0U);
    memset((void *)&socCfg, 0, sizeof(SOC_Cfg));
    socCfg.clockCfg = SOC_SysClock_INIT;
    gSocHandle = SOC_init(&socCfg, &errCode);
    if (gSocHandle == NULL) {
        return -1;
    }
    Task_Params_init(&taskParams);
    taskParams.priority  = L3_INIT_TASK_PRIORITY;
    taskParams.stackSize = 8 * 1024;
    Task_create(l3_initTask, &taskParams, NULL);
    BIOS_start();
    return 0;
}
