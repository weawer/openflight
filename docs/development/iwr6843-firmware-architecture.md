# IWR6843 Firmware: Architecture and Components

This document describes the OpenFlight IWR6843 firmware as it stands after the
detect-on-DSP work (September 2026): what runs where, how a frame travels from
the antenna to a shot result, what each module is responsible for, and how
each piece is verified. It is the map; `firmware.md` in this directory is the
build/flash/operate guide and the change history.

- [1. The chip and its cores](#1-the-chip-and-its-cores)
- [2. Images and build](#2-images-and-build)
- [3. Memory map](#3-memory-map)
- [4. Tasks and priorities (MSS)](#4-tasks-and-priorities-mss)
- [5. Acquisition: from chirps to the L3 ring](#5-acquisition-from-chirps-to-the-l3-ring)
- [6. The pre-impact pipeline (every frame)](#6-the-pre-impact-pipeline-every-frame)
- [7. The fire](#7-the-fire)
- [8. The post-impact pipeline](#8-the-post-impact-pipeline)
- [9. The DSS subsystem](#9-the-dss-subsystem)
- [10. The club's angles off the decision path](#10-the-clubs-angles-off-the-decision-path)
- [11. Module map](#11-module-map)
- [12. Host side](#12-host-side)
- [13. CLI reference](#13-cli-reference)
- [14. Verification strategy](#14-verification-strategy)
- [15. Measured numbers and what still needs the board](#15-measured-numbers-and-what-still-needs-the-board)

---

## 1. The chip and its cores

The TI IWR6843 has three processors and two DMA/accelerator blocks:

| Block | What it is | What OpenFlight runs on it |
|---|---|---|
| **MSS** | ARM Cortex-R4F, 200 MHz, no cache | Everything that decides: the capture plan, the CLI, the detect task (trigger, trackers, shot machine), the angle task, the notices to the host |
| **DSS** | TI C674x DSP, 600 MHz, L1P/L1D 16 KB cache each, 256 KB L2 SRAM | The detector's bin scoring (`trackCfg detectCore dss\|verify`), the link diagnostics; the ported post-shot solve stages (`solve/`) are built in but not yet dispatched |
| **BSS** | The radar subsystem (TI firmware) | Chirps, the RF front end; OpenFlight only configures it through mmWave |
| **HWA** | Hardware accelerator | The range FFT of every chirp, triggered by the front end |
| **EDMA** | Two transfer controllers: TPCC0 (instance 0), TPCC1 (instance 1) | Instance 0 (MSS): range bins from the HWA into the L3 ring. Instance 1 (DSS): the gather of a frame's window into DSS L2 |

The MSS owns the system: it initialises clocks (`SOC_SysClock_INIT`), unhalts
the BSS, opens mmWave (FULL, ISOLATION mode, so the BSS is driven from the
MSS alone) and opens the MSS<->DSS mailbox. The DSS is loaded and released
by the boot ROM from the same flash image, initialises only itself
(`SOC_SysClock_BYPASS_INIT`) and serves requests.

## 2. Images and build

One flashable meta-image (`firmware/releases/*.bin`) carries three images,
assembled by the SDK's `generateMetaImage.sh` / `MulticoreImageGen`:

| Image | Built from | Toolchain |
|---|---|---|
| MSS `l3_dump_mss.xer4f` | `firmware/iwr6843/makefile` (`SOURCES`: `l3_dump.c` and the `l3_*.c` modules) | TI ARM CGT 20.2.7, SYS/BIOS 6.73, mmWave SDK 3.06 |
| BSS `xwr6xxx_radarss_rprc.bin` | TI's prebuilt radar firmware | — |
| DSS `l3_dump_dss.xe674` | `firmware/iwr6843/dss/makefile` (`dss_main.c`, the shared `l3_dsp_ipc.c`, `l3_bin_score.c`, `l3_iq16_stats.c`, and `solve/`) | TI C6000 CGT 8.3.3, DSPLIB |

Build in the SDK container:

```bash
make -C firmware docker-build RELEASE_NAME=l3_dump_dsp_link_test.bin
```

`build-native` cleans and rebuilds the MSS objects, links the DSS, and writes
the meta-image and its sha256. The build defines select the production data
path: `HWA_CHAINED_SNAPSHOT_RING`, `L3_RING_IQ8`, `L3_IQ8_EDMA_PACK`,
`N_TX=3`.

**Cache rule for the DSS** (learned on the board): the DSS keeps the
platform's caches (`ti/platform/xwr68xx/c674x_linker.cmd`: L1P and L1D 16 KB,
**L2 all SRAM**). An override that turned 32 KB of L2 into cache killed the
DSS inside its BIOS module startups, before `main`.

## 3. Memory map

### MSS

| Region | Size | Use |
|---|---|---|
| PROG_RAM (TCMA) | 512 KB | Code. **Read-only on a flashed board** (the SDK's prebuilt SOC library maps it RO unless loaded from CCS): nothing is written there at run time |
| DATA_RAM (TCMB) | 192 KB | Globals, `.bss`, BIOS heap, task stacks. ~4.3 KB free: it is the scarce resource |
| L3_RAM `0x51000000` | 768 KB | `.l3ring`, the capture ring (the whole arena) |
| HS_RAM `0x52080000` | 32 KB | Shared with the DSS (which sees it at `0x21080000`); see below |

HS-RAM layout (offsets):

| Offset | What | Owner |
|---|---|---|
| `0x0000`–`0x2AF3` | `.hsramMss`: MSS-only data moved out of DATA_RAM — `gProfile`, `gTiming`, the angle queue and the angle task's stack, the steering-rotor table, CLI line buffers, l3sparse's power rows | MSS (linker-placed) |
| `0x7400` | SCORE result block (`l3_dsp_result_t`, 1,320 B) | DSS writes, MSS reads |
| `0x7E00` | The `trackCfg dsp hw` read-back probe word | MSS |
| `0x7F00` | The DSS boot status (`l3_dsp_status_t`) | DSS writes, MSS reads |

`0x7400`–`0x7FFF` is reserved in the MSS link (`.hsramDss`), so a growing
`.hsramMss` fails the link instead of overwriting the DSS's words.

### DSS

| Region | Use |
|---|---|
| L1P / L1D | 16 KB cache each (platform) |
| L2 UMAP1 `0x007E0000` (128 KB) | `.vecs`, `.text`, **`.dssGather`** (48 KB: a frame's gathered window) |
| L2 UMAP0 `0x00800000` (128 KB) | Data, `.const`, `.stack`, the system heap, the solve scratch |
| L3 `0x20000000` | The MSS's ring, read in place or gathered; the DSS owns no L3 buffer |
| HS-RAM `0x21080000` | The result block and the boot status (above) |

## 4. Tasks and priorities (MSS)

SYS/BIOS, preemptive; higher number runs first.

| Priority | Task | Job |
|---|---|---|
| 6 | `l3_mmwaveCtrlTask` | mmWave control events |
| 5 | `l3_hwaRearmTask` | Re-arms the HWA/EDMA chain between frames; packs IQ8 |
| 4 | `l3_detectTask` | The whole per-frame decision: ball detector, self-trigger, trackers, shot machine |
| 3 | CLI task | Host commands |
| 3 | `l3_noticeTask` | Writes queued notices (`Triggered`, debug lines) whole, so a CLI reply cannot interleave |
| 2 | `l3_angleTask` | The club's queued angles (§10) |
| 2 | `l3_initTask` | Start-up; ends after creating the others |
| 1 | snapshot worker | Live snapshot streaming |

The detect task outranks the CLI on purpose (a polled `stats` reply takes
~8 ms and would otherwise drop frames), so anything it does costs the CLI
its time: this is why the per-frame budget matters (§6, §15).

## 5. Acquisition: from chirps to the L3 ring

1. The BSS fires TDM chirps: chirp *c* is TX *c mod 3*, loop *c div 3*
   (3 TX, 16 loops, 4 RX).
2. The front end triggers the **HWA** range FFT per chirp; HWA completion
   triggers **EDMA instance 0**, which copies the selected range bins into
   the **L3 ring** (the processing window, e.g. 53 bins from global bin 20).
3. `l3_hwaRearmTask` re-arms one frame at a time in the inter-frame gap.
4. The ring holds `preFrames` pre-impact slots (a rolling history) and a
   post-impact tail. A frame's layout in a slot is
   `[loop][tx][rx][bin][Im, Re]`, 2 bytes a component (`captureFormat iq16`)
   or 1 (`iq8`, with a per-frame scale). The compact formats keep a wider
   IQ16 processing window in MSS scratch and retain fewer bins in L3.
5. A completed slot is published to the **detect queue** (`detect_queue.c`)
   with its epoch and an acquisition timestamp; the detect task is woken.
6. On a fire (§7) the ring freezes after the post tail; `l3dump`, `l3sparse`
   or `l3track` read it out; `l3release` re-arms.

## 6. The pre-impact pipeline (every frame)

`l3_detectTask` pops a slot, checks it is still live (not overwritten), and
notes whether it is **behind** (a newer frame already landed). Then:

### 6.1 Ball detector (`l3_considerBall`, `l3_ball.c`)

- Runs every **8th** frame (`L3_BALL_FRAMES_PER_UPDATE`; shed entirely when
  behind). Static (non-MTI) power of every window bin, loops subsampled by 4
  (`l3_channels_static_power`).
- `l3_ball_update`: learns the empty background, then a new compact stable
  riser locks as the ball (`building → waiting → candidate → locked`). Its
  counts and learning rates are per update and scaled to the cadence
  (`l3_ball_cfg_defaults_at`), so lock/release times are the same in
  milliseconds as the old every-second-frame cadence.
- The locked ball's direction (`l3_channels_snapshot_static` →
  `l3_angle_estimate`) only when due (`l3_ball_angle_due`: a new or moved
  lock, then every 20 updates ≈ 0.5 s).
- With `ball cfg ... follow`, the locked bin becomes the trigger's
  destination; else the configured tee bin.

### 6.2 Self-trigger (`l3_considerSelfTrigger`)

1. **Destination and region**: the ball's (or tee's) bin; the trigger's
   watch region (`l3_trig_region`).
2. **Tee band** (`l3_band.c`): the band of bins around the ball that carry an
   MTI ridge for the whole capture; placed on the noisiest idle bins near the
   destination (per-bin noise map with history), frozen while a club track
   is active. Targets inside it are dropped.
3. **Scan plan** (`l3_scan.c`): which bins this frame scores — the club's
   approach short of the band (16) plus the edge, the ball-leave stretch
   beyond it (10), the region clipped short of the band, and on idle frames
   a rotating 2-bin chunk of the band for its noise map: ≤ 29 bins.
4. **Scoring** (`l3_scoreSpans`): each planned bin once (a 64-bit mask),
   routed by `l3_detect_core` to the **MSS** (`l3_channels_residual` →
   `l3_bin_score_iq16`), the **DSS** (SCORE, §9), or **both** (verify). Per
   bin: burst-MTI residual energy over every loop, the strongest loop, loop
   0, and the lag-1 autocorrelation (Doppler), summed over the vertical TX
   pair (TX0, TX2) and all RX, in exact integers.
5. **Stale re-check**: a slot the writer overtook during the read is
   discarded (`stale_read`).
6. **Trigger floor** (`l3_trigger.c`): the running noise floor the
   extraction reads against.
7. **Targets** (`l3_observation.c`, `l3_preImpactClubTargets`): ranked
   targets in the club's span (sub-bin peak, SNR, Doppler, confidence), and
   the leave detector's candidates beyond the band.
8. **Club track** (`l3_club_track.c`): predictive association into a
   32-point trajectory; its fitted range rate resolves the TDM alias.
9. **Band noise map**: idle frames feed each scored span to the map.
10. **Club angle**: the associated target's channel snapshot
    (`l3_channels_snapshot`) is **queued** for the angle task (§10) — no
    estimate on this path, never shed.
11. **Delivery** (`l3_track_delivery`): the club's velocity vector, path and
    attack from its newest 8 points; and the ball's position.
12. **Range impact** (`l3_impact.c`): the club-in line's crossing of the
    ball's range, or the approach ending within 0.40 m of it
    (`cause=crossing|end`), fires.
13. **Ball-leave fallback** (`l3_leave.c`): when the club rules miss, the
    first return beyond the band stepping outward at a ball's speed
    (22–90 m/s), armed only just after the club reached the band, fires.

Behind, the detect task sheds the ball detector and the band map chunk:
what the fire does not need. The club's angle snapshot is kept (§10).

`l3_timing.c` stamps every frame (acquired, dequeued, score start/end,
decided) and keeps throughput (mean service vs the frame interval,
`over_budget`) apart from latency (the ring slot reuse margin,
`margin_negative`); `l3_profile.c` keeps per-stage costs. Both print in
`triggerLog perf` / `timing`.

## 7. The fire

When the range impact or the leave fallback fires, in this order:

1. **Freeze request** (under `Hwi_disable`): the HWA chain stops re-arming
   after the post-impact tail; the self-trigger latches.
2. **`Triggered` notice** queued first: the host's `S!` to the OPS243 waits
   on it.
3. **Drain the angle queue** on the detect task (§10), then recompute the
   delivery from the complete angles.
4. **`l3_shotObserve`** → `l3_shot_update`: the shot machine declares impact,
   freezes the club trajectory, the delivery and the ball's position, and
   arms the ball tracker at the ball's bin with the post-impact floor frozen.
5. A leave fire seeds the ball track with the fallback's two points.

Steps 3–4 run after the freeze request, so the fire's latency is unchanged.

## 8. The post-impact pipeline

Each kept post frame (`l3_considerBallTrack`):

- **Post scan plan** (`l3_scan_post`): 12 ball bins following the predicted
  ball (3 behind, from the band's far edge) plus 4 club bins: 16 bins, scored
  through the same routed path.
- **Ball track** (`l3_ball_track.c`): the departing ball, confirmed with a
  confidence gate; a confident departure displaces an unconfirmed smeared
  first point; its points' angles are estimated inline.
- **Hypotheses** (`l3_ball_hyp.c`, off by default: `L3_BALL_HYPOTHESES=0`)
  for the ball found rather than assumed.
- **Ball fit** (`l3_ball_fit.c`): at RESULT the ball's direction is fitted
  from the departing track's points, anchored at the tee (the ball track
  origin's slant range and bearing at `teeBallHeightM - radarHeightM`), with
  every point scored against the nearer of the direct path and the floor
  reflection. It reports HLA and VLA with a reason (`l3_ball_fit_why_name`).
  An uncertainty gate (finite-difference Hessian of the cost, covariance
  scaled by observed rms squared over `angleSigma` squared, limit
  `maxAngleSigmaRad` 3 deg) turns an ill-conditioned direction into an
  invalid result ("uncertain") rather than a confident wrong one. It runs
  once at RESULT, in the profile stage `reconstruct`.
- **Club filter** (`l3_track_kf.c`): an extended Kalman filter over the
  club's range and angles with a Rauch-Tung-Striebel smoother, plus a
  filtered delivery (`l3_track_delivery_filtered`). It is host-only: the
  replay runs it once at the end of a shot so the dump viewer can draw the
  club's reconstructed points beside the raw angle points. The board does
  not run it and the club's frozen delivery stays unfiltered, because the
  filtered delivery regressed club speed and path on a recording and on
  synthetic swings (about five approach points cannot pin the direction at
  a 15 deg angle sigma). The library function is host-tested for when it is
  tuned on labelled captures.
- **Shot machine** (`l3_shot.c`) → on RESULT: **impact fit**
  (`l3_impact_fit.c`: impact from the club-in, club-out and ball-out tracks
  either side of the band), **launch** (`l3_launch.c`: the ball's fitted
  departure walked back to impact), and the **result** packet
  (`l3_result.c`: each metric with a confidence and validity).

## 9. The DSS subsystem

### 9.1 Boot and its diagnostics

`dss_main.c` starts with `SOC_SysClock_BYPASS_INIT` (the MSS owns the clock).
Because a DSS that dies is otherwise invisible, it records its boot stage on
two channels the MSS reads without its help — the status block in HS-RAM
(`0x7F00`) and `DSSREG DSSGPREG0` (tagged `0xD55000xx`):

`reset` (xdc Reset hook, before C init) → `startup_first` / `startup_last`
(xdc Startup first/last functions, around the BIOS module startups) →
`main` → `soc_init` → `task` → `mailbox_init` → `link_open` (serving; a
heartbeat counts read timeouts, `served` counts requests). A BIOS exception
hook records `exception` with the program counter (NRP) and flags (EFR).

`trackCfg dsp status` prints the block; `trackCfg dsp hw` reads the DSS's
halt and power bits (DSSREG `GEMPWRSMCFG4`/`3`), the ROM self-test flag, the
ESM status registers, `DSSGPREG0`, and checks HS-RAM with a write/read. Both
print automatically after any unanswered `dsp` command.

### 9.2 The link (`l3_dsp_ipc.h`)

Mailbox channel 0, MSS <-> DSS. Fixed-width messages:

| Command | Request | Answer |
|---|---|---|
| `PING` | magic, seq | reply echoes seq |
| `PROBE` | a frame's L3 **offset** (the cores map L3 at different addresses), geometry, a bin range | the reply's sums, scoring and preparation cycles, `gathered` |
| `SCORE` | offset, geometry, up to 4 local spans, the detect queue's epoch | observations in the HS-RAM result block (written back out of the cache before the reply); the reply only signals it |

The DSS refuses any request that would read outside L3
(`l3_dsp_request_check`, subtraction-safe). The MSS accepts a SCORE result
only for its own seq and epoch with a consistent bin count
(`l3_dsp_result_check`), from a snapshot copy, then merges it. A reply that
belongs to an earlier, timed-out request is skipped by sequence number.
Timeouts: 6 ms (two frames) on the detect path.

### 9.3 The gather

Scoring in place, the DSS read one 4-byte sample of each (loop, TX, RX) row
per bin from L3 through a 16 KB L1D: 48 µs a bin. `l3_dsp_gather_plan` finds
the window of local bins a request reads (PROBE's bins; SCORE's spans from
the first to the end of the last); the DSS copies that window of every row
into `.dssGather` in **one AB-synchronised EDMA transfer on instance 1**
(aCount = a row's window, bCount = rows, source stride = a frame row),
polls it with a 1 ms bound, invalidates L1D over the copy and scores it
through the same scorer (`l3_dsp_serve_gathered`), so the answer is bit for
bit the in-place one. A refused plan, an untranslatable address or an
unfinished transfer falls back to scoring in place (invalidating L1D over
the L3 frame first). The preparation time is reported (`dss_prep_us`,
SCORE's `dss_inv_us`).

### 9.4 Which core scores (`l3_detect_core.c`)

`trackCfg detectCore dss|verify`:

- `dss` (default): the detect task sends SCORE and blocks on the reply (the
  CLI and notices run meanwhile). A failed frame falls back to the MSS;
  three in a row latch the MSS and queue `dsp detect latched to mss`, until
  `dss` or `verify` is chosen again.
- `verify`: both cores score the same bins at once and are compared bit for
  bit (`l3_dsp_result_compare`); the MSS's are used; never latches. Needs an
  IQ16 ring and the DSS link.
- `mss` is not a choice. Frames that are not IQ16 ring frames in L3 (IQ8,
  `compact16`, `adaptive16`), or that find the link held by a CLI `dsp`
  command or down, go to the MSS as `ineligible`, so the MSS scorer stays.

## 10. The club's angles off the decision path

The fire uses the club track's range only; its angles feed the delivery and
the shot. So (`l3_angle_queue.c`):

- The detect task takes the associated target's channel snapshot while the
  ring slot is valid and **pushes** it keyed by the track point's
  **timestamp** (unique per frame, robust to track resets and to the
  32-point history rolling). A full queue (12) drops its oldest.
- **`l3_angleTask`** (priority 2) wakes on a semaphore: **peeks** the oldest
  job under `Task_disable`, **estimates unlocked**, then under
  `Task_disable` **finishes**: only if its job is still the oldest is it
  taken and applied to the point with that timestamp; otherwise a fire
  frame's drain already applied it and the result is superseded. So no job
  is ever in flight outside the queue.
- **A fire drains** the queue on the detect task after the freeze request,
  so the shot freezes every angle (§7).
- Counters in `triggerLog perf`:
  `angles queued= done= stale= failed= dropped= pending=`.

The angle estimate itself (`l3_angle.c`): TDM phase correction with the
track's range rate, an alias search, then a 161-step Bartlett elevation scan
whose steering rotors are a **table filled once** (`l3_angle_tables_init`,
in HS-RAM) with the same `sinf`/`cosf` the scan used to call per step.

## 11. Module map

All `l3_*.c` modules are pure C with no TI headers: the host builds them into
one library (`firmware_host.HOST_SOURCES`) and tests them through ctypes.
`l3_dump.c` is the board glue (drivers, tasks, globals, CLI) and is tested by
source-structure tests.

| Module | Responsibility |
|---|---|
| `l3_dump.c` | MSS main: SOC/mmWave/HWA/EDMA setup, the capture ring, tasks, the CLI, the glue of every module below |
| `capture_plan.c` | Capture-plan arithmetic (windows, frames, bytes) |
| `detect_queue.c` | Completed-slot queue from the writer to the detect task; slot liveness by epoch |
| `compact_iq16.c`, `l3_iq8.c`, `l3_retain.c`, `l3_adaptive.c` | Capture formats: IQ16 compaction, IQ8 quantisation, retention policy, adaptive windows around the locked ball |
| `l3_iq16_stats.c` | Exact integer per-channel statistics (each sample read once) |
| `l3_bin_score.c` | One bin's burst-MTI observation from an IQ16 frame: the scoring **both cores** build |
| `l3_channels.c` | The per-channel loops over a detect frame: residual (IQ16 → `l3_bin_score`, IQ8 float path), static power, moving and static angle snapshots |
| `l3_scan.c` | The scan plan: which bins a frame scores, before and after impact |
| `l3_trigger.c` | The trigger front end: region, running floor, trace |
| `l3_observation.c` | Per-bin observations → ranked targets (sub-bin peak, SNR, Doppler, confidence) |
| `l3_band.c` | The tee band and its per-bin noise map |
| `l3_club_track.c` | The trajectory core: 32-point ring, predictive association, fits, delivery, point lookup by timestamp |
| `l3_impact.c` | Range-only impact: crossing and approach-end rules |
| `l3_leave.c` | Ball-leave fallback |
| `l3_ball.c` | Ball-placement detector with a scalable cadence; angle-due clock |
| `l3_angle.c` | Angle estimation; steering-rotor table |
| `l3_angle_queue.c` | The club's pending angles: push, peek/finish, drain, apply by timestamp |
| `l3_frames.c` | Radar and golf coordinate frames, the board calibration |
| `l3_shot.c` | Shot state machine: waiting → ready → club → impact → post → result |
| `l3_ball_track.c`, `l3_ball_hyp.c` | The ball after impact: track, hypotheses (off by default) |
| `l3_impact_fit.c` | Impact from the tracks either side of the band |
| `l3_launch.c` | Launch from the ball's departure |
| `l3_ball_fit.c` | Tee-anchored ball direction (HLA/VLA) with an uncertainty gate; board, at RESULT |
| `l3_track_kf.c` | Club EKF + RTS smoother and filtered delivery; host/viewer only |
| `l3_result.c` | Shot result packet with confidences |
| `l3_profile.c`, `l3_timing.c` | Per-stage costs; latency and throughput |
| `l3_detect_core.c` | Which core scores; fallback and latch; verify comparison bookkeeping |
| `l3_dsp_ipc.c` | The MSS<->DSS messages, checks, SCORE serving, the gather plan and copy, status and hardware formatting |
| `track_select.c`, `live_selector.c` | The `l3track` cell selection; the live selector |
| `l3_text.c` | Integer-only float formatting for the CLI |
| `dss/dss_main.c` | DSS: boot stages, the mailbox listener, the gather (EDMA instance 1), SCORE/PROBE serving |
| `solve/*.c` | DSS ports of the post-shot solve stages (FFT, tracking, LCMF) — built, not yet dispatched |

## 12. Host side

| Component | Role |
|---|---|
| `openflight/iwr6843/driver.py` (`IWR6843Radar`) | The CLI transport: `cmd`, config, dumps, `release_sparse_freeze`, `dsp_ping/probe/status/hw`, `detect_core`, `detect_timing` |
| `openflight/iwr6843/monitor.py` (`IWR6843CaptureMonitor`) | The kiosk's capture worker: owns the port, arms the self-trigger, reads captures, and **always re-arms** the board after a capture (`l3release`, else a restart as start-up configured it, with restart hooks re-sending `ball cfg`) |
| `openflight/iwr6843/dsp_link.py` | Parsers for every link/detect line; `summarize_probes`; `evaluate_acceptance` |
| `openflight/iwr6843/firmware_host.py` | Builds the pure-C modules for the host (cc/gcc/clang, else the bundled `zig cc`) with ctypes mirrors of every struct |
| `openflight/iwr6843/firmware_replay.py` | Replays a recorded `.l3dump` through the same C modules frame by frame, mirroring the board's orchestration (scan plan, scoring, trackers, angle queue, shot) |
| `scripts/hardware-test/iwr6843_dsp_probe.py` | Board check: ping, probe (both cores, timed, bit-for-bit), and `--acceptance` (verify then dss while swinging, judged) |

## 13. CLI reference

The SDK caps the table at `CLI_MAX_CMD` (32) with the mmWave extension's
commands; new diagnostics are sub-modes.

| Command | Purpose |
|---|---|
| `sensorStart` / `sensorStop` | Configure and start / stop |
| `captureCfg`, `phaseCaptureCfg`, `captureFormat`, `iq8Scale` | The capture plan and format |
| `l3dump`, `l3sparse`, `l3track`, `l3release` | Read the frozen ring (whole, sparse, tracked cells); rearm |
| `triggerCfg <bin> <snr> <on>` | The self-trigger |
| `trackCfg ...` | The tracker; sub-modes `cal`, `elem`, `impact`, `impactFit`, `ballSnr`, `subbin`, **`dsp ping\|probe [bins]\|status\|hw`**, **`detectCore [mss\|dss\|verify]`** |
| `ball [status] \| scan \| cfg` | The ball-placement detector |
| `triggerLog [trace\|track\|shot\|result\|perf\|timing\|frames\|cal\|clear]` | Diagnostics; `perf` includes the angle queue and detect core |
| `stats`, `hwastats`, `debugCfg` | Health counters (`detect dropped= stale= ... stale_read=`), HWA state, streamed decisions |

## 14. Verification strategy

1. **Host unit tests** of every pure-C module through ctypes
   (`tests/test_iwr6843_firmware_*.py`), including bit-for-bit equivalence
   where code moved or was optimised: gathered vs in-place scoring, the
   steering table vs the recorded scan outputs, read-once vs two-pass, the
   angle queue vs an immediate estimate.
2. **Replay parity**: every committed recording (`tests/radar/recordings`)
   replayed through the C modules; the labelled swings' bars (fire offsets,
   launch speeds) and work budgets (≤ 29 bins pre-impact, ≤ 16 post, ≤ 1 club
   snapshot per pre-impact frame). The September changes were checked to
   leave all 41 recordings' fire frames, club points and angles, delivery,
   launch and shot identical.
3. **Source-structure tests** pin the board glue that cannot run on the host
   (task order, lock placement, memory placement, CLI table limits).
4. **The firmware build** itself (warnings are errors; the MSS HS-RAM
   reservation fails the link on overlap).
5. **The board**: `iwr6843_dsp_probe.py` (and `--acceptance`), `triggerLog
   perf`/`timing`, `stats`.

## 15. Measured numbers and what still needs the board

Measured on the board (2026-09-30):

| What | Value |
|---|---|
| MSS bin scoring | ~73 µs a bin undisturbed (the probe reads ~103 µs: the CLI task it runs on is preempted by the detect task) |
| DSS ping round trip | 22 µs |
| DSS scoring in place | 27 bins 1,303 µs (48 µs a bin); 53 bins 2,558 µs; 40/40 bit-for-bit matches |
| Per-frame budget | 3 ms (frame period) |

Waiting on the board:

- The **gathered** DSS timing (`dss_prep_us` + scoring) from
  `iwr6843_dsp_probe.py`.
- `--acceptance`: `verify` then `dss` through real swings with no mismatch,
  failure, fallback, stale frame or dropped angle, and the detect task's
  mean service under 3 ms. Then `dss` becomes the default (the review's 2A).
- Whether gathering only the vertical TX pair is worth its complexity
  (review item 14): decided by the measured `dss_prep_us`.
- Real `triggerLog perf` per-stage costs after the ball-detector cadence,
  the angle queue and the steering table.
