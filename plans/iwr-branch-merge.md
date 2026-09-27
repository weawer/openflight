# Plan: combine `feat/iwr-calcs` and `feat/iwr-offloading`

> Date: 2026-09-27.
> Branches: `Cormac131/feat/iwr-calcs` at `f72c352` (99 commits since the shared
> base) and `feat/iwr-offloading` at `483f11b` (38 commits since the base).
> Shared base: `3989ac9`.
> Status (2026-09-27): selective detector port implemented on
> `feat/iwr-2ms-trigger-merge`, preserving the offloading capture architecture.
> The original full-merge direction below remains background, not the scope
> of this implementation. Hardware validation is pending.

## Implemented detector slice

**Hardware follow-up (2026-09-27):** the static smoke test failed. Its initial
stats already show `latched=1`, firing after two detector observations,
`frame_work_max_us=3224`, and `hwa_missed=1`. The 2 ms timing acceptance is
therefore blocked. Native regression tests reproduced an approach-rate
bypass when a strong return stays at the same bin or moves backward inside
the gate; that bypass is fixed. The original log has no frame records, so
this is not proof of the exact return that caused the hardware trigger.
The test now preserves failure stats and `triggerLog`, including failures
already present at its initial baseline. Performance work on the combined
selector/detector path is a separate remaining change; do not accept this
release for 2 ms operation based solely on the gate fix.

**Timing follow-up (2026-09-27):** the new diagnostic image records stage costs
from the same frame that sets `frame_work_max_us`. Adaptive pre-trigger frames
now seed only the impact-window baseline instead of running the full shadow
selector whose decisions are discarded before capture. Raw detector trace is
opt-in (`triggerLog trace on|off`); candidate/event records and health counters
remain enabled. The hardware checker uses the combined same-frame total, not a
sum of independent stage maxima, with a 1,500 us engineering target and the
frame period as the hard limit. On a false latch, it saves the frozen dump when
`--capture-dir` is supplied. Build artifact and operator command:
[`l3_dump_2ms_iq16_timingfix_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix_20260927.md).
This build is not yet hardware-validated.

**Capture-phase timing result (2026-09-27):** the first timing-fix smoke run
passed its static soak at 1,426 us, but a capture-phase frame reached 1,832 us.
The maximum-work frame attributed 1,473 us to shadow selection, 314 us to
compaction, 39 us to rearm, and zero to trigger processing; no HWA frame was
missed and no unexpected trigger occurred. The follow-up image limits the
adaptive impact-phase shadow reduction to its configured range window and
re-primes the baseline on transition to full-range flight selection. Image,
checksum, and repeat-smoke command:
[`l3_dump_2ms_iq16_timingfix2_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix2_20260927.md).
The follow-up image has not yet been hardware-tested.

**Stationary false-trigger follow-up (2026-09-27):** `timingfix2` fired after
repeated observations in bin 38 followed by a one-bin shift to bin 39. The
approach timer was reset on equal-bin observations, making the final shift
appear fast. The detector now resets the approach origin only on actual
retreat; a native regression reproduces the hardware sequence. The independent
test image and checksum are recorded in
[`l3_dump_2ms_iq16_timingfix3_trigger_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix3_trigger_20260927.md).
This image has not yet been hardware-tested.

**Second stationary trigger trace (2026-09-27):** `timingfix3` fired on bin 39
after a missed frame, retreat to bin 38, and one-bin return to bin 39. The
rate threshold was met exactly over one frame. Multi-observation triggers now
require two bins of net approach as well as the configured rate; one-frame
mode is unchanged. The separate image and stationary test are documented in
[`l3_dump_2ms_iq16_timingfix4_trigger_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_timingfix4_trigger_20260927.md).
This build is not yet hardware-validated.

The donor `l3_trigger.c/h` detector now consumes all-loop vertical-TX residuals
from compacted IQ16 frames. Host configuration uses global bins with the
existing range calibration. Adaptive16 retention and RF-stop retry behavior
remain in place. Dump/release paths reset the fired detector while preserving
its learned noise floor, allowing subsequent captures.

The one-shot trigger notification is deferred to a CLI-priority task;
per-frame UART debug streaming stays disabled. Trigger logs use a stable
snapshot. Stats expose detector time and combined frame-work time so the
2 ms budget can be checked with the detector enabled. The test parser keeps
acquisition frame counts separate from detector frame counts.

This is not the full donor architecture: the separate detection task,
general notice queue, ball-placement/follow feature, UART RX interrupt change,
and DSS solve are not ported. The detector still runs in the rearm task;
real-board timing is a prerequisite for accepting that arrangement.

Release and operator checks:
[`l3_dump_2ms_iq16_adaptive_trigger_20260927.md`](../firmware/releases/l3_dump_2ms_iq16_adaptive_trigger_20260927.md).
The user runs flashing and hardware tests. Static soak validates timing and
capture/rearm mechanics; successful real swings and repeated self-triggers
remain separate acceptance checks.

> **Ported ahead (2026-09-27, uncommitted, for a new branch):** from the
> calcs branch, `l3release` (section 4), the host `cmd()` reading through the
> prompt (section 5, host half of `294fdbf`), and readbacks dropping a stale
> notice. See
> `firmware/releases/l3_dump_2ms_iq16_adaptive_l3release_20260927.md`.
> The detector slice above supersedes this earlier status. The general notice
> queue, `48a81a8`, and ball-placement/follow work remain deferred.

## Summary

The branches mostly changed different parts of the system:

- **`feat/iwr-calcs`:** the self-trigger, firmware and serial reliability,
  on-board diagnostics, and an on-chip (DSS) solve.
- **`feat/iwr-offloading`:** what is retained after the trigger (adaptive16
  capture, live selector, coherent gate), the measured range bias, and OPS
  trigger options.

`feat/iwr-calcs` contains our Phase 3 selector (`36f8531`) but none of the
later work: no adaptive16, no confirmation/coasting, no coherent gate.

**Direction:** start from `feat/iwr-calcs` and port the offloading work onto
it. `feat/iwr-calcs` restructured `l3_dump.c` far more (the trigger moved into
`l3_trigger.c`, a detect task was added, `CONFIGURABLE_CAPTURE` was removed),
so porting in that direction produces fewer and simpler conflicts.

## Decisions by area

### 1. Trigger detector — take `feat/iwr-calcs`

- **Offloading:** the "ball leaves the tee" rule in `l3_considerSelfTrigger`.
  It has never fired on a real swing on hardware.
- **Calcs:** `l3_trigger.c`, which tracks the clubhead toward the tee using
  the burst-MTI residual over every loop, with:
  - an adaptive median noise floor;
  - range continuity across frames, bridging one missed frame;
  - an impact gate around the tee with minimum track age and approach rate;
  - an optional Doppler speed gate against body movement;
  - a 128-record flight-recorder log and a raw-input trace.

  It is also not yet proven on a real swing: the last recorded run found the
  club at bins 46–50 while the trigger watched bin 34 (`f72c352`).

Firing at impact suits adaptive16's split: the 14 pre-trigger frames (28 ms)
hold the club's approach, and the 6 impact frames hold the launch.

**Check:** the detector reads the retained pre-trigger slot
(`l3_verticalResidual`). With adaptive16, pre-trigger slots are compacted IQ16
of bins 22–53. The watch region (tee minus 12 bins to tee plus 3) must stay
inside that window, and the sample reads must match the adaptive16 slot
layout.

### 2. Tee position — take `feat/iwr-calcs`, add the measured range bias

- **Calcs:**
  - all bins are global;
  - a ball-placement detector (`l3_ball.c`, `ball` command) finds the placed
    ball against a learned static background;
  - with `follow` enabled, the trigger aims at the located ball, falling back
    to the configured tee.
- `tee_global_bin` converts distance to bin without any range-bias
  correction.
- **Offloading:** measured a constant bias of +1.5 to +1.8 bins (about
  7.5 cm) at 1.20 m and 1.845 m (`71a7f67`). This agrees with
  `range_bias_const_m` = 0.066 m in `config/iwr6843_calibration_reference.json`.

**Port:** add a `range_bias_m` argument to `tee_global_bin`, loaded from
`--iwr6843-cal`, as in `3adbfe9`. It applies to the fallback only; a located
ball needs no correction.

### 3. Stopping RF after a self-triggered freeze — take `feat/iwr-calcs`, plus one refinement

Both branches found and fixed the same bug independently: the firmware's own
freeze leaves the radar transmitting, so the next rearm fails with
"RF restart failed", and `sensorStop` closes the radar while it is still
running.

- **Calcs:** `l3_awaitFrozenRing` (`c2a5d61`, `06439be`), shared by
  `l3sparse`, `l3track`, `l3release` and `sensorStop`.
- **Offloading:** `483f11b`.

**Refinement to port:** in `l3_awaitFrozenRing`, clear `gSelfTriggerLatched`
only after `l3_finishCaptureStop()` succeeds. The calcs version clears it
first. If the stop then fails, the retry takes the unlatched path, which
returns early once `gCaptureActive` is 0 and never stops RF. The native C
harness test `test_failed_rf_stop_keeps_the_latch_so_a_retry_stops_rf_again`
(`483f11b`) covers this.

### 4. Releasing an unwanted freeze — take `feat/iwr-calcs`

- **Offloading:** `eb8e895` releases an adaptive16 freeze by reading the whole
  dump and discarding it, because `l3sparse` rejects adaptive16. That costs
  about 5.3 s each time.
- **Calcs:** `l3release` (`006525f`) rearms with one command and sends no
  data.

**Port:** `l3release` must end in the offloading version of
`l3_sparseRearm`, which also resets the adaptive16 state (`gRetentionReason`,
`gRetentionPostFrames`, `gShadowState`, `gShadowHavePrevious`). Once it does,
drop the discard branch from `_listen_for_self_trigger`.

### 5. Serial and CLI reliability — take `feat/iwr-calcs`

- `294fdbf`:
  - `cmd()` reads through the CLI prompt;
  - the firmware queues notices (including "Triggered") through
    `l3_noticeTask`, so they are no longer spliced into command replies.
- `48a81a8`: the `l3sparse` cell line is read through the UART RX interrupt.
- `3ff13a6`: `l3dump` waits for a full ring; the detector is waited for after
  disarm.

The offloading branch has none of these, and they plausibly explain several
of its board wedges and lost notices.

### 6. Session start state — take `feat/iwr-calcs`

Both branches carry the `a38fbe8` lifecycle fixes (ported as `e00aee5` and
`7c4f992`). The calcs branch also makes the detector wait for a full
pre-trigger ring before arming.

### 7. Post-trigger retention — take `feat/iwr-offloading`

The offloading branch keeps:

- adaptive16: 36 frames of IQ16 at 2 ms, with all 3 TX, 4 RX and 12 loops.
  Retention windows are 14 frames × 32 bins, 6 frames × 53 bins, and 16
  frames × 12 bins.
- confirmation (3 frames) and coasting (up to `maxMisses`);
- the coherent gate, computed over the active window only.

Hardware evidence:

- a 100/100 static soak with no flight frames kept;
- full 36-frame captures of a moving object at about 1.9 m;
- worst-case timing of 1560 µs selector and 255 µs compaction per 2 ms frame.

The calcs branch plans a different route ("longer movie", `2026-09-25`):
51 frames of IQ8 at 2 ms, with the IQ16 scratch buffers moved into DATA_RAM.
That trades sample precision for length, and our retention plan explicitly
excludes IQ8. Moving scratch out of L3 does not directly help adaptive16:
its scratch reservation is 196 KB, more than the free DATA_RAM.

### 8. OPS trigger options — take `feat/iwr-offloading`

- `c866ff9`: opt-in `--ops-trigger-speed-mph` / `--ops-trigger-magnitude`
  send `ST`/`SM` in rolling-buffer mode only.
- `0921617`: these flags are refused with `--trigger speed`.
- `c2caa37`: when the OPS onboard trigger is on, the IWR self-trigger stops
  sending `S!` to the OPS but still drives the camera.

The calcs branch has none of this, and its `S!` relay code is unchanged from
the base, so these ports should be straightforward.

### 9. Tools and diagnostics — keep both

- **Calcs:**
  - the firmware CLI check suite (`firmware_checks.py`,
    `scripts/hardware-test/test_iwr_firmware.py`);
  - `scripts/iwr6843/swing_trigger.py`;
  - `triggerLog` (log, trace, clear);
  - `teeScan` / `ball scan`.
- **Offloading:**
  - `check_iwr_2ms_iq16.py` with `--expect-no-flight-frames` and the
    per-cycle flight-frame count;
  - `scripts/analysis/iwr_selector_replay.py`.

### 10. On-chip solve (DSS) — no action

The calcs branch ports tracking and the LCMF vertical decision onto the DSP.
Onboard measurement is on hold for this work; it stays on the calcs branch
unchanged.

## Risks

1. **Processing time per 2 ms frame.**
   - The adaptive16 selector and coherent gate already reach about 1.88 ms
     worst case, leaving about 120 µs.
   - The calcs detector runs every frame in `l3_detectTask`, below the rearm
     task.
   - Together they may overrun the frame, or starve the detector, which would
     show up as `detect dropped=` / `stale=` in `stats`.
   - Needs a hardware measurement before trusting either.
   - If it does not fit, the first lever is computing the coherent difference
     over the active window only (see `plans/iwr-coherent-gate.md`, risk 4).
2. **`l3_dump.c` conflicts.** Expect to re-apply the offloading firmware
   changes by hand rather than merge them:
   - capture plan and `captureFormat adaptive16`;
   - `l3_storeCompletedScratchFrame` (selector, coherent rows, retention);
   - dump header versions 8/9;
   - `sensorStop` / freeze.
3. **DATA_RAM.** The offloading branch uses 98,393 of 196,608 bytes,
   including 18,432 bytes of previous-frame rows. The calcs branch adds the
   detector log and trace, the ball detector, and the detect queue. Check the
   linker map after the port.
4. **`triggerCfg` arguments changed.** The calcs form is
   `triggerCfg <globalBin> <snr> <frames> [approach gate minCoh minStep stat minSpeed]`,
   replacing the offloading `<localBin> <level> <hits>`. The host
   `SelfTriggerConfig`, CLI flags and tests must follow the calcs form.
5. **Out-of-date image.** The latest calcs release image (`708eb8f`) predates
   the calcs commits `4e15528`, `274349a`, `513ee7e` and `f72c352`. Rebuild
   before any hardware run.
6. **Default config.** The calcs branch defaults `--iwr6843-config` to
   `wide_24f3ms_53bin_iq16`. The offloading branch defaults to adaptive16
   (branch-only). Keep adaptive16 as the default only on the combined test
   branch, and never carry it into `main` before Phase 5 of
   `plans/iwr-iq16-2ms-onboard-retention.md` passes.

## Steps

1. Create `feat/iwr-combined` from `Cormac131/feat/iwr-calcs`. Work in a
   separate worktree.
2. **Firmware, in this order, each with its tests first:**
   1. adaptive16 capture plan, `captureFormat adaptive16`, retention report,
      and dump versions 8/9 (from `f8d0a94`);
   2. `live_selector.c`/`.h` with confirmation, coasting, `l3_retention_window`
      and `l3_coherent_gate` (from `a90f3ad`, `cdedd5a`, `f9718a8`);
   3. selector input, coherent rows and gating in
      `l3_storeCompletedScratchFrame`, gating over the active window only;
   4. `l3release` → the adaptive-aware `l3_sparseRearm`;
   5. clear the latch after the RF stop in `l3_awaitFrozenRing`.
3. **Host:**
   1. `live_selector.py` (confirmation, coasting, `retention_window`,
      `coherent_gate`);
   2. `dump.py` retention parsing;
   3. `monitor.py` adaptive readout, releasing through `l3release`;
   4. `check_iwr_2ms_iq16.py` and `iwr_selector_replay.py`;
   5. the calibrated adaptive config (`preStart` 22, `impactStart` 34);
   6. `range_bias_m` in `tee_global_bin`;
   7. OPS `ST`/`SM` options and `S!` relay gating.
4. Rebuild the firmware image under a new release name and check the linker
   map for L3 and DATA_RAM.
5. Run the full test suite, both branches' tests together.
6. **Hardware, in order:**
   1. the calcs firmware check suite;
   2. `check_iwr_2ms_iq16.py` 1,000-frame smoke with the detector armed —
      record `rearm`/`compact`/`selector` maxima and `detect dropped`/`stale`;
   3. 100-cycle static soak with `--expect-no-flight-frames`, with the
      detector armed and the room empty;
   4. `ball scan` / `ball` lock at the 1.84 m tee, comparing the locked bin
      with the bias-corrected tee bin (expected about 41);
   5. `swing_trigger.py` with a real swing, reading `triggerLog` on a miss;
   6. kiosk start with `--iwr6843-self-trigger`, then with the OPS `ST`/`SM`
      options.

## Acceptance

- [ ] All tests from both branches pass on the combined branch.
- [ ] TI build succeeds with warnings as errors; L3 and DATA_RAM fit, with
  the margins recorded.
- [ ] Per-frame timing with detector and selector both active stays under
  2,000 µs worst case, and `detect dropped`/`stale` stay 0 over 100,000
  frames.
- [ ] 100/100 static cycles keep no flight frames with the detector armed.
- [ ] The trigger does not fire in an empty lane over the soak.
- [ ] The located ball's bin and the bias-corrected tee bin agree within
  1 bin at the measured tee.
- [ ] A real swing fires the trigger and yields a complete or explicitly
  early-stopped adaptive16 capture. A miss is explained by `triggerLog`.
- [ ] `l3release` rearms an adaptive16 freeze without streaming, and no run
  needs a hardware reset.
