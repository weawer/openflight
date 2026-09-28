# IWR 2ms Pipeline Handoff

Updated: 2026-09-28

## Review corrections and next hardware run

The armed run in `openflight_sessions/iwr-armed-soak/run.jsonl` subsequently
failed with nine stale scratch reads and no HWA misses. CLI/notice output
outranked detection while using polled UART writes; the residual wall-time
maximum reached about 99 ms during diagnostic output. Scheduling now orders
control > rearm > detection > CLI/notice. Performance counters reset at
sensor start, and the soak rechecks health after its final diagnostic output.
See `firmware/releases/l3_dump_pipeline_wip_cli_fix_20260928.md` for the
WIP-based test image; it still requires hardware validation.

Frame-rate equivalence is a separate open issue: `minStepBins=1` requires
15.625 m/s radial progress at 3 ms but 23.4375 m/s at 2 ms. The eight-bin
continuation limit and frame-count timeouts likewise change their physical
meaning. Ball limits expressed in m/s and timestamp-based fits retain their
units. Chirp/loop timing is unchanged. Normalize the frame-based rules to
elapsed time and replay identical physical trajectories at both cadences
before claiming equivalent detection behavior; no gates are changed by the
scheduling fix.

The earlier cadence soaks did **not** arm the self-trigger. `sensorStart`
disables it, and the original soak only loaded the config. Those runs validate
capture cadence; they do not validate club-detection or post-impact tracking
workload. No armed-pipeline hardware validation has been performed by the agent.

The host tools now use SNR ratios consistently. `swing_trigger.py` previously
passed an absolute floor-times-SNR threshold as the SNR argument, defaulted
to the old 3 ms profile, and replayed the retired ball-leave detector. It now
defaults to the 2 ms adaptive profile, saves `triggerLog`, trace and performance
reports, and saves self-triggered raw dumps without that obsolete PASS/FAIL
judgment. Automatic tee-bin selection in the server and live diagnostic tools
adds the selected calibration's range bias; an explicit server bin remains
an override. No firmware or trigger thresholds changed for these fixes.

Keep the soak-tested `l3_dump_pipeline_wip.bin` on the board. Stop the kiosk,
keep the scene still, and replace 1.845 below with the measured antenna-to-tee
slant distance. Run this short **armed pre-trigger** test first:

```bash
uv run python scripts/hardware-test/iwr6843_cadence_soak.py \
  --config config/iwr6843_l3dump_adaptive_47f2ms_53bin_a16.cfg \
  --self-trigger-tee-m 1.845 --frames 1000 \
  --output openflight_sessions/iwr-armed-soak/run.jsonl
```

It verifies enable/latch state and detection queue/scratch counters, recording
raw stats and final trigger diagnostics. If successful, repeat with 50,000
frames. This still does not exercise post-impact tracking; that needs a real
trigger and capture. Then run:

```bash
uv run python scripts/iwr6843/swing_trigger.py --tee-m 1.845 \
  --capture-dir openflight_sessions/iwr-trigger
```

Both tools accept `--cal` for the same calibration used by the app. Share the
JSONL and any dumps, including when no trigger fires. The viewer firmware is
still untested. The historical results and investigation context below remain
useful subject to these corrections.

## Goal

Converge on Cormac's firmware pipeline (trigger, club track, ball track,
angle estimation, shot state machine) running at our 2 ms frame period,
validated on this rig's real hardware. This branch (`feat/iwr-2ms-pipeline`)
starts from `Cormac131/feat/iwr-calcs`, not from our old selector-based
branch (`feat/iwr-2ms-trigger-merge`, kept as-is and pushed for reference).

## Current State

- Branch: `feat/iwr-2ms-pipeline`. Base: `776c266` (Cormac131/feat/iwr-calcs
  tip at the time we branched).
- Working tree clean at `00e75c10` when this handoff was written.
- Two firmware images exist under `firmware/releases/`:
  - `l3_dump_pipeline_wip.bin`, SHA-256 `903bfd99...ced13c1`. **Soak-tested
    clean on hardware** at both 3 ms and 2 ms (see Hardware Evidence). This
    is the one to flash for real-shot testing.
  - `l3_dump_pipeline_viewer_20260928.bin`, SHA-256 `54e29ef0...97f96af7`.
    Adds the dump viewer's `l3_track_follow` (post-impact club follow).
    Built, links clean, **not run on hardware**. Do not flash this for the
    open trigger investigation below; it changes club-track behavior, an
    unrelated variable.
- Configs: `config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg` (3 ms) and
  `config/iwr6843_l3dump_adaptive_47f2ms_53bin_a16.cfg` (2 ms) differ in
  exactly one field (`frameCfg`'s period), pinned by
  `test_the_2ms_adaptive_profile_differs_from_3ms_only_in_frame_period`.
- Trigger path in use: `--trigger sound` (OPS passive, waits for `S!`) +
  `--iwr6843-self-trigger`. No physical SEN-14262 sound sensor on this rig
  -- the IWR's self-trigger relaying `S!` is the *only* way a capture
  starts. OPS's own onboard autonomous trigger (ST/SM) is the other option
  this project chooses between, not currently selected.

## Work Completed

1. **Fixed a variable-shadowing compile bug** in `l3_considerBallTrack`
   (`731eae7`): a frame index was declared with the same name as the frame
   struct already in scope; `armcl` rejects it, host tests never compile
   `l3_dump.c` so nothing caught it. Renamed to `shotFrame`. Cormac
   independently found and fixed the identical bug on his branch
   (`4bd31cd`, named it `frameIndex`) -- external confirmation of the
   diagnosis.
2. **Fixed a `DATA_RAM` overflow** (`731eae7`): the full combined feature
   set didn't fit in the chip's 196,608 B `DATA_RAM` region (43 B free
   before even placing the stack). Trimmed the trigger's diagnostic flight
   recorder (128->64 records, trace 64->32) and compiled `l3sparse`/
   `l3track` out behind `L3_SPARSE_READBACK` (off by default; their combined
   buffers, `gTrackWorkspace` alone 17 KB, cost ~22 KB). Result: 23,251 B
   free (later 23,155 B after the viewer pull), policy minimum 16,384 B.
   The host already falls back cleanly to `l3dump` when `l3track`/`l3sparse`
   refuse -- confirmed by reading `driver.py`'s `_read_packet`, not assumed.
3. **Ported host reliability fixes** from the old branch (`a9ced07`): dump
   version 8/9 read support (so old session `.l3dump` files stay openable),
   the UART stall-tolerance bump (4.0s -> 8.0s) with a warning log, and the
   OPS `wait_for_hardware_trigger` blocking-read fix (was sleep-polling,
   adding latency/jitter to `trigger_delta_ms`).
4. **Unarmed cadence soak passed clean at both 3 ms and 2 ms** (see Hardware
   Evidence) -- `scratch_stale=0` both times. The self-trigger was disabled;
   these runs do not establish timing for active detection or post-impact
   tracking. Added the `scratch_stale` check itself to
   `iwr6843_cadence_soak.py` (it wasn't gated on before) and extracted its
   pass/fail logic into a tested `evaluate()` function (`855d283`).
5. **Pulled the dump viewer** (`00e75c10`) from Cormac's branch
   (`776c266..4bd31cd`, stopping before the joint ball/club hypothesis
   search that follows it, which is still gated off there and unjudged).
   Confirmed working with a live smoke test, not just imported. Brought
   `l3_track_follow` along since the viewer's host code was rewritten in
   the same commit to depend on it -- not separable. This is on
   `l3_dump_pipeline_viewer_20260928.bin`, a separate, not-yet-soaked image
   from `wip.bin`.
6. **Caught and fixed my own mistake**: an intermediate build during the
   viewer pull used the wrong output filename and silently overwrote
   `wip.bin` (the soak-tested image) with the new build. Caught via hash
   mismatch before committing, restored. Worth remembering when running
   `fwbuild.sh` by hand -- always pass an explicit `RELEASE_NAME`.

## Hardware Evidence

- 3 ms soak (`l3_dump_pipeline_wip.bin`,
  `config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg`, 50,001 frames):
  `missed=0 rate=0.000000% iq8_overrun=0 iq8_edma_err=0`,
  `rearm_max_us=43 (1.4% of 3000us)`, `scratch_stale=0`. PASS.
- 2 ms soak (same image, `config/iwr6843_l3dump_adaptive_47f2ms_53bin_a16.cfg`,
  50,002 frames): `missed=0 rate=0.000000%`, `rearm_max_us=39 (1.9% of
  2000us)`, `scratch_stale=0`. PASS. Both soaks only exercise timing --
  neither says anything about whether the ball tracker finds the ball.
- **Real-swing session at 2 ms: zero self-trigger fires.** The IWR's
  self-trigger, running this integrated pipeline live for the first time
  (see Important Context below), did not fire once across the session.
  Root cause not yet diagnosed -- this is the open item.

## Important Context

- **This was very likely the first time this whole pipeline ran live on
  real hardware, not just the first time at 2 ms.** The full build (trigger
  + club track + ball track + angle estimation + shot machine, all running
  together on the chip during a real swing) never compiled until this
  branch's first build attempt (`731eae7`) -- see Work Completed item 1.
  Whatever real-swing testing Cormac did before was necessarily against
  something else: the trigger fix alone (his own docs mark only that as
  hardware-validated, 2 swings), or the offline replay harness against
  recorded dumps (host-compiled C modules, not live silicon). Treat any
  claim that this architecture "works on real swings" as unverified until
  we see it ourselves.
- The old Phase-0 diagnostic approach (fixed 53-bin window, held-still,
  bin-by-bin visibility check) belonged to the previous selector
  architecture and does not directly apply here -- this pipeline's windows
  move dynamically per `l3_retain.c`'s shot-state policy, not a fixed wide
  window you can inspect frame-by-frame the same way. It would still be
  useful as a fallback diagnosis (RF visibility vs. tracking-logic problem)
  if real swings keep failing, but it is not a prerequisite.
- Session dump location: default is `~/openflight_sessions/iwr6843/`
  (home-directory-relative), **not** the repo-relative
  `openflight_sessions/` this whole project's historical data lives under.
  Unconfirmed whether the Pi bridges these automatically (symlink) or a
  manual step does. Pass `--log-dir <repo>/openflight_sessions` explicitly
  to be certain, or check `ls -la ~/openflight_sessions` on the Pi for a
  symlink.

## Next Steps

1. **Diagnose the zero-trigger real-swing session.** Run
   `uv run python scripts/iwr6843/swing_trigger.py --tee-m 1.845` (stop the
   kiosk first, it owns the UART) against a few real swings and read what
   it prints. The corrected tool described above arms `triggerCfg` with an
   SNR ratio, saves detector logs, and saves the raw dump when triggered.
   Three distinguishable outcomes:
   - Nothing ever changes: the detector sees no moving return in its watch
     region at all. Check tee-bin/geometry first (is 1.845 m still correct
     for wherever this rig physically is right now?).
   - A track forms and advances but never reaches `Triggered`: it sees the
     club but a gate condition rejects it (speed, approach distance,
     coherence). See the lead below.
   - Something else (never arms, errors).
2. **One concrete lead, not yet confirmed against evidence**: the
   self-trigger's approach-speed gate (`L3_TRIG_DEFAULT_MIN_STEP_BINS =
   1.0F`, `l3_trigger.h:72`) is specified in bins *per frame*, not per
   second. Its own comment says "~15 m/s radial at 3 ms" -- at 2 ms the
   same nominal value implies a higher required velocity (~23.4 m/s vs
   ~15.6 m/s, since less real time elapses per frame). Nothing adjusted
   this when the 2 ms config was added. This alone probably isn't strict
   enough to block every swing (real club speeds are well above 23 m/s),
   but it moved in the wrong direction and could compound with something
   else. **Do not change this from assumption** -- `swing_trigger.py`'s
   trace output is what turns this from a guess into evidence, per the
   project's standing rule below.
3. Once real triggers are firing: judge whether the ball tracker finds the
   ball or the golfer/club, using the dump viewer
   (`scripts/iwr6843/dump_viewer.py`) on the resulting captures. This is
   the actual question the whole branch pivot exists to answer; nothing so
   far (soaks, viewer pull) has tested it yet.
4. Decide whether to bring over the joint ball/club hypothesis search
   (`5b2ef85d..6e16bbb` on Cormac's branch) once it's judged on his side --
   deliberately not pulled yet, still gated off there.

## Standing Rules

- **Tune only from evidence.** Do not change a trigger/detector threshold
  (gate width, approach bins, min step, min speed, coherence) without a
  real capture or trace showing the specific failure it fixes, and add a
  regression that reproduces it first. This bit us already on the previous
  branch (see `feat/iwr-2ms-trigger-merge`'s handoff, superseded but the
  rule carries over).
- **Never claim hardware validation without hardware evidence.** A soak
  passing tests timing only. A clean build is not a validated image --
  `l3_dump_pipeline_viewer_20260928.bin` is built and untested; say so
  plainly whenever it comes up.
- **After any firmware change**: run the native/host test suite, build a
  distinctly named image (explicit `RELEASE_NAME`, learn from the mistake
  in Work Completed item 6), record its SHA-256, and say exactly which
  tests ran and which didn't before asking for a hardware run.
- **`DATA_RAM` margin is tight** (~23 KB free of 196,608 B total). Any new
  static buffer should be checked against the linker map, not assumed to
  fit.
- Read this handoff, Cormac's `docs/development/firmware.md` roadmap
  table, and the joint-tracking design doc
  (`docs/superpowers/plans/2026-09-28-iwr-joint-club-ball-tracking.md`, not
  yet acted on) before proposing further firmware changes.
