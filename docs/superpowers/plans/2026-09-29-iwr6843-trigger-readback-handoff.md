# IWR6843 trigger + readback handoff

Updated: 2026-09-29

## Branch and starting point

`test_build`, branched from Cormac's `feat/iwr-calcs` at `bb06293e` ("iwr:
place the impact and ball capture windows on the tee") -- his newest branch
as of 2026-09-29, superseding this project's own earlier `feat/iwr-calc` and
`feat/iwr-2ms-trigger-merge`/`feat/iwr-2ms-pipeline` lines (kept, unmerged,
for reference). Everything below is on top of that commit.

## What was ported from this project's own earlier work

Before any new fix, four things were pulled from `feat/iwr-2ms-pipeline`
(this project's now-superseded branch, itself Cormac's `feat/iwr-calcs` plus
some of this project's own commits) and adapted:

1. **Host reliability fixes**: dump v8/9 read support, UART stall-tolerance
   bump (4.0s -> 8.0s, now known to have masked the readback bug below
   rather than fixed anything), OPS `wait_for_hardware_trigger` blocking-read
   fix.
2. **Soak health check**: gate the armed soak on `scratch_stale`, with a
   tested `evaluate()`.
3. **Trigger tools default to the 3 ms adaptive profile**, not 2 ms -- this
   branch keeps 3 ms frames until the trigger is proven; the 2 ms profile
   is intentionally not brought over yet.
4. **`l3sparse`/`l3track` compiled out** (`L3_SPARSE_READBACK`) to restore
   DATA_RAM margin. Cormac's checked-in linker map was stale (last
   regenerated 2026-09-26); a fresh build of his actual `bb06293e` tip
   showed only 1,382 B of DATA_RAM free, under the 16,384 B policy floor.
   Disabling the sparse readback path (its buffers cost ~22 KB,
   `gTrackWorkspace` alone 17 KB) restored 23,814 B free. The host already
   falls back to `l3dump` when `l3sparse`/`l3track` refuse.

A task-priority reorder (detect above CLI, to fix `scratch_stale` during
diagnostic output) was ported too, then reverted the same day -- see below.

## New work: an optional Hann range window

`trackCfg window none|hann`: the detector can now score bins through a
periodic Hann window instead of the plain range-FFT output. Because the
range FFT is exactly 128 points over 128 samples, this is implemented as
the exact 3-tap kernel `X[k] - (X[k-1] + X[k+1]) / 2` on the FFT output
(gain-compensated), not a signal-processing approximation. A strong return's
sidelobes drop from about -13 dB to -31 dB, so a weak ball a few bins away
is no longer buried under the club's sidelobes; the cost is a main lobe
roughly twice as wide. Stored samples are untouched -- only where the
detector *scores* bins changes, so the saved movie is identical to the
plain profile's, and the change is reversible per-capture. New profile
`config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16_hann.cfg` (the plain 3 ms
adaptive profile plus one `trackCfg window hann` line). The replay harness
(`firmware_replay.py`, `ReplayConfig.range_window`) mirrors it exactly, with
a test that runs the board's memory layout through the compiled C to prove
parity.

Two bugs were found and fixed in this feature before it was usable, both
from real hardware testing, not host tests (host tests cannot catch either
-- they only prove the arithmetic is correct, not that it fits in 3 ms):

1. **Redundant computation** (`777703df`). The two-pass mean/residual
   algorithm computed the windowed value twice per sample (once in each
   pass) instead of caching it. Harmless for the plain path; for Hann it
   doubled the extra-neighbour-read cost, which missed the 3 ms detect-task
   deadline (`scratch_stale=13` of 106 frames on the armed soak, plus a
   false self-trigger latch on a still scene). Fixed by caching each loop's
   windowed value once. Confirmed on the rig afterward: armed soak clean,
   `scratch_stale=0`, 1046 frames.
2. See "The actual trigger/readback bug" below -- unrelated to Hann, but
   found while testing it.

## The actual trigger/readback bug (today's real finding)

**The self-trigger works.** `swing_trigger.py`'s live trace on real swings
shows the detector correctly tracking a club's approach and firing inside
the impact gate (`state=fired why=fired`, held for the required frames,
gated within the configured bins). This happened reliably, repeatedly, on
real hardware. This was never in question, despite how it looked from the
kiosk.

**The readback was broken.** Every one of 13 real-swing captures came back
as exactly 18 bytes: the literal text `Error -1\nl3dump:/>`, not a
truncated transfer. From the kiosk's side this is indistinguishable from
"never triggered" -- no shot ever completes -- which is why it looked like
a trigger problem when the trigger was never the issue.

Two hypotheses were tried, in order:

1. **Wrong: CLI task starvation** (`6d01b27e`, since reverted). Earlier the
   same day, `L3_DETECT_TASK_PRIORITY` had been raised above
   `L3_CLI_TASK_PRIORITY` (from `feat/iwr-2ms-pipeline`, meant to stop
   `scratch_stale` during diagnostic CLI output). `l3dump`'s readback also
   runs on the CLI task with polled UART writes, so a detect task that
   outranks it could in principle starve that polling loop. This was a
   reasonable first hypothesis given the "8s stall, 18 bytes" symptom, but
   it was wrong: a starvation race would be flaky, not the same 18-byte
   `Error -1` on 13/13 swings every time. The revert was harmless (the
   `scratch_stale` fix above already covers what the reorder was for) but
   did not fix the real bug.
2. **Right: `l3_cli_dump` never checked for an already-latched freeze**
   (`f16ae54d`). `l3_cli_dump` unconditionally called
   `l3_stopCaptureAtBoundary()`, which requests a *new* HWA freeze and
   waits up to 250 ticks for it. Once a self-trigger has already frozen the
   ring, re-arming has already stopped, so nothing will ever produce that
   new completion -- the wait always times out and returns -1. Two other
   commands, `l3_cli_release` and `sensorStop`, already handle this
   correctly via `l3_awaitFrozenRing()`, which waits on the *existing*
   freeze instead of requesting a new one. `l3_cli_dump` was simply never
   updated to match. This is a pre-existing bug in Cormac's code, not
   something introduced this week -- self-trigger and `l3dump` had
   apparently never been exercised together on real hardware before this
   session (every prior soak and armed test measured timing only, never an
   actual triggered capture).

The fix: `l3_cli_dump` now calls `l3_awaitFrozenRing()`, the same helper the
other two commands use. A regression test
(`test_dump_waits_on_an_already_latched_freeze_instead_of_requesting_a_new_one`
in `tests/test_iwr6843_firmware_sparse.py`) pins it by source inspection,
since `l3_dump.c` cannot be compiled on the host (needs the mmWave SDK).

## Current image

`firmware/releases/l3_dump_hann_dumpfix_3ms_20260929.bin`
SHA-256 `53fd9bfeaf834411314734e658ba44954a710d32103dec0ff68ba5b4f64ea7f7`

Host tests pass (1435 passed across the IWR/swing/soak suites; 30 errors in
`test_iwr6843_compact_iq16.py`/`test_iwr6843_live_selector.py` are a
pre-existing build-machine compiler-setup issue, present on Cormac's clean
branch too, unrelated to any of today's work). Links with the TI toolchain.

**Not yet run on the rig.**

## Next steps

1. Flash `l3_dump_hann_dumpfix_3ms_20260929.bin`. Confirm the SHA-256.
2. Re-run `swing_trigger.py` on a few real swings, same as before:
   ```bash
   uv run python scripts/iwr6843/swing_trigger.py --tee-m <measured> \
     --config config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg \
     --capture-dir openflight_sessions/iwr-trigger/diag3
   ```
   Success looks like a full-size `.l3dump` (hundreds of KB), not 18 bytes.
   If it still fails, it is not (or not only) this bug, and the readback
   path needs a fresh look -- but the "Error -1" mechanism is now
   understood and fixed, so a different failure mode would be a genuinely
   new finding, not a variant of this one.
3. Once readback works, resume the plan from earlier today: kiosk sessions
   alternating plain and `_hann` configs in blocks, labeling every attempt
   (hit/miss/practice/walk-up), to judge whether Hann actually improves
   ball visibility in the saved post-impact data -- the armed soak only
   proved the timing budget, not detection quality. This is also the input
   data for `docs/superpowers/plans/2026-09-29-iwr6843-split-trigger.md`'s
   Phase 0 (measuring split-confirmation latency from real captures).
4. Consider whether `l3sparse`/`l3track` (compiled out for DATA_RAM margin)
   should come back for the split-trigger work, which will need the freed
   space differently (a live split detector, not sparse readback).

## Standing rules (carried over from this project's earlier handoffs)

- **Tune only from evidence.** Do not change a trigger/detector threshold
  without a real capture or trace showing the specific failure it fixes.
- **Never claim hardware validation without hardware evidence.** A clean
  host-test run or a clean build is not a validated image -- say plainly
  when something is untested on the rig, as every release note above does.
- **After any firmware change**: run the host test suite, build a
  distinctly named image (explicit `RELEASE_NAME`), record its SHA-256, and
  say exactly which tests ran before asking for a hardware run.
- **A deterministic, 100%-reproducible failure is not a race.** The first
  hypothesis here (CLI starvation) was plausible but wrong because it
  predicted flaky behavior against a symptom that was perfectly consistent
  across every swing; check that a hypothesis's failure signature actually
  matches the observed pattern before rebuilding around it.
