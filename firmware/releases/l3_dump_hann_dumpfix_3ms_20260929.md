# l3_dump_hann_dumpfix_3ms_20260929.bin

SHA-256: `53fd9bfeaf834411314734e658ba44954a710d32103dec0ff68ba5b4f64ea7f7`

Supersedes `l3_dump_hann_cliprio_3ms_20260929.bin`. Same base and Hann fix,
plus the actual cause of the post-trigger readback failure that image
(wrongly) tried to fix by reverting task priorities.

## The bug (found on the rig, 2026-09-29)

13/13 real swings in `swing_trigger.py` fired correctly (`state=fired
why=fired`, confirmed in the trigger's own trace) but every readback came
back as exactly 18 bytes: the literal text `Error -1\nl3dump:/>`, not a
truncated transfer. Deterministic across every swing, which ruled out the
scheduling-race theory in `l3_dump_hann_cliprio_3ms_20260929.md`.

Root cause: `l3_cli_dump` (the `l3dump` command) never checked whether a
self-trigger had already frozen the ring. It called
`l3_stopCaptureAtBoundary()` unconditionally, which requests a *new* HWA
freeze and waits up to 250 ticks for it. Once self-triggered, nothing
produces that new completion (re-arming already stopped), so it always
times out and returns -1. `l3_cli_release` and `sensorStop` already handle
this correctly via `l3_awaitFrozenRing()`, which waits on the *existing*
freeze instead of requesting a new one; `l3_cli_dump` was never updated to
match. Pre-existing in Cormac's code -- self-trigger + `l3dump` had
apparently never been exercised together on real hardware before this
session.

## The fix

`l3_cli_dump` now calls `l3_awaitFrozenRing()`, the same helper
`l3_cli_release` and `sensorStop` use.
`tests/test_iwr6843_firmware_sparse.py::test_dump_waits_on_an_already_latched_freeze_instead_of_requesting_a_new_one`
pins it.

## Status

Host tests pass (1435 passed across the IWR/swing/soak suites; the 30
errors in `test_iwr6843_compact_iq16.py`/`test_iwr6843_live_selector.py` are
the pre-existing build-machine compiler issue, present on Cormac's clean
branch too). The image links with the TI toolchain.

**Not yet run on the rig.** This is the fix that should actually resolve
"it does not seem to trigger at all" -- the trigger was always firing
correctly; only the readback was broken. Flash and repeat the
`swing_trigger.py` diagnostic; a full-size `.l3dump` (hundreds of KB, not
18 bytes) confirms it.
