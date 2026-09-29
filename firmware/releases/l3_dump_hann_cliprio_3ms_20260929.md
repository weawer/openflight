# l3_dump_hann_cliprio_3ms_20260929.bin

SHA-256: `acb029fdcac5242d7d87600c4a245bf6d9493f22c4f57452cc24481fe53f9120`

Supersedes `l3_dump_hann_fast_3ms_20260929.bin`. Same base and features, plus
a task-priority revert for a real hardware failure that image had.

## The bug (found on the rig, 2026-09-29)

`scripts/iwr6843/swing_trigger.py` against `l3_dump_hann_fast_3ms_20260929.bin`
(plain config, no Hann involved) correctly detected and gated two real
swings — `state=fired why=fired`, confirmed in the trigger's own trace — but
both readbacks failed:

```
[IWR6843] Dump stream stalled >= 8.0s, giving up (18/None bytes received)
```

18 bytes of a roughly 450,000-byte capture. From the kiosk's side this is
indistinguishable from "never triggered": the shot never completes.

Root cause: an earlier commit today (`09c71a11`, ported from
`feat/iwr-2ms-pipeline`) raised `L3_DETECT_TASK_PRIORITY` above
`L3_CLI_TASK_PRIORITY` to stop `scratch_stale` during `triggerLog`/`debugCfg`
diagnostic output. `l3dump`'s readback also runs on the CLI task, with polled
(not interrupt-driven) UART writes (`firmware/iwr6843/l3_dump.c`, `l3dump`
command handler). With detect outranking CLI, a detect task still being
invoked every frame (ball tracking during the post-impact movie, and
self-trigger scoring once rearmed) can starve that polling loop instead of
yielding between frames long enough for the CLI to send.

This reorder was never tested against a real trigger+readback sequence
before today — every prior soak and armed test exercised timing only, never
an actual capture. It happened to make yesterday's `scratch_stale` failure
worse, but that failure's actual cause (fixed in
`l3_dump_hann_fast_3ms_20260929.bin`) was the Hann kernel's redundant
computation, not this priority order; the armed soak already passes clean
without it.

## The fix

Reverted the priority macros to the original ordering (CLI above detect,
matching Cormac's `feat/iwr-calcs`): `L3_DETECT_TASK_PRIORITY 1`,
`L3_HWA_REARM_TASK_PRIORITY (L3_CLI_TASK_PRIORITY + 1U)`,
`L3_CTRL_TASK_PRIORITY 5`. Kept the rest of `09c71a11`
(`sensorStart` profile-counter reset, the soak's `scratch_stale` gate).
`tests/test_iwr6843_firmware_rearm.py::test_detection_stays_below_the_cli`
pins the reverted ordering and records why.

## Status

Host tests pass (1434 passed across the IWR/swing/soak suites; the 30 errors
in `test_iwr6843_compact_iq16.py`/`test_iwr6843_live_selector.py` are a
pre-existing compiler-setup issue on the build machine, present on Cormac's
clean branch too). The image links with the TI toolchain.

**Not yet run on the rig.** Needs, in order:
1. Armed soak (plain and `_hann` configs) — confirm `scratch_stale=0` still
   holds with detect back below CLI.
2. `swing_trigger.py` on a few real swings — confirm the dump readback
   actually completes this time (a full-size `.l3dump`, not 18 bytes).
3. If (2) still stalls, the priority order was not the (or not the only)
   cause and the CLI-starvation hypothesis needs to be dropped in favour of
   something else in the freeze/readback path.
