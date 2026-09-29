# l3_dump_hann_dumpfix_detectprio_3ms_20260929.bin

SHA-256: `8e829c0c025798fda1b48ebe7e86429ae9d0c762886d2240d328334351dac066`

Supersedes `l3_dump_hann_dumpfix_3ms_20260929.bin`. Same source plus one
change: the detect task now runs above the CLI task
(CLI 3 < detect 4 < HWA rearm 5 < control 6).

## Why

`swing_trigger.py` on the dumpfix image failed its own health check within
about two seconds of arming, before any swing: `scratch_stale=3`, the same
count on both quiet runs, and 30 on the earlier run with verbose traces
(`openflight_sessions/iwr-trigger/readback-check*/run.jsonl`).

The count matches the health poll itself. Each `stats` reply is ~825 B;
the CLI writes it by polling the UART at 1,041,667 baud, which takes
~7.9 ms, about 2.6 frames at 3 ms. The detect task was priority 1, below
the CLI (3), so it could not run during that write, and ~3 scratch slots
were reused before it read them. Verbose traces were ~10x the output and
cost ~10x the frames. The check was producing the failure it reported, and
any `stats`/`ball status` poll from the kiosk during a swing would have
dropped frames the same way.

This ordering was tried once before (`l3_dump_hann_cliprio_3ms_20260929.bin`
was the revert) because real self-triggered readbacks came back as 18
bytes and CLI starvation was suspected. That failure was the missing
`l3_awaitFrozenRing()` in `l3_cli_dump`, fixed in the dumpfix image. The
cliprio image raised detect priority *without* that fix, so its failure
says nothing about priority. During readback the ring is frozen and no
frames arrive, so the detect task has nothing to preempt the dump with.

## What was run

- TI toolchain build (MSS + DSS + RADARSS + CRC). DATA_RAM 23,814 B free
  (unchanged; priorities don't move memory).
- Host: `test_iwr6843_firmware_rearm.py`, `test_iwr6843_firmware_sparse.py`,
  `test_swing_trigger.py`, `test_iwr6843_cadence_soak.py`,
  `test_iwr6843_firmware_trigger.py`, `test_iwr6843_memory_layout.py`:
  201 passed. These pin the source; they cannot show scheduling behavior.

## Status

**Not yet run on the rig.** This is the first image with both the dump fix
and detection above the CLI. Two things to confirm, in order:

1. `swing_trigger.py` armed and still for ~30 s with health polling on:
   `scratch_stale` stays 0.
2. A few real swings: each readback is a full-size `.l3dump` (hundreds of
   KB), not 18 bytes. This is what shows raising detect priority does not
   starve the readback.

If (1) fails, the detect task genuinely does not fit the 3 ms frame and
the next step is `triggerLog perf`, not further priority changes. If (2)
fails with detection above the CLI, that is new evidence for the
starvation theory and the ordering needs revisiting.
