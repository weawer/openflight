# Experimental Phase 4: adaptive 2 ms IQ16 retention

- Binary: `l3_dump_2ms_iq16_adaptive_20260926.bin`
- SHA-256: `d909b1ac4065af20e2674476f9c29df261ea18deb83439c80d5dc82b4ec5dae2`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg`
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS.
- Built with the configurable three-TX options in `firmware/Makefile`.
- Hardware status: not flashed or tested by the agent. The operator runs hardware tests.

## Capture behavior

`captureFormat adaptive16` enables 14 pre-trigger frames of 32 bins, six broad
post-trigger frames of 53 bins, and up to 16 flight frames of 12 bins. Every
retained bin keeps all 12 loops, three TX and four RX, with complex IQ16 values
and a 2 ms frame interval. The broad windows stay fixed; only flight windows
follow the selector. The impact interval is relative to the trigger and has not
been qualified against real ball impact timing.

At the start of the broad post-trigger interval, association restarts inside
that interval's configured range window. Flight retention requires an existing
track, an accepted candidate, and two bins of margin on each side. Two ambiguous
candidates must both fit with those margins. Otherwise retention ends before
that uncertain frame and records `track_lost`, `ambiguous`, or `range_edge`.
No later frame is retained in that capture. RF acquisition still finishes the
configured post-trigger interval so the existing freeze/rearm sequence is used.
There is no explicit net-distance stop in this build.

A complete record has 36 frames (nominal 72 ms, 70 ms between first and last
frame starts). The payload is 551,808 bytes; with temperature and metadata the
file is 552,004 bytes. The full UART transfer alone is approximately 5.30 s at
1,041,667 baud, 8N1. This change targets retention capacity, not faster transfer.

The arena reserves 196,608 bytes for two maximum-size, 16-loop scratch frames,
leaving 589,824 bytes for retained samples. This profile leaves 38,016 bytes
unused within that partition. Descriptor and selector globals occupy DATA_RAM.
The linked image uses 78,865 of 196,608 DATA_RAM bytes; runtime stack peaks and
adaptive timing still require hardware measurements.

## Host compatibility and diagnostics

Deploy the accompanying Python changes with this firmware. Adaptive captures
use version 8 without temperature or version 9 with temperature. An eight-byte
little-endian report follows the fixed header and optional temperature report:
`uint16 reason, pre_frames, planned_frames, reserved`. Reason codes 0–4 mean
complete, track lost, ambiguity, range edge, and insufficient prehistory.
Timed frame descriptors and IQ16 sample order are unchanged.

The application reads the full retained dump to preserve all TX channels. It
saves early-ended captures when dump saving is enabled, reports the reason, and
withholds measurements from them. Offline measurement preparation applies the
same check. Existing IQ16, compact16, IQ8 and shadow-reference profiles remain
available. No default application profile or trigger mode is changed.

## First operator test

After deploying the host changes, flashing this image and resetting in run mode:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --shadow --allow-early-stop \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-adaptive-smoke \
  --output openflight_sessions/iwr-adaptive-smoke/run.jsonl
```

This can run at home, including with no movement. `--allow-early-stop` permits
explicit track-loss outcomes for lifecycle testing; PASS does not mean a full
36-frame flight was captured. Without that option the test requires complete
retention. It checks decoded layout, all antenna channels, timing, error
counters, and stored flight windows against frozen selector decisions. It logs
payload bytes and the full freeze/transfer/restart round-trip duration; this is
not a separate UART-only or processing-latency measurement.

After reviewing the smoke files, increase to 100,000 frames and 100 cycles.
Real-shot window coverage, uncertainty margins, club/ball association and
measurement accuracy remain unqualified. Adjust fixed pre/impact window origins
to the measured tee geometry before range tests. The Phase 3 reference profile
remains the comparison path for data discarded by the adaptive mode.

## Local verification

- `uv run pytest tests/test_iwr6843* -q -rs`: 508 passed, five skipped because
  local recorded-capture fixtures are absent.
- Targeted Ruff lint: passed. Pylint on changed backend modules: 9.88/10.
- Format check: five checked files passed; `driver.py` has two pre-existing
  line-wrapping differences in the dump command calls.
- TI native build and meta-image packaging: passed; image is 363,076 bytes with
  the TI `MSTR` signature. No hardware validation of this image was performed.
- The split-header regression reproduced loss of shadow decisions before the
  driver fix and passed afterward.

The binary is available in the repository's `firmware/releases/` directory.
Copying it to the previously requested `/home/firmware/` location was blocked by
filesystem ownership; creating that directory requires an operator sudo password.
