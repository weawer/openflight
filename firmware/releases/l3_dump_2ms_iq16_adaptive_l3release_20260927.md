# Adaptive 2 ms IQ16 + self-trigger: `l3release`

- Binary: `l3_dump_2ms_iq16_adaptive_l3release_20260927.bin`
- SHA-256: `dbd9d5cb8d8973083783d1d833235ee59274cc2627b4264e719cc9afde7b163f`
- Profile: `config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg` (unchanged)
- Toolchain: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, native build.
- Supersedes `l3_dump_2ms_iq16_adaptive_selftrigger_20260927.bin`.
- Hardware status: not flashed or tested by the agent.

**The host now requires this firmware.** It releases unwanted self-trigger
freezes with `l3release` and reports "flash the current firmware" if the
command is missing.

## What changed

These trigger-handling changes are ported from `Cormac131/feat/iwr-calcs`.
The rest of that branch's trigger work (new detector, ball detector, global
bins, Doppler gate, firmware notice task) is not included; see
`plans/iwr-branch-merge.md`.

### Firmware

- New `l3release` CLI command (table entry 17). It stops RF and rearms the
  ring without streaming anything, for every capture format including
  adaptive16. Previously the only options were `l3sparse`, which adaptive16
  rejects, or reading and discarding a whole dump (about 5.3 s).
- `l3_freezeCapture` is split into two steps:
  - `l3_stopFrozenRing` takes the frozen ring with RF stopped;
  - the readback-only check for an incomplete capture.
- `l3release` discards the capture, so it clears `gCaptureIncomplete` and
  rearms even after an overrun. Before this, an overrun kept every later
  freeze failing until `sensorStart`.

### Host

- `IWR6843Radar.cmd()` reads a reply through the CLI prompt instead of
  stopping at the first "Done"/"Error". This keeps a reply that arrives byte by
  byte from bleeding into the next command's reply, and keeps a trailing debug
  line or "Triggered" notice out of it. The notice is still remembered for the
  listener.
- Readbacks (`l3dump`, `l3shadow`, `l3track`, `l3sparse`) drop a pending
  "Triggered" notice. That notice belongs to the capture being read, and
  keeping it fired a phantom capture of the freshly rearmed ring.
- `release_sparse_freeze()` sends `l3release`. The monitor no longer reads and
  discards a dump to release an adaptive16 freeze.

## Verification

- The native C harness compiles the real `l3_stopFrozenRing`,
  `l3_freezeCapture` and `l3_cli_release`. It checks that a release:
  - stops RF, rearms, and discards an incomplete capture;
  - does not rearm when the RF stop fails or the freeze times out.

  A readback still refuses an incomplete capture.
- Ported host tests cover prompt-terminated replies (from `294fdbf`), the
  `l3release` round trip and its failures, and notice discard for all four
  readbacks.
- Full suite: 1986 passed, 8 skipped. TI build passes with warnings as
  errors. DATA_RAM is unchanged (98,393 of 196,608 bytes). The CLI table uses
  18 of 32 entries plus `help`; the mmWave extension commands are held in a
  separate table.

## Operator test

Flash this image, set functional mode, press RESET, then start kiosk mode
with `--iwr6843-self-trigger`. At startup, "Releasing an unaccepted
self-trigger capture" should be followed by the server finishing startup, with
no dump transfer and no "RF restart failed".
