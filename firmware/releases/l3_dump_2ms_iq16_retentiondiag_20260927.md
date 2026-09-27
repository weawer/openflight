# Adaptive 2 ms IQ16: selector-stop diagnostics

- Binary: `l3_dump_2ms_iq16_retentiondiag_20260927.bin`
- SHA-256: `6ca284433457baac4b3d4be5be547da1b06cec159b6da425d804e58323e3cfb7`
- Supersedes `l3_dump_2ms_iq16_timingfix4_trigger_20260927.bin`, the last
  image confirmed on hardware (`plans/iwr-trigger-timing-handoff.md`). This
  build adds `db29133` (preserve approach timing across stationary returns),
  `ae3b868` (limit impact-phase selection to configured bins; reset baseline
  for full-range flight tracking) and `e2abb79` (this diagnostic) on top of
  it. None of those three have been hardware-tested before this build.
- Hardware status: built, not flashed or tested on the board.
- Validation (this machine, Linux, TI toolchain): 2077 tests passed, 7
  skipped; TI build, link, image packaging and CRC generation passed with
  warnings as errors. DATA_RAM: 112,512 of 196,608 bytes used (83,584 free).
  The prior report of "native tests pass with ziglang cc" (`e2abb79`) was
  from a different machine/compiler; this is the first TI-toolchain build of
  that commit.

## Why

Every real-shot capture in the 2026-09-27 range sessions stopped adaptive
retention early (`ambiguous` or `track_lost`), including operator-labeled
hits, and the retained dump alone cannot show which candidate bins or powers
caused the stop -- the rejected frame's data is never retained. This was
next on the handoff's list before any further threshold tuning.

## What changed

Firmware writes one `RST` line to the CLI text prefix, immediately before an
adaptive16 `l3dump`'s binary payload, whenever the live selector (not a
timeout or short-history case) ended retention:

```
RST r=<reason> f=<frame> cc=<candidateCount> c=<bin0>,<bin1> p=<power0>,<power1>
    s=<selectedBin> h=<heldBin> ok=<accepted> a=<ambiguous> co=<coasting>
    n=<noise> w=<windowStart>,<windowBins>
```

`f` is the dump-order index of the first frame retention did not keep (the
frame that was rejected, not a retained one). `reason` matches
`L3_RETENTION_*` in `live_selector.h`. The binary dump format itself is
unchanged; this line only exists when `gRetentionStopRecorded` was set by
`l3_storeCompletedScratchFrame`, and is cleared on every rearm/sensorStart.

Host: `IWR6843Radar.read_adaptive_dump()` parses the line via
`parse_retention_stop()` into `IWR6843Capture.retention_stop`, logged as
`iwr6843_capture.retention_stop` in the session JSONL alongside the existing
capture event.

## Verification

- Native selector tests (`tests/test_iwr6843_live_selector.py`) cover the
  `RST` line round-trip for `track_lost` (both a lost track and one still
  coasting) and `ambiguous`, plus a 128-byte output buffer bound.
- Host parser, monitor and session-logger tests cover
  `parse_retention_stop()`, `IWR6843Capture.retention_stop`, and the JSONL
  field.
- Full suite (this machine): 2077 passed, 7 skipped.
- TI native build: passed, warnings as errors.

## First operator test

Flash, set functional mode, reset, then repeat the labeled real-shot session
per the handoff's step 3 (`plans/iwr-trigger-timing-handoff.md`) with trigger
and selector parameters unchanged from the last session
(`triggerCfg 41 6.0 2 12 3 0.0 1.0 1 1.5`). This build's only purpose is to
make the *next* set of early-stop captures explain themselves; it does not
change detection or selection behavior. Check the session JSONL for
`iwr6843_capture.retention_stop` on each capture and use it, plus the
existing detector log/trace, before changing any threshold.

Do not flash straight to a real-shot session without first repeating the
stationary soak (`check_iwr_2ms_iq16.py --self-trigger-tee-m 1.845 --shadow
--allow-early-stop --expect-no-flight-frames`) once, since `db29133` and
`ae3b868` in this build have not been hardware-validated on their own.
