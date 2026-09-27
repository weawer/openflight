# Adaptive 2 ms IQ16 one-bin-jitter trigger guard

- Binary: `l3_dump_2ms_iq16_timingfix4_trigger_20260927.bin`
- SHA-256: `ff7b1f3ef070279f070aedde0d3723c9859844c232cdab73948fbd73e651a363`
- Hardware status: built, not flashed or tested on the board.
- Validation: 639 IWR tests passed, 4 skipped; Ruff passed; TI build, link,
  image packaging, and CRC generation passed.

The `timingfix3` static run again latched. Its detector log showed bin 39,
one missed frame, bin 38, then bin 39. The actual retreat correctly reset the
approach origin, but the one-bin return met the configured one-bin-per-frame
rate exactly and fired. Multi-observation triggers now require at least two
bins of net approach in addition to the configured rate check. Explicit
`trackFrames=1` mode retains its existing immediate behavior.

The checker also waits for a full pre-trigger buffer between capture cycles;
deploy its updated Python script as well as this image. The wait checks for
unexpected triggers and capture/timing errors while frames accumulate.

After flashing and resetting this image, stop the kiosk and run the stationary
test with the matching source deployed:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --self-trigger-tee-m 1.845 \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-timingfix4-smoke \
  --output openflight_sessions/iwr-timingfix4-smoke/run.jsonl
```

Use the measured sensor-to-tee distance in place of `1.845` when different.
Require no unexpected trigger, no new HWA/rearm errors, full pre-trigger
history in each dump, and combined frame work below the 2,000 us period. The
1,500 us figure is informational. Do not proceed to real-swing testing until
the stationary run passes; then validate that the two-bin gate still triggers
repeatably on real swings.