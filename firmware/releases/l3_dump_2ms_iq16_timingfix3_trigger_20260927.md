# Adaptive 2 ms IQ16 stationary-trigger correction

- Binary: `l3_dump_2ms_iq16_timingfix3_trigger_20260927.bin`
- SHA-256: `8f539e7b802a31a0645cd6b06aebf18ac2345e50f307b0bba1d99a6ca735677e`
- Hardware status: built, not flashed or tested on the board.
- Validation: 638 IWR tests passed, 4 skipped; Ruff passed; TI build, link,
  image packaging, and CRC generation passed.

The hardware checker waits for the configured pre-trigger history to refill
after every capture/rearm before starting the next cycle. This prevents a
back-to-back diagnostic dump from reporting a partial prebuffer merely because
the prior UART transfer and rearm finished shortly beforehand.

The `timingfix2` static test exposed a repeatable false trigger. The saved
trigger log followed bin 38 through repeated frames and fired after a one-bin
change to bin 39, near tee bin 41. Coherence and Doppler filtering were
disabled. The detector reset its approach origin on every equal-bin return,
so the final one-bin shift was compared with only one frame of elapsed time.

The detector now resets its approach origin only on an actual retreat to a
smaller bin. Equal-bin returns preserve the elapsed approach history, so a
one-bin drift after repeated stationary observations does not pass the
minimum approach-rate check. Genuine retreats still restart the approach
measurement. A native regression reproduces the observed repeated-bin then
one-bin sequence.

After flashing and resetting this image, run the short stationary smoke test
with the matching source deployed and kiosk stopped:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --self-trigger-tee-m 1.845 \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-timingfix3-smoke \
  --output openflight_sessions/iwr-timingfix3-smoke/run.jsonl
```

Use the measured sensor-to-tee distance in place of `1.845` when different.
Require no unexpected trigger, no new HWA/rearm errors, and combined frame
work below the 2,000 us frame period. The 1,500 us figure is informational,
not a pass/fail target for this test. Do not proceed to real-swing validation
until the stationary run passes.