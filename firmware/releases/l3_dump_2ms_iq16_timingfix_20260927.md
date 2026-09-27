# Adaptive 2 ms IQ16 timing diagnostics and pre-trigger work reduction

- Binary: `l3_dump_2ms_iq16_timingfix_20260927.bin`
- SHA-256: `872fb68a7ee1964542e54a3c641f554cb95a3c5dc3be0a7cc0c5a7c541c3996d`
- Hardware status: built, not flashed or tested on the board.
- Validation: 635 IWR tests passed (4 skipped); Ruff passed; TI build, link, image
  packaging, and CRC generation passed.

This image includes the existing stationary/receding approach-rate fix and the
following timing changes:

- `frame_work_max_us` remains the combined rearm-task work maximum.
- The `workmax_*` stats line records rearm, selector/baseline, compaction,
  trigger, and total costs from the same frame that set that maximum.
- Adaptive rolling pre-trigger frames seed only the configured impact-window
  history. They do not run the full shadow selector before the trigger, where
  its results were discarded at capture start.
- Raw trigger trace and per-bin max-hold collection are disabled by default.
  Enable them temporarily with `triggerLog trace on`, disable with
  `triggerLog trace off`, and read the collected evidence with
  `triggerLog trace`.
- The hardware checker accepts timing only when combined frame work is at or
  below its 1,500 us engineering target and below the configured frame period.
  The independent selector/compaction maxima are diagnostic only; they may
  have occurred on different frames.
- If an armed static test unexpectedly triggers and `--capture-dir` is set,
  the checker saves the frozen `.l3dump` along with stats and `triggerLog` in
  its JSONL failure event.

After deploying the matching source and flashing this image, stop the kiosk
and run a short static test with the unit stationary:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --self-trigger-tee-m 1.845 \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-timingfix-smoke \
  --output openflight_sessions/iwr-timingfix-smoke/run.jsonl
```

Use the measured sensor-to-tee distance in place of `1.845` when different.
The run must show no unexpected latch, no new HWA/rearm errors, and total
frame work no greater than 1,500 us. This static test does not validate
real-swing detection.