# Adaptive 2 ms IQ16 impact-selector timing reduction

- Binary: `l3_dump_2ms_iq16_timingfix2_20260927.bin`
- SHA-256: `30ef1499a975faf837427e61ef1879d7c64cfd22cb1ee86ea8221050d99c2d58`
- Hardware status: built, not flashed or tested on the board.
- Validation: focused selector and adaptive-retention tests passed; TI build,
  link, image packaging, and CRC generation passed.

This image follows the first timing-fix smoke run. That run passed the
1,000-frame static soak with a maximum of 1,426 us, but a capture-phase frame
reached 1,832 us. Same-frame attribution showed 1,473 us in shadow selection,
314 us in compaction, 39 us in rearm, and no trigger work. There were no
missed HWA frames, rearm errors, or unexpected self-trigger.

The impact-phase shadow scan now visits only the configured impact range-bin
window instead of scanning all 128 bins and masking the rest afterward. The
first full-range flight frame re-primes the previous-frame baseline, so its
differential score is intentionally empty; following full-range frames resume
selection. Pre-trigger baseline seeding and all stored IQ16 samples are
unchanged.

After deploying the matching source and flashing this image, stop the kiosk
and rerun the short stationary test:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --self-trigger-tee-m 1.845 \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-timingfix2-smoke \
  --output openflight_sessions/iwr-timingfix2-smoke/run.jsonl
```

Use the measured sensor-to-tee distance in place of `1.845` when different.
Require no unexpected latch, no new HWA/rearm errors, and combined frame work
at or below 1,500 us. Then inspect the `workmax_*` fields to confirm the
impact selector cost fell. This static run does not validate real-swing
detection or selector quality during ball flight.