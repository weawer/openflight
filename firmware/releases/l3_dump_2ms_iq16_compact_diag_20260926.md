# IWR6843 2 ms IQ16 compaction diagnostic

- Binary: `l3_dump_2ms_iq16_compact_diag_20260926.bin`
- SHA-256: `bb78c41f9c238b3d6ae10bc05f33d1b088c2a1c86a790c80dc02c0593930ab74`
- Config: `config/iwr6843_l3dump_compact_16f2ms_32bin_iq16.cfg`

This opt-in phase-2 image keeps the proven 2 ms, 12-loop, three-TX profile. The
HWA writes each complete 128-bin IQ16 frame into alternating scratch buffers.
The rearm task switches buffers and restarts acquisition before copying the
fixed 32-bin window into retained L3 storage. It records rearm and compaction
latency, scratch generations, overruns, and incomplete captures.

The diagnostic capture contains 16 frames (32 ms). It is intentionally short
so this test isolates lossless compaction timing before increasing retention.

After flashing and resetting the IWR, run from the repository root:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_compact_16f2ms_32bin_iq16.cfg \
  --soak-frames 100000 \
  --cycles 100 \
  --output ~/openflight_sessions/iwr-2ms-iq16-compact-validation.jsonl
```

The test fails if RF, HWA, scratch-overrun, compaction, or incomplete-capture
counters increase. It also verifies the timed IQ16 dump geometry on every
capture cycle.
