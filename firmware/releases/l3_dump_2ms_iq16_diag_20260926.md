# IWR 2 ms IQ16 diagnostic — 2026-09-26

Built from the phase-one timing and diagnostic changes based on
`98a44f3dd86a2b5596021cc6fe0816c3e7ba93ab`.

- Image: `l3_dump_2ms_iq16_diag_20260926.bin` (359364 bytes)
- SHA-256: `134f95519510bf796da70ca1a62fea909596f76cdcd3479a8f1b96b42ecf3046`
- `l3_dump.c` SHA-256: `826598fbc018eae26da23636421ac90e09d23a9163d5eb8ddc91ac614897905c`
- Diagnostic config SHA-256: `605f6e22b34d42ce296b66a73b324bba718a9aac01c9bb3fcbf67adce74fc081`
- SDK: TI mmWave 3.6.2 LTS; ARM compiler: 20.2.7 LTS
- SYS/BIOS: 6.73.1.1; XDCtools: 3.61.0.16
- Packaging: TI `out2rprc`, multicore image generator and CRC tools through Mono

The image uses the existing configurable three-transmitter build with IQ16 and
IQ8 support. The matching diagnostic config records 24 IQ16 frames at 2 ms per
frame, giving a 48 ms capture that fits in the existing L3 arena. Firmware stats
report the latest and maximum end-to-end rearm latency from completed-frame queue
time through the next HWA arm. The HWA rearm task preempts CLI diagnostics, and
the latency fields use a separate CLI line to stay within the output buffer.

After flashing, run:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --soak-frames 100000 \
  --cycles 100 \
  --output iwr-2ms-iq16-validation.jsonl
```

Compilation, linking, image generation and CRC generation succeeded. On the
IWR6843, 103,003 continuously acquired frames completed with zero missed frames,
RF faults or rearm errors. Maximum measured rearm latency was 39 microseconds.
One hundred consecutive freeze, timed-IQ16 transfer and rearm cycles completed
without a timeout, malformed capture or lockup. The validation log is
`~/openflight_sessions/iwr-2ms-iq16-validation.jsonl` on the test Raspberry Pi.
