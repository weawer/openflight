# Adaptive 2 ms IQ16 with the club-track self-trigger

- Binary: `l3_dump_2ms_iq16_adaptive_trigger_20260927.bin`
- SHA-256: `8fc6dd32703d1a5d48cdf86284ec10233743773ef71355eaee9879348b24020d`
- Build: TI mmWave SDK 3.6.2 LTS, ARM compiler 20.2.7 LTS, warnings as errors.
- DATA_RAM: 113,000 / 196,608 bytes (includes the configured task heap).
- Hardware status: not flashed or tested by the agent.
- Validation: 2,050 tests passed, 7 skipped; targeted Ruff checks passed;
  Pylint on the changed backend modules scored 9.76/10. Native C tests
  exercise the detector and capture/rearm integration; these are not hardware
  validation.

This release integrates the `feat/iwr-calcs` club-track detector into the
existing adaptive16 capture firmware. It replaces the previous ball-leave
detector and requires the matching host changes using global tee bins and
SNR/track-frame settings. It retains IQ16 and the 2 ms profile. IQ8 is rejected
when enabling this detector.

The review fixed detector rearming after capture/release, validated compacted
frame addressing against a NumPy reference, moved the trigger notice out of
the rearm task, and made log readback use a stable snapshot. Stats include
`trigger_max_us` and `frame_work_max_us`; the latter measures the combined
capture work rather than adding maxima from unrelated frames.

## Operator checks

Deploy the matching source, flash this named binary, return to functional
mode, and reset. Stop the kiosk before running either script: each owns the
IWR serial port. Use your measured sensor-to-tee distance instead of 1.845
if different; both tools load the existing range calibration (`--cal` can
select another calibration file).

First run a short static test indoors, keeping people and objects still:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --self-trigger-tee-m 1.845 \
  --shadow --allow-early-stop --expect-no-flight-frames \
  --soak-frames 1000 --poll-s 2 --cycles 3 \
  --capture-dir openflight_sessions/iwr-trigger-port-smoke \
  --output openflight_sessions/iwr-trigger-port-smoke/run.jsonl
```

This checks timing with the detector armed, unexpected triggers, and forced
capture/rearm cycles. Combined frame work must stay below 2,000 us with no
new capture errors. If it passes, repeat with 100,000 frames and 100 cycles.
It does not prove that a golf swing triggers correctly.

Then inspect detection and repeated releases:

```bash
uv run python scripts/iwr6843/watch_trigger.py \
  --config config/iwr6843_l3dump_adaptive_36f2ms_iq16.cfg \
  --tee-m 1.845
```

This prints `triggerLog` and releases each triggered capture automatically.
Arm waving can exercise observations but may not satisfy the club-track
gates. Real-shot acceptance requires repeatable triggering, retained impact
and ball-flight evidence, and no missed frame deadlines. This utility
discards captures; use normal session logging for real-shot data analysis.

The full donor detection-task architecture, ball placement, and DSS solver
are outside this release. See `plans/iwr-branch-merge.md` for the remaining
merge scope. Do not substitute the earlier `l3_dump_test_build.bin`.
