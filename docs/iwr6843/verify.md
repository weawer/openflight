---
icon: lucide/play
---

# Start and Verify

Bring the system up with the geometry you measured, then confirm the first
capture looks right before hitting a full session.

## Start OpenFlight

For the first run, use `--debug`. This retains each TI dump for inspection and
offline replay. This example uses the Option A GPIO UART path. Replace the
example geometry with your measurements:

```bash
scripts/start-kiosk.sh --debug \
  --radar-port /dev/ttyAMA0 \
  --iwr6843 \
  --iwr6843-port /dev/ttyUSB0 \
  --iwr6843-config config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg \
  --iwr6843-tee-m 1.575 \
  --iwr6843-net-m 4.6 \
  --iwr6843-tilt-deg 10.4 \
  --iwr6843-radar-height-m 0.1524 \
  --iwr6843-ball-height-m 0.040 \
  --session-location home
```

For Option B, replace `/dev/ttyAMA0` after `--radar-port` with the OPS USB serial
device, preferably its stable `/dev/serial/by-id/...` path.

The example uses the recommended wide profile. To test the 2 ms profile with
the 54 ms ball phase, change only the config argument to:

```text
--iwr6843-config config/iwr6843_l3dump_dense_45f2ms_53bin_iq8.cfg
```

To test dense sampling while retaining the wide profile's late-flight window,
use the experimental profile:

```text
--iwr6843-config config/iwr6843_l3dump_dense_36f2ms_53bin_iq8_wide_late.cfg
```

Passing `--iwr6843-config` explicitly keeps the selected profile visible in the
launch command and session log.

The OPS port can also be supplied as `--ops-port /dev/ttyAMA0`. `--port` means
the web-server port, so do not use it for the OPS serial device.

The TI port can be omitted after the custom firmware is running; OpenFlight
probes available USB serial ports for the expected CLI. Supplying
`--iwr6843-port` is clearer during initial setup and avoids ambiguity when
multiple USB serial devices are connected.

Once the setup is stable, remove `--debug` for normal operation. The server
still processes TI captures in memory, but it does not write a dump for
every shot. Session JSONL entries only contain a dump path when debug capture
is enabled.

## Verify The First Capture

Healthy startup includes messages similar to:

```text
[IWR6843] Configured on BCM17 using /dev/ttyUSB0 (..., waiting for OPS)
[IWR6843] Armed on BCM17
[SERVER] IWR6843 initialized (... firmware boundary freeze)
```

Use one clap to verify the shared trigger and dump transfer. A clap is not a
golf ball, so `rejected_by_ball_tracker` is expected. The important result is a
complete capture:

```text
[IWR6843] Trigger #1: dumping firmware-frozen L3 ring
[IWR6843] Capture #1 complete: 732812 bytes
```

Firmware health should show an active sensor, increasing frame/wrap counters,
and no RF faults:

```text
active=1 ... rf_faults=0
```

Then hit a ball. A trusted result logs `Angle source: radar`. A shot may still
appear in the UI with an estimated angle when the TI capture completes but the
ball track does not meet the acceptance gates.

In debug mode, verify that the session contains an `iwr6843_capture` entry, a
`temperature_report` object, and a `capture_path` pointing to the saved
`.l3dump` file.

## Firmware Feature Check

After flashing a firmware image, or after any firmware change, run the CLI
test suite. Stop the kiosk first; the suite owns the TI UART.

```bash
uv run python scripts/hardware-test/test_iwr_firmware.py
```

It exercises every command the firmware registers on its CLI and prints one
`PASS`, `FAIL`, or `SKIP` line per check, grouped into sections:

| Section | What it proves | Hands-off? |
|---|---|---|
| `lifecycle` | `sensorStart`/`sensorStop`/`stats` behave; config commands are refused while active; a restart resets counters | yes |
| `profiles` | `captureCfg`, `phaseCaptureCfg`, `captureFormat`, `iq8Scale` validate their arguments; every shipped `config/iwr6843_*.cfg` loads with the declared format and stride | yes |
| `readback` | `l3dump`, `l3sparse` (limit, oversized, late request), `trackCfg`, `l3track`, and `l3release` stream and rearm | yes |
| `trigger` | a fresh session is untriggered, `triggerCfg` arms and disarms, the detector goes live only once the pre-trigger ring is full, `debugCfg` streams parsable change-only lines, the floor measurement works, and reconfiguring clears a previous arm | yes |
| `trigger-swing` | with `--swing`: a ball on the tee reaches `watching`, a swing fires `Triggered` (the notice must survive a `stats` reply), the frozen ring reads back in under 1.0 s, the host club-track replay fires too, the ring rearms, and a latched session is cleared by reconfigure | no, prompts you |
| `solve` | always `SKIP`: the on-chip DSS solve is in the image but the MSS exposes no CLI entry point for it yet | yes |

The `l3track without trackCfg` check can only prove the refusal on the first run
after a power cycle; on later runs it reports `SKIP` (the firmware never clears
`gTrackConfigured`, so `l3track` streams instead of refusing).

Run only one section, or add the prompted checks:

```bash
uv run python scripts/hardware-test/test_iwr_firmware.py --only trigger
uv run python scripts/hardware-test/test_iwr_firmware.py --ball --tee-m 1.575
uv run python scripts/hardware-test/test_iwr_firmware.py --swing --tee-m 1.575 --shots 2
```

`--ball` runs `ball-detect`: it asks for an empty tee, starts the firmware's
ball detector and scans the static (non-MTI) power of the global bins around
the expected tee bin with `ball scan`, asks for the ball, scans again, reports
where the return grew most and how the range sits in the supported setup
envelope, then checks that the firmware's own detector locked on the same bin:

```text
PASS  ball-detect/stationary return: expected=1.575m expected_bin=34 detected_bin=35
      detected_range=1.64m baseline=328441 occupied=2841977 ratio=8.65x
      (searched bins 28-40) setup=ideal: Ready
PASS  ball-detect/firmware detector locks on the placed ball: firmware bin=35
      ratio=7.65x, scan bin=35 (offset +0), range=1.64m
```

Bins are global range-FFT bins (46.9 mm; bin 34 is 1.59 m). The self-trigger
works from the MTI residual, which removes a stationary ball, so this is the
check that proves the radar sees a ball at the tee at all, and where.
`--swing` runs it first and arms the swing checks on the detected bin. Repeat
it a few times: a detected bin that wanders by one is the hand-measured tee
range; one that is always off by the same amount is the range conversion. A
`setup=too-far` line with "move OpenFlight about 65 cm closer" is the ten
captures that started this: the club at 2.2-2.4 m and a tee configured at
1.575 m.

To make the firmware aim the trigger at the detected ball rather than the
configured tee, arm the detector with follow on before `triggerCfg`:

```text
ball cfg 1 1
```

With no ball locked it falls back to the configured tee and counts the
frames in `stats` (`trig dest=... source=tee fallback=N`).

A swing that does not fire within `--swing-wait-s` (default 10 s) is not
silent: the check prints the detector's raw-input trace and frame log, then a
diagnosis that follows the same decision table you would apply by hand:

```text
  diagnosis:
    Ball:
      expected bin: 14
      observed bin: 15 (stationary ratio 8.7x)
    Detector:
      statistic:  peak
      floor:      325611
      snr:        6
      threshold:  1953666
      candidates: 0  acquired: 0  jumped: 0  lost: 0  young: 0  slow: 0  short: 0  lowcoh: 0  slowdop: 0  fired: 0
    Swing observation:
      traced frames:       4
      strongest bin:       14
      max energy:          1124211  (3.44x floor)
      max peak:            12531121  (38.30x floor)
    Likely failure:
      swing visible but the configured energy statistic peaked at 3.44x floor, under snr 6; peak reached 38.30x and would have crossed
```
`--swing` runs it first and arms the swing checks on the detected bin. Repeat
it a few times: a detected bin that wanders by one is the hand-measured tee
range; one that is always off by the same amount is the range conversion.

`--list` prints every check without opening a port. `--json path` writes the
results for a report. The suite exits 1 on any `FAIL`; `SKIP` lines (older
firmware, `--swing` not given, the solve placeholder) never fail the run.

The check logic is unit-tested without hardware in
`tests/test_iwr6843_firmware_checks.py` against a scripted serial port, and a
test pins the suite's command list to the CLI table in
`firmware/iwr6843/l3_dump.c`, so a new firmware command without a check fails
CI.

## Shot Evidence After A Fire

After `--swing` fires, the suite reads back what the onboard pipeline made of
the swing and prints it under `shot evidence (after the fire)`:

```text
clubtrack active=0 why=idle count=6 ... speed=22.34 fit=6 residual=0.11 ...
delivery points=5 az=5 el=5 speed=22.40 radial=22.20 path=2.70 attack=0.00 residualmm=0.80 conf=0.58 valid=spa
 angle az=-0.20 el=0.00 coh=1.00 peak=8.1 psi=3.50 valid=ae estimates=5
range impact fired=1 why=fired offsetms=3.22 t=23216 pending=2 passed=0 fired_n=1
shot state=result since=14 impact=23216 source=range origin=1.36,0.00,0.00 club=6 post=9 transitions=6
balltrack armed=1 confirmed=1 done=1 why=lost count=7 origin=29.00 bin=60.03 impact=20000 acq=1 slow=0 fast=0 lost=4 post=9
launch points=5 speed=61.00 radial=60.40 hla=2.00 vla=12.10 residualmm=5.60 conf=0.71 valid=shv
result v2 shot=1 verdict=valid valid=0x1ff quality=0x3ff impact=23216 source=range club=6 ball=7 smash=1.50 ready=1
```

Read it as a chain of evidence. `delivery valid=spa` means speed, path and
attack were all measured (angles came through); `valid=s` alone means the
club was seen in range only. `source=range` is the range-only impact, the
club track's line crossing the ball's range, which fires the self-trigger.
`gate`, `geometry` and `both` (and the `geometric_impact` quality bit) come
only from firmware before 2026-09-30, when those detectors were removed. `shot state=result` with
`launch valid=shv` is a complete post-impact measurement; `balltrack
confirmed=0` means nothing left the origin fast enough to be a ball (a
practice swing, or the ball was not where the destination bin said).
`result verdict=` is the firmware's own VALID / PARTIAL / INVALID call, and
`triggerLog result` also prints the packet `shot_result.py` parses.

The server reads that packet itself after every capture, self- or
sound-triggered. The shot's launch angles, club path and attack angle are
the packet's usable values. The shot record carries the whole packet as
`iwr6843_onboard`, and the kiosk shows a "TI onboard" strip under the Live
tiles with MEASURED / ESTIMATED on each metric. Only `--debug` reads the
ring back; then the log line `IWR6843 onboard <verdict>: ball ... vs OPS ...`
puts the firmware's numbers beside the host LCMF-v1's, which are never
published.
While the ball detector is on (the default), the kiosk's setup banner
shows the same `setup=` advice as the `ball status` line above, refreshed
every second.

`triggerLog perf` prints what each stage costs per frame in microseconds;
collect it on a soak before deciding what moves to the HWA or DSP.

## Cadence Acceptance Soak

This is the acceptance gate for any change to the capture-path DMA/CPU memory
layout (for example, moving a scratch buffer between L3 and `DATA_RAM`). It
runs the sensor for tens of thousands of frames and fails on any sign that
the firmware could not keep up with the inter-frame budget:

```bash
uv run python scripts/hardware-test/iwr6843_cadence_soak.py \
    --config config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg --frames 50000
uv run python scripts/hardware-test/iwr6843_cadence_soak.py \
    --config config/iwr6843_l3dump_dense_51f2ms_53bin_iq8.cfg --frames 50000
```

Run it against **both** shipped profiles — the wide/iq16 default and the
dense/iq8 profile — since they run the same firmware image at different
frame periods (3 ms and 2 ms respectively, read automatically from each
`.cfg`'s `frameCfg` line).

The script reads the firmware's `stats` CLI response
(`firmware/iwr6843/l3_dump.c`, `l3_cli_stats`) and checks:

- `hwa_frames` reached at least 90% of the requested `--frames` (a low count
  means the sensor stalled or `sensorStop` cut the run short, not that the
  cadence held).
- `hwa_missed` / `hwa_frames` (the HWA frame-start miss rate) does not exceed
  twice the recorded baseline of 0.0089% for the shipped profile. Doubling
  the baseline is a materiality band: DMA/CPU contention from a bad
  relocation shows up as a large jump, not a rate that hovers just above the
  baseline.
- `iq8_overrun` (IQ8 pack overruns) is zero.
- `iq8_edma_err` (IQ8 EDMA errors) is zero.

**Pass criteria:** the script prints `PASS` and exits 0 for both profiles.
**A failure means the relocation broke the inter-frame budget on real
silicon — revert to the L3 fallback rather than tuning around it.**

The stats parser (`parse_stats` in the script) is unit-tested without
hardware in `tests/test_iwr6843_monitor.py`
(`test_cadence_soak_parses_firmware_stats` and related tests), against the
exact field names the firmware emits.

### Readback measurement (manual, alongside the soak)

While soaking the dense/iq8 profile, take at least 20 shots and record:

- `l3track` readback latency — must stay under 1.0 s per shot.
- Frequency of `l3sparse` truncation warnings (`monitor.py:510`) — must be no
  more frequent than on the 45-frame profile.

Record both numbers alongside the soak's PASS/FAIL output when reporting
results for a DATA_RAM relocation change.
