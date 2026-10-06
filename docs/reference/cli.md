---
icon: lucide/terminal
---

# CLI Flags

Every flag accepted by the server, grouped by subsystem.

`scripts/start-kiosk.sh` accepts most of these and forwards them. Use
`--dry-run` to print the exact command the script would run:

```bash
scripts/start-kiosk.sh --iwr6843 --dry-run
```

!!! note "Generated from the source"

    This table is derived from the argparse definitions in
    `src/openflight/server.py`. If a flag here disagrees with the code, the
    code is right — please open an issue.

## Server & web

Binding, ports, and debug output.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--mock`, `-m` | flag | Run in mock mode without radar |
| `--mock-swing-speed` | flag | Run swing speed training mode with simulated reps and no OPS radar |
| `--host` | default `0.0.0.0` | Host to bind to (default: 0.0.0.0) |
| `--web-port` | int; default `8080` | Web server port (default: 8080) |
| `--startup-status-file` | path | Write structured initialization progress for the optional kiosk splash |
| `--debug`, `-d` | flag | Enable verbose FFT/CFAR debug output. With `--iwr6843`, also read the TI ring back after every shot, save it, and log the host LCMF-v1 numbers beside the firmware's. The shot always carries the firmware's numbers. |
| `--radar-log` | flag | Log raw radar data to console (Python logging) |
| `--show-raw` | flag | Show raw radar readings in console (signed values) |

## OPS243 radar & transport

Serial port, baud, and sample rate.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--port`, `-p` | — | Serial port for radar |
| `--ops-baud` | int | — |
| `--sample-rate` | int; default `30` | Radar sample rate in ksps (default: 30). Lower = longer buffer but lower max speed. 25=174mph/164ms, 27=187mph/152ms |

## Trigger & capture

How a capture is initiated and framed.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--trigger` | choices: `sound`, `speed`; default `sound` | Trigger strategy |
| `--sound-pre-trigger` | int | Pre-trigger segments S#n, 0-32 (default: 16 = 50/50 split, 24 with --iwr6843-self-trigger; each segment ~4.27ms at 30ksps) |

## IWR6843 angle radar

The supported angle radar.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--iwr6843` | flag | Enable TI IWR6843 L3 capture and LCMF-v1 vertical launch angle |
| `--iwr6843-port` | — | TI serial port (auto-detect by default) |
| `--iwr6843-config` | default `config/iwr6843_l3dump_wide_24f2ms_53bin_iq16_window_hann.cfg` | TI RF config matching the flashed L3 firmware |
| `--iwr6843-cal` | default `config/iwr6843_calibration_reference.json` | TI complex array/range calibration JSON |
| `--iwr6843-trigger-pin` | int; default `17` | BCM GPIO receiving the shared sound-trigger edge (default: 17) |
| `--iwr6843-tee-m` | float; default `1.575` | Distance in metres from the enclosure front to the centre of the ball, not the golfer's feet (horizontal is fine: the ball's height barely changes it). The array sits 30 mm behind the front and that is added internally (default: 1.575, minimum: 1.4) |
| `--iwr6843-net-m` | float; default `4.6` | Distance in metres from the enclosure front to the net or screen; the array's 30 mm depth is added internally. Only used with `--iwr6843-flight net`: ball tracks are kept 0.25 m short of it (default: 4.6) |
| `--iwr6843-tee-band-bins` | float; default `6` (`0` = off) | **Experimental.** Width, in range bins, of the tee band the IWR6843 club and ball trackers ignore; the firmware places it on the noisiest bins within 10 bins of the tee (learned from idle frames, frozen while the club swings) and impact is then fitted from the tracks either side (`trackCfg impactFit`). Before 2026-09-29 the value was a half width. 0 = off (default). The value is sent at every start, 0 included, so a restart clears a band a previous run set. A 13-bin band (the old ±6 half width) failed acceptance on the recorded sessions (club tracking collapsed); leave it off unless you are testing a cluttered setup |
| `--iwr6843-flight` | choices: `net`, `range`, `course`; default `net` | net clamps tracks at the net. range or course keeps returns past it and measures the late-window descent after the shot is published |
| `--iwr6843-self-trigger` | flag | Freeze the IWR ring when the firmware club track is predicted to cross the tee's range and send S! to the OPS, instead of the sound-gate edge. Disconnect the SEN-14262 GATE from HOST_INT. Requires --iwr6843 and --trigger sound |
| `--iwr6843-full-capture` | flag | Transfer all samples and TX channels instead of selected cells; about 7 seconds for the default profile. Requires `--debug`, which turns it on by itself. The firmware tracker stays configured, so the onboard result is unchanged. |
| `--no-iwr6843-onboard-track` | flag | Select cells on the Pi instead of onboard; still transfers selected samples unless `--iwr6843-full-capture` is set. |
| `--iwr6843-ball-detector` | choices: `off`, `on`, `follow`; default `off` | Firmware ball-placement detector: `on` locks the ball for the onboard shot machine and drives the kiosk setup banner; `follow` also aims the self-trigger at the locked ball; `off` keeps the configured tee bin |
| `--iwr6843-setup-poll-s` | float; default `1.0` | Seconds between `ball status` polls for the setup banner |
| `--iwr6843-self-trigger-bin` | int | Global range-FFT bin the trigger watches, inside the cfg's first capture window (default: two bins short of the ball, from `--iwr6843-tee-m`, moved by `--iwr6843-self-trigger-offset-m`: bin 36 for the default 1.575 m, 1.605 m from the array). Requires --iwr6843-self-trigger |
| `--iwr6843-self-trigger-offset-m` | float; default `0.2` | Move the default trigger bin this far downrange, in whole bins (0.2 m = 4 bins); negative moves it toward the radar. A swing's line must carry past the ball, which backswings and waggles do not. The board's tee band, ball search and retained cells move with it. Not with `--iwr6843-self-trigger-bin`. Requires --iwr6843-self-trigger |
| `--iwr6843-self-trigger-snr` | float | Club target threshold as a multiple of the firmware's running noise floor, at least 1 (default: 1). Requires --iwr6843-self-trigger |
| `--iwr6843-ball-snr` | float | The firmware ball tracker's target threshold as a multiple of its noise floor, 1..1e6, set apart from the trigger's (`trackCfg ballSnr`; default: the firmware's, 1). The Pi sends it at every start (0 on the wire restores the firmware default) |
| `--iwr6843-no-confirm-flight` | flag | Fire the self-trigger on every club or ball-leave candidate, as before, instead of only once the ball's flight confirms it (`trackCfg confirm`). Confirmation keeps backswings and static scenes from firing; firmware without it fires on every candidate and the Pi logs a warning. Requires --iwr6843-self-trigger |
| `--iwr6843-tilt-deg` | float | Override mount tilt from the TI calibration JSON |
| `--iwr6843-radar-height-m` | float | Override antenna-center height from the TI calibration JSON |
| `--iwr6843-ball-height-m` | float; default `0.04` | Ball-center height above the floor/mat (default: 0.040) |
| `--iwr6843-tx-order` | choices: `auto`, `normal`, `reversed`; default `auto` | TI TDM chirp order; auto reads the chirp masks from the cfg |
| `--iwr6843-capture-timeout` | float; default `16.0` | Maximum seconds an OPS shot waits for its TI UART dump |
| `--iwr6843-output-dir` | — | Raw TI dump directory when --debug is enabled (default: <session-log-dir>/iwr6843) |
| `--iwr6843-azimuth-offset-deg` | float | Azimuth of the radar boresight relative to the target line, in degrees. Positive means boresight points right of the target line. Added to the measured club path; 0 reports club path relative to boresight. |
| `--iwr6843-horizontal-phase-reference-rad` | float | Static target-line phase measured by horizontal aim calibration. Subtracted from the TX2 horizontal proxy before angle conversion. |

Self-trigger mode uses serial messages; no microphone or trigger GPIO is required,
and the trigger pin is left unallocated. The firmware extracts club targets of
at least the configured SNR times its running noise floor from the bins short
of the tee, the club track follows the club through them, and the capture
freezes when the line fitted to the club's approach is predicted to cross the
tee's range within 4 ms. It does not wait for the club to leave again. (The
range gate that fired from its own short track, and `--iwr6843-self-trigger-frames`
which tuned it, were removed on 2026-09-30.) The trigger also waits until the pre-trigger ring has
wrapped, so the saved movie has its full history. These checks reduce false
triggers but do not prove ball identity; validate tee geometry and thresholds
with real shots. Self-trigger needs an IQ16 profile: the IQ8 dense profiles are
rejected at startup. Full capture is a diagnostic alternative, not a longer
recording window.

The front end (floor, watch region, trace) is `firmware/iwr6843/l3_trigger.c`;
the club track and its range-only impact are `l3_club_track.c` and `l3_impact.c`.
`tests/test_iwr6843_firmware_trigger.py` and `test_iwr6843_firmware_replay.py`
build them on the host.
Flash an image built from the matching firmware.

## IWR6843 dump viewer and label tools

These are scripts, not server flags. Run them with `uv run python`.

| Command | Description |
|---------|-------------|
| `scripts/iwr6843/dump_viewer.py [--dir DIR] [--host H] [--port P] [--no-browser]` | Dump viewer. **Annotate tracks** lets you click the ball or club onto the range-time map (click the same frame again to move a point, shift-click to remove, **Seed from firmware** to start from the firmware's points); **reviewed** + **Save labels** writes `<dump>.l3dump.labels.json` next to the dump. Annotate works only on a capture chosen from the list, not an uploaded file. Reload the page to pick up edits to it |
| `scripts/analysis/fit_constants.py --update-baseline` | Rescore every labelled dump and write `tests/radar/recordings/label_baseline.json`. The labelled-replay test fails if a dump has no baseline entry or scores below it |
| `scripts/analysis/fit_constants.py [--dir DIR] [--passes N] [--only PREFIX]` | Constants sweep. Replays the labelled dumps in `--dir` (default `tests/radar/recordings`) with each runtime config field in `src/openflight/iwr6843/tunables.py` varied, `--passes` times (default 2), optionally only fields whose name starts with `PREFIX`. Prints a report and edits nothing; leave rows with flat n/n alone. It does not cover the `#define`s in the C headers |

## Inclinometer

LIS3DH enclosure tilt compensation.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--inclinometer` | flag | Enable LIS3DH enclosure pitch compensation for IWR6843 tilt |
| `--inclinometer-zero-offset` | float | Degrees added to raw LIS3DH pitch (default: 0) |

## Ballistics & spin

Carry model and spin handling.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--ballistics` | flag | Use the physics-based carry simulator (drag + Magnus, RK4). This is the default; shots without a vertical launch angle fall back to the legacy table estimator. |
| `--no-ballistics` | flag | Disable the physics simulator and use the legacy carry table for all shots. |
| `--calculated-spin` | flag | Replace radar-measured spin with the kinematic estimate (170*v*sin(LA)^1.2) when the launch angle was measured. The 24 GHz OPS return carries no usable spin line (see src/openflight/spin_estimate.py); the measured value is kept in spin_rpm_measured for offline scoring |

## Swing speed

Club-only training mode.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--swing-speed` | flag | Run club-only swing speed training mode (no impact or ball required) |
| `--swing-speed-threshold` | float; default `30.0` | Outbound speed threshold that starts a swing speed rep (default: 30 mph) |
| `--swing-speed-max` | float; default `130.0` | Maximum plausible swing speed accepted from OPS reports; use 0 to disable (default: 130 mph) |
| `--swing-speed-min-readings` | int; default `3` | Minimum qualifying radar readings required to count a swing speed rep (default: 3) |
| `--swing-speed-single-peak` | float; default `60.0` | Peak speed that can count as a swing from one radar reading (default: 60 mph) |
| `--swing-speed-num-reports` | int; default `8` | Number of OPS speed candidates to report per sample cycle (default: 8) |
| `--swing-speed-end-ms` | float; default `1000.0` | Milliseconds below threshold before ending a swing speed rep (default: 1000) |
| `--swing-speed-cooldown-ms` | float; default `750.0` | Cooldown after a swing speed rep before accepting another (default: 750) |
| `--swing-speed-rejected-cooldown-ms` | float; default `100.0` | Cooldown after an ignored short motion before re-arming (default: 100) |

## Logging & session data

Where session logs go and what they capture.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--session-location`, `-l` | default `range` | Location identifier for session logs (e.g., 'range', 'course', 'home') |
| `--log-dir` | — | Directory for session logs (default: ~/openflight_sessions) |
| `--profiles-path` | path | Profile store (default: `OPENFLIGHT_PROFILES_PATH` or `~/.config/openflight/profiles.json`) |
| `--no-logging` | flag | Disable session logging |

## Simulators & power

Outbound connectors and battery status.

| Flag | Type / default | Description |
| --- | --- | --- |
| `--battery` | — | Show battery and external-power status using the selected provider |
| `--sim` | flag | Enable simulator connectors from config/sim.json (GSPro / OpenGolfSim / PAR-TEE). Off by default. |

## High-speed camera capture

Optional rolling-buffer capture and replay. See [camera setup](../camera/README.md).

| Flag | Type / default | Description |
| --- | --- | --- |
| `--camera-capture` | flag | Enable high-speed rolling-buffer capture and replay |

Flight model air. The ballistic carry uses the day's air density instead of
the standard 1.225 kg/m3 when these are given (`openflight.environment`).

| Flag | Type | Description |
|------|------|-------------|
| `--temperature-c` | float | Air temperature, default 15 C |
| `--pressure-hpa` | float | Station pressure; default standard, or from altitude |
| `--humidity` | float | Relative humidity 0..1, default 0 |
| `--altitude-m` | float | Range altitude for the pressure when no barometer |
| `--camera-capture-width` | int; default `640` | Capture width |
| `--camera-capture-height` | int; default `400` | Capture height |
| `--camera-capture-fps` | float; default `300` | Capture frame rate |
| `--camera-capture-pre-ms` | float; default `150` | Milliseconds retained before the trigger |
| `--camera-capture-post-ms` | float; default `50` | Milliseconds retained after the trigger |
| `--camera-capture-exposure-us` | int; default `1000` | Exposure seed for startup calibration |
| `--camera-capture-gain` | float; default `4.0` | Analogue-gain seed for startup calibration |
| `--camera-capture-mount-height-m` | float; default `0.20955` | Camera optical-center height above the hitting surface |
| `--camera-capture-horizontal-offset-deg` | float; default `0` | Target-line correction added to horizontal launch angles |
| `--camera-capture-lateral-offset-m` | float; default `0` | Camera position relative to radar center; positive is target-right |
| `--camera-capture-roll-deg` | float; default `0` | Clockwise image-roll correction for preview and geometry |
| `--camera-capture-stream` | `raw` or `main-y`; default `raw` | Camera stream to persist |
| `--camera-capture-scaler-crop` | `X,Y,W,H` | Optional Picamera2 scaler crop |
| `--camera-capture-rotate-180` | flag | Rotate saved frames 180 degrees |
| `--camera-capture-mirror-horizontal` | flag | Mirror saved frames left-to-right after rotation |

## K-LD7 (deprecated)

Retained for existing builds only. See [Legacy (K-LD7)](../legacy/index.md).

| Flag | Type / default | Description |
| --- | --- | --- |
| `--kld7` | flag | [DEPRECATED] Enable K-LD7 vertical angle radar (launch angle) |
| `--kld7-port` | — | K-LD7 vertical serial port (auto-detect if not specified) |
| `--kld7-angle-offset` | float; default `1.5` | K-LD7 vertical boresight offset in degrees. Not user-measurable without a corner reflector; 1.5 is the calibrated default for the standard mount (default: 1.5) |
| `--kld7-mount-tilt` | float | K-LD7 vertical radar mount tilt in degrees. REQUIRED with --kld7 — measure it with a phone inclinometer against the radar face; there is no default because a wrong tilt silently corrupts the launch angle |
| `--kld7-ball-distance` | float; default `5.0` | Radar-to-tee distance in feet (default: 5.0) |
| `--net-distance` | float; default `10.0` | Ball-to-net/screen distance in feet (two_ray). For nets beyond the ~11ft FSK range wrap, far-flight frames are de-aliased and kept instead of dropped (default: 10.0; nets at/inside the wrap are unaffected). |
| `--kld7-radar-height-inches` | float; default `4.0` | K-LD7 radar height above the ball in inches, used by the ball-speed cosine correction geometry (default: 4.0) |
| `--kld7-vertical-raw` | flag | TEST MODE: show the raw vertical launch angle for every shot the estimator produces, bypassing all display guardrails (plausibility, soft-lane, estimator-agreement, confidence floor). Default off. |
| `--kld7-horizontal` | flag | [DEPRECATED] Enable K-LD7 horizontal angle radar (club path) |
| `--kld7-horizontal-port` | — | K-LD7 horizontal serial port |
| `--kld7-horizontal-offset` | float | K-LD7 horizontal angle offset in degrees (default: 0.0) |

## Wrapper-only flags

Handled by `scripts/start-kiosk.sh` itself rather than passed through.

| Flag | Description |
| --- | --- |
| `--dry-run` | Print the command that would run, then exit |
| `--startup-splash` | Show component progress while the kiosk starts |
| `--startup-splash-port` | Port used by the temporary splash server |
| `--port`, `--web-port` | Set the kiosk web port |
| `--radar-port`, `--ops-port` | Forward the OPS serial port as the server's `--port` |
| `--buffer-split` | Buffer split preset (`balanced`, `post-heavy`, `pre-heavy`) or raw segment count |

## Related

- [Running & modes](../using/running.md) — the common invocations
- [Configuration files](configuration.md) — settings that are not flags
- [Constants](constants.md) — values compiled in rather than passed
