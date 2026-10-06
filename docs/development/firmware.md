# IWR6843 Firmware Developer Guide

OpenFlight uses custom firmware on the TI IWR6843LEVM to preserve a short radar
movie around impact in the chip's on-board L3 RAM. The firmware continuously
processes chirps, stores selected complex range bins in a circular frame ring,
and streams that ring to the Raspberry Pi after the shared sound trigger.

Most builders do **not** need to compile firmware. A validated flashable image is
checked into the repository. Build the firmware only when changing capture
geometry, range windows, HWA/EDMA processing, or the binary dump contract.

For hardware wiring, mounting, geometry, calibration, and normal OpenFlight
startup, use the [IWR6843 Operator Guide](../iwr6843/index.md).

## Current Release

One firmware image supports multiple runtime capture profiles (see
[Choose A Capture Profile](#choose-a-capture-profile) below). Flash the image
once, then choose a profile by passing its `.cfg` to OpenFlight.

| Component | Current value |
|---|---|
| Flash image | `firmware/releases/l3_dump_configurable_capture_20260818.bin` |
| Runtime configs | See [Choose A Profile](../iwr6843/index.md#choose-a-profile) |
| Reference calibration | `config/iwr6843_calibration_reference.json` |
| Native build | `make -C firmware build-native` |
| Container build | `make -C firmware docker-build` |
| Flash image size | 430,916 bytes |
| Flash SHA-256 | `664b2360dc7d6f8e7300eb53f980faf5294d25d7c1e8c93692f8756a62ea8dde` |
| Validate on hardware | `uv run python scripts/hardware-test/test_iwr_firmware.py` (see [Firmware Feature Check](../iwr6843/verify.md#firmware-feature-check)) |
| Dump format | Variable-width, timed complex range-FFT snapshots |
| Release version | `firmware/VERSION` (semver); the image above predates versioning |

Verify the checked-in image before flashing:

```bash
sha256sum firmware/releases/l3_dump_configurable_capture_20260818.bin
```

After flashing, run the firmware CLI test suite on the Pi. It covers every
registered CLI command; the on-chip solve reports `SKIP` until the MSS gains a
command that invokes it.

## Choose A Capture Profile

The full set of runtime capture profiles (config file, frame count, spacing,
payload size, and when to use each) is maintained in one place:
[Choose A Profile](../iwr6843/index.md#choose-a-profile) in the IWR6843
Operator Guide. This firmware doc previously kept its own copy of that table;
it went stale (two new profiles landed there and were never mirrored here), so
this section now points at that single source of truth instead of duplicating
it. All profiles use 3 TX, 4 RX, 12 TDM loops, and 128 acquired ADC samples.
Changing profiles does not require reflashing.

The supported normal-TX profiles use a fixed positive TDM sign. This physical
registration keeps the full eight-element vertical channel aligned with OPS
radial speed. Automatic sign selection is useful for offline diagnostics, but
it is not the production policy because multipath can select the mirrored sign
and collapse that channel while leaving the range track apparently healthy.

## On-Chip Data Path

```text
RF chirp
  -> ADCBUF
  -> HWA 128-point range FFT
  -> EDMA copies the configured moving range window
  -> IQ16 is stored directly, EDMA compacts HWA-scaled IQ8 into L3, or
     (compact16/adaptive16) the detect task reads the IQ16 scratch and
     the rearm task copies only the retained window into L3
  -> circular frame ring in L3 RAM
  -> sound trigger freezes the completed pre/post-impact movie
  -> header, timing/window metadata, scale table, and IQ payload stream to Pi
  -> firmware rearms the ring for the next shot
```

The leave detector does not run in the rearm task. A finished pre-trigger slot is
published once its samples are in the ring: immediately for IQ16, and after the
IQ8 pack for the dense profile. A lower-priority task then reads that slot while
the accelerator writes the next frame. `stats` adds `detect dropped` and
`detect stale` when that queue falls behind or the slot has already been reused.
The launch-angle fit still runs on the Pi after the frozen movie is transferred.

The saved bins remain complex I/Q so the host retains phase for vertical and
horizontal direction of arrival. Every frame carries its absolute range-window
start, bin count, and measured time delta. IQ8 frames also carry their scale,
allowing the host to restore the physical sample amplitude before processing.

For IQ8, `iq8Scale` selects a fixed power-of-two scale before `sensorStart`.
The HWA applies that scale, then EDMA copies the compact byte from each signed
component into L3 without a CPU packing loop. Firmware `stats` report completed
EDMA packs, waits, errors, clipped components, and missed HWA starts rather than
silently hiding cadence failures.

## Onboard Self-Trigger

With `--iwr6843-self-trigger` the firmware, not the sound gate, decides when
impact is imminent. Once per completed frame the detect task scores the range
bins around the tee. The front end in `firmware/iwr6843/l3_trigger.c` keeps the
floor and the trace, and the club track fires:

```text
completed frame (every loop of the vertical TX pair, all RX)
  -> burst-MTI residual per bin: mean over loops removed, so the stationary
     ball and the room vanish and only movers remain
  -> per bin: residual energy integrated over ALL loops, the strongest single
     loop's residual power, plus the lag-1 loop autocorrelation (Doppler
     phase and coherence)
  -> noise floor = smoothed median of the watched bins in the chosen
     statistic (strongest loop by default: a fast club can be in a bin for
     only part of a frame); threshold = floor x snr        (l3_trigger.c)
  -> club targets above the threshold into the club track, which predicts
     the club through bins a standing return holds and reads past the tee
     band                                                  (l3_club_track.c)
  -> range-only impact: the club-in line fitted to the track crosses the
     tee's range within 4 ms of this frame               (l3_impact.c)
  -> the freeze the sound gate would have requested, then "Triggered" on the CLI
```

Until 2026-09-30 a range gate in `l3_trigger.c` fired instead, from its own
short track entering a gate around the tee. It held the tee's standing clutter
(hands, body, a mat edge) and on the 2026-08-24 recordings fired on 10 of 20
swings at the kiosk's settings. The club track fires on 19 of them, 18 within
three frames of the recorded freeze, so the gate, its track, its flight
recorder and its thresholds were removed.

The club is never required to be seen moving away again: that only makes the
trigger late and adds a condition a real swing can fail. The reported Doppler
velocity is a readout, not a condition: with three TX at 45 us chirps a loop
is 135 us, so Doppler is unambiguous only to about +/- 9 m/s and a clubhead
aliases. Range rate across frames is what separates a clubhead (2-3 bins per
3 ms frame for a driver) from a player walking up to the ball (a bin every few
frames).

The self-trigger is armed and tuned over the CLI:

```text
triggerCfg <globalBin> <snr> <on> [approach past stat]
triggerLog [trace|track|shot|result|perf|timing|clear]
trackCfg detectCore [dss|verify]
trackCfg cal <pitchDeg> <yawDeg> <rollDeg> <azOffsetRad> <elOffsetDeg> <rangeBiasM>
trackCfg elem <index> <phaseRad> <gain>
trackCfg impact <horizonS> [endM [endMinMps]]
captureCfg adaptive <enabled> <approachBins> <marginBins>
```

`snr` is the club-target threshold over the running noise floor (default 1 on
the kiosk). `on` of 0 turns the self-trigger off and any other value turns it
on; it was the gate's track-frame count, so existing arming lines still work.
The optional values default to 12 approach bins (about 0.56 m short of the
tee), 3 bins past it, and the strongest-loop statistic (`stat` 1; 0 selects the
energy over all loops once the club is known to be seen). A longer line, one
still carrying the gate's `minCoh minStep minSpeed minApproach`, is refused
rather than half applied. `trackCfg impact <horizonS>` sets how close to the
frame's time the club's predicted crossing must be (default 4 ms); the old
five-value line of the removed geometric detector is refused.

The crossing alone often never comes. On the 34 labelled swings the club's
radar range when the ball leaves is 3-12 bins (median 7.4) short of the
ball's: at impact the club's return merges with the ball's and the club track
stops taking points. So the range impact also fires on the first frame an
approaching track (one with a usable club-in estimate) takes no point after a
point within `endM` short of the ball's range (default 0.40 m, about 8.5
bins; 0 turns it off), dated to that last point. That approach must be at
least `endMinMps` (default 20 m/s; 0 accepts any usable approach): on the
bench (2026-10) a backswing's downrange crossing, which the club-in fit
accepts from 17 m/s, armed the end and fired 0.5-0.8 s before impact, while
every approach that armed it on the labelled swings was 20.6 m/s or faster.
A point past the ball's range never arms it. `triggerLog track` prints which rule
fired (`cause=crossing` or `cause=end`) and whether the end rule is armed.

Both rules need a usable club-in estimate, and that estimate must span time,
not just points. The club-in fit takes `fitPoints` (4), then older points
until it spans `fitSpanUs` (8.5 ms), so at 2 ms frames it fits 6 points where
3 ms fits 4. A fit whose points still span under `minSpanUs` (5.5 ms) is
`short_span` and arms nothing: at 2 ms a new track's three points span 4 ms,
and a third of a bin of range jitter then reads about 4 m/s. Replaying the
bench's takeaway fires (2026-10) showed 7 of the 8 that went through these
rules came from just such a track, the club at address 0-0.4 m short of the
ball fitted at 20-57 m/s, past both `clubMinMps` and `endMinMps`. Every
range-rule fire on the labelled swings fitted 6 ms or more, and 3 points at
3 ms frames span 6 ms, so 3 ms behaviour is unchanged. Both values are
compile-time defaults (`l3_impact_fit_cfg_defaults`); replays set them through
the `fit` tunables, and 0 turns either off.

When the club is not seen before launch (the early 2026-08-09 captures lose it
3-5 frames out), neither rule fires, so the ball leaving is a fallback
(`l3_leave.c`, tee band on only). Before impact the band hides the ball; the
first return to stand beyond the band's far edge, starting within 4 bins of it
and stepping outward on the next frame at 22-90 m/s, is the ball. On every
labelled swing the ball goes first and the club follows 1-10 frames later,
slower. It reads its own returns: the bins beyond the band against their own
median (6x), because the trigger's floor, learned where the club swings, sits
above the leaving ball. Three guards keep it off clutter and noise: it is armed
only for 10 frames after the club track came within 10 bins of the band, a
start from nothing must have had no return within a bin of it on the frame
before (a standing ridge beyond a band set short of the ball always does), and
the step must be a ball's speed. Impact is dated by running the two points back
to the band's centre. By then the departing ball is too smeared for the ball
tracker to start on, so the fallback's two points seed the flight
(`l3_ball_track_seed`), whichever rule dated impact; the ball tracker's own
first point needs only confidence 0.05 (the club tracker's 0.2 let the club's
follow-through be taken as the ball). It fires about two frames after launch.
`triggerLog track` prints a `leave` line after the `range impact` line.

#### The scan plan: fitting the 3 ms frame

Scoring a range bin (`l3_verticalResidual`) costs ~73 us on the R4F
(`triggerLog perf`, 2026-09-30), and the detect task outranks the CLI and the
trigger notices. With the tee band on, the armed path once scored the trigger
region and then the whole window, ~5.1 ms of a 3 ms frame: the board stopped
answering the moment the trigger was armed. `l3_scan.c` now says which bins a
frame scores, each once:

| When | Scored | Bins |
|---|---|---|
| before impact | club: 16 short of the band's near edge; fallback: 10 beyond its far edge; trigger region clipped to short of the band | 27 |
| idle frames | the same, plus 2 bins of the band's interior for its noise map | 29 |
| after impact | 12 following the ball (just beyond the band until tracked) + 4 following the club, merged | 16 |

After impact the floor is frozen at impact (the fallback's median beyond the
band, else the trigger's floor). When the detect task is behind (a newer frame
landed before this one was taken) it sheds the ball detector, the map chunk
and the pre-impact angle estimate; `stats` counts them as `shed=`. At the
kiosk's settings the labelled swings fire on 32 of 34 with 25 good launches;
the replay records `scored_bins` per frame and a labelled test holds the plan
to 29 and 16. Wider spans measured better in the replay (the whole window: 34
and 34) but cannot run in the frame until the residual is cheaper.

`triggerLog` prints the front end's frame count, floor and threshold, then its
configuration. After a missed swing, read `triggerLog track` (the club track,
its delivery, and the `range impact` verdict: `nodelivery`, `pending`,
`passed` or `fired`) and `triggerLog trace` (what the radar was offered)
before re-arming: `triggerCfg` clears the trace.
### Observation layer and club track

The per-bin residuals are not the trigger's alone. `l3_observation.c` owns
what is done with them before anyone decides anything: the statistic
(`stat`), the adaptive floor, and target extraction (local maxima at or above
floor x snr, strongest first, at most eight) with a sub-bin range centroid
over the peak and its neighbours, SNR, coherence (|lag-1 autocorrelation| /
energy), the aliased Doppler readout and a 0..1 confidence from the margin
over threshold and the coherence. Angle fields exist on every target and read
invalid until azimuth and elevation estimation lands. The trigger reads its
statistic and floor from this layer, so the two never disagree about what a
bin is worth.

`l3_club_track.c` keeps the persistent trajectory the trigger's own
`trackBin`/`trackAge` is not: the last 32 clubhead observations as a ring.
Each frame's targets (the same observations the trigger just scored, ranked
by `l3_obs_extract` against the trigger's floor) are associated by predicting
where the club should be, `lastBin + velocity x elapsed frames`, and scoring
every target inside a 3-bin gate by range error, Doppler continuity (wrapped
across the alias span) and quality (1 - confidence), so the shaft, hands and
body cannot steal the track just by being stronger. A frame with nothing in
the gate coasts the prediction; two in a row drop the track. A least-squares
fit of range bin against time over the held points gives a club speed that
does not depend on the aliased Doppler. `triggerLog track` prints the status
line (`clubtrack active= count= bin= dest= dist= vel= speed= fit= residual=
acq= assoc= coast= drop=`) then every held point oldest first (`p frame= t=
bin= dist= range= vr= vd= coh= conf= angles=`), distances in bins short of
the destination. The track is reset with every ring rearm and configured
with every `triggerCfg`: bin width from the accepted `trackCfg` range
resolution, Doppler span from the profile's loop period.

Both modules are pure C. `tests/test_iwr6843_firmware_observation.py` and
`tests/test_iwr6843_firmware_club_track.py` build them with the host compiler
through `openflight.iwr6843.firmware_host`, which holds the ctypes mirrors of
every `l3_*.h` structure in one place.

### Coordinate frames and angles

Two frames, defined once in `l3_frames.h` and nowhere else. The RADAR frame
has x out of the antenna face, y to the radar's right seen from behind, z
up; azimuth is positive right and elevation positive up. The GOLF frame has x
along the target line, y right of it, z up, origin at the antenna. The two
differ by the enclosure's roll, pitch (nose up positive) and yaw (boresight
right of the target line positive), held in `l3_radar_cal_t` together with
the per-virtual-element complex corrections from the corner-reflector solve,
the electrical zeros of the two baselines and the range bias. Every angle a
metric reports follows one rule: horizontal angles (club path, horizontal
launch) are `atan2(vy, vx)`, positive right, in-to-out for a right-hander;
vertical angles (angle of attack, vertical launch) are `atan2(vz, hypot(vx,
vy))`, positive up. `tests/test_iwr6843_firmware_frames.py` is the executable
statement of those conventions.

`l3_angle.c` reads a target's angles from its antenna channels at its range
bin. RX0..RX3 sit lambda/2 apart and the vertical TX pair (TX0 and TX2 of a
three-TX loop) is 2 lambda apart on the same axis, so `[txA.rx0..3,
txB.rx0..3]` is the 8-element elevation array once flipped into the
calibration's physical order and corrected; a Bartlett beamformer over +/-40
degrees with parabolic refinement gives elevation, and its peak-to-mean
power ratio the quality. TX1 sits lambda/2 off the pair's centre on the axis
the rotation made horizontal: the coherent mean over RX of TX1 against that
centre has phase `-pi sin(azimuth)` (TX1 is physically left), so azimuth is
positive right. The TDM phase between TX blocks is the per-loop Doppler
phase the observation layer measured, unwrapped by the alias the track's
range rate selects, divided by the TX count; `l3_dump.c` builds the channel
snapshot by summing each channel's burst-MTI residual coherently over the
loops with that per-loop phase unwound, and estimates angles for the
associated club target only, once its track has a range rate.

### Club delivery, range impact, ball flight, result

Each club-track point carries a golf-frame position from its range and
whatever angles it measured. `l3_track_delivery` fits x, y and z against
time over the newest points: the velocity vector gives club speed, its
direction club path and angle of attack, each valid only when the fitted
points carried the angle it needs (range alone gives the radial speed;
elevation adds attack; azimuth adds path), with the radial speed kept for
cross-checking. `triggerLog track` prints the delivery line and the newest
angle estimate beside the track.

`l3_impact.c` is the range-only impact that fires the self-trigger: the
club-in line fitted to the track's range points (`l3_impact_fit_track`)
crosses the ball's range, and impact is declared when that crossing is within
the horizon of the current frame's time, which dates impact between frames.
It needs no angles. A geometric detector once judged the delivery's 3D line
against the ball's position; it was removed on 2026-09-30, since the kiosk
never armed it and it never fired on the recorded swings. `triggerLog track`
prints the `range impact` verdict every frame.

`l3_shot.c` is the explicit per-shot sequence: WAITING_FOR_BALL, READY,
CLUB_ACQUIRE, CLUB_TRACK, IMPACT, BALL_TRACK, SOLVE, RESULT. IMPACT freezes
the ball origin, the delivery, the impact time and the club trajectory;
nothing after reads a live tracker. Kept post-impact frames are published to
the detect task (IQ16 rings) and routed to `l3_ball_track.c`, which looks
for a coherent return leaving the origin against the post window's own
noise floor and a lower threshold than the trigger's (the ball is a weak
return): acquired a bin or more beyond the origin within a short gate, which
excludes the impact echo, the resting club and the clubhead's follow-through
behind the ball; confirmed by its second point's range rate; once flying,
never associated with anything behind its last point; dropped when it is
too slow or impossibly fast. The launch is fitted over the earliest clean
points and extrapolated to the impact time: ball speed, horizontal launch
and vertical launch. When the angled positions do not lie on a line (a fast
ball crosses bins within a burst), the launch keeps the radial speed and
reports no angles rather than a precise-looking wrong direction; the club
delivery applies the same rule. `triggerLog shot` prints the machine, the
ball track, the launch and the ball points.

The first ten recorded swings (`tests/radar/recordings/`) shaped these
rules: a strong return at bin 44 (hands or body) is the most confident
target in every frame, so club acquisition prefers a target whose aliased
Doppler reads at least 1 m/s; the ball departs from about bin 49 at 33 to
41 m/s radial and is tracked for 14 frames on five of the six real shots.

`l3_result.c` assembles the versioned result when the machine reaches
RESULT: nine measurements (ball speed, vertical and horizontal launch, club
speed, path, attack, spin rate, spin axis, impact range) each with value,
confidence and flags (valid; measured rather than inferred; radial-only;
implausible; the tee stood in for a locked ball), evidence and plausibility
flags, and a VALID, PARTIAL or INVALID verdict. Spin stays invalid until it
is measured. `triggerLog result` prints the lines and the fixed 164-byte
little-endian packet (version 2: the version-1 100 bytes followed by the
impact fit — verdict, fused time, spread and each track's estimate) as two
hex lines; `openflight.iwr6843.shot_result` parses it and labels every metric
MEASURED or ESTIMATED for the UI. The Pi still parses version-1 (100-byte)
packets from older firmware, but older Pi code refuses version 2, so
**update the Pi before flashing this firmware.** An `inconsistent` impact fit
sets the `impact_uncertain` quality bit.

The host reads that packet on every capture, self-triggered or sound-triggered,
before any readback (`l3track`/`l3sparse` rearm the ring, which resets the
result). It rides on the capture as `onboard_result`, on the shot as
`iwr6843_onboard` (the session JSONL keeps it), and the kiosk shows it under
the Live tiles with each metric's provenance and confidence. The packet is
the shot's only IWR source:

- **Launch angles.** Its vertical and horizontal launch go on the shot with
  source `radar`, unless the metric is missing, implausible or a tee fallback.
- **Club delivery.** Its usable club path and attack angle go on the shot
  with status `onboard`.
- **Host corrections.** The host adds `--iwr6843-azimuth-offset-deg` to
  horizontal launch and club path, because the board runs with azimuth
  offset 0. It adds the inclinometer's effective-minus-configured tilt to
  vertical launch and attack angle.

Without `--debug` the ring is never read back; the frozen ring is released
straight away. With `--debug` it is read back, saved, and run through the
host LCMF-v1 and club-path pipeline. Those results are logged beside the
board's but never published. OPS ball speed is never replaced.

The detect path reads IQ8 rings as well as IQ16: every ring reader takes
the component width from the capture format and multiplies int8 samples by
the frame's HWA scale, so the dense IQ8 profiles get the same trigger,
club track and ball track as the wide IQ16 one. Firmware older than this
scores garbage on IQ8; the host logs which format is in use at start.

### IQ16 processing and exact IQ8 emulation

The measurement algorithms read IQ16. In the IQ16 profiles the ring holds
the HWA's int16 output and the detect task processes it as is; the plan is
for the compact profiles to process the IQ16 scratch frame before it is
compacted, so that IQ8 (and the compact IQ16 formats that follow) are
storage formats, not signal-processing formats.

What IQ8 costs a measurement is answered offline, not argued.
`firmware/iwr6843/l3_iq8.c` holds the board's three quantisers (the CPU
pack with a per-frame shift, the EDMA low-byte copy after the fixed
`iq8Scale` shift, which wraps rather than clips, and the dump-time divide)
and both `l3_dump.c` and the host library compile it, so
`openflight.iwr6843.iq8_emulation` turns an IQ16 recording into the IQ8 dump
the firmware would have stored, scale table included.
`scripts/analysis/ab_iq16_iq8.py` replays each recording both ways through
the trigger, club track, impact detector, ball track and launch fit and
prints the measurement table with a delta column, then a corpus summary
(`mean |delta|`, `max |delta|`, bias, and how often only one path produced
a measurement). On the five recorded swings the shipped EDMA path at scale
128 moves the gate's fire frame on two captures, the ball speed by under
0.15 m/s, and the club path and attack angle by degrees where a 3D fit
existed: the club is the measurement IQ8 hurts. The HWA's own shift
rounding is settled on a board with
`scripts/hardware-test/iwr6843_iq8_hwa_probe.py` (a truncating shift biases
every stored component by half a step) and passed to the tool as
`--hwa-rounding`.

`scripts/analysis/baseline_dataset.py` freezes the current firmware's
per-shot numbers (OPS speeds, the onboard result with every confidence,
the host launch angle and club path, capture cost) from session logs into
one CSV row per shot, tagged with the firmware SHA, before any of the
representation work changes them.

### IQ16 precision in the observations

`l3_iq16_stats.c` computes a bin's burst-MTI residual energy, per-loop power
and lag-1 autocorrelation in integers: the residual is taken scaled by the
loop count (exact in int32), its products and sums in int64, and the totals
are divided by loops squared once at the end. `l3_verticalResidual` takes
that path for every IQ16 frame (ring or scratch) and keeps the float path
for IQ8, so the precision the capture holds is not spent in float rounding
before the detector sees it.

Targets read their sub-bin range from the parabola through the LOG of the
statistic at the peak and its two neighbours (`trackCfg subbin
parabolic|centroid`, `l3_obs_parabolic_offset`), the centroid standing in
at a region edge. A Gaussian-shaped lobe is fitted exactly; the unwindowed
128-point range FFT's sinc-squared lobe is not a parabola in any domain, and
the log fit's worst error on it is 0.17 bin (8 mm) against 0.28 for the
linear parabola, which `tests/test_iwr6843_firmware_iq16_stats.py` pins. A
Hann window on the HWA (`windowEn`, a waveform decision for the profile
work) would make it almost exact. On the five recorded swings the two
estimators give the same speeds to 0.3 m/s and the same fit residuals to
half a millimetre; parabolic is the default because it is the exact fit
for a smooth lobe and its bias is characterised, where the centroid's
depends on the floor estimate.

### Validation against the OPS and a reference monitor

Every shot with an onboard result writes an `iwr_ops_comparison` entry to
the session log (`openflight.iwr6843.ops_compare`): the OPS ball and club
speed beside the IWR's, the IWR's confidence for each, the verdict, the
capture format and the impact range. The two are never averaged; the OPS
stays the validator. `scripts/analysis/ops_validation.py` reduces sessions
to bias, MAE, RMSE and P95, overall and grouped by club, capture format,
verdict and confidence band. `scripts/analysis/reference_validation.py`
replays every labelled shot under `tests/radar/datasets/` and compares
ball speed, launch angles, club speed, path and attack with the sidecar's
reference values, per field and per label, after first naming the thin
cells of the club x speed x shape matrix (`coverage`), because a hundred
identical 7-irons validate nothing. `openflight.iwr6843.confidence_calibration`
then turns (confidence, error) pairs from either source into error bounds
per confidence band and the lowest confidence that meets a chosen bound at
95% coverage: the number the shot validation should reject below, measured
rather than designed. The host packet exposes a per-domain confidence
(club, ball, angle, spin: the weakest usable metric of each) for that use.

### Angle confidence, calibration and angular validation

Every angle estimate now carries a confidence (`l3_angle_confidence`): the
elevation beam's peak-to-mean ratio mapped from 1 (flat) to
`L3_ANGLE_PEAK_RATIO_FULL` (6), capped by the azimuth coherence when azimuth
was measured; `l3_angle_format` prints it as `conf=` and the replay carries
it per point (`aconf=`). The per-element calibration is set and read as a
gain and a phase offset (`l3_cal_set_element`, `l3_cal_element`; `trackCfg
elem <index> <phaseRad> <gain>` as before) and `triggerLog cal` prints the
calibration in force: offsets, attitude, range bias and every element.

Validation needs a rig, so the tooling is ready before the rig is. Static:
`scripts/hardware-test/iwr6843_angle_static.py` walks the protocol's
positions (azimuth -20 to +20 in 5 degree steps, elevation -15 to +15),
locks the ball detector on the corner reflector and reads its measured
direction (`ball status` -> `ballangle`, which the host now parses) thirty
times per placement, saving a JSON set that
`openflight.iwr6843.angle_validation` summarises: bias, standard
deviation, P95, quality numbers per position, and the repeatability across
placements. Moving: `scripts/analysis/iwr6843_angle_moving.py` replays
recordings of a swinging reflector, compares every tracked point's angles
with the truth direction, bins the error by radial speed (where the TDM
correction and the alias resolution are exercised) and, with `--iq8`, runs
the firmware-exact IQ8 of the same captures beside IQ16. Whether IQ8's
angle noise rises with speed faster than IQ16's is answered by that table,
not argued.

### Compact IQ16 capture (compact16, adaptive16)

`captureFormat compact16|adaptive16` (with `captureCfg retain <preBins>
<impactBins> <postBins>`, default 16/24/16, and `captureCfg retainPolicy
...` for `l3_retain_cfg_t`) route the HWA's range-FFT output to the IQ16
scratch in DATA_RAM, as the IQ8 path does. The detect task then reads each
frame's wide processing window FROM THAT SCRATCH at full precision
(`l3_detectFrameOf`), and the rearm task copies the retained window into
the frame's L3 slot with `l3_compact_iq16` after restarting the HWA on the
other scratch (`l3_compactCompletedFrame`): compact16 centres the window in
the processing window, adaptive16 asks `l3_retain_window` where the shot
is. The plan's slot widths are the retain widths (`L3CapturePlan.compact`,
`retain*Bins`); the dump keeps format 4 with per-frame start and count, so
the host parser needs no change and `l3sparse`/`l3track` read the retained
windows. `l3_retain_budget` fits a phased plan that asks for too much by
cutting the oldest club history first and the flight's tail second, never
an impact frame, and prints the cut.

The scratch is ping/pong, so a detect frame is valid until the HWA is
aimed at its scratch again: about one frame period after the frame
completed. `l3_detectFrameStale` (the scratch is busy, or completed another
frame since) is checked after the observations are computed and before any
decision; a stale frame is dropped and counted (`stats`: `compact frames=
errors= max_us= scratch_stale= retain=` and the last window chosen). The
compaction's worst-case cost is timed into `max_us`, `triggerLog frames`
lists every stored slot's descriptor (window, processing window, shot
state, priority, reason), and IQ8 keeps reading its packed ring as before.
What is not yet known from hardware: the detect task's per-frame cost at 3
ms against that one-frame deadline (`scratch_stale` says), the rearm
budget with the compaction added (`max_us` and `hwa_missed` say), and
whether the 141 ms adaptive movie changes the shot numbers (the A/B tool
and the baseline dataset say).

### Processing region, retention region and the policy

The HWA window a frame is processed from and the bins a frame stores need
not be the same. `l3_retain.c` separates them: the PROCESSING region (the
wide window the detect task reads for its floor, candidates and
association) from the RETENTION region (the narrower IQ16 window that goes
into L3). Each capture phase has a fixed retained slot width; the policy
decides where the slot looks from what the trackers know before the frame
lands: around the tee or the locked ball while waiting, around the
predicted club while it approaches (`lastBin + velocityBinsPerFrame`),
spanning club and ball once they are within `approachBins`, on the ball
biased toward the arriving club for the impact frames, from the origin
outward while the departing ball is sought, and ahead of the prediction
once the flight is confirmed. Every frame gets a retention priority (low,
track, ball, impact, spin) and a reason, and `l3_frame_desc_t` records
what each stored frame is. The spin tag marks the first `spinFrames` post
frames (default 16, about the ball's 35-47 ms in view; `captureCfg
retainPolicy` sets it, at most 32), the confirmed flight frames included;
it only labels a frame and never moves its window. `l3_retain_budget` spends L3 in priority order:
every impact frame first, then the first ball frames, then the last club
frames; when the request does not fit it cuts the oldest club history
before the flight's tail and never the impact.

The policy is mirrored in the replay harness: `replay_iwr_track.py
--retain 16/24/16/7` decides a window for every frame of a recording from
the same tracker state the board would have and reports how many of the
points the trackers appended fell inside it. On the five recorded swings
the state-aware windows hold every club and ball point while keeping
37-40% of the processed bins; the centred windows of a plain compact
format miss the club.

### Profiling and adaptive windows

`triggerLog perf` prints per-stage counts, last, mean and maximum in
microseconds (residual, trigger, extraction, club track, angle, impact, ball
detector, ball tracker, `reconstruct`, the ball's direction fit at RESULT,
once per shot, and `dspwait`, the part of the residual the
MSS spent blocked on the DSS; the frame total excludes both `dspwait` and
`reconstruct`) from
`l3_profile.c` and the R4F cycle counter. That
is the evidence for moving a stage to the HWA or DSP; nothing is moved until
the numbers say which. `captureCfg adaptive 1 <approachBins> <marginBins>`
lets `l3_adaptive.c` move the plan's windows to the locked ball between
shots (at rearm, before the HWA restarts): pre starts `approachBins` short of
the ball, impact and post `marginBins` short, late half a window further
out. L3 is then spent on where the shot is rather than on fixed ranges.

### Detect task on the DSS

The self-trigger's detect task runs on the R4F (MSS) today: ~73 us a range
bin, so the scan plan scores 27 bins before impact and 16 after to leave the
CLI and the notices time in a 3 ms frame. It is moving to the C674x (DSS),
with the MSS path kept behind a switch so the two can be compared on the
board.

Phase 0 proves the link before anything moves:

- `l3_bin_score.c` is the IQ16 per-bin scoring, pulled out of
  `l3_verticalResidual` so both cores build the same code
- `l3_dsp_ipc.c` is the mailbox message (channel 0, MSS <-> DSS). A frame
  travels as its byte offset into L3, since the MSS sees L3 at `0x51000000`
  and the DSS at `0x20000000`, and the DSS refuses a request that would read
  outside it. The DSS invalidates its cache over the frame before reading
  (the EDMA writes the ring behind it), or gathers it (below)
- `trackCfg dsp ping` and `trackCfg dsp probe [bins]` are sub-modes, because
  the CLI table is at the SDK's `CLI_MAX_CMD`. The probe scores the newest
  pre-impact ring frame on the MSS and then the DSS, and prints both times
  and whether every sum matched bit for bit. IQ16 rings only
- `scripts/hardware-test/iwr6843_dsp_probe.py` runs both and summarizes

Frames in the compact formats' processing scratch live in MSS DATA_RAM,
which the DSS cannot read, so those formats stay on the MSS path.

#### Getting the DSS to boot

The DSS image had never run before the link: the first board run answered
"link open" and nothing else. The DSS now records how far it got, on two
channels the MSS reads without its help: a status block in HS-RAM at
`0x7F00` and the stage mirrored into DSSREG `DSSGPREG0`. The stages are
`reset` (an xdc Reset hook, before C init and BIOS), `startup_first` and
`startup_last` (xdc `Startup.firstFxns`/`lastFxns`, either side of the
BIOS module startups), `main`, `soc_init`, `task`, `mailbox_init` and
`link_open`, plus `exception` (a BIOS exception hook records the program
counter and flags). `trackCfg dsp status` prints the block; `trackCfg dsp
hw` reads the DSS's halt and power state, the ROM self-test flag, the ESM
status and `DSSGPREG0`, and checks HS-RAM with a write and read. Both print
whenever a `dsp` command goes unanswered.

What it found, run by run on 2026-09-30:

1. `SOC_init` on the DSS used `SOC_SysClock_INIT`, which re-runs the MSS's
   BSS unhalt and APLL wait; TI's mmw demo DSS uses `BYPASS_INIT`. Fixed,
   but not the cause
2. powered, not halted, stage `reset`: it died after the reset hook
3. stage `startup_first`: it died in the BIOS module startups, with no
   exception and no DSS ESM flag (ESMSR4 bit 2 is channel 34, HVMODE, a
   supply monitor)
4. the cause: this image alone overrode the platform's
   `ti_sysbios_family_c64p_Cache_l2Size = 0` with 32 KB of L2 as cache
   (linker warning #10190 on every build), which the Cache module applies
   in exactly that window. With the platform's caches (L1P and L1D 16 KB
   each, L2 all SRAM, as TI's demo) the DSS boots, answers `ping` in 22 us,
   and scores the live ring bit for bit as the MSS does

The first board probe: 27 bins MSS 2,796 us, DSS 1,303 us (48 us a bin);
53 bins MSS 5,488 us, DSS 2,558 us; 40/40 matched. The MSS figure is
inflated: the probe runs on the CLI task, which the detect task preempts
while the capture runs (~73 us a bin undisturbed).

#### The gather

Scoring in place, the DSS reads one 4-byte sample of every (loop, TX, RX)
row a bin, from L3, through a 16 KB L1D and no L2 cache: 48 us a bin, only
2.1x the R4F. `l3_dsp_gather_plan` finds the window of local bins a request
reads (PROBE's bins; SCORE's spans from the first to the end of the last)
and the DSS copies that window of every row from L3 into a 48 KB L2 buffer
(`.dssGather`) in one EDMA transfer on its own instance (1; the MSS's
capture runs on 0), AB-synchronised: aCount a row's window, bCount the
rows, the source stride a frame row. It polls for completion (1 ms bound),
invalidates L1D over the copy and scores from it through the same scorer
(`l3_dsp_serve_gathered`), so the answer is bit for bit the in-place one
(host-tested over random frames and the committed recordings). A gather
that is refused or does not finish falls back to scoring in place. The
probe line gains `dss_prep_us` (the gather, or the invalidate) and
`gathered`; SCORE reports the same preparation as its `dss_inv_us`.

#### A cheaper frame

See `docs/development/iwr6843-firmware-architecture.md` for the whole
pipeline. What changed, all with the replay's outcomes unchanged over every
committed recording:

- the ball detector updates every eighth frame (`L3_BALL_FRAMES_PER_UPDATE`;
  `l3_ball_cfg_defaults_at` rescales its counts, rates and persistence
  window to the same times) and estimates the ball's angle only when due
  (`l3_ball_angle_due`: a new or moved lock, then every 20 updates)
- the club's angle is queued, not estimated, on the decision path
  (`l3_angle_queue.c`): a snapshot keyed by the point's timestamp, estimated
  by `l3_angleTask` (priority 2, below the CLI; its stack and the queue in
  HS-RAM). The task peeks, estimates unlocked and takes the job only if it
  is still the oldest, so a fire frame's drain (after the freeze request,
  before `l3_shotObserve`) never misses one. `triggerLog perf` prints
  `angles queued= done= stale= failed= dropped= pending=`
- `l3_angle_bartlett` reads its steering rotors from a table filled once
  (`l3_angle_tables_init`, in HS-RAM: the program TCM is read-only on a
  flashed board)
- `l3_iq16_channel_stats` and the float paths read each sample once
- `l3_channels.c` holds the four per-channel loops `l3_dump.c` had

#### MSS memory

The detect timing's buffers overflowed DATA_RAM by 1,568 B (`.myFiqStack`
would not fit). MSS-only diagnostics and CLI scratch (`gProfile`,
`gTiming`, the detect and timing line buffers, l3sparse's `powerRow`, the
triggerLog, ball and stats lines) moved to HS-RAM's unused lower 29 KB
(`.hsramMss`). The DSS's words at the top (the SCORE result at `0x7400`,
the probe word at `0x7E00`, the status at `0x7F00`) are reserved in the MSS
link (`.hsramDss`), so a growing `.hsramMss` fails the link instead of
overwriting them. DATA_RAM free: 4,623 B.

#### Detect timing (step 2)

`l3_timing.c` stamps every frame the detect task finishes with the R4F cycle
counter: `acquired` (the HWA/EDMA stored it and it was queued; the stamp
travels in the detect queue), `dequeued`, `scoreStart`/`scoreEnd`, and
`decided` (the trigger decided and the slot was released). It keeps two
deadlines apart, because they are different things:

- **throughput**: `service` (dequeued -> decided) must average below the
  frame interval (`budget_us`, the frame period); `over_budget` counts the
  frames whose service did not
- **latency**: `latency` (acquired -> decided) may exceed the interval as
  long as the frame's ring slot is not reused first. `slot reuse margin` =
  (ring - 2) x interval - latency; `margin_min_us` is the tightest seen and
  `margin_negative` the frames that ran past it

`triggerLog perf` prints the summary and each statistic (wait, score,
service, latency, arrival) after the per-stage lines; `triggerLog timing`
adds the last 16 frames' timelines (`slot`, `epoch`, `core`, each duration,
queue `depth`, flags `post|behind|stale|fired`). Timing starts over with
each `sensorStart` and each `trackCfg detectCore <core>`.

A slot is checked for reuse when the detect task pops it and **again after
it has been read** (`l3_detectFrameStale`): a read that straddled the writer
reaching the slot is discarded and counted as `stale_read` in `stats`.

#### DSS bin scoring (steps 3-4)

`trackCfg detectCore dss|verify` chooses how the detector's bins are
scored (`dss` from boot); the observation -> tracker -> trigger path stays on the
MSS, unchanged. Every scan-plan read goes through `l3_scoreSpans`, one call
a frame, so both cores are always asked for exactly the same bins:

- `l3_dsp_spans_localize` turns the plan's global spans into the frame's
  local bins; `l3_dsp_spans_score` scores each once (both cores run it)
- `SCORE` (`l3_dsp_ipc.h`) carries the frame's L3 offset, geometry, up to
  four spans and the queue epoch. The DSS invalidates the whole frame
  (timed apart from the scoring), runs `l3_dsp_serve`, writes the
  observations to the result block in HS-RAM at `0x7400` (1320 B, below the
  boot status at `0x7F00`), writes it back out of its cache, and only then
  replies. The mailbox reply is the signal; the data is in HS-RAM
- the MSS copies the block and accepts it only for its own request (magic,
  seq, epoch, and a count that matches the bins marked:
  `l3_dsp_result_check`), then merges it (`l3_dsp_result_merge`)
- `dss`: the detect task blocks on the reply (the CLI and notices run
  meanwhile) for at most 6 ms, two frames. A DSS that fails a frame has the
  MSS score it (a `fallback`); three in a row latch the MSS until `dss` or
  `verify` is chosen again, and queue the notice `dsp detect latched to mss`
- `verify`: the request goes first, the MSS scores the same bins while the
  DSS does, then the two are compared bit for bit
  (`l3_dsp_result_compare`); the MSS's are used. A frame costs about what it
  did before, so it can stay on through real swings. Never latches
- `mss` is not a choice: the MSS scores only the frames the DSS cannot
  take, the fallbacks, and every frame once latched. Only an IQ16 ring frame
  in L3 can go to the DSS: `verify` is refused for any other capture or
  without the link; `dss` is not, and a frame that is not one (IQ8,
  `compact16`, `adaptive16`, or the format changed since) goes to the MSS as
  `ineligible`, as does a frame that finds the link down or held by a CLI
  `dsp` command (the detect task never waits for it)

The `detect core=...` line (printed by `trackCfg detectCore`, `triggerLog
perf` and `triggerLog timing`) has the counts, the latch, the DSS's
invalidate and scoring microseconds (last/max), and the first verify
mismatch as `slot:bin:field`. `IWR6843Radar.detect_core()` and
`detect_timing()` parse them (`openflight.iwr6843.dsp_link`).

Acceptance on the board, after `trackCfg dsp probe` shows repeated
`match=1`: an armed session in `verify` through real swings with
`mismatches=0` and `failures=0`; then `dss` with the same swings firing,
`fallbacks=0`, `dropped=0 stale=0 stale_read=0` in `stats`, and `timing
service` mean well below `budget_us`.

### Hardware-gated work

The code above is complete and host-tested; what needs the rig is listed
here so nobody mistakes it for done (the table and protocols below say
where each item stands). The ball detector's acceptance list
(empty tee reaches waiting, a ball locks within a bin, the golfer does not
create false locks, removal releases, dest minus club bin behaves) has not
been run. The angle estimator wants a corner reflector at 0, +/-10 and
+/-20 degrees and known heights, and the reference calibration loaded with
`trackCfg cal` and `trackCfg elem`. The six core measurements need a
reference monitor over 30 to 50 shots per club (`tests/radar/datasets/`).
The spin probe's thresholds are
placeholders until stationary, low-spin and high-spin balls have been
recorded. On the 34 labelled swings in `tests/radar/recordings` the ball
stands only 3-9 dB over its window's median, so the micro-Doppler spread
there (1.1-2.5 cells, every ball "low-spin") is noise-dominated; the
rotation scan claims no line on any of them. Loop counts are chosen from
`scripts/analysis/evaluate_iwr_profiles.py` on real captures, not from
frame rate; nothing moves to the HWA or DSP before `triggerLog perf` has
numbers.

### IQ16 roadmap: what is done and what waits for the rig

| Phase | Status | Where |
| --- | --- | --- |
| 0 baseline dataset | tool ready; run on the rig | `scripts/analysis/baseline_dataset.py` |
| 1 IQ16 processing, exact IQ8 emulation | done | `l3_iq8.c`, `iq8_emulation`, `l3_iq16_stats.c` |
| 2 compact16 primitive and tests | done | `compact_iq16.c`, `test_iwr6843_compact_iq16.py` |
| 3 processing vs retention ROI | done | `l3_retain.h` (`l3_roi_t`), `L3CapturePlan.retain*` |
| 4 IQ16 retention ring, variable widths | done per phase (pre / impact / post widths), descriptors in `l3_frame_desc_t`; a fully variable-width arena is not needed while widths are per phase | `l3_retain.c`, `l3_dump.c` |
| 5-7 state-aware retention, impact protection, adaptive post-impact | done | `l3_retain_window`, `l3_compactCompletedFrame` |
| 8 compact track history apart from raw IQ | done (the trackers' rings and the result packet outlive the raw frames) | `l3_club_track.c`, `l3_ball_track.c`, `l3_result.c` |
| 9 IQ16 in the observations | done | `l3_iq16_stats.c` |
| 10 sub-bin range | done, bias characterised | `l3_obs_parabolic_offset` |
| 11 club speed IQ16 vs IQ8 | tool ready (the A/B); on five swings IQ8 moves the fit by under 0.04 m/s | `scripts/analysis/ab_iq16_iq8.py` |
| 12 IQ16 angle pipeline | done (float from IQ16, confidence added) | `l3_angle.c` |
| 13 antenna calibration | representation and CLI done; the numbers need a reflector | `l3_cal_set_element`, `trackCfg elem`, `triggerLog cal` |
| 14-15 static and moving angular validation | tools ready; need the rig | `iwr6843_angle_static.py`, `iwr6843_angle_moving.py` |
| 16-17 path, attack, ball speed, launch from the 3D fits | done | `l3_track_delivery`, `l3_ball_track_launch` |
| 18 IQ16 vs IQ8 replay tool | done | `ab_iq16_iq8.py` |
| 19 selectable capture format | done (`iq8`, `iq16`, `compact16`, `adaptive16`) | `captureFormat` |
| 20-21 memory budget and retention priorities | done | `l3_retain_budget`, `L3_RETAIN_*` |
| 22-23 spin IQ16 retention and probe | retention priority, the IQ16-vs-IQ8 probe and the label-tracked rotation scan done; a marked-ball recording with a reference spin is needed | `scripts/analysis/spin_probe.py --iq8`, `--labels` |
| 24-26 firmware spin, spin axis, face angle | not started: no evidence yet that the observable exists in these captures | — |
| 27 OPS validation | done | `ops_compare`, `scripts/analysis/ops_validation.py` |
| 28 reference validation | loop ready; needs the labelled dataset | `scripts/analysis/reference_validation.py` |
| 29 confidence from real error | tool ready; needs 27 and 28 to have run | `confidence_calibration` |
| 30 cadence experiment | protocol below; needs the rig | — |
| 31 HWA/DSP | waits for `triggerLog perf` numbers on the rig | `l3_profile.c` |
| 32 production result path | done (164-byte v2 packet with the impact fit; v1 still parsed on the Pi — update the Pi before the firmware; per-domain confidence on the host) | `l3_result.c`, `shot_result.py` |

### Rig protocols the code is waiting on

Run these in this order; each one's tool exists and each one's numbers
change what comes next.

1. **Baseline** (`baseline_dataset.py --firmware-sha`) on the shipped
   firmware before flashing anything from this branch.
2. **Compact formats on hardware.** Flash, run `stats` under
   `config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg` for a few hundred
   frames: `scratch_stale` must stay 0 (the detect task finishes inside one
   frame), `compact max_us` plus the rearm time must fit the frame's idle
   gap (`hwa_missed` stays at the wide profile's rate), `triggerLog frames`
   must show the windows following the tee, then the club, then the ball.
   `iwr6843_cadence_soak.py` is the acceptance gate.
3. **HWA rounding** (`iwr6843_iq8_hwa_probe.py`), so the A/B's IQ8 is the
   board's.
4. **Angles**: the static reflector protocol, then a calibration from its
   biases (`trackCfg elem`), then the static protocol again, then the
   moving one with `--iq8`.
5. **Shots**: sessions on the adaptive profile with the OPS
   (`ops_validation.py`), a labelled reference session
   (`reference_validation.py`), and only then the confidence thresholds
   (`confidence_calibration`).
6. **Cadence** (phase 30): only after 5 has numbers at 3 ms. Compare 3 ms
   adaptive16 against 2 ms adaptive16 with the same retain widths, never a
   cadence change and a format change in one step; 1 ms needs fewer loops
   than 12 and is a profile experiment of its own
   (`evaluate_iwr_profiles.py` chooses loop counts from real captures).
7. **HWA/DSP** (phase 31): move a stage only when `triggerLog perf` names it
   as the one that does not fit; the residual, the Bartlett search and any
   spin FFT are the candidates, the state machine and the result stay on
   the R4F.
8. **Spin and face** (phases 22-26): record stationary, low-spin and
   high-spin balls on the adaptive profile (the first post frames are
   tagged `L3_RETAIN_SPIN`), first a marked (taped or striped) ball, then
   plain balls, each with a reference spin. Label the ball in the dump
   viewer and run `spin_probe.py --labels --reference-rpm <rpm>` on each
   (and `--iq8` on a fixed bin for the IQ8 question). The rotation scan
   only reports a spin when the window holds 1.5 revolutions: about
   2000-2500 rpm for the 35-47 ms the ball is in view, so a driver's spin
   is at or below the floor while an iron's or a wedge's is not. A
   firmware estimator is written only if the marked ball gives a line at
   the reference and the plain balls separate too, and a face angle only
   if the club's own signature does, never as launch minus path.

### End-state architecture

```text
              IWR6843 ADC
                   |
                   v
             HWA RANGE FFT
                   |
          +--------+--------+
          |                 |
     STATIC PROFILE       MTI (l3_verticalResidual)
          |                 |
          v                 v
    BALL DETECTOR     OBSERVATION LAYER (l3_observation)
     (l3_ball)              |
          |                 v
          |           CLUB TRACKER (l3_club_track + l3_angle)
          |                 |
          |        speed / path / attack (l3_track_delivery)
          |                 |
          +------> IMPACT <-+   range-only impact (l3_impact)
                    |
                    v      shot machine (l3_shot)
               BALL TRACKER (l3_ball_track)
                    |
           speed / HLA / VLA (l3_ball_track_launch)
                    |
                    v
              SPIN PROCESSOR (host spin_probe, experimental)
                    |
                    v
             SHOT VALIDATOR + RESULT PACKET (l3_result)
                    |
                    v
              Raspberry Pi: shot_result.py, ballistics.solve_flight
              with environment.py, delivery.py (face, smash, gates)
```

### Replaying recorded swings

`openflight.iwr6843.firmware_replay` runs a recorded `.l3dump` through the
same compiled modules the board runs. It computes the per-bin observations
exactly as `l3_verticalResidual` does in `l3_dump.c` (vertical TX pair, all
RX, burst-MTI residual, energy, strongest loop, loop 0, lag-1
autocorrelation), takes the watch region from `l3_trig_region`, feeds
`l3_trig_update`, `l3_obs_extract` and `l3_track_update` frame by frame, and
reports what the board would have decided: the fired frame, every trajectory
point, the longest unbroken run of points, acquisitions, coasts and drops,
the share of steps that closed on the destination, and the fitted speed.

```bash
uv run python scripts/analysis/replay_iwr_track.py capture.l3dump --tee-range-m 1.575 --points
uv run python scripts/analysis/replay_iwr_track.py tests/radar/recordings
```

Captures copied into `tests/radar/recordings/` with a `manifest.json` (see
its README) are replayed by `tests/test_iwr6843_firmware_replay.py`, which
also proves the harness on a synthetic swing: one acquisition, no missed
frame, a fitted speed equal to the club's radial speed. Judge a change to the
trigger or the track against every recorded swing there before flashing it;
the acceptance criterion is a continuous approach trajectory without
constant reacquisition.

### Global range bins

Every bin the trigger and the ball detector speak of is a global range-FFT
bin (0..127 on the 128-point FFT over 6 m, 46.9 mm each; bin 34 is 1.59 m),
never a capture-window offset. The window start moves between profiles and
between the pre and post phases (20 on the wide profile's pre window, 47 on
its late window), and a tee bin read as a window offset watched an empty
stretch of air: ten captures put the club's motion at global bins 46-50
(2.2-2.4 m) while `triggerCfg` was watching around bin 34. `triggerCfg`'s
first argument, the `bin=`/`dest=` fields of `triggerLog`, `ball scan` and
`ball status` are all global; the firmware converts to a window offset only
when it indexes a frame (`l3_trig_region`). The host side does the same:
`tee_global_bin` replaces `tee_local_bin`.

### Ball placement detector

```text
ball                 status: state, bin, ratio, confidence, reason, window
ball status          the same plus a balldbg line: centroid, width, persistence
ball scan <bin> <n>  static power of n global bins, pre frames averaged
ball cfg <enable> <follow> [minRatio stableUpdates buildUpdates]
```

The detector is off by default: each poll holds the serial port, a
sound-trigger edge that arrives meanwhile is dropped, and with a golfer over
the ball the detector cannot see it anyway. With `--iwr6843-ball-detector on`
(`follow` also aims the self-trigger at the locked ball; `off` keeps the
configured tee bin) the server turns it on at startup through the capture
worker's job queue, and
polls `ball status` every `--iwr6843-setup-poll-s` seconds
(`openflight.iwr6843.setup_poll`). Each poll becomes an `iwr_setup` socket
event with the detector state, the ball range and the placement advice
(`too-close`, `close`, `ideal`, `far`, `too-far`, with how far to move
OpenFlight); the kiosk shows it as a one-line setup banner above the Live
tiles. A firmware without `ball cfg` fails the job, not startup, and the
banner stays hidden.

The trigger's MTI residual removes a stationary ball entirely, and the
strongest static reflector in the lane is usually furniture (those captures
had one at 2.06 m), so neither motion nor "the biggest static return" finds
the ball. What does is the thing the golfer always does: place it.
`l3_ball.c` keeps a per-bin background of the static (non-MTI) power learned
while the tee is empty, then watches for a compact new reflector against it:

```text
BUILDING   ~64 updates (every other frame) learning the empty background
WAITING    background known, no ball; background keeps adapting slowly
CANDIDATE  a bin rose to minRatio (default 1.0: power doubled) over its
           background, no wider than two bins; the background around it
           is frozen so the ball is not learned in
LOCKED     the candidate held still for stableUpdates (12, ~70 ms): the
           ball's bin is frozen until it leaves, so the club and the
           launched ball cannot drag it; release needs goneUpdates (20)
           consecutive updates under 30 % of the settled return, longer
           than acquisition, so one bad frame moves nothing
```

The status reports a confidence (contrast x width x persistence x range
stability, 0..1), a delta-weighted centroid across the ball's cluster for a
sub-bin range, the width of the rise, how many of the last 50 updates saw
the ball, and why the last update did not lock: `no_delta`, `too_wide`,
`unstable` or `gone`. With `follow` set, the self-trigger aims at the locked
ball's bin instead of the configured tee; with no ball locked it falls back
to the tee, motion only, and counts the frames (`trig dest= source=
fallback=` in `stats`), so the detector cannot make a shot uncapturable
while it is being proven. Every `triggerLog` record and trace line carries
`dest=` and `dist=`, the destination bin and the candidate's distance short
of it, which is what the tracker actually decides on and what makes
captures at 1.4 m and 2.0 m comparable. The setup envelope (too close,
close, ideal, far, too far, and how far to move the unit) is classified on
the host from the detected range, in `openflight.iwr6843.tee_scan`, ready
for the kiosk to show; it is not wired into the UI yet.

`ball scan` is the raw view behind the check suite's `ball-detect` section,
which compares an empty tee with an occupied one and then checks that the
firmware's own detector locked on the same bin. `ball` is entry 18 of the
CLI table; with the mmWave extension's commands that table is at the SDK's
limit, so further diagnostics ride existing commands as sub-modes.

`triggerLog trace` answers the question the log cannot when it stays empty:
did the radar see anything at all? Every frame, the detector notes the
region's strongest bin in a per-bin maximum since arming, and records the
frame whenever that bin reaches twice the floor (well under any usable
`snr`), with its all-loop energy, strongest-loop power, loop-0 power and the
floor. `triggerLog clear` empties the trace and the maxima without touching
the log or the arm. A fire that nobody reads back leaves the ring frozen
with the front end still chirping; `l3release` thaws it, `sensorStop` now
copes with it, and the check suite releases it whenever `stats` show
`latched=1`, after printing the log that explains the fire. `test_iwr_firmware.py --swing` prints both the trace and
the log when a swing does not fire, or when it is interrupted, before its
cleanup disarms and clears them. Read the trace against `floor x snr`: flat
maxima and no entries mean the club was not seen; entries under the
threshold mean it was seen and thresholded out; a large `peak` beside a
modest `energy` means the club is in a bin for part of a frame and the
strongest-loop statistic is the right one.

`Triggered` and the `debugCfg 1` phase lines are not written by the detect
task itself. It queues them and a task at the CLI task's priority writes
them, because a host command arriving mid-line would otherwise let the CLI
task splice its reply into the notice. The host's `cmd()` in turn reads a
reply through the prompt that ends it, not to the first `Done` or `Error`
word, so a reply split across reads cannot leak into the next command's.

The detector has no hardware dependencies, so
`tests/test_iwr6843_firmware_trigger.py` builds it with the host C compiler
and drives synthetic swings, walkers, backswings and noise steps through it.
Change the tracking rules there first.

## Firmware And Host Contract

The wire format is defined in two places that must stay synchronized:

- Firmware: [`iwr6843/dump_format.h`](https://github.com/jewbetcha/openflight/blob/main/firmware/iwr6843/dump_format.h)
- Host parser: [`../src/openflight/iwr6843/dump.py`](https://github.com/jewbetcha/openflight/blob/main/src/openflight/iwr6843/dump.py)

The configurable version 7 transfer contains:

1. A packed 20-byte little-endian `l3_dump_header_t`.
2. A packed 24-byte `l3_temperature_report_t` captured immediately before streaming.
3. A `(start bin, valid bins, elapsed microseconds)` descriptor per frame.
4. A per-frame scale table when `sample_fmt` is IQ8.
5. Complex IQ16 or IQ8 samples ordered by frame, chirp, RX, and local range bin.
6. Each complex sample in TI's native imaginary-then-real order.

The header carries:

| Field | Meaning |
|---|---|
| `magic` | `ILD1` synchronization marker |
| `version` | Dump contract version |
| `n_frames` | Number of ring frames |
| `chirps_per_frame` | `n_tx x loops` |
| `n_tx`, `n_rx` | Virtual-array geometry |
| `n_samples` | Stored bins per chirp/RX for snapshot formats |
| `sample_fmt` | IQ16 or scaled IQ8 variable-width timed range snapshots |
| `trigger_frame` | Oldest circular-ring slot for chronological rotation |
| `frame_period_us` | Frame spacing used by trajectory fitting |

Changing the header, sample order, frame metadata, or sample format requires a
matching host-parser change and regression tests in the same commit.

## Repository Layout

| Path | Responsibility |
|---|---|
| `firmware/iwr6843/l3_dump.c` | RF control, HWA/EDMA pipeline, circular ring, freeze/rearm, CLI, and dump streaming |
| `firmware/iwr6843/l3_observation.c`, `l3_observation.h` | Observation layer: statistic, adaptive floor, target extraction with sub-bin range, coherence, Doppler readout, confidence (host-testable, no hardware) |
| `firmware/iwr6843/l3_trigger.c`, `l3_trigger.h` | Self-trigger front end: watch region, noise floor and raw-input trace; the club track fires (host-testable, no hardware) |
| `firmware/iwr6843/l3_club_track.c`, `l3_club_track.h` | Persistent club trajectory: predictive association, coasting, 3D delivery fit (host-testable, no hardware) |
| `firmware/iwr6843/l3_frames.c`, `l3_frames.h` | Radar and golf coordinate frames, the calibration structure, velocity-angle conventions |
| `firmware/iwr6843/l3_angle.c`, `l3_angle.h` | Azimuth and elevation of a target from its antenna channels, TDM alias resolution |
| `firmware/iwr6843/l3_impact.c`, `l3_impact.h` | Geometric impact detector: closest approach of the club line to the ball, impact time between frames |
| `firmware/iwr6843/l3_shot.c`, `l3_shot.h` | Shot state machine with the frozen impact record |
| `firmware/iwr6843/l3_ball_track.c`, `l3_ball_track.h` | Post-impact ball tracker and launch fit (ball speed, HLA, VLA) |
| `firmware/iwr6843/l3_result.c`, `l3_result.h` | Measurements with confidence, shot validation, the versioned result packet |
| `firmware/iwr6843/l3_profile.c`, `l3_profile.h` | Per-stage cycle counters for `triggerLog perf` |
| `firmware/iwr6843/l3_timing.c`, `l3_timing.h` | Per-frame detect timing: latency, throughput and slot reuse margin (`triggerLog timing`) |
| `firmware/iwr6843/l3_detect_core.c`, `l3_detect_core.h` | Which core scores a frame's bins, fallbacks and the latch (`trackCfg detectCore`) |
| `firmware/iwr6843/l3_adaptive.c`, `l3_adaptive.h` | Capture windows that follow the locked ball between shots |
| `firmware/iwr6843/l3_text.c`, `l3_text.h` | Integer-only fixed-point text for the CLI |
| `firmware/iwr6843/l3_ball.c`, `l3_ball.h` | Ball placement detector: static background, compact-reflector appearance, confidence (host-testable, no hardware) |
| `firmware/iwr6843/dump_format.h` | Packed firmware-side wire contract |
| `firmware/iwr6843/fw_version.h` | Image identity reported by the CLI `stats version` sub-mode |
| `firmware/VERSION` | Release version (semver); names and stamps each release |
| `firmware/iwr6843/makefile` | TI mmWave SDK application build and meta-image generation |
| `firmware/iwr6843/mss.cfg` | SYS/BIOS configuration |
| `firmware/iwr6843/mss_linker.cmd` | Places the ring and optional scratch buffers in L3 RAM |
| `firmware/Makefile` | Toolchain setup and production firmware build target |
| `firmware/releases/` | The single checked-in, validated flash image |
| `firmware/flash_iwr6843.py` | Pi-compatible IWR6843 ROM bootloader client |
| `config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg` | Default wide IQ16 capture profile |
| `config/iwr6843_l3dump_dense_45f2ms_53bin_iq8.cfg` | Dense IQ8 capture profile with a 54 ms ball phase |
| `config/iwr6843_l3dump_dense_36f2ms_53bin_iq8_wide_late.cfg` | Experimental dense IQ8 capture with the wide late-flight window |
| `src/openflight/iwr6843/dump.py` | Python decoder and executable format reference |
| `src/openflight/iwr6843/firmware_host.py` | Host build of the pure-C modules and their ctypes mirrors |
| `src/openflight/iwr6843/firmware_replay.py` | Replays recorded captures through the compiled trigger, trackers, impact detector and shot machine |
| `src/openflight/iwr6843/shot_result.py` | Parses the firmware's result packet into labelled measurements |
| `src/openflight/iwr6843/spin_probe.py` | Experimental spin observable: ball ROI, micro-Doppler spread, rotation-rate scan along a labelled ball track |
| `src/openflight/iwr6843/datasets.py` | Labelled calibration dataset schema and loader (`tests/radar/datasets/`) |
| `src/openflight/environment.py`, `src/openflight/delivery.py` | Air density for the flight model; inferred face, smash and plausibility gates |
| `tests/radar/recordings/` | Recorded `.l3dump` swings with a `manifest.json` for the replay test |
| `src/openflight/iwr6843/firmware_version.py` | Release semver and `stats version` reply parsing; `openflight-firmware` CLI lives in `firmware_cli.py` |

## Where To Build, Flash, And Run

| Operation | Supported environment |
|---|---|
| Build | Native x86_64 Linux or the provided Docker image |
| Build on Apple Silicon | Docker Desktop emulating the x86_64 build image; UTM is a fallback |
| Build on Raspberry Pi 5 | Not currently reliable because TI's x86/i386 installer stubs can fail under QEMU and a 16 KiB host page size |
| Flash | Raspberry Pi using `flash_iwr6843.py`, or TI UniFlash as a fallback |
| Run | Raspberry Pi through OpenFlight |

The Pi can flash and run the image, but it should not be treated as the
canonical compiler host.

## Build On Apple Silicon With Docker

Install Docker Desktop, start its engine, and place the five TI installers
listed below in `firmware/ti_installers/`. The installers are license-gated and
are intentionally excluded from Git.

Build the reusable x86_64 toolchain image once:

```bash
make -C firmware docker-image
```

Build the supported firmware after any source change:

```bash
make -C firmware docker-build
```

Docker runs the same `build-native` recipe under `linux/amd64` and writes the
release artifact back into the host worktree at:

```text
firmware/releases/openflight_iwr6843_v<VERSION>.bin
```

See [Versioning Releases](#versioning-releases) for how `<VERSION>` is set.

Use the UTM workflow below only when Docker emulation is unavailable.

## Build On Apple Silicon With UTM

### 1. Create An x86_64 Debian VM

In UTM:

1. Select **Create a New Virtual Machine**.
2. Select **Emulate**, not Virtualize.
3. Select **Linux** and an amd64 Debian netinst ISO.
4. Use `Intel ICH9 based PC (2009, x86_64)`.
5. Allocate at least 4 GB RAM and 30 GB storage.
6. Install `SSH server` and `standard system utilities`; a desktop is optional.
7. Eject the installer ISO before the first reboot into the installed system.

Confirm the guest architecture and page size:

```bash
uname -m
getconf PAGE_SIZE
```

Expected output is `x86_64` and `4096`.

### 2. Put OpenFlight In The VM

Clone the repository inside the VM or copy your existing worktree with `rsync`:

```bash
sudo apt-get update
sudo apt-get install -y git rsync openssh-server
git clone https://github.com/jewbetcha/openflight.git
cd openflight
```

To copy an existing worktree from the Mac instead:

```bash
rsync -av --exclude '.venv' ~/Projects/openflight/ \
  openflight@VM_ADDRESS:~/openflight/
```

Find the VM address with `ip addr` inside Debian.

### 3. Supply The TI Installers

TI's installers are large and license-gated, so they are intentionally ignored
by git. Download them from TI and place these exact files under
`firmware/ti_installers/` inside the VM:

```text
mmwave_sdk_03_06_02_00-LTS-Linux-x86-Install.bin
ti_cgt_tms470_20.2.7.LTS_linux-x64_installer.bin
bios_6_73_01_01.run
sysconfig-1.10.0_2163-setup.run
xdctools_3_61_00_16_core_linux.zip
```

The current application is MSS/R4F-only, but the SDK install now also enables
the C674x DSP compiler (`cl6x`) and the DSPLIB/MATHLIB C674x libraries for the
in-progress on-chip DSS solve; the build container is correspondingly larger
(roughly +440 MB) than a strictly R4F-only image would be.

Verify the installer set:

```bash
make -C firmware check-installers
```

### 4. Install The Build Environment

Install Debian packages, probe every installer stub, and install the TI tools
under `/opt/ti`:

```bash
make -C firmware install-ti-deps-native
make -C firmware probe-installers-native
make -C firmware install-ti-tools-native
```

The resulting layout is:

```text
/opt/ti/sdk/mmwave_sdk_03_06_02_00-LTS
/opt/ti/sdk/ti-cgt-c6000_8.3.3
/opt/ti/sdk/dsplib_c674x_3_4_0_0
/opt/ti/sdk/mathlib_c674x_3_1_2_1
/opt/ti/cgt-arm/ti-cgt-arm_20.2.7.LTS
/opt/ti/bios/bios_6_73_01_01
/opt/ti/xdc/xdctools_3_61_00_16_core
/opt/ti/sysconfig
```

### 5. Build The Current Firmware

From the repository root inside the VM:

```bash
make -C firmware build-native
```

The target performs the application build, generates the flashable TI
meta-image, and copies the production image into `firmware/releases/`:

```text
firmware/releases/openflight_iwr6843_v<VERSION>.bin
```

Generated `.xer4f`, `.map`, and intermediate `.bin` files stay under
`firmware/iwr6843/` and are ignored by Git. Current production images and
intentional rollback images live under `releases/`.

### 6. Copy Artifacts Out Of The VM

From the Mac:

```bash
mkdir -p artifacts/firmware_build
rsync -av \
  openflight@VM_ADDRESS:~/openflight/firmware/releases/ \
  artifacts/firmware_build/
```

## Versioning Releases

Firmware releases use semantic versioning. `firmware/VERSION` holds
`MAJOR.MINOR.PATCH` and is the single source of truth: the build stamps it into
the image, names the release file after it, and the flashed board reports it
through the CLI `stats version` sub-mode.

| Change | Bump |
|---|---|
| Breaks the host contract: dump/packet layout, a removed or renamed CLI command, changed `.cfg` semantics | `major` |
| Adds a CLI command, capture feature, or backwards-compatible field | `minor` |
| Fixes a bug without changing the host contract | `patch` |

Bump and commit before building, so the stamped commit is the release source
rather than a `-dirty` tree, then commit the new image:

```bash
make -C firmware bump-version PART=minor     # or patch / major
git commit -am "firmware: release v$(cat firmware/VERSION)"
make -C firmware docker-build                # or build-native
git add firmware/releases/ && git commit -m "firmware: add v$(cat firmware/VERSION) image"
```

The build refuses a malformed `VERSION`, and refuses to overwrite a release
file that already exists, so two different images never share a version. Pass
`ALLOW_OVERWRITE=1` only to rebuild an unreleased version in place.

Alongside the version, the build stamps the source commit (`git describe
--always --dirty`, so an image built from uncommitted changes reports
`-dirty`) and the UTC build time. A bare `make` inside `firmware/iwr6843/`
reports `0.0.0-dev` and `unknown` instead.

## Build On Native x86_64 Linux

Use the same installer files and Make targets as the UTM VM. Confirm `uname -m`
reports `x86_64`, then start at **Supply The TI Installers** above.

The tool paths can be overridden when a machine does not use `/opt/ti`:

```bash
make -C firmware build-native \
  TI_ROOT=/custom/ti
```

## Supported Build Target

`make -C firmware build-native` and `make -C firmware docker-build` produce the
same configurable image. Capture timing, frame plan, moving windows, and IQ16
or IQ8 storage are selected by the runtime config. Use Git history for earlier
experiments rather than distributing those images or targets as installation
choices.

## Flash From The Raspberry Pi

The checked-in Python flasher uses the IWR6843 ROM UART bootloader and does not
require TI Cloud Agent. Flash over the CP2105 **Enhanced/UARTA** interface,
normally interface `00` and `/dev/ttyUSB0`. Do not use the Standard interface,
normally `/dev/ttyUSB1`.

### 1. Stop Serial Users

Stop OpenFlight and any calibration or test process using the TI port:

```bash
pgrep -af 'openflight|calibrate|shot_test'
sudo fuser -v /dev/ttyUSB0
```

### 2. Enter Flash Mode And Probe

Set the IWR6843LEVM switches to:

```text
S1.1 ON, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF
```

Start the non-destructive probe:

```bash
uv run python firmware/flash_iwr6843.py \
  --probe \
  --port /dev/ttyUSB0
```

Follow the prompts exactly:

1. Type `READY` so the script opens UART and settles the control lines.
2. Press and release RESET only when requested.
3. Wait one second.
4. Type `PROBE`.

Do not continue until the ROM bootloader handshake passes.

### 3. Flash The Current Image

Leave the board in flash mode and run:

```bash
uv run python firmware/flash_iwr6843.py \
  firmware/releases/l3_dump_configurable_capture_20260818.bin \
  --port /dev/ttyUSB0
```

Type `READY`, press RESET when prompted, wait one second, and type `FLASH`. The
default workflow erases SFLASH, writes acknowledged chunks, closes the image,
and verifies the final ROM bootloader status.

Expected completion:

```text
Erasing existing SFLASH...
Opening firmware image...
Writing firmware...
Writing: 100% (346,820/346,820 bytes)
Closing and verifying firmware...

Flash verified by the IWR6843 ROM bootloader.
```

Do not reset, disconnect, or remove power while erase or write is active. A
failed write is recoverable because the ROM bootloader is not stored in SFLASH.
Leave the board in flash mode and rerun the complete command.

### 4. Return To Functional Mode

Set the switches to:

```text
S1.1 OFF, S1.2 OFF, S1.3 ON, S1.4 ON, S1.5 OFF
```

Press and release RESET. The firmware CLI and binary dumps now share the
Enhanced UART at 1,041,667 baud. Flashing itself always uses the ROM
bootloader's 115,200-baud protocol.

The flasher follows TI application note
[SWRA627, IWR6843 Bootloader Flow](https://www.ti.com/lit/an/swra627/swra627.pdf).

## Verify The Installed Firmware

Ask the board which image it is running (functional mode, OpenFlight stopped):

```bash
uv run openflight-firmware query              # auto-detects the CLI port
uv run openflight-firmware query --port /dev/ttyUSB0
```

```text
Flashed firmware: 1.0.0 (git b6f4c36d2286, hybrid-cadence, built 2026-09-30T15:37:51Z)
```

The same line comes from typing `stats version` at the `l3dump:/>` prompt, and the
CLI banner shows the version and commit at boot. OpenFlight also logs it at
startup (`[IWR6843] Firmware 1.0.0 ...`). An image built before versioning
ignores the argument and prints its counters instead; the query reports it as
unversioned and OpenFlight logs a warning but still starts. `version` is a
sub-mode of `stats` because the CLI table is at the SDK's `CLI_MAX_CMD`.

Run OpenFlight with the matching config as described in the
[Operator Guide](../iwr6843/verify.md#start-openflight). With `--debug`, a
healthy capture reports:

```text
[IWR6843] Trigger #1: dumping firmware-frozen L3 ring
[IWR6843] Capture #1 complete: 732812 bytes
```

The firmware/config geometry is checked at `sensorStart`. A mismatch in TX
masks, loop count, frame count, or ADC samples is rejected rather than silently
capturing a differently shaped cube.

## Changing Capture Geometry

The runtime config controls the capture without rebuilding firmware:

| Config command | Purpose |
|---|---|
| `frameCfg` | TDM loop count and RF frame period |
| `captureFormat iq16\|iq8` | L3 sample representation |
| `iq8Scale 16\|32\|64\|128\|256` | Fixed power-of-two IQ8 quantization scale used by EDMA packing |
| `phaseCaptureCfg` | Pre/impact/ball window starts, widths, counts, and stride |

Before increasing loops, frames, transmitters, or bins, calculate the ring:

```text
IQ16 bytes = TX x loops x frames x RX x saved bins x 4
IQ8 bytes  = TX x loops x frames x RX x saved bins x 2
```

The result must fit within 786,432 L3 bytes; the linker places `.l3ring` in
`L3_RAM` and fails the build if it overflows. The IQ16 ping/pong frame
scratch (`g_iq16FrameScratch`) lives in the `.dataScratch` section in
`DATA_RAM`, not in L3 — it no longer competes with the capture ring for L3
space.

`DATA_RAM` (192 KB) holds that 96 KB scratch, the 32 KB SYS/BIOS heap, and all
of the firmware's static state. The tracker state outgrew what was left, so the
board image compiles some features out through `L3_FEATURE_DEFS` in
`firmware/iwr6843/makefile`. The code stays in the tree, and the host build
(the replay and the tests) keeps all of it:

| Switch | Board | Host | What it drops |
|---|---|---|---|
| `L3_BALL_HYPOTHESES` | `0` | `1` | The ball-hypothesis search. It is off at run time until the recorded captures justify it (~1.6 KB). |
| `L3_BALL_RECOVER` | `0` | `1` | The post-impact target history and the backward recovery of the frames an adopted ball hypothesis missed. Needs `L3_BALL_HYPOTHESES`. ~4.6 KB static: the history and its configuration in the track (2632 B) plus function-static buffers (2016 B: `usable`/`original` in `l3_ball_track_update_joint`, 576 + 32 B; `merged` in `l3_ball_track_adopt`, 1408 B); and 1056 B of stack (`found` in `l3_ball_recover`). Host `ctypes` sizes; a TI link map is still needed for the board figure. |
| `L3_TRIG_LOG_DEPTH` | `48U` | `128U` | Older trigger flight-recorder records (~2.2 KB) |
| `L3_TRIG_TRACE_DEPTH` | `24U` | `64U` | Older trigger raw-input trace entries (~1.3 KB) |

To put a feature back, override the list on the make line and rebuild from
clean. For example: `make clean && make bin
L3_FEATURE_DEFS="--define=L3_BALL_HYPOTHESES=1"`. Check the map's `DATA_RAM`
unused bytes afterwards. `tests/test_iwr6843_firmware_board_image.py` builds
the modules with the makefile's list and checks the result against the host
build.

The firmware rejects invalid windows, frame plans, and L3 budgets at
`sensorStart`. The dense IQ8 profile also has only about 380 microseconds
between its 1.62 ms RF burst and the next 2 ms frame. Its EDMA packer moves the
low byte of each HWA-scaled IQ16 component into the compact ring without a CPU
copy loop. Cadence testing at scale `128` reduced the observed HWA miss rate
from 28.2% with CPU packing to 0.0089% with EDMA packing, with no IQ8 overruns
or EDMA errors. That proves scheduling headroom, but ball-signal fidelity and
launch-angle accuracy must still be validated against the IQ16 baseline.

## Validation Before Flashing A New Variant

Run the firmware contract and host-pipeline tests:

```bash
uv run pytest \
  tests/test_iwr6843_firmware_rearm.py \
  tests/test_iwr6843_firmware_sparse.py \
  tests/test_iwr6843_firmware_trigger.py \
  tests/test_iwr6843_firmware_observation.py \
  tests/test_iwr6843_firmware_club_track.py \
  tests/test_iwr6843_firmware_replay.py \
  tests/test_iwr6843_firmware_frames.py \
  tests/test_iwr6843_firmware_angle.py \
  tests/test_iwr6843_firmware_impact.py \
  tests/test_iwr6843_firmware_shot.py \
  tests/test_iwr6843_firmware_ball_track.py \
  tests/test_iwr6843_firmware_board_image.py \
  tests/test_iwr6843_firmware_result.py \
  tests/test_iwr6843_firmware_profile.py \
  tests/test_iwr6843_pipeline.py \
  tests/test_iwr6843_driver.py \
  tests/test_iwr6843_monitor.py \
  tests/test_iwr6843_bootloader.py
```

Also check:

1. The `.cfg` matches all compile-time capture geometry.
2. The map file keeps `.l3ring` occupying all of `L3_RAM` (0 unused) and
   `.dataScratch` inside `DATA_RAM`, with `DATA_RAM` unused staying above the
   16,384 B floor.
3. The first static capture has the expected version, dimensions, frame period,
   per-frame window table, and total byte count.
4. Repeated dump/rearm cycles work without resetting the board.
5. Vertical and horizontal estimators can replay the new format offline.
6. Source-of-truth testing is repeated if timing, loops, frame spacing, TX
   schedule, or saved range coverage changed.
7. Every capture in `tests/radar/recordings/` still replays as one continuous
   approach track (`scripts/analysis/replay_iwr_track.py tests/radar/recordings`).

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| TI installer exits immediately | Build host is ARM, installer lacks execute permission, or i386 compatibility is missing | Use x86_64 Debian, run `install-ti-deps-native`, then `probe-installers-native` |
| VM returns to the Debian installer | ISO remains attached | Eject the ISO from the UTM CD/DVD drive and reboot |
| `check-installers` reports missing files | Installer name or location differs | Use the exact filenames under `firmware/ti_installers/` |
| Build cannot find `/opt/ti/...` | Tool installation did not complete or uses a custom root | Run `install-ti-tools-native` or pass `TI_ROOT=/custom/ti` |
| Link fails with L3 overflow | Ring or scratch allocation exceeds 768 KiB | Reduce frames, loops, TX count, or saved bins and inspect the map file |
| Probe receives no ROM response | Wrong CP2105 interface or RESET timing | Use Enhanced/UARTA, type `READY`, then RESET only when prompted |
| Flash fails after erase | Image transfer was interrupted | Leave flash mode enabled and rerun the full flash command; the ROM bootloader remains available |
| No CLI after flashing | Board remains in flash mode or was not reset | Restore functional switches and press RESET |
| Server rejects `captureFormat`, `iq8Scale`, or `phaseCaptureCfg` | Older firmware is flashed | Flash `l3_dump_configurable_capture_20260818.bin`, reset in functional mode, and retry |
| Dump length differs from the selected profile | Wrong config, interrupted UART transfer, or stale process | Verify firmware SHA-256, use Enhanced/UARTA, stop serial owners, reset, and retry |
| Dense profile reports sustained `hwa_missed`, `iq8_overrun`, or `iq8_edma_err` | The requested cadence exceeds processing time or EDMA packing failed | Return to the wide profile and inspect `stats`; do not trust descriptor cadence from a missed-frame run |
| `openflight-firmware query` reports an unversioned image | The flashed image predates `stats version` | Build and flash a versioned release |
| Build stops with `... already exists` | `firmware/VERSION` was not bumped since the last release | `make -C firmware bump-version PART=patch`, then rebuild |
| First run works but restart hangs | Retired v1 image or incomplete shutdown | Flash the current release image and reset in functional mode |

## Historical Context

The [IWR6843 field report](../how-it-works/launch-angle.md) explains
why the project moved capture into on-chip L3 and how the estimator evolved.
The implementation has since advanced from full raw ADC rings to HWA-generated,
dynamically windowed complex range snapshots; this README is the authoritative
description of the current firmware.
