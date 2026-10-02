---
icon: lucide/ruler
---

# Mount, Aim, and Measure

Where the radar sits and how it is angled determines every launch angle it
reports. Measure the geometry rather than estimating it — the numbers you
record here go straight into the runtime.

## Mount And Aim The Radar

Mount the radar behind the ball with the antenna face pointing down the target
line. The validated enclosure rotates the board so its vertical virtual array
is physically vertical, with the TX antennas above the RX antennas.

The IWR6843 can start at approximately the same upward tilt as the OPS243,
typically around 10 degrees, when both antenna faces are mounted parallel. Treat
10 degrees as a mounting starting point, not a universal calibration value.
Measure the IWR6843 antenna-face tilt independently and enter that measured
value in OpenFlight.

Start with the IWR6843 antenna center approximately 6 inches (`0.1524 m`) above
the floor surface under the radar. Measure vertically from that surface to the
center of the antenna array, not to the enclosure bottom or mounting feet. This
is the validated starting height, not a substitute for entering the actual
measured height.

Mounting requirements:

- Aim the antenna face toward the intended start line, not diagonally across
  the hitting area.
- Keep the antenna face unobstructed.
- Keep the board rotation consistent with the validated enclosure.
- Use a rigid mount. Small mechanical shifts can appear as angle bias.
- Measure tilt against the antenna face or a known-parallel enclosure surface.
- Re-measure after moving to a different floor, mat, bay, or stand.

A corner reflector placed on the target line can verify horizontal aim. It is
useful for alignment and static health checks, but it does not replace moving
golf-ball validation.

## Measure The Geometry

OpenFlight needs these physical inputs:

| Argument | Measurement |
|---|---|
| `--iwr6843-tee-m` | Distance from the enclosure front to the ball center (the array's depth is added for you) |
| `--iwr6843-net-m` | Distance from the enclosure front to the net or screen (the array's depth is added for you) |
| `--iwr6843-tilt-deg` | Antenna-face mount tilt from an inclinometer |
| `--iwr6843-radar-height-m` | Antenna-center height above the floor reference |
| `--iwr6843-ball-height-m` | Ball-center height above the same floor reference |

Measurement guidance:

- Measure `tee-m` from the enclosure front to the centre of the ball, along the line the radar looks down. The array sits 30 mm behind the front (`ARRAY_DEPTH_M` in `iwr6843/calibration.py`) and the software adds that itself, so do not add it to the tape reading. Measure to the ball, not the golfer's feet: the golfer stands beside the ball and is a strong standing return of their own, and their feet are not where the club meets the ball.
- Measure `net-m` from the enclosure front too. It only matters when hitting into a net (`--iwr6843-flight net`, the default), where ball tracks stop 0.25 m short of it; in `range` and `course` flight it is ignored.
- Measure the radar height and the tilt from the antenna center, not the enclosure edge or mounting feet.
- The horizontal distance to the ball is fine for `tee-m`: at 2 m the ball's height above the radar changes the slant range by under a centimetre.
- Keep `net-m` honest so late net reflections can be excluded.
- Measure radar and ball height from the same floor reference. If the radar and
  ball sit on different surfaces, extend a common level reference between them.
- Add an elevated mat to ball height. A 1 inch mat adds approximately `0.0254 m`.
- A typical iron ball center is around `0.040 m`; a driver tee is higher.
- Do not reuse a tilt value after moving the rig unless you verify it again.

The checked-in reference calibration contains the array correction used by the
validated radar. It provides a known starting point, not a universal factory
calibration. The operator calibration session below checks geometry and
estimator consistency; it does not regenerate the file's per-element complex
array correction. A different radar board or antenna orientation may require a
new corner-reflector array calibration before source-of-truth accuracy can be
expected.

