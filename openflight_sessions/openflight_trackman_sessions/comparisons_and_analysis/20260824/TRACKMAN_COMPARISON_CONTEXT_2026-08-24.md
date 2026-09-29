# August 24, 2026 TrackMan Comparison Context

The canonical aligned dataset is:

`/Users/john.pacino/openflight_sessions/trackman/20260824/trackman_openflight_aligned_2026-08-24.csv`

## Geometry Definitions

- `physical_tx2_height_in` and `physical_tx2_height_m` are the measured height of the center of the IWR6843LEVM TX2 antenna above the floor. They are not the height of the enclosure bottom.
- `of_configured_radar_height_*` is the height passed to OpenFlight during the live session.
- `of_configured_ball_height_*` is the configured ball-center height above the floor at launch.
- `of_configured_tee_slant_range_*` is the configured slant distance from the TI radar phase center to the launch point.

## Physical Height A/B

| Block | Physical TX2 height | Aligned rows | Assignment |
|---|---:|---:|---|
| `low_2.875in` | 2.875 in / 0.073025 m | 36 | Preferred low-enclosure position |
| `high_5.25in` | 5.25 in / 0.133350 m | 48 | Radar and camera raised together |
| Unassigned | Unknown | 7 | Warm-up or transition captures outside the controlled height blocks |

The row-level `height_assignment_basis` column records the club/session boundary used for every assigned shot.

## Important Live-Configuration Mismatch

All sessions were run with these configured values:

- TI radar height: 6.500 in / 0.165100 m
- Ball-center launch height: 1.575 in / 0.040000 m
- Radar-to-tee slant range: 60.000 in / 1.524000 m

The live TI radar-height input therefore did not match either measured physical height. Offline geometry comparisons must use `physical_tx2_height_m` rather than `of_configured_radar_height_m`.

## Height Assignment Summary

- 9-iron low block: before the noon elevation change.
- 9-iron high block: after the 12:03 restart.
- 5-iron high block: after the 12:03 restart.
- 5-iron low block: second-session low-height segment.
- Driver low block: before shot 24 in the driver transition session.
- Driver high block: shot 24 onward.

The CSV remains the source of truth for individual rows and artifact paths.
