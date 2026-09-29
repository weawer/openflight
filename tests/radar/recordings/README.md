# IWR6843 swing recordings

Recorded `.l3dump` captures from a real rig, replayed through the firmware's
own trigger, observation layer and club track by
`tests/test_iwr6843_firmware_replay.py` and
`scripts/analysis/replay_iwr_track.py`. The replay computes exactly the
per-bin observations the R4F computes (`l3_verticalResidual` in
`firmware/iwr6843/l3_dump.c`) and feeds them to the compiled C modules, so a
change to `l3_trigger.c`, `l3_observation.c` or `l3_club_track.c` can be
judged against every swing here before it is flashed.

The captures here are the swings the trackers were tuned on (see
`manifest.json` for what each one holds and what a replay must reproduce).
To add captures, copy them from the Pi's session directory
(`~/openflight_sessions/iwr6843_<timestamp>_<seq>.l3dump`) and describe them
in `manifest.json`:

```json
{
  "default": {"tee_bin": 34, "snr": 6.0, "track_frames": 2, "stat": "peak"},
  "iwr6843_20260920_181204_003.l3dump": {"dest_bin": 46, "notes": "ball locked at 2.16 m"},
  "iwr6843_20260920_181330_004.l3dump": {"notes": "practice swing, no ball"}
}
```

An `expect` entry (per file or in `default`) states ranges the replay must
land in; the test asserts them and the script prints `expectations: ok` or
the failures and exits 2:

```json
{
  "iwr6843_20260920_181204_003.l3dump": {
    "dest_bin": 46,
    "expect": {"impact_frame": [10, 12], "club_direction": "approaching",
               "club_points_min": 7, "acquisitions_max": 1,
               "ball_origin_bin": [47, 49], "ball_speed_mps": [55, 75]}
  }
}
```

For captures the sound trigger froze (everything recorded before the
self-trigger), set `post_from_frame` to the plan's pre frame count (9 on the
wide profile): the recorded freeze is the impact, so the ball tracker is
judged on the true post frames whatever the range gate did earlier. Set
`dest_bin` to where the ball actually was when the configured tee was wrong.

Keys: `fires`, `impact_frame`, `geometric_frame`, `club_points_min`,
`club_direction` ("approaching"), `acquisitions_max`, `ball_origin_bin`,
`ball_speed_mps`, `club_speed_mps`.

`ball_origin_bin` is the first point the core ball track appended. With the
hypothesis search on (the firmware default since 2026-09-29) that is the
first point after a line wins, typically 3-4 frames past the tee, not the
tee itself; the launch still fits the winning line's earlier points.

Every key other than `notes` and `expect` is a `ReplayConfig` field
(`openflight.iwr6843.firmware_replay`). `tee_bin` and `dest_bin` are GLOBAL
range-FFT bins (bin = range / (6 m / 128) on the shipped profiles; bin 34 is
1.59 m). `dest_bin` is the ball detector's locked bin when the firmware was
following it; without it the tee bin is the destination, as on the board.

Run the replay by hand with:

```bash
uv run python scripts/analysis/replay_iwr_track.py tests/radar/recordings --points
```

The acceptance criterion for the club track is a continuous approach
trajectory: one acquisition per swing, a longest run covering the approach,
and a fitted speed in the range a clubhead reaches. The test asserts the
weaker, capture-independent form (a track exists and did not reacquire more
than once); read the report for the rest.
