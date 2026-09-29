# l3_dump_hann_trim_3ms_20260929.bin

SHA-256: `757aab5f070f42a416e4d7766ada0b1892dde7fb778719755272a00e1d332230`

Built on `test_build` from Cormac's `feat/iwr-calcs` (`bb06293e`) with:

- Host reliability fixes, soak `scratch_stale` gate and trigger tools from
  `feat/iwr-2ms-pipeline` (tools default to the 3 ms adaptive profile).
- Detect task above CLI output (`71214121`).
- `l3sparse`/`l3track` compiled out (`L3_SPARSE_READBACK`); DATA_RAM free
  23,814 B (was 1,382 B at `bb06293e`).
- Range window for the detector: `trackCfg window none|hann`, reset by
  `sensorStop`. Stored samples and angle snapshots are never windowed.

Runtime profiles, 3 ms frames:

| Config | Detector window |
|---|---|
| `config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg` | none |
| `config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16_hann.cfg` | Hann |

Status: host tests pass (IWR, swing and soak suites); the image links with the
TI toolchain. The live split trigger is not in this image; see
`docs/superpowers/plans/2026-09-29-iwr6843-split-trigger.md`.

## Hardware evidence (2026-09-29, jamorro@Openflight rig)

- **Plain profile (no window): armed soak clean.** 1000-frame run on
  `iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg` at tee 1.845 m passed with no
  `scratch_stale`, no missed frames, no false trigger.
- **Hann profile: FAILS the armed soak.** Same rig, same tee, `_hann.cfg`:
  `scratch_stale=13` of 106 frames captured, then a false self-trigger latch
  (`active=0 latched=1`) with a still scene, ending the soak early. The
  windowed residual reads each bin's two neighbours as well as itself (about
  3x the memory reads of the plain path per bin/channel/loop) across the full
  53-bin wide processing region every frame; this misses the 3 ms detect-task
  deadline often enough to corrupt the trigger's floor tracking.
- **Conclusion: do not arm the `_hann` profile on the rig until the kernel is
  narrowed or cheapened.** The plain profile is unaffected (Hann is off by
  default; `gRangeWindow` only changes when a cfg explicitly sends `trackCfg
  window hann`) and is safe to keep using for trigger-reliability testing.
