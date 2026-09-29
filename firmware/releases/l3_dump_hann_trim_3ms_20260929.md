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
TI toolchain. **Not run on hardware.** The live split trigger is not in this
image; see `docs/superpowers/plans/2026-09-29-iwr6843-split-trigger.md`.
