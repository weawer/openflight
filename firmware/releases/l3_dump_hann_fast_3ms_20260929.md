# l3_dump_hann_fast_3ms_20260929.bin

SHA-256: `191355c27263f4033b2ad74d981d868303e27e0376a4adc8e2d21ea022e49c85`

Supersedes `l3_dump_hann_trim_3ms_20260929.bin`. Same base and features (see
that release note), plus a fix for the deadline miss it recorded.

## The bug

`l3_iq16_channel_stats_windowed`'s two-pass mean/residual algorithm called
`l3_iq16_sample` twice per loop: once to accumulate the mean, once again in
the residual pass, re-deriving the identical windowed value both times. For
`L3_RANGE_WINDOW_NONE` that redundant read was cheap and harmless. For
`L3_RANGE_WINDOW_HANN` (3 neighbouring reads plus a combine, instead of 1
plain read) it meant that cost ran twice per sample, which is what missed the
3 ms detect-task deadline on the rig (`scratch_stale=13` of 106 frames,
armed soak, `l3_dump_hann_trim_3ms_20260929.bin`). The IQ8/float fallback
path in `l3_verticalResidual` (`firmware/iwr6843/l3_dump.c`) had the same
pattern with `l3_ringComponentWindowed`.

## The fix

Both paths now read each loop's windowed value once, into a small
loop-count-sized cache (`valueIm`/`valueRe`), and the residual pass reads
the cache instead of re-deriving the value. `L3_RANGE_WINDOW_NONE` is
unaffected (same single read either way). DATA_RAM cost: negligible
(stack-local arrays, `L3_MAX_LOOPS` x 2 x 4 bytes each); free margin
unchanged from the previous image (23,814 B).
`tests/test_iwr6843_firmware_iq16_stats.py::test_the_windowed_sample_is_read_once_per_loop_not_twice`
pins the single-read invariant by source inspection, and
`tests/test_iwr6843_firmware_sparse.py::test_residual_walks_loops_by_stride_instead_of_recomputing_indices`
covers the float path the same way.

## Status

Host tests pass; the image links with the TI toolchain.

**Armed soak passed on the rig (2026-09-29, jamorro@Openflight):**
`config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16_hann.cfg`, tee 1.845 m,
1000-frame request: `frames=1046 missed=0 rate=0.000000% scratch_stale=0`,
`PASS`. The `iq8_overrun`/`iq8_edma_err` fields in the stats line are always
printed regardless of capture format; this profile is `adaptive16` (IQ16),
and the self-trigger only arms on IQ16 profiles, so the windowed IQ16 fast
path (`l3_iq16_channel_stats_windowed`, not the IQ8/float fallback) is what
was exercised and fixed.

The `DO NOT USE` warning is lifted from the `_hann` config. Still open:
real-swing trigger behaviour and ball-track quality with Hann on — the
armed soak only proves the timing budget, not detection.
