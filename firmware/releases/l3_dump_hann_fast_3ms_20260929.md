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

Host tests pass; the image links with the TI toolchain. **Not yet run on the
rig — the armed soak with `_hann.cfg` needs to be repeated on this image**
before the `DO NOT USE` warning is removed from
`config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16_hann.cfg`. This fix removes
the known 2x redundant-computation cost; it has not been measured to confirm
the windowed path now fits the 3 ms deadline on real hardware.
