"""Rebuild the firmware shadow selector's input from a retained IQ16 dump.

Mirrors ``l3_storeCompletedScratchFrame`` in ``firmware/iwr6843/l3_dump.c``: sum
|I|+|Q| over every third loop, all TX and RX 0/2, then take the rise over the
previous frame, masked to the active analysis window for frames inside the
pre-trigger/impact interval. Frame 0, and any frame whose predecessor kept a
different bin window, cannot be rebuilt from a retained dump alone.

Also implements the coherent gate evaluated in
``plans/iwr-coherent-gate.md``: a candidate bin is kept only if the coherent
(complex, not rectified) frame-to-frame difference at that bin is a multiple of
the in-window mean coherent difference. Static clutter cancels almost exactly
under coherent differencing; the magnitude-rise detector alone does not.

Run directly to reproduce the false-confirmation counts from that plan:

    uv run python scripts/analysis/iwr_selector_replay.py \\
        openflight_sessions/iwr-adaptive-confirm-soak-no-fail/*.l3dump
"""

from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[2] / "src"))

from openflight.iwr6843.dump import parse_dump  # noqa: E402
from openflight.iwr6843.live_selector import (  # noqa: E402
    RETENTION_COMPLETE,
    SelectorParams,
    SelectorState,
    retention_window,
    select_window,
)

SHADOW_LOOP_STRIDE = 3
SHADOW_RX = (0, 2)
N_BINS = 128


def _rows(cube: np.ndarray, frame: int, count: int) -> np.ndarray:
    """The sampled (chirp, rx, bin) rows the firmware sums for one frame."""
    chirps = [loop * 3 + tx for loop in range(0, 12, SHADOW_LOOP_STRIDE) for tx in range(3)]
    return cube[frame][chirps][:, list(SHADOW_RX), :count]


def load_dump(path: Path | str) -> tuple[dict, np.ndarray, list[int], list[int]]:
    meta, cube = parse_dump(Path(path).read_bytes())[:2]
    n = meta["n_frames"]
    starts = list(meta.get("range_bin_starts") or [meta["range_bin_start"]] * n)
    counts = list(meta.get("range_bin_counts") or [meta["n_samples"]] * n)
    return meta, cube, starts, counts


def _to_window(values: np.ndarray, start: int) -> list[int]:
    power = [0] * N_BINS
    for offset, value in enumerate(values):
        power[start + int(offset)] = int(value)
    return power


def magnitude_rise(cube, frame, starts, counts) -> np.ndarray:
    """The firmware's current selector input: rise in summed |I|+|Q|."""
    now = _rows(cube, frame, counts[frame])
    prev = _rows(cube, frame - 1, counts[frame - 1])
    now_power = (np.abs(now.real) + np.abs(now.imag)).sum(axis=(0, 1))
    prev_power = (np.abs(prev.real) + np.abs(prev.imag)).sum(axis=(0, 1))
    return np.maximum(0, now_power - prev_power)


def coherent_difference(cube, frame, starts, counts) -> np.ndarray:
    """Magnitude of the complex (not rectified) frame-to-frame difference."""
    delta = _rows(cube, frame, counts[frame]) - _rows(cube, frame - 1, counts[frame - 1])
    return (np.abs(delta.real) + np.abs(delta.imag)).sum(axis=(0, 1))


def hybrid_gated_rise(cube, frame, starts, counts, gate_q8: int = 512) -> np.ndarray:
    """Magnitude rise, kept only where the coherent difference clears the gate.

    ``gate_q8 = 512`` requires the coherent difference to be at least 2x the
    in-window mean coherent difference (the plan's recommended factor).
    """
    rise = magnitude_rise(cube, frame, starts, counts)
    coherent = coherent_difference(cube, frame, starts, counts)
    threshold = coherent.mean() * gate_q8 / 256
    return np.where(coherent >= threshold, rise, 0)


def masked_powers(
    detector, meta: dict, cube: np.ndarray, starts: list[int], counts: list[int]
) -> tuple[list[list[int] | None], int | None]:
    """Full 128-bin selector input per frame, matching the firmware's masking.

    Frames below ``pre_frames + impact_frames`` (from the retention report) are
    masked to the fixed impact window, exactly as ``l3_storeCompletedScratchFrame``
    does; frame 0 and any window change are None (not reconstructable offline).

    Returns ``(powers, pre_frames)`` where ``pre_frames`` is the first impact
    frame index (where the firmware resets tracker state, and where the fixed
    analysis window it resets into is read from), or None if the dump has no
    retention report.
    """
    retention = meta.get("retention")
    pre_frames = None
    impact_start = impact_bins = None
    if retention is not None:
        pre_frames = retention["pre_frames"]
        impact_start, impact_bins = starts[pre_frames], counts[pre_frames]

    result: list[list[int] | None] = [None]
    for frame in range(1, meta["n_frames"]):
        same_window = (starts[frame], counts[frame]) == (starts[frame - 1], counts[frame - 1])
        if not same_window:
            result.append(None)
            continue
        values = detector(cube, frame, starts, counts)
        power = _to_window(values, starts[frame])
        # The firmware masks every frame below preFrames + impactFrames to the
        # fixed impact window, not just the impact frames themselves.
        if pre_frames is not None and frame < pre_frames + _impact_frame_count(
            starts, counts, pre_frames
        ):
            for bin_index in range(N_BINS):
                if bin_index < impact_start or bin_index >= impact_start + impact_bins:
                    power[bin_index] = 0
        result.append(power)
    return result, pre_frames


def _impact_frame_count(starts: list[int], counts: list[int], pre_frames: int) -> int:
    """How many frames from ``pre_frames`` share the same fixed impact window."""
    count = 0
    while pre_frames + count < len(starts) and (
        starts[pre_frames + count],
        counts[pre_frames + count],
    ) == (
        starts[pre_frames],
        counts[pre_frames],
    ):
        count += 1
    return count


def run_selector(
    powers: list[list[int] | None],
    params: SelectorParams | None = None,
    reset_at_frame: int | None = None,
) -> list[tuple[int, object]]:
    """Replay the selector frame by frame.

    ``reset_at_frame``, when given, clears tracker state before that frame
    index, matching the firmware's reset of ``gShadowState`` at the first
    impact frame (``slot == gCapturePlan.preFrames``). Returns
    ``(frame_index, result)`` pairs so callers can pick out a specific frame.
    """
    params = params or SelectorParams()
    state = SelectorState()
    results = []
    for frame, power in enumerate(powers):
        if power is None:
            continue
        if frame == reset_at_frame:
            state = SelectorState()
        results.append((frame, select_window(power, params, state)))
    return results


def would_retain_flight_frames(
    detector, meta: dict, cube: np.ndarray, starts: list[int], counts: list[int]
) -> bool:
    """Whether this static capture's tracker would gate open a flight frame.

    Mirrors what actually matters on hardware: the firmware only decides
    retention for frames at/after ``pre_frames + impact_frames``, running
    ``l3_retention_window`` (margin and ambiguity checks included) against
    whatever tracker state the last impact frame left behind. A frame anywhere
    in the pre-trigger interval reporting ``accepted`` is harmless on its own,
    as the 2026-09-26 soak logs confirm (accepted flags inside pre-trigger
    frames, but 0 flight frames kept, in the large majority of cycles).
    """
    powers, pre_frames = masked_powers(detector, meta, cube, starts, counts)
    if pre_frames is None:
        return False
    impact_frames = _impact_frame_count(starts, counts, pre_frames)
    last_impact_frame = pre_frames + impact_frames - 1
    results = run_selector(powers, reset_at_frame=pre_frames)
    for frame, result in results:
        if frame == last_impact_frame:
            reason, _ = retention_window(result, N_BINS)
            return reason == RETENTION_COMPLETE
    return False


def _summarize(paths: list[str]) -> None:
    detectors = {
        "magnitude (firmware)": magnitude_rise,
        "hybrid gated": hybrid_gated_rise,
    }
    for name, detector in detectors.items():
        confirmed = 0
        for path in paths:
            meta, cube, starts, counts = load_dump(path)
            confirmed += would_retain_flight_frames(detector, meta, cube, starts, counts)
        print(f"{name:22s} false-confirmed static captures: {confirmed}/{len(paths)}")


if __name__ == "__main__":
    args = sys.argv[1:] or ["openflight_sessions/iwr-adaptive-confirm-soak-no-fail/*.l3dump"]
    paths = [p for pattern in args for p in sorted(glob.glob(pattern))]
    if not paths:
        raise SystemExit("no .l3dump files matched")
    _summarize(paths)
