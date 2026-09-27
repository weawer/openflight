"""Deterministic reference for the firmware's bounded live range selector."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RETENTION_COMPLETE = 0
RETENTION_TRACK_LOST = 1
RETENTION_AMBIGUOUS = 2
RETENTION_RANGE_EDGE = 3
RETENTION_MARGIN_BINS = 2
MAX_TRACK_HITS = 255
RANGE_FFT_SIZE = 128
# l3_storeCompletedScratchFrame sums every third loop and every second RX.
SHADOW_LOOP_STRIDE = 3
SHADOW_RX = (0, 2)
# l3_dump.c L3_COHERENT_GATE_Q8: keep a rise bin only at >= 2x the mean coherent change.
COHERENT_GATE_Q8 = 512


@dataclass(frozen=True)
class SelectorParams:
    window_bins: int = 12
    max_jump_bins: int = 4
    max_misses: int = 2
    snr_q8: int = 768
    # Consecutive associated frames before a track is trusted. Recorded static
    # scenes produced two-frame noise chains; a real target needs three.
    confirm_frames: int = 3


@dataclass
class SelectorState:
    selected_bin: int = 0
    velocity_q8: int = 0
    misses: int = 0
    active: int = 0
    hits: int = 0

    def copy(self) -> SelectorState:
        return SelectorState(
            self.selected_bin, self.velocity_q8, self.misses, self.active, self.hits
        )


@dataclass(frozen=True)
class SelectorResult:
    candidate_bins: tuple[int, ...]
    candidate_power: tuple[int, ...]
    noise: int
    selected_bin: int
    window_start: int
    window_bins: int
    confidence_q8: int
    accepted: bool
    ambiguous: bool
    held_bin: int = 0
    coasting: bool = False


def _window_start(selected_bin: int, window_bins: int, total_bins: int) -> int:
    return max(0, min(selected_bin - window_bins // 2, total_bins - window_bins))


def _trunc_div(numerator: int, denominator: int) -> int:
    """C integer division: truncate toward zero."""
    quotient = abs(numerator) // abs(denominator)
    return quotient if (numerator >= 0) == (denominator > 0) else -quotient


def _start_track(state: SelectorState, chosen: int) -> None:
    state.selected_bin = chosen
    state.velocity_q8 = 0
    state.misses = 0
    state.active = 1
    state.hits = 1


def _drop_track(state: SelectorState) -> None:
    state.active = 0
    state.hits = 0
    state.velocity_q8 = 0


def select_window(
    powers: list[int], params: SelectorParams, state: SelectorState
) -> SelectorResult:
    """Select at most two peaks and update a constant-velocity association.

    A new track is tentative until ``confirm_frames`` consecutive associations;
    only confirmed frames are accepted. A confirmed track coasts on its velocity
    through up to ``max_misses`` missed frames, reporting the predicted bin.
    """
    if not powers or not 0 < params.window_bins <= len(powers):
        raise ValueError("invalid selector layout")
    noise = max(1, sum(powers) // len(powers))
    candidates = [
        index
        for index in range(1, len(powers) - 1)
        if powers[index] >= powers[index - 1]
        and powers[index] >= powers[index + 1]
        and powers[index] * 256 >= noise * params.snr_q8
    ]
    candidates.sort(key=lambda index: (-powers[index], index))
    candidates = candidates[:2]
    chosen: int | None = None
    if state.active:
        steps = state.misses + 1
        predicted_q8 = state.selected_bin * 256 + state.velocity_q8 * steps
        eligible = [
            index
            for index in candidates
            if index >= state.selected_bin - 1
            and abs(index - state.selected_bin) <= params.max_jump_bins * steps
            and abs(index * 256 - predicted_q8) <= params.max_jump_bins * 256
        ]
        if eligible:
            chosen = min(
                eligible,
                key=lambda index: (abs(index * 256 - predicted_q8), -powers[index], index),
            )
            step_q8 = _trunc_div((chosen - state.selected_bin) * 256, steps)
            state.velocity_q8 = _trunc_div(state.velocity_q8 + step_q8, 2)
            state.selected_bin = chosen
            state.misses = 0
            state.hits = min(MAX_TRACK_HITS, state.hits + 1)
    confirmed_before = state.active and state.hits >= params.confirm_frames
    coasting = False
    selected = state.selected_bin
    if chosen is None and candidates and not confirmed_before:
        # No track, or a tentative one that failed to associate: restart on the
        # strongest candidate rather than spending a frame on the stale guess.
        chosen = candidates[0]
        _start_track(state, chosen)
    elif chosen is None and not state.active:
        state.misses += 1
    elif chosen is None and not confirmed_before:
        _drop_track(state)
        state.misses = 0
    elif chosen is None:
        state.misses += 1
        if abs(state.velocity_q8) > params.max_jump_bins * 256:
            state.velocity_q8 = 0  # not produced by association: stale
        if state.misses > params.max_misses:
            _drop_track(state)
        else:
            coasting = True
            predicted_q8 = state.selected_bin * 256 + state.velocity_q8 * state.misses
            predicted_q8 = max(0, min(predicted_q8, (len(powers) - 1) * 256))
            selected = (predicted_q8 + 128) // 256
    if chosen is not None:
        selected = chosen
    accepted = chosen is not None and state.hits >= params.confirm_frames
    confidence = min(65535, powers[chosen] * 256 // noise) if accepted else 0
    ambiguous = len(candidates) == 2 and powers[candidates[1]] * 256 >= powers[candidates[0]] * 230
    return SelectorResult(
        candidate_bins=tuple(candidates),
        candidate_power=tuple(powers[index] for index in candidates),
        noise=noise,
        selected_bin=selected,
        window_start=_window_start(selected, params.window_bins, len(powers)),
        window_bins=params.window_bins,
        confidence_q8=confidence,
        accepted=accepted,
        ambiguous=ambiguous,
        held_bin=state.selected_bin,
        coasting=coasting,
    )


def retention_window(result: SelectorResult, n_bins: int) -> tuple[int, int]:
    """Return ``(reason, window_start)`` for retaining a flight frame.

    Mirrors ``l3_retention_window``: the window must keep a two-bin margin
    around the accepted target, every ambiguous candidate, or, while coasting,
    both the last measured bin and the prediction.
    """
    margin = RETENTION_MARGIN_BINS
    if not result.accepted and not result.coasting:
        return RETENTION_TRACK_LOST, result.window_start
    low = high = result.selected_bin
    if result.coasting:
        low, high = min(low, result.held_bin), max(high, result.held_bin)
    elif result.ambiguous:
        low = min(low, *result.candidate_bins)
        high = max(high, *result.candidate_bins)
    if low < margin or high + margin >= n_bins:
        return RETENTION_RANGE_EDGE, result.window_start
    if high - low + 2 * margin + 1 > result.window_bins:
        reason = RETENTION_TRACK_LOST if result.coasting else RETENTION_AMBIGUOUS
        return reason, result.window_start
    start = min(result.window_start, low - margin)
    if start + result.window_bins <= high + margin:
        start = high + margin + 1 - result.window_bins
    return RETENTION_COMPLETE, start


def coherent_gate(
    rise: list[int], coherent: list[int], gate_q8: int = COHERENT_GATE_Q8
) -> list[int]:
    """Suppress rise bins whose coherent difference is not well above average.

    ``rise`` is the existing magnitude-rise selector input; ``coherent`` is the
    magnitude of the complex (not rectified) frame-to-frame difference at each
    bin, over the same window. Static and quasi-static clutter cancels almost
    exactly under coherent differencing (recorded static captures never
    exceeded 1.51x their in-window mean); a real departing target does not,
    because the magnitude-rise detector that finds it is already looking at a
    fall in rectified amplitude, not a bin a target merely moved through.

    ``gate_q8 = 512`` requires at least 2x the mean coherent difference over
    ``coherent`` (plans/iwr-coherent-gate.md); every recorded static capture
    stayed under that, and it kept slightly more injected synthetic targets
    than the ungated detector in an offline A/B.
    """
    if len(rise) != len(coherent):
        raise ValueError("rise and coherent must cover the same bins")
    mean_coherent = max(1, sum(coherent) // len(coherent))
    threshold = mean_coherent * gate_q8 // 256
    return [value if coherent[index] >= threshold else 0 for index, value in enumerate(rise)]


def _shadow_frames(meta: dict, cube: np.ndarray):
    """Sampled rows (every third loop, all TX, RX 0/2) and window per frame."""
    frames = meta["n_frames"]
    n_tx = meta["n_tx"]
    loops = meta["chirps_per_frame"] // n_tx
    starts = meta.get("range_bin_starts") or [meta["range_bin_start"]] * frames
    counts = meta.get("range_bin_counts") or [meta["n_samples"]] * frames
    chirps = [
        loop * n_tx + tx for loop in range(0, loops, SHADOW_LOOP_STRIDE) for tx in range(n_tx)
    ]
    for frame in range(frames):
        rows = cube[frame][chirps][:, list(SHADOW_RX), : counts[frame]]
        yield rows.reshape(-1, counts[frame]), starts[frame], counts[frame]


def _l1(values: np.ndarray) -> np.ndarray:
    return (np.abs(values.real) + np.abs(values.imag)).sum(axis=0)


def _placed(start: int, values) -> list[int]:
    power = [0] * RANGE_FFT_SIZE
    for offset, value in enumerate(values):
        power[start + offset] = int(value)
    return power


def selector_powers(meta: dict, cube: np.ndarray) -> list[list[int] | None]:
    """Rebuild the firmware's magnitude-rise input from a parsed IQ16 dump.

    Mirrors ``l3_storeCompletedScratchFrame``: sum |I|+|Q| over every third
    loop, all TX and RX 0/2, then keep only the rise over the previous frame,
    placed at global range bins. Frames whose predecessor retained a different
    bin window cannot be rebuilt and are None, as is frame 0. This is the
    input before the coherent gate; see ``gated_selector_inputs``.
    """
    frames = list(_shadow_frames(meta, cube))
    sums = [_placed(start, _l1(rows)) for rows, start, _count in frames]
    rises: list[list[int] | None] = [None]
    for frame in range(1, len(frames)):
        same_window = frames[frame][1:] == frames[frame - 1][1:]
        rises.append(
            [max(0, now - before) for now, before in zip(sums[frame], sums[frame - 1])]
            if same_window
            else None
        )
    return rises


def gated_selector_inputs(
    meta: dict, cube: np.ndarray, gate_q8: int = COHERENT_GATE_Q8
) -> list[list[int] | None]:
    """The rise after the firmware's coherent gate: what ``l3_live_select`` sees.

    The coherent difference is |dI|+|dQ| of the same sampled rows between
    consecutive frames. The firmware takes the gate's mean over its analysis
    window (the impact bins, or all 128 in flight); a dump only has the
    retained window, so the mean here is over that window instead.
    """
    frames = list(_shadow_frames(meta, cube))
    gated: list[list[int] | None] = []
    for frame, rise in enumerate(selector_powers(meta, cube)):
        if rise is None:
            gated.append(None)
            continue
        rows, start, count = frames[frame]
        coherent = _l1(rows - frames[frame - 1][0]).astype(int).tolist()
        kept = coherent_gate(rise[start : start + count], coherent, gate_q8)
        gated.append(_placed(start, kept))
    return gated
