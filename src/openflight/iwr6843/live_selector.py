"""Deterministic reference for the firmware's bounded live range selector."""

from __future__ import annotations

from dataclasses import dataclass

RETENTION_COMPLETE = 0
RETENTION_TRACK_LOST = 1
RETENTION_AMBIGUOUS = 2
RETENTION_RANGE_EDGE = 3
RETENTION_MARGIN_BINS = 2
MAX_TRACK_HITS = 255


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
