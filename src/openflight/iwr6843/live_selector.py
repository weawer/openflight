"""Deterministic reference for the firmware's bounded live range selector."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SelectorParams:
    window_bins: int = 12
    max_jump_bins: int = 4
    max_misses: int = 2
    snr_q8: int = 768


@dataclass
class SelectorState:
    selected_bin: int = 0
    velocity_q8: int = 0
    misses: int = 0
    active: int = 0

    def copy(self) -> SelectorState:
        return SelectorState(
            self.selected_bin, self.velocity_q8, self.misses, self.active
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


def _window_start(selected_bin: int, window_bins: int, total_bins: int) -> int:
    return max(0, min(selected_bin - window_bins // 2, total_bins - window_bins))


def select_window(
    powers: list[int], params: SelectorParams, state: SelectorState
) -> SelectorResult:
    """Select at most two peaks and update a constant-velocity association."""
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
    before_bin = state.selected_bin
    chosen: int | None = None
    if candidates and not state.active:
        chosen = candidates[0]
    elif candidates:
        predicted_q8 = state.selected_bin * 256 + state.velocity_q8
        eligible = [
            index
            for index in candidates
            if index >= state.selected_bin - 1
            and abs(index - state.selected_bin) <= params.max_jump_bins
            and abs(index * 256 - predicted_q8) <= params.max_jump_bins * 256
        ]
        if eligible:
            chosen = min(
                eligible,
                key=lambda index: (
                    abs(index * 256 - predicted_q8), -powers[index], index
                ),
            )
    if chosen is None:
        state.misses += 1
        selected = state.selected_bin
        state.velocity_q8 = 0
        if state.misses > params.max_misses:
            state.active = 0
            state.velocity_q8 = 0
    else:
        if state.active:
            state.velocity_q8 = int(
                (state.velocity_q8 + (chosen - before_bin) * 256) / 2
            )
        else:
            state.velocity_q8 = 0
        state.selected_bin = chosen
        state.misses = 0
        state.active = 1
        selected = chosen
    confidence = min(65535, powers[chosen] * 256 // noise) if chosen is not None else 0
    ambiguous = (
        len(candidates) == 2
        and powers[candidates[1]] * 256 >= powers[candidates[0]] * 230
    )
    return SelectorResult(
        candidate_bins=tuple(candidates),
        candidate_power=tuple(powers[index] for index in candidates),
        noise=noise,
        selected_bin=selected,
        window_start=_window_start(selected, params.window_bins, len(powers)),
        window_bins=params.window_bins,
        confidence_q8=confidence,
        accepted=chosen is not None,
        ambiguous=ambiguous,
    )
