"""Host replay of the firmware ball-leave detector.

``l3_considerSelfTrigger`` in ``firmware/iwr6843/l3_dump.c`` watches loop-0
vertical residual power. This steps the same state machine over a saved dump
so a swing can be checked without the board.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.dump import parse_dump, range_data
from openflight.iwr6843.shot import geometry_from_header
from openflight.iwr6843.sparse import vertical_loop_power

# Clubhead is short of the ball. Twelve bins is about 0.6 m on the wide profile.
APPROACH_BINS = 12
# Same names ``l3_triggerPhaseName`` prints on the debug UART.
PHASES = (
    "off",
    "no-frame",
    "bin-outside",
    "tee-low",
    "occupying",
    "watching",
    "no-approach",
    "toward",
    "away",
    "fired",
)
# Production ``triggerCfg`` defaults used with the wide profile.
DEFAULT_LEVEL = 1000.0
DEFAULT_HITS = 2


@dataclass(frozen=True)
class TriggerObservation:
    """One frame of the detector, matching a ``trig phase=...`` debug line."""

    frame: int
    phase: str
    tee: float
    approach: float
    ready: bool
    toward: bool
    away: bool
    run: int
    peak_bin: int
    have_peak: bool

    @property
    def fired(self) -> bool:
        """True on the frame that latches the freeze."""
        return self.phase == "fired"


class BallLeaveDetector:
    """Per-frame state machine from ``l3_considerSelfTrigger``."""

    def __init__(
        self,
        level: float = DEFAULT_LEVEL,
        hits: int = DEFAULT_HITS,
        approach_bins: int = APPROACH_BINS,
        frame_period_s: float = 0.003,
    ) -> None:
        if level <= 0.0:
            raise ValueError(f"self-trigger level must be > 0, got {level}")
        if hits < 1:
            raise ValueError(f"self-trigger hits must be >= 1, got {hits}")
        if approach_bins < 1:
            raise ValueError(f"approach bins must be >= 1, got {approach_bins}")
        if not 0.0 < frame_period_s <= 0.06:
            raise ValueError("frame period must be positive and at most 60 ms")
        self.frame_period_s = frame_period_s
        self.level = float(level)
        self.hits = int(hits)
        self.approach_bins = int(approach_bins)
        self._tee = 0.0
        self._clear()
        self._latched = False
        self._approach_power = 0.0

    def _clear(self) -> None:
        self._ready = False
        self._toward = False
        self._away = False
        self._run = 0
        self._peak_bin = 0
        self._have_peak = False
        self._departure_bin = None
        self._motion_frames = 0
        self._missed_frames = 0

    def step(
        self,
        frame: int,
        power: np.ndarray,
        tee_local: int | None,
        valid_bins: int,
    ) -> TriggerObservation:
        """Advance one frame. ``tee_local`` is None when that bin was not stored."""
        if self._latched:
            return self._observe(frame, "fired", float(self._tee), self._approach_power)
        if tee_local is None or tee_local < 0 or tee_local >= valid_bins or tee_local >= len(power):
            self._clear()
            return self._observe(frame, "bin-outside", 0.0, 0.0)
        tee = float(power[tee_local])
        self._tee = tee
        if self._toward:
            self._motion_frames += 1
            if self._motion_frames * self.frame_period_s > 0.06:
                self._clear()
        if tee < self.level and not self._toward:
            self._clear()
            self._approach_power = 0.0
            return self._observe(frame, "tee-low", tee, 0.0)
        if not self._ready:
            self._run += 1
            if self._run >= self.hits:
                self._ready = True
            return self._observe(frame, "watching" if self._ready else "occupying", tee, 0.0)
        if self._toward:
            last = min(valid_bins, len(power), tee_local + 1 + self.approach_bins)
            past = power[tee_local + 1 : last]
            if past.size and float(np.max(past)) >= self.level:
                past_bin = tee_local + 1 + int(np.argmax(past))
                if self._departure_bin is not None and past_bin > self._departure_bin:
                    self._latch()
                    return self._observe(frame, "fired", tee, float(np.max(past)))
                if self._departure_bin is not None and past_bin < self._departure_bin:
                    self._clear()
                    return self._observe(frame, "watching", tee, 0.0)
                self._departure_bin = past_bin
                self._missed_frames = 0
                return self._observe(frame, "away", tee, float(np.max(past)))
        self._departure_bin = None
        first = max(0, tee_local - self.approach_bins)
        approach = power[first:tee_local]
        if not approach.size or float(np.max(approach)) < self.level:
            self._missed_frames += 1
            if self._missed_frames > 2:
                self._clear()
            return self._observe(frame, "no-approach", tee, 0.0)
        self._missed_frames = 0
        peak_bin = first + int(np.argmax(approach))
        peak = float(power[peak_bin])
        if self._have_peak and peak_bin > self._peak_bin:
            self._toward = True
        elif self._toward and self._have_peak and peak_bin < self._peak_bin:
            self._clear()
        self._peak_bin = peak_bin
        self._have_peak = True
        self._approach_power = peak
        return self._observe(frame, "toward" if self._toward else "watching", tee, peak)

    def _latch(self) -> None:
        self._clear()
        self._latched = True

    def _observe(self, frame: int, phase: str, tee: float, approach: float) -> TriggerObservation:
        return TriggerObservation(
            frame=frame,
            phase=phase,
            tee=tee,
            approach=approach,
            ready=self._ready,
            toward=self._toward,
            away=self._away,
            run=self._run,
            peak_bin=self._peak_bin,
            have_peak=self._have_peak,
        )


def loop0_vertical_power(cube: np.ndarray, n_tx: int) -> np.ndarray:
    """Loop-0 vertical residual power, one row per memory-order frame."""
    summary = vertical_loop_power(cube, n_tx=n_tx)
    frames = cube.shape[0]
    return np.asarray(summary.power).reshape(frames, summary.n_loops, -1)[:, 0, :]


def replay_dump(
    raw: bytes,
    *,
    tee_range_m: float = DEFAULT_TEE_RANGE_M,
    level: float = DEFAULT_LEVEL,
    hits: int = DEFAULT_HITS,
) -> list[TriggerObservation]:
    """Run the detector over a dump in capture order.

    Configurable-capture dumps are already oldest-first and store
    ``trigger_frame`` 0. Older rings store the oldest slot in
    ``trigger_frame``; both become time order here. Each frame watches the
    tee's absolute bin when that frame stored it.
    """
    meta, cube = parse_dump(raw)
    ranged = range_data(meta, cube)
    geometry = geometry_from_header(meta)
    absolute = int(round(tee_range_m / geometry.range_res_m))
    power = loop0_vertical_power(ranged, meta["n_tx"])
    detector = BallLeaveDetector(level=level, hits=hits, frame_period_s=geometry.frame_period_s)
    observations: list[TriggerObservation] = []
    n_frames = meta["n_frames"]
    origin = meta["trigger_frame"] % n_frames
    for time_index in range(n_frames):
        slot = (origin + time_index) % n_frames
        start = geometry.frame_bin_start(slot)
        count = geometry.frame_bin_count(slot)
        local = absolute - start
        tee_local = local if 0 <= local < count else None
        observations.append(detector.step(time_index, power[slot], tee_local, count))
    return observations


def iter_dump_files(directory: Path) -> list[Path]:
    """ILD1 captures under ``directory``, sorted by name."""
    if not directory.is_dir():
        return []
    found = []
    for path in sorted(directory.rglob("*")):
        if not path.is_file():
            continue
        with path.open("rb") as handle:
            if handle.read(4) == b"ILD1":
                found.append(path)
    return found
