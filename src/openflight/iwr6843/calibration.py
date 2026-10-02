"""Array and geometry calibration for the IWR6843 elevation array.

The checked-in reference was produced with a corner reflector at three
tape-measured positions. It is a validated starting point, not a universal
factory calibration. Conventions:

- Element corrections apply AFTER the physical-orientation flip (x8[::-1]).
- ``tilt_rad`` rotates measured angles into ground coordinates.
- ``range_bias_m`` subtracts from measured range (radar-face referenced).

Re-run array calibration whenever the board, enclosure, or antenna orientation
changes, and measure installation geometry whenever the mount moves.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

DEFAULT_CAL_PATH = "config/iwr6843_calibration_reference.json"

# Mount tilt used when no session log or --iwr6843-tilt-deg supplies one;
# 0 silently zeros VLA. Round number: per-board cal can override.
DEFAULT_PITCH_DEG = 10.0


# Antenna-center to tee slant range when the setup has not measured one.
# Tape readings for the kiosk and scripts are from the enclosure front; add
# ARRAY_DEPTH_M (or call antenna_range_m) before converting to a range bin.
DEFAULT_TEE_RANGE_M = 1.575

# Closest tee the kiosk accepts, tape from the enclosure front. Nearer than
# this the self-trigger's watch region starts under a metre from the radar,
# where the golfer's hands and body stand (and a desk test fires it).
MIN_TEE_RANGE_M = 1.4

# Enclosure face to the antenna array, 30 mm. A tape measured from the face
# reads this much less than the radar's range, which is measured from the array.
ARRAY_DEPTH_M = 0.030


def antenna_range_m(face_range_m: float) -> float:
    """Range from the antenna array for a distance measured from the enclosure face."""
    return face_range_m + ARRAY_DEPTH_M


@dataclass
class Calibration:
    """Loaded calibration constants, ready to apply to snapshots."""

    elem_correction: np.ndarray  # complex, len n_virtual elements
    tilt_rad: float
    range_bias_m: float
    source: str = "unset"
    tee_range_m: float | None = None  # tape-measured launch point (slant)
    # Ball height at launch ABOVE THE FLOOR (ball on mat ~0.04 m). The old
    # field meant "above the radar plane" and silently anchored every tee
    # fit ~0.15 m high on a 0.152 m mount: -0.15/lever-arm of slope bias
    # (SW read -12 deg). Floor-referenced + radar height fixes the anchor.
    tee_ball_height_m: float = 0.04
    meta: dict = field(default_factory=dict)

    @property
    def radar_height_m(self) -> float:
        """Antenna center height above the floor (from the cal solve)."""
        return float(self.meta.get("radar_height_m", 0.152))

    @property
    def tee_anchor_h_m(self) -> float:
        """Tee anchor height in radar-plane coordinates (h=0 at radar)."""
        return self.tee_ball_height_m - self.radar_height_m

    @classmethod
    def load(cls, path: str = DEFAULT_CAL_PATH) -> "Calibration":
        """Load a cal JSON written by the corner-reflector solve."""
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
        corr = np.exp(-1j * np.asarray(raw["elem_phase_rad"])) / np.asarray(raw["elem_gain"])
        return cls(
            elem_correction=corr,
            tilt_rad=float(np.radians(raw["tilt_deg"])),
            range_bias_m=float(raw["range_bias_const_m"]),
            source=path,
            meta=raw,
        )

    @classmethod
    def identity(cls, n_elements: int = 8) -> "Calibration":
        """No-op calibration (uncalibrated array, zero tilt/bias)."""
        return cls(
            elem_correction=np.ones(n_elements, dtype=complex),
            tilt_rad=0.0,
            range_bias_m=0.0,
            source="identity",
        )

    def apply(self, snapshot: np.ndarray) -> np.ndarray:
        """Correct a physical-order snapshot (post-flip) element-wise."""
        return snapshot * self.elem_correction

    def true_range(self, measured_m: float) -> float:
        """Bias-corrected range from a measured (apparent) range."""
        return measured_m - self.range_bias_m
