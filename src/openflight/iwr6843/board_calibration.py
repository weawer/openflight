"""What the board (``trackCfg cal`` / ``trackCfg elem``) and the replay are
told about the array: one mapping from the loaded ``Calibration``, so the
onboard angles and the host LCMF share a source (late-flight spec 2026-09-29)."""

from __future__ import annotations

import logging
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from openflight.iwr6843 import tunables
from openflight.iwr6843.calibration import Calibration

logger = logging.getLogger(__name__)

N_ELEMENTS = 8


@dataclass(frozen=True)
class BoardCalibration:  # pylint: disable=too-many-instance-attributes
    """``trackCfg cal`` values in the firmware's units, and the 8 elements."""

    pitch_deg: float
    yaw_deg: float
    roll_deg: float
    az_offset_rad: float
    el_offset_deg: float
    range_bias_m: float
    elem_phase_rad: tuple[float, ...]
    elem_gain: tuple[float, ...]
    radar_height_m: float = 0.152  # antenna height above the floor, for the ball fit

    def __post_init__(self) -> None:
        if len(self.elem_phase_rad) != N_ELEMENTS or len(self.elem_gain) != N_ELEMENTS:
            raise ValueError(f"board calibration needs {N_ELEMENTS} element phases and gains")
        if not all(math.isfinite(g) and g > 0.0 for g in self.elem_gain):
            raise ValueError("every element gain must be finite and positive")
        if not all(math.isfinite(v) for v in (*self.cal_args, *self.elem_phase_rad)):
            raise ValueError("board calibration values must be finite")

    @classmethod
    def identity(cls) -> BoardCalibration:
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0,) * N_ELEMENTS, (1.0,) * N_ELEMENTS)

    @classmethod
    def from_calibration(cls, cal: Calibration) -> BoardCalibration:
        correction = np.asarray(cal.elem_correction, dtype=complex)
        return cls(
            pitch_deg=math.degrees(cal.tilt_rad),
            yaw_deg=0.0,
            roll_deg=0.0,
            # The firmware applies azimuthOffsetRad once, as a phase, in l3_angle.c; it stays 0
            # until a board calibration measures it.
            az_offset_rad=0.0,
            el_offset_deg=0.0,
            range_bias_m=float(cal.range_bias_m),
            elem_phase_rad=tuple(float(-np.angle(c)) for c in correction),
            elem_gain=tuple(float(1.0 / abs(c)) for c in correction),
            radar_height_m=float(cal.radar_height_m),
        )

    @classmethod
    def from_file(cls, path: str | Path) -> BoardCalibration:
        return cls.from_calibration(Calibration.load(str(path)))

    @property
    def is_identity(self) -> bool:
        return self == BoardCalibration.identity()

    @property
    def cal_args(self) -> tuple[float, ...]:
        """``trackCfg cal <pitchDeg> <yawDeg> <rollDeg> <azOffsetRad> <elOffsetDeg> <rangeBiasM>``."""
        return (
            self.pitch_deg,
            self.yaw_deg,
            self.roll_deg,
            self.az_offset_rad,
            self.el_offset_deg,
            self.range_bias_m,
        )

    def replay_overrides(self) -> dict:
        """The same values as ``ReplayConfig`` fields."""
        return {
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "roll_deg": self.roll_deg,
            "azimuth_offset_rad": self.az_offset_rad,
            "elevation_offset_deg": self.el_offset_deg,
            "range_bias_m": self.range_bias_m,
            "elem_phase_rad": self.elem_phase_rad,
            "elem_gain": self.elem_gain,
        }

    def tunable_overrides(self) -> dict[str, float]:
        """Firmware tunables the calibration sets (``ReplayConfig.overrides``): the radar
        height the ball fit measures the tee from. Skipped, with a warning, outside the
        tunables registry's bounds."""
        tunable = tunables.BY_NAME["ball.fit.radarHeightM"]
        if not tunable.low <= self.radar_height_m <= tunable.high:
            logger.warning(
                "radar height %.3f m is outside %.2f..%.2f m; the ball fit keeps its default",
                self.radar_height_m,
                tunable.low,
                tunable.high,
            )
            return {}
        return {tunable.name: self.radar_height_m}

    def to_dict(self) -> dict:
        return asdict(self)
