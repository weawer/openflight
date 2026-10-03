"""Firmware config constants a sweep may override, and how they reach the C configs.

Each entry names a field of one of the ``*_cfg_t`` structs the replay builds
from the C ``*_cfg_defaults()``. The defaults are never copied here: they are
read from the firmware (``read_defaults``), so the C stays the one source.
"""

from __future__ import annotations

import ctypes
import math
from collections.abc import Mapping
from dataclasses import dataclass

from openflight.iwr6843 import firmware_host as fw


@dataclass(frozen=True)
class Tunable:
    """One overridable constant and the box a sweep may search."""

    root: str  # "trig", "club", "fit" or "ball": which config it lives in
    path: str  # dotted field path inside that config
    kind: str  # "int" or "float"
    low: float
    high: float
    step: float

    @property
    def name(self) -> str:
        """``root.path``: how overrides and reports name it."""
        return f"{self.root}.{self.path}"


def _t(root: str, path: str, kind: str, low: float, high: float, step: float) -> Tunable:
    return Tunable(root, path, kind, low, high, step)


TUNABLES: tuple[Tunable, ...] = (
    _t("trig", "approachBins", "int", 4, 24, 2),
    _t("trig", "pastBins", "int", 1, 8, 1),
    _t("club", "gateBins", "float", 1.0, 6.0, 0.5),
    _t("club", "maxMisses", "int", 0, 5, 1),
    _t("club", "minConfidence", "float", 0.0, 0.8, 0.1),
    _t("club", "minAcquireDopplerMps", "float", 0.0, 4.0, 0.5),
    _t("club", "maxSameBinPoints", "int", 1, 5, 1),
    _t("club", "approachMaxSameBinPoints", "int", 1, 5, 1),
    _t("club", "standingFrames", "int", 0, 6, 1),
    # The club "kf.*" constants affect only the replay/viewer reconstruction; the board
    # does not run the club filter.
    _t("club", "kf.accelSigmaMps2", "float", 100.0, 5000.0, 100.0),
    _t("club", "kf.rangeSigmaM", "float", 0.005, 0.1, 0.005),
    _t("club", "kf.angleSigmaRad", "float", 0.02, 0.6, 0.02),
    _t("club", "kf.chi2Gate", "float", 2.0, 20.0, 1.0),
    _t("club", "acquireMinStepBins", "float", 0.25, 2.0, 0.25),
    _t("club", "acquireMaxStepBins", "float", 0.0, 6.0, 0.5),  # 0: single-frame acquisition
    _t("club", "acquireDopplerTolMps", "float", 1.0, 9.0, 1.0),  # 9: any (half the alias span)
    _t("club", "acquireMinConfidence", "float", 0.0, 0.4, 0.05),
    _t("club", "acquireExpectedStepBins", "float", 1.0, 3.5, 0.25),
    _t("ball", "minDepartureMps", "float", 5.0, 25.0, 2.5),
    _t("ball", "originGateBins", "float", 2.0, 16.0, 2.0),
    _t("ball", "minDepartureBins", "float", 0.5, 3.0, 0.5),
    _t("ball", "launchPoints", "int", 3, 10, 1),
    _t("ball", "core.gateBins", "float", 2.0, 12.0, 1.0),
    _t("ball", "core.maxMisses", "int", 0, 4, 1),
    _t("ball", "hyps.spawnBehindM", "float", 0.0, 0.2, 0.025),
    _t("ball", "hyps.spawnBeyondM", "float", 0.2, 0.75, 0.05),
    _t("ball", "hyps.gateM", "float", 0.025, 0.15, 0.025),
    _t("ball", "hyps.gateMps", "float", 2.0, 16.0, 2.0),
    _t("ball", "hyps.coastUs", "int", 0, 12000, 3000),
    _t("ball", "hyps.impactCoastUs", "int", 6000, 30000, 3000),
    _t("ball", "hyps.classifyPoints", "int", 2, 8, 1),
    _t("ball", "hyps.maxResidualBins", "float", 0.25, 2.5, 0.25),
    _t("ball", "hyps.dopplerToleranceMps", "float", 1.0, 6.0, 0.5),
    _t("ball", "hyps.fastSupportFraction", "float", 0.3, 0.9, 0.05),
    _t("ball", "fit.angleSigmaRad", "float", 0.02, 0.6, 0.02),
    _t("ball", "fit.gateK", "float", 1.0, 5.0, 0.25),
    _t("ball", "fit.minAccepted", "int", 3, 8, 1),  # the firmware clamps to 3
    _t("ball", "fit.maxAngleSigmaRad", "float", 0.01, 0.2, 0.01),
    _t("ball", "fit.maxRmsRad", "float", 0.05, 0.6, 0.05),
    _t("ball", "fit.radarHeightM", "float", 0.05, 0.6, 0.01),
    _t("ball", "fit.teeBallHeightM", "float", 0.0, 0.6, 0.01),
    _t("fit", "fitPoints", "int", 3, 8, 1),
    _t("fit", "fitSpanUs", "int", 0, 12000, 500),  # 0: fitPoints alone
    _t("fit", "minPoints", "int", 2, 5, 1),
    _t("fit", "ballMinMps", "float", 5.0, 30.0, 2.5),
    _t("fit", "clutterSigmas", "float", 0.0, 8.0, 0.5),  # 0: no clutter filter
)
BY_NAME: dict[str, Tunable] = {t.name: t for t in TUNABLES}

_DEFAULTS_FN = {
    "trig": ("l3_trig_cfg_defaults", fw.TrigCfg),
    "club": ("l3_track_cfg_defaults", fw.TrackCfg),
    "fit": ("l3_impact_fit_cfg_defaults", fw.ImpactFitCfg),
    "ball": ("l3_ball_track_cfg_defaults", fw.BallTrackCfg),
}


def check_overrides(overrides: Mapping[str, float]) -> None:
    """Refuse unknown constants and values outside the registry's bounds."""
    unknown = sorted(set(overrides) - set(BY_NAME))
    if unknown:
        raise ValueError(f"unknown constants {unknown}; known: {sorted(BY_NAME)}")
    for name, value in overrides.items():
        t = BY_NAME[name]
        if not (isinstance(value, (int, float)) and math.isfinite(value)):
            raise ValueError(f"{name}: {value!r} is not a finite number")
        if not t.low <= value <= t.high:
            raise ValueError(f"{name}: {value} outside {t.low}..{t.high}")


def _field_owner(cfg: ctypes.Structure, path: str) -> tuple[ctypes.Structure, str]:
    *parents, leaf = path.split(".")
    for name in parents:
        cfg = getattr(cfg, name)
    return cfg, leaf


def apply_overrides(overrides: Mapping[str, float], root: str, cfg: ctypes.Structure) -> None:
    """Write the overrides that belong to ``root`` into ``cfg``; integer fields round."""
    for name, value in overrides.items():
        t = BY_NAME.get(name)
        if t is None or t.root != root:
            continue
        owner, leaf = _field_owner(cfg, t.path)
        setattr(owner, leaf, int(round(value)) if t.kind == "int" else float(value))


def read_defaults(lib: ctypes.CDLL) -> dict[str, float]:
    """Every registered constant's firmware default, read from the C ``*_cfg_defaults``."""
    configs = {}
    for root, (function, struct) in _DEFAULTS_FN.items():
        cfg = struct()
        getattr(lib, function)(ctypes.byref(cfg))
        configs[root] = cfg
    out = {}
    for t in TUNABLES:
        owner, leaf = _field_owner(configs[t.root], t.path)
        out[t.name] = float(getattr(owner, leaf))
    return out
