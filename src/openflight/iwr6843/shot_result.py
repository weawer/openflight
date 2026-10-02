"""The IWR6843 shot result packet, as the firmware serialises it.

``firmware/iwr6843/l3_result.h`` defines version 1: a fixed 100-byte
little-endian record of nine metrics (m/s and radians on the wire) each with
a confidence, validity and quality flag words, the impact time and source,
the point counts and the smash factor. Version 2 appends the impact fit
(``l3_impact_fit_t``), 64 bytes: the fused verdict and impact time, and each
of the three tracks' (club in, club out, ball out) own estimate. The firmware
prints the packet as a hex line after ``triggerLog result``; this module
parses either version into measurements the UI can label, keeping what was
measured apart from what was inferred and never turning an invalid or
implausible value into a number.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, replace

from openflight.iwr6843 import firmware_host as fw

PACKET_V1 = struct.Struct("<II9fII9fIBBBBf")
IMPACT_FIT = struct.Struct("<BBBBfff" + "BBHfff" * 3)
PACKET_V2_SIZE = PACKET_V1.size + IMPACT_FIT.size
assert PACKET_V1.size == fw.RESULT_V1_PACKET_BYTES
assert PACKET_V2_SIZE == fw.RESULT_PACKET_BYTES
_SIZES = {1: PACKET_V1.size, 2: PACKET_V2_SIZE}

ANGLE_METRICS = frozenset(
    {"vertical_launch", "horizontal_launch", "club_path", "angle_of_attack", "spin_axis"}
)
# gate and geometry come only from firmware before 2026-09-30, when both
# detectors were removed; current firmware reports range alone.
_IMPACT_SOURCE_BITS = (
    ("gate", fw.SHOT_IMPACT_GATE),
    ("geometry", fw.SHOT_IMPACT_GEOMETRY),
    ("range", fw.SHOT_IMPACT_RANGE),
)
# l3_shot_source_name keeps its original names below the range bit.
_LEGACY_IMPACT_SOURCES = {
    0: "none",
    fw.SHOT_IMPACT_GATE: "gate",
    fw.SHOT_IMPACT_GEOMETRY: "geometry",
    fw.SHOT_IMPACT_GATE | fw.SHOT_IMPACT_GEOMETRY: "both",
}


def _impact_source_name(mask: int) -> str:
    if mask in _LEGACY_IMPACT_SOURCES:
        return _LEGACY_IMPACT_SOURCES[mask]
    return "+".join(name for name, bit in _IMPACT_SOURCE_BITS if mask & bit)


IMPACT_SOURCES = {
    mask: _impact_source_name(mask)
    for mask in range(sum(bit for _, bit in _IMPACT_SOURCE_BITS) + 1)
}


@dataclass(frozen=True)
class Measurement:
    """One metric: SI value (m/s, degrees for angles, m for range) with its provenance."""

    name: str
    value: float | None  # None when the firmware marked it invalid
    confidence: float
    measured: bool  # by the radar, as opposed to inferred or modelled
    radial_only: bool
    implausible: bool
    fallback: bool  # the configured tee stood in for a locked ball

    @property
    def usable(self) -> bool:
        return self.value is not None and not self.implausible

    @property
    def label(self) -> str:
        """MEASURED / ESTIMATED / -, for a display that must not blur the two."""
        if self.value is None:
            return "-"
        return "MEASURED" if self.measured else "ESTIMATED"


@dataclass(frozen=True)
class ShotResultPacket:
    version: int
    shot_id: int
    metrics: dict[str, Measurement]
    quality: frozenset[str]
    impact_timestamp_us: int
    verdict: str  # invalid, partial, valid
    impact_source: str
    club_points: int
    ball_points: int
    smash: float | None
    impact_fit: dict | None = None

    def __getitem__(self, name: str) -> Measurement:
        return self.metrics[name]

    @property
    def ball_flight(self) -> bool:
        """The board measured a ball leaving: a ball speed, plausible or not.

        Ball points alone do not count; a stationary return near the tee
        leaves some. An implausible speed still counts, so the no-ball veto
        errs toward keeping the capture.
        """
        return self.metrics["ball_speed"].value is not None

    def with_onboard_angles_doubted(self) -> ShotResultPacket:
        """The same packet with every angle-derived metric marked implausible
        (launch angles, club path, angle of attack): the board ran without
        the calibration the Pi uses."""
        metrics = dict(self.metrics)
        for name in ("vertical_launch", "horizontal_launch", "club_path", "angle_of_attack"):
            if metrics[name].value is not None:
                metrics[name] = replace(metrics[name], implausible=True)
        return replace(self, metrics=metrics)

    @property
    def domain_confidence(self) -> dict[str, float]:
        """One confidence per measurement domain, the weakest usable metric's in each.

        club: club speed, path and attack; ball: ball speed; angle: the two
        launch angles; spin: spin rate. A domain with no usable metric reads 0.
        """
        domains = {
            "club": ("club_speed", "club_path", "angle_of_attack"),
            "ball": ("ball_speed",),
            "angle": ("vertical_launch", "horizontal_launch"),
            "spin": ("spin_rate",),
        }
        out: dict[str, float] = {}
        for domain, names in domains.items():
            usable = [
                self.metrics[n].confidence
                for n in names
                if n in self.metrics and self.metrics[n].usable
            ]
            out[domain] = round(min(usable), 3) if usable else 0.0
        return out

    def to_dict(self) -> dict:
        """JSON for the Shot record and the UI: every metric with its provenance."""
        return {
            "version": self.version,
            "shot_id": self.shot_id,
            "verdict": self.verdict,
            "impact_source": self.impact_source,
            "impact_timestamp_us": self.impact_timestamp_us,
            "club_points": self.club_points,
            "ball_points": self.ball_points,
            "smash": self.smash,
            "impact_fit": self.impact_fit,
            "quality": sorted(self.quality),
            "domains": self.domain_confidence,
            "metrics": {
                name: {
                    "value": m.value,
                    "confidence": m.confidence,
                    "label": m.label,
                    "measured": m.measured,
                    "radial_only": m.radial_only,
                    "implausible": m.implausible,
                    "fallback": m.fallback,
                    "usable": m.usable,
                }
                for name, m in self.metrics.items()
            },
        }


def parse_packet(raw: bytes) -> ShotResultPacket:
    """Decode one packet, version 1 or 2; raises ValueError on a wrong size or version."""
    if len(raw) < 4:
        raise ValueError(f"shot result packet is {len(raw)} bytes")
    (version,) = struct.unpack_from("<I", raw)
    if version not in _SIZES:
        raise ValueError(f"shot result version {version}, this host reads {sorted(_SIZES)}")
    if len(raw) != _SIZES[version]:
        raise ValueError(
            f"shot result v{version} packet is {len(raw)} bytes, expected {_SIZES[version]}"
        )
    fields = PACKET_V1.unpack_from(raw)
    shot_id = fields[1]
    values = fields[2:11]
    valid_flags, quality_flags = fields[11], fields[12]
    confidences = fields[13:22]
    impact_us, verdict, source, club_points, ball_points, smash = fields[22:28]
    metrics: dict[str, Measurement] = {}
    for index, name in enumerate(fw.RESULT_METRIC_NAMES):
        valid = bool(valid_flags & (1 << index))
        value = values[index]
        if valid and name in ANGLE_METRICS:
            value = math.degrees(value)
        metrics[name] = Measurement(
            name=name,
            value=value if valid else None,
            confidence=confidences[index] if valid else 0.0,
            # The per-metric flag word is not on the wire; validity is the bit
            # mask and the rest is read from the quality word where it applies.
            measured=valid and name not in ("spin_rate", "spin_axis"),
            radial_only=valid
            and name in ("ball_speed", "club_speed")
            and not (valid_flags & _angle_bits_for(name)),
            implausible=valid and not _plausible(name, quality_flags),
            fallback=valid and not (quality_flags & fw.QUALITY_FLAGS["ball_locked"]),
        )
    packet = ShotResultPacket(
        version=version,
        shot_id=shot_id,
        metrics=metrics,
        quality=frozenset(name for name, bit in fw.QUALITY_FLAGS.items() if quality_flags & bit),
        impact_timestamp_us=impact_us,
        verdict=fw.RESULT_VERDICT_NAMES[verdict] if verdict < 3 else "?",
        impact_source=IMPACT_SOURCES.get(source, "?"),
        club_points=club_points,
        ball_points=ball_points,
        smash=smash if smash > 0.0 else None,
    )
    impact_fit = _impact_fit(IMPACT_FIT.unpack_from(raw, PACKET_V1.size)) if version >= 2 else None
    return replace(packet, impact_fit=impact_fit)


def _impact_fit(fields: tuple) -> dict:
    """Decode the version-2 tail: the fused verdict and each track's estimate."""
    verdict, dropped, no_lock, _pad, impact_us, spread_us, dtrig_us = fields[:7]
    tracks = {}
    for index, name in enumerate(fw.FIT_TRACK_NAMES):
        why, points, _pad2, time_us, sigma_us, speed = fields[7 + 6 * index : 13 + 6 * index]
        why_name = fw.FIT_WHY_NAMES[why] if why < len(fw.FIT_WHY_NAMES) else "?"
        timed = fw.fit_track_timed(why_name)
        tracks[name] = {
            "why": why_name,
            "points": points,
            "time_us": time_us if timed else None,
            "sigma_us": sigma_us if timed else None,
            "speed_mps": speed,
        }
    verdict_name = fw.FIT_VERDICT_NAMES[verdict] if verdict < len(fw.FIT_VERDICT_NAMES) else "?"
    decided = fw.fit_verdict_decided(verdict_name)
    return {
        "verdict": verdict_name,
        "impact_us": impact_us if decided else None,
        "spread_us": spread_us,
        "refined_minus_trigger_us": dtrig_us if decided else None,
        "dropped": fw.FIT_TRACK_NAMES[dropped] if dropped < len(fw.FIT_TRACK_NAMES) else None,
        "no_lock": bool(no_lock),
        "tracks": tracks,
    }


def _angle_bits_for(name: str) -> int:
    names = fw.RESULT_METRIC_NAMES
    if name == "ball_speed":
        return (1 << names.index("vertical_launch")) | (1 << names.index("horizontal_launch"))
    return (1 << names.index("club_path")) | (1 << names.index("angle_of_attack"))


# The ball's own measurements, doubted together when the ball left slower than
# the club arrived (quality ball_slower_than_club); the club's stay trusted.
BALL_METRICS = frozenset({"ball_speed", "vertical_launch", "horizontal_launch"})


def _plausible(name: str, quality_flags: int) -> bool:
    if name in BALL_METRICS and quality_flags & fw.QUALITY_FLAGS["ball_slower_than_club"]:
        return False
    if name in ("ball_speed", "club_speed"):
        return bool(quality_flags & fw.QUALITY_FLAGS["speeds_plausible"])
    if name in ANGLE_METRICS:
        return bool(quality_flags & fw.QUALITY_FLAGS["angles_plausible"])
    return True


def parse_hex(text: str) -> ShotResultPacket:
    """The ``packet <hex>`` line ``triggerLog result`` prints, or the bare hex."""
    token = text.strip().split()[-1] if text.strip() else ""
    try:
        raw = bytes.fromhex(token)
    except ValueError as error:
        raise ValueError(f"shot result hex is not hex: {token[:40]!r}") from error
    return parse_packet(raw)


def parse_result_reply(reply: str) -> ShotResultPacket | None:
    """The whole ``triggerLog result`` reply; None when no packet line is present.

    The firmware prints the 200 hex characters as ``packet <first half>`` and
    ``packet+ <second half>`` because one CLI line cannot carry them all.
    """
    halves: list[str] = []
    for line in reply.splitlines():
        stripped = line.strip()
        if stripped.startswith("packet ") or stripped.startswith("packet+ "):
            halves.append(stripped.split(maxsplit=1)[1])
    if not halves:
        return None
    return parse_hex("".join(halves))


__all__ = [
    "ANGLE_METRICS",
    "IMPACT_FIT",
    "PACKET_V1",
    "PACKET_V2_SIZE",
    "Measurement",
    "ShotResultPacket",
    "parse_hex",
    "parse_packet",
    "parse_result_reply",
]
