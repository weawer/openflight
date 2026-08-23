"""Stable domain-event contracts shared by HTTP/SSE and BLE adapters."""

from __future__ import annotations

import json
import uuid
from typing import Mapping

API_VERSION = 1
SCHEMA_VERSION = 1

OPTIONAL_SHOT_FIELDS = (
    "club_speed_mph",
    "smash_factor",
    "launch_angle_vertical",
    "launch_angle_horizontal",
    "spin_rpm",
    "club_path_deg",
    "spin_axis_deg",
)


def build_club_event(club: str) -> dict:
    """Build the V1 event broadcast whenever the authoritative club changes."""
    if not isinstance(club, str) or not club:
        raise ValueError("Club must be a non-empty string")
    return {
        "schema_version": SCHEMA_VERSION,
        "type": "club_changed",
        "club": club,
    }


def encode_club_event(club: str) -> bytes:
    """Encode a club-state event as deterministic, compact UTF-8 JSON."""
    return _encode_json(build_club_event(club))


def build_shot_event(shot_data: Mapping, *, event_id: str | None = None) -> dict:
    """Build the stable, display-focused V1 event from a completed shot."""
    event = {
        "schema_version": SCHEMA_VERSION,
        "event_id": event_id or str(uuid.uuid4()),
        "timestamp": shot_data["timestamp"],
        "club": shot_data["club"],
        "ball_speed_mph": shot_data["ball_speed_mph"],
        "estimated_carry_yards": shot_data["estimated_carry_yards"],
    }
    event.update({field: shot_data.get(field) for field in OPTIONAL_SHOT_FIELDS})
    return event


def encode_shot_event(shot_data: Mapping, *, event_id: str | None = None) -> bytes:
    """Encode a completed-shot event as deterministic, compact UTF-8 JSON."""
    return _encode_json(build_shot_event(shot_data, event_id=event_id))


def _encode_json(payload: Mapping) -> bytes:
    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
