"""Persistent user-defined golf clubs backed by an atomic JSON file."""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Union

from .clubs import ClubType

logger = logging.getLogger(__name__)

DEFAULT_CLUBS_PATH = Path.home() / ".config" / "openflight" / "clubs.json"
CLUBS_PATH_ENV = "OPENFLIGHT_CLUBS_PATH"
CLUBS_SCHEMA_VERSION = 1
MAX_CUSTOM_CLUBS = 100
MAX_NAME_LENGTH = 80
MAX_LABEL_LENGTH = 12
MIN_LOFT_DEG = 1.0
MAX_LOFT_DEG = 90.0


def resolve_clubs_path(path: Union[str, Path, None] = None) -> Path:
    """Resolve an explicit path, environment override, or user default."""
    if path is not None and str(path).strip():
        return Path(path).expanduser()
    env_path = (os.environ.get(CLUBS_PATH_ENV) or "").strip()
    if env_path:
        return Path(env_path).expanduser()
    return DEFAULT_CLUBS_PATH


def _clean_text(raw: Any, max_length: int) -> str:
    if raw is None:
        return ""
    return str(raw).strip()[:max_length]


def _parse_loft(raw: Any) -> Optional[float]:
    try:
        loft = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(loft) or not MIN_LOFT_DEG <= loft <= MAX_LOFT_DEG:
        return None
    return loft


@dataclass(frozen=True)
class CustomClub:
    """User equipment that inherits protected behavior from a built-in type."""

    id: str
    name: str
    label: str
    group: str
    base_type: ClubType
    loft_deg: float
    enabled: bool = True
    sort_order: int = 0

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "label": self.label,
            "group": self.group,
            "base_type": self.base_type.value,
            "loft_deg": self.loft_deg,
            "enabled": self.enabled,
            "sort_order": self.sort_order,
        }

    @classmethod
    def from_dict(cls, raw: Any) -> Optional["CustomClub"]:
        if not isinstance(raw, dict):
            return None
        club_id = _clean_text(raw.get("id"), MAX_NAME_LENGTH)
        name = _clean_text(raw.get("name"), MAX_NAME_LENGTH)
        label = _clean_text(raw.get("label"), MAX_LABEL_LENGTH)
        group = _clean_text(raw.get("group"), MAX_NAME_LENGTH)
        loft = _parse_loft(raw.get("loft_deg"))
        enabled = raw.get("enabled", True)
        if not isinstance(enabled, bool):
            return None
        try:
            base_type = ClubType(raw.get("base_type"))
            sort_order = int(raw.get("sort_order", 0))
        except (TypeError, ValueError):
            return None
        if not club_id or not name or not label or not group or loft is None:
            return None
        return cls(
            id=club_id,
            name=name,
            label=label,
            group=group,
            base_type=base_type,
            loft_deg=loft,
            enabled=enabled,
            sort_order=sort_order,
        )


class ClubStore:
    """Load, mutate, and atomically persist user-defined clubs."""

    def __init__(self, path: Union[str, Path, None] = None):
        self._path = resolve_clubs_path(path)
        self._lock = threading.Lock()
        self._clubs: list[CustomClub] = []
        self._load()

    def list(self) -> list[CustomClub]:  # pylint: disable=redefined-builtin
        """Return custom clubs in display order."""
        with self._lock:
            return sorted(self._clubs, key=lambda club: (club.sort_order, club.name.casefold()))

    def get(self, club_id: Any) -> Optional[CustomClub]:
        """Return one custom club by stable id."""
        with self._lock:
            return self._find(club_id)

    def snapshot(self) -> dict:
        """Return the versioned disk/wire representation."""
        with self._lock:
            return self._payload()

    def add(
        self,
        *,
        name: Any,
        label: Any,
        group: Any,
        base_type: Any,
        loft_deg: Any,
        enabled: bool = True,
        sort_order: int = 0,
    ) -> Optional[CustomClub]:
        """Create and persist a custom club, or return None when invalid."""
        raw = {
            "id": uuid.uuid4().hex,
            "name": name,
            "label": label,
            "group": group,
            "base_type": base_type.value if isinstance(base_type, ClubType) else base_type,
            "loft_deg": loft_deg,
            "enabled": enabled,
            "sort_order": sort_order,
        }
        club = CustomClub.from_dict(raw)
        if club is None:
            return None
        with self._lock:
            if len(self._clubs) >= MAX_CUSTOM_CLUBS:
                return None
            self._clubs.append(club)
            if not self._save_locked():
                self._clubs.remove(club)
                return None
        return club

    def update(self, club_id: Any, **changes: Any) -> Optional[CustomClub]:
        """Update editable club metadata while preserving its stable id."""
        allowed = {"name", "label", "group", "base_type", "loft_deg", "enabled", "sort_order"}
        if not changes or not set(changes).issubset(allowed):
            return None
        with self._lock:
            current = self._find(club_id)
            if current is None:
                return None
            raw = current.to_dict()
            raw.update(changes)
            if isinstance(raw.get("base_type"), ClubType):
                raw["base_type"] = raw["base_type"].value
            updated = CustomClub.from_dict(raw)
            if updated is None:
                return None
            index = self._clubs.index(current)
            self._clubs[index] = updated
            if not self._save_locked():
                self._clubs[index] = current
                return None
        return updated

    def remove(self, club_id: Any) -> bool:
        """Remove and persist a custom club."""
        with self._lock:
            club = self._find(club_id)
            if club is None:
                return False
            index = self._clubs.index(club)
            self._clubs.pop(index)
            if not self._save_locked():
                self._clubs.insert(index, club)
                return False
        return True

    def save(self) -> bool:
        """Write all custom clubs atomically and report whether it succeeded."""
        with self._lock:
            return self._save_locked()

    def _save_locked(self) -> bool:
        """Persist the current catalog while the caller holds ``self._lock``."""
        payload = self._payload()
        temp_path = self._path.with_name(f"{self._path.name}.{uuid.uuid4().hex}.tmp")
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with open(temp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_path, self._path)
            return True
        except OSError as error:
            logger.error("[clubs] could not save custom clubs to %s: %s", self._path, error)
            try:
                temp_path.unlink()
            except OSError:
                pass
            return False

    def _payload(self) -> dict:
        return {
            "version": CLUBS_SCHEMA_VERSION,
            "clubs": [club.to_dict() for club in self._clubs],
        }

    def _find(self, club_id: Any) -> Optional[CustomClub]:
        wanted = str(club_id or "").strip()
        return next((club for club in self._clubs if club.id == wanted), None)

    def _load(self) -> None:
        raw: Any = None
        try:
            with open(self._path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)
        except FileNotFoundError:
            self.save()
            return
        except (OSError, json.JSONDecodeError) as error:
            logger.warning("[clubs] could not read %s: %s", self._path, error)
            return

        if not isinstance(raw, dict) or raw.get("version") != CLUBS_SCHEMA_VERSION:
            version = raw.get("version") if isinstance(raw, dict) else None
            logger.warning(
                "[clubs] unsupported schema version %r in %s",
                version,
                self._path,
            )
            return

        entries = raw.get("clubs")
        if not isinstance(entries, list):
            return
        parsed = [CustomClub.from_dict(entry) for entry in entries]
        unique: dict[str, CustomClub] = {}
        for club in parsed:
            if club is not None and club.id not in unique:
                unique[club.id] = club
        self._clubs = list(unique.values())[:MAX_CUSTOM_CLUBS]
