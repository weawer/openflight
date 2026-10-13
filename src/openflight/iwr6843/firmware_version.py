"""IWR6843 firmware identity: release semver and the CLI ``stats version`` reply.

``firmware/VERSION`` holds the release version as ``MAJOR.MINOR.PATCH``,
optionally with a fork tag of lowercase letters (``1.0.3.j``): two forks once
released different images as 1.0.3, so this branch's versions carry ``.j``.
The firmware build reads it, stamps it into the image and names the release file
after it, so the file is the single source of truth. Bump it with::

    make -C firmware bump-version PART=patch   # or minor / major

The flashed image answers the CLI ``stats version`` sub-mode with one line of
space-separated ``key=value`` pairs (``firmware/iwr6843/fw_version.h``)::

    version=1.0.0 git=b6f4c36d2286 variant=hybrid-cadence built=2026-09-30T15:37:51Z

It is a sub-mode because the firmware's CLI table is full. Images built before
it ignore the argument and print their counters, so they carry no version line.

``openflight-firmware`` (``firmware_cli.py``) wraps this on the command line.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

VERSION_FILE = Path(__file__).resolve().parents[3] / "firmware" / "VERSION"
BUMP_PARTS = ("major", "minor", "patch")

_SEMVER = re.compile(r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:\.([a-z]+))?$")
_REPLY_KEYS = ("version", "git", "variant", "built")


@dataclass(frozen=True, order=True)
class SemVer:
    """A release version: ``MAJOR.MINOR.PATCH[.tag]``; the tag names a fork
    (lowercase letters), never a pre-release, and a bump keeps it."""

    major: int
    minor: int
    patch: int
    tag: str = ""

    @classmethod
    def parse(cls, text: str) -> "SemVer":
        """Parse ``MAJOR.MINOR.PATCH[.tag]``; raise ValueError on anything else."""
        match = _SEMVER.match(text.strip())
        if match is None:
            raise ValueError(f"not a MAJOR.MINOR.PATCH[.tag] version: {text.strip()!r}")
        major, minor, patch, tag = match.groups()
        return cls(int(major), int(minor), int(patch), tag or "")

    def bump(self, part: str) -> "SemVer":
        """Next version; a higher part resets the lower ones to zero."""
        if part == "major":
            return SemVer(self.major + 1, 0, 0, self.tag)
        if part == "minor":
            return SemVer(self.major, self.minor + 1, 0, self.tag)
        if part == "patch":
            return SemVer(self.major, self.minor, self.patch + 1, self.tag)
        raise ValueError(f"unknown version part {part!r}; expected one of {BUMP_PARTS}")

    def __str__(self) -> str:
        tag = f".{self.tag}" if self.tag else ""
        return f"{self.major}.{self.minor}.{self.patch}{tag}"


def read_release_version(path: Path = VERSION_FILE) -> SemVer:
    """The release version recorded in ``firmware/VERSION``."""
    return SemVer.parse(path.read_text(encoding="utf-8"))


def bump_release_version(part: str, path: Path = VERSION_FILE) -> tuple[SemVer, SemVer]:
    """Bump ``firmware/VERSION`` in place; return (old, new)."""
    old = read_release_version(path)
    new = old.bump(part)
    path.write_text(f"{new}\n", encoding="utf-8")
    return old, new


@dataclass(frozen=True)
class FirmwareVersion:
    """What the flashed image reports about itself."""

    version: str
    git: str
    variant: str
    built: str

    def __str__(self) -> str:
        return f"{self.version} (git {self.git}, {self.variant}, built {self.built})"


def find_version_line(reply: str) -> str | None:
    """The ``version=`` line of a ``stats version`` reply, or None without one.

    The reply may carry CLI echo, prompt and ``Done`` lines around it. Only a
    line that starts with ``version=`` counts, so a counter whose name merely
    ends in ``version`` cannot pass for it.
    """
    for line in reply.splitlines():
        line = line.strip()
        if line.startswith("version="):
            return line
    return None


def parse_version_reply(reply: str) -> FirmwareVersion:
    """Parse the ``stats version`` reply; raise ValueError when it is malformed."""
    line = find_version_line(reply)
    if line is None:
        raise ValueError(f"no version line in firmware reply: {reply.strip()!r}")
    fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
    missing = [key for key in _REPLY_KEYS if not fields.get(key)]
    if missing:
        raise ValueError(f"firmware version reply is missing {missing}: {line!r}")
    return FirmwareVersion(**{key: fields[key] for key in _REPLY_KEYS})
