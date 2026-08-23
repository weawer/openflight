"""Explicit application operations consumed by the HTTP route adapter."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import Any, Protocol


class EventStream(Protocol):
    """Minimum broker surface needed to expose an SSE response."""

    def subscribe(self) -> Any:
        """Register a subscriber or raise when capacity is exhausted."""

    def frames(self, subscriber: Any) -> Iterator[str]:
        """Yield SSE frames for a registered subscriber."""

    def unsubscribe(self, subscriber: Any) -> None:
        """Release a registered subscriber."""


ApiOperation = Callable[..., Any]


@dataclass(frozen=True)
class ApiDependencies:
    """Operations supplied by the running OpenFlight application."""

    capabilities: ApiOperation
    state: ApiOperation
    read_club: ApiOperation
    write_club: ApiOperation
    read_orientation_calibration: ApiOperation
    write_orientation_calibration: ApiOperation
    event_stream: Callable[[], EventStream]
    shutdown: ApiOperation
