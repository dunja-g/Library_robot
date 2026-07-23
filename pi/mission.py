"""Hardware-independent mission progress for outbound and return routes."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum

try:
    from .route_db import get_marker, get_route
except ImportError:  # Supports running modules directly from ``pi``.
    from route_db import get_marker, get_route


class MissionPhase(str, Enum):
    OUTBOUND = "OUTBOUND"
    AT_DESTINATION = "AT_DESTINATION"
    RETURNING = "RETURNING"
    COMPLETE = "COMPLETE"


@dataclass
class Mission:
    book: dict
    outbound_route: dict
    return_route: dict
    phase: MissionPhase = MissionPhase.OUTBOUND
    step_index: int = 0

    @classmethod
    def from_book(cls, book: dict) -> "Mission":
        if not isinstance(book, dict):
            raise TypeError("book must be a dictionary")
        required = {"title", "outbound_route", "return_route", "destination_marker"}
        missing = sorted(required - set(book))
        if missing:
            raise ValueError(f"Book record is missing fields: {missing}")
        outbound = get_route(book["outbound_route"])
        return_route = get_route(book["return_route"])
        if outbound["steps"][-1]["marker_id"] != book["destination_marker"]:
            raise ValueError("Outbound route does not end at the book destination")
        return cls(deepcopy(book), outbound, return_route)

    @property
    def current_route(self) -> dict:
        if self.phase in (MissionPhase.OUTBOUND, MissionPhase.AT_DESTINATION):
            return self.outbound_route
        return self.return_route

    @property
    def current_step(self) -> dict | None:
        steps = self.current_route["steps"]
        if self.step_index >= len(steps):
            return None
        return deepcopy(steps[self.step_index])

    @property
    def current_marker_id(self) -> int | None:
        step = self.current_step
        return None if step is None else int(step["marker_id"])

    @property
    def is_destination_reached(self) -> bool:
        return self.phase in (
            MissionPhase.AT_DESTINATION,
            MissionPhase.RETURNING,
            MissionPhase.COMPLETE,
        )

    @property
    def is_complete(self) -> bool:
        return self.phase == MissionPhase.COMPLETE

    def advance_waypoint(self) -> dict:
        """Complete and return the current step, then advance mission phase."""
        completed = self.current_step
        if completed is None:
            raise RuntimeError("Mission has no current waypoint to advance")
        self.step_index += 1
        if self.step_index >= len(self.current_route["steps"]):
            if self.phase == MissionPhase.OUTBOUND:
                self.phase = MissionPhase.AT_DESTINATION
            elif self.phase == MissionPhase.RETURNING:
                self.phase = MissionPhase.COMPLETE
        return completed

    def start_return(self) -> None:
        if self.phase != MissionPhase.AT_DESTINATION:
            raise RuntimeError("Return route can only start at the destination")
        self.phase = MissionPhase.RETURNING
        self.step_index = 0

    def reset(self) -> None:
        self.phase = MissionPhase.OUTBOUND
        self.step_index = 0

    def status(self) -> dict:
        current_marker_id = self.current_marker_id
        marker = get_marker(current_marker_id) if current_marker_id is not None else None
        location = {
            "zone": self.book["zone"],
            "shelf": self.book["shelf_code"],
            "level": self.book["level"],
            "slot": self.book["slot"],
        }
        return {
            "phase": self.phase.value,
            "book": self.book["title"],
            "location": location,
            "current_marker_id": current_marker_id,
            "current_marker_name": None if marker is None else marker["name"],
            "waypoint_index": min(self.step_index + 1, len(self.current_route["steps"])),
            "waypoint_count": len(self.current_route["steps"]),
            "route": [step["marker_id"] for step in self.outbound_route["steps"]],
            "return_route": [step["marker_id"] for step in self.return_route["steps"]],
            "route_name": self.current_route["name"],
        }
