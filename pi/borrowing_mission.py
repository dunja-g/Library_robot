"""In-memory transaction state for one borrowing-only robot mission."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from enum import Enum


class BorrowingState(str, Enum):
    PENDING = "pending"
    CONFIRMED = "confirmed"
    CANCELLED = "cancelled"


@dataclass
class BorrowingMission:
    mission_id: str
    student_id: str
    student_name: str
    book_id: str
    book_title: str
    box_id: str
    location_code: str
    layer: int
    position: int
    created_at: float
    timeout_seconds: float
    mission_type: str = "borrow"
    state: BorrowingState = BorrowingState.PENDING
    cancel_reason: str | None = None
    confirmed_at: float | None = None

    @classmethod
    def create(
        cls,
        student: dict,
        book: dict,
        *,
        mission_type: str = "borrow",
        timeout_seconds: float,
        clock=time.monotonic,
    ) -> "BorrowingMission":
        if timeout_seconds <= 0:
            raise ValueError("Mission timeout must be positive")
        return cls(
            mission_id=uuid.uuid4().hex,
            student_id=str(student["id"]),
            student_name=str(student["name"]),
            book_id=str(book["book_id"]),
            book_title=str(book["title"]),
            box_id=str(book["box_id"]),
            location_code=str(book["location_code"]),
            layer=int(book["layer"]),
            position=int(book["position"]),
            mission_type=str(mission_type),
            created_at=float(clock()),
            timeout_seconds=float(timeout_seconds),
        )

    @property
    def is_active(self) -> bool:
        return self.state in {
            BorrowingState.PENDING,
            BorrowingState.CONFIRMED,
        }

    def is_expired(self, *, clock=time.monotonic) -> bool:
        return (
            self.state == BorrowingState.PENDING
            and float(clock()) - self.created_at >= self.timeout_seconds
        )

    def confirm(self, *, clock=time.monotonic) -> None:
        if self.state != BorrowingState.PENDING:
            raise ValueError("Only a pending mission can be confirmed")
        self.state = BorrowingState.CONFIRMED
        self.confirmed_at = float(clock())

    def cancel(self, reason: str) -> None:
        if self.state != BorrowingState.PENDING:
            raise ValueError("Only a pending mission can be cancelled")
        self.state = BorrowingState.CANCELLED
        self.cancel_reason = str(reason)

    def as_dict(self) -> dict:
        return {
            "mission_id": self.mission_id,
            "mission_type": self.mission_type,
            "state": self.state.value,
            "student_id": self.student_id,
            "student_name": self.student_name,
            "book_id": self.book_id,
            "book_title": self.book_title,
            "box_id": self.box_id,
            "location_code": self.location_code,
            "layer": self.layer,
            "position": self.position,
            "cancel_reason": self.cancel_reason,
            "pickup_confirmation_required": self.state
            == BorrowingState.PENDING,
        }
