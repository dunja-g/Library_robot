import pytest

from pi.borrowing_mission import BorrowingMission, BorrowingState


STUDENT = {"id": "S001", "name": "Alice"}
BOOK = {
    "book_id": "BK001",
    "title": "Deep Learning",
    "box_id": "1A",
    "location_code": "1A-L3-P21",
    "layer": 3,
    "position": 21,
}


def test_pending_mission_can_be_confirmed_once():
    mission = BorrowingMission.create(
        STUDENT, BOOK, timeout_seconds=60, clock=lambda: 10
    )
    assert mission.state == BorrowingState.PENDING
    assert mission.is_active

    mission.confirm(clock=lambda: 12)

    assert mission.state == BorrowingState.CONFIRMED
    assert mission.confirmed_at == 12
    with pytest.raises(ValueError):
        mission.confirm()


def test_pending_mission_can_be_cancelled_without_confirmation():
    mission = BorrowingMission.create(
        STUDENT, BOOK, timeout_seconds=60, clock=lambda: 10
    )
    mission.cancel("encoder_stall")

    assert mission.state == BorrowingState.CANCELLED
    assert not mission.is_active
    assert mission.as_dict()["cancel_reason"] == "encoder_stall"
    with pytest.raises(ValueError):
        mission.confirm()


def test_only_pending_missions_expire():
    now = [10.0]
    mission = BorrowingMission.create(
        STUDENT, BOOK, timeout_seconds=5, clock=lambda: now[0]
    )
    now[0] = 15.0
    assert mission.is_expired(clock=lambda: now[0])

    mission.confirm(clock=lambda: now[0])
    now[0] = 100.0
    assert not mission.is_expired(clock=lambda: now[0])
