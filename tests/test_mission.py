from pi.book_db import get_book
from pi.mission import Mission, MissionPhase


def test_mission_advances_outbound_then_return_route():
    mission = Mission.from_book(get_book("Deep Learning"))
    assert mission.current_marker_id == 101

    mission.advance_waypoint()
    assert mission.current_marker_id == 105
    mission.advance_waypoint()
    assert mission.current_marker_id == 203
    mission.advance_waypoint()
    assert mission.phase == MissionPhase.AT_DESTINATION

    mission.start_return()
    assert mission.phase == MissionPhase.RETURNING
    assert mission.current_marker_id == 105
    mission.advance_waypoint()
    assert mission.current_marker_id == 101
    mission.advance_waypoint()
    assert mission.current_marker_id == 0
    mission.advance_waypoint()
    assert mission.is_complete


def test_mission_status_contains_location_and_both_routes():
    status = Mission.from_book(get_book("Deep Learning")).status()
    assert status["location"] == {"zone": "B", "shelf": "B3", "level": 3, "slot": 12}
    assert status["route"] == [101, 105, 203]
    assert status["return_route"] == [105, 101, 0]
