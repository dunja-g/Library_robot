import pytest

from pi.route_db import MARKERS, ROUTES, get_marker, get_route, validate_routes


def test_b3_routes_have_required_marker_order():
    assert [step["marker_id"] for step in get_route("B3_OUTBOUND")["steps"]] == [101, 105, 203]
    assert [step["marker_id"] for step in get_route("B3_RETURN")["steps"]] == [105, 101, 0]
    assert get_marker(203)["arrival_distance_cm"] == 35.0


@pytest.mark.parametrize(
    "markers,routes",
    [
        (MARKERS, {"bad": {"steps": [{"marker_id": 999, "turn_after": "NONE"}]}}),
        (MARKERS, {"bad": {"steps": [{"marker_id": 0, "turn_after": "SPIN"}]}}),
        (MARKERS, {"bad": {"steps": []}}),
        ({0: {"arrival_distance_cm": 0}}, ROUTES),
    ],
)
def test_route_validation_rejects_unsafe_configuration(markers, routes):
    with pytest.raises(ValueError):
        validate_routes(markers, routes)
