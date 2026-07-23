"""Configurable ArUco marker catalogue and deterministic routes."""

from __future__ import annotations

from copy import deepcopy


VALID_TURNS = {"NONE", "LEFT", "RIGHT", "UTURN"}

MARKERS = {
    0: {"name": "Dock", "type": "dock", "arrival_distance_cm": 30.0},
    1: {"name": "Legacy marker 1", "type": "destination", "arrival_distance_cm": 25.0},
    2: {"name": "Legacy marker 2", "type": "destination", "arrival_distance_cm": 25.0},
    3: {"name": "Legacy marker 3", "type": "destination", "arrival_distance_cm": 25.0},
    4: {"name": "Legacy marker 4", "type": "destination", "arrival_distance_cm": 25.0},
    101: {"name": "Main Corridor", "type": "waypoint", "arrival_distance_cm": 35.0},
    105: {"name": "Zone B Junction", "type": "waypoint", "arrival_distance_cm": 35.0},
    203: {"name": "Shelf B3", "type": "destination", "arrival_distance_cm": 35.0},
}

ROUTES = {
    "B3_OUTBOUND": {
        "initial_turn": "NONE",
        "steps": [
            {"marker_id": 101, "turn_after": "RIGHT"},
            {"marker_id": 105, "turn_after": "LEFT"},
            {"marker_id": 203, "turn_after": "NONE"},
        ],
    },
    "B3_RETURN": {
        "initial_turn": "UTURN",
        "steps": [
            {"marker_id": 105, "turn_after": "RIGHT"},
            {"marker_id": 101, "turn_after": "LEFT"},
            {"marker_id": 0, "turn_after": "NONE"},
        ],
    },
    "LEGACY_RETURN": {
        "initial_turn": "UTURN",
        "steps": [{"marker_id": 0, "turn_after": "NONE"}],
    },
    **{
        f"LEGACY_{marker_id}_OUTBOUND": {
            "initial_turn": "NONE",
            "steps": [{"marker_id": marker_id, "turn_after": "NONE"}],
        }
        for marker_id in range(5)
    },
}


def validate_routes(markers: dict | None = None, routes: dict | None = None) -> None:
    """Raise ``ValueError`` when marker or route configuration is unsafe."""
    markers = MARKERS if markers is None else markers
    routes = ROUTES if routes is None else routes

    for marker_id, marker in markers.items():
        distance = marker.get("arrival_distance_cm")
        if not isinstance(marker_id, int):
            raise ValueError(f"Marker ID must be an integer: {marker_id!r}")
        if not isinstance(distance, (int, float)) or distance <= 0:
            raise ValueError(f"Marker {marker_id} arrival distance must be positive")

    for route_name, route in routes.items():
        steps = route.get("steps")
        initial_turn = route.get("initial_turn", "NONE")
        if initial_turn not in VALID_TURNS:
            raise ValueError(f"Route {route_name} has invalid initial turn")
        if not isinstance(steps, list) or not steps:
            raise ValueError(f"Route {route_name} cannot be empty")
        for step in steps:
            marker_id = step.get("marker_id")
            turn_after = step.get("turn_after", "NONE")
            if marker_id not in markers:
                raise ValueError(f"Route {route_name} references unknown marker {marker_id}")
            if turn_after not in VALID_TURNS:
                raise ValueError(f"Route {route_name} has invalid turn {turn_after}")


def get_marker(marker_id: int) -> dict:
    if marker_id not in MARKERS:
        raise KeyError(f"Unknown marker: {marker_id}")
    marker = deepcopy(MARKERS[marker_id])
    marker["id"] = marker_id
    return marker


def get_route(route_name: str) -> dict:
    if route_name not in ROUTES:
        raise KeyError(f"Unknown route: {route_name}")
    route = deepcopy(ROUTES[route_name])
    route["name"] = route_name
    return route


validate_routes()
