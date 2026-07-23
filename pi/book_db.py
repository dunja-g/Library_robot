"""Book catalogue with physical shelf and route metadata."""

from __future__ import annotations

from copy import deepcopy


BOOK_DATABASE = {
    "Deep Learning": {
        "book_id": "BK001",
        "zone": "B",
        "shelf_number": 3,
        "shelf_code": "B3",
        "level": 3,
        "slot": 12,
        "destination_marker": 203,
        "outbound_route": "B3_OUTBOUND",
        "return_route": "B3_RETURN",
    },
    # Existing Version 1 titles remain available through direct-marker routes.
    "The Great Gatsby": {
        "book_id": "LEGACY000",
        "zone": "Legacy",
        "shelf_number": 0,
        "shelf_code": "Marker 0",
        "level": 1,
        "slot": 0,
        "destination_marker": 0,
        "outbound_route": "LEGACY_0_OUTBOUND",
        "return_route": "LEGACY_RETURN",
    },
    "1984": {
        "book_id": "LEGACY001",
        "zone": "Legacy",
        "shelf_number": 1,
        "shelf_code": "Marker 1",
        "level": 1,
        "slot": 1,
        "destination_marker": 1,
        "outbound_route": "LEGACY_1_OUTBOUND",
        "return_route": "LEGACY_RETURN",
    },
    "To Kill a Mockingbird": {
        "book_id": "LEGACY002",
        "zone": "Legacy",
        "shelf_number": 2,
        "shelf_code": "Marker 2",
        "level": 1,
        "slot": 2,
        "destination_marker": 2,
        "outbound_route": "LEGACY_2_OUTBOUND",
        "return_route": "LEGACY_RETURN",
    },
    "Harry Potter": {
        "book_id": "LEGACY003",
        "zone": "Legacy",
        "shelf_number": 3,
        "shelf_code": "Marker 3",
        "level": 1,
        "slot": 3,
        "destination_marker": 3,
        "outbound_route": "LEGACY_3_OUTBOUND",
        "return_route": "LEGACY_RETURN",
    },
    "The Hobbit": {
        "book_id": "LEGACY004",
        "zone": "Legacy",
        "shelf_number": 4,
        "shelf_code": "Marker 4",
        "level": 1,
        "slot": 4,
        "destination_marker": 4,
        "outbound_route": "LEGACY_4_OUTBOUND",
        "return_route": "LEGACY_RETURN",
    },
}


def get_book(title: str) -> dict | None:
    """Return a defensive copy of a complete book record."""
    record = BOOK_DATABASE.get(title)
    if record is None:
        return None
    book = deepcopy(record)
    book["title"] = title
    return book


def get_book_location(title: str) -> dict | None:
    """Return only user-facing physical location metadata."""
    book = get_book(title)
    if book is None:
        return None
    return {
        "zone": book["zone"],
        "shelf_number": book["shelf_number"],
        "shelf_code": book["shelf_code"],
        "level": book["level"],
        "slot": book["slot"],
        "destination_marker": book["destination_marker"],
    }


def get_aruco_id(title: str) -> int | None:
    """Compatibility helper returning the final destination marker."""
    book = get_book(title)
    return None if book is None else int(book["destination_marker"])


def get_all_books() -> list[str]:
    """Return book titles without changing the existing frontend contract."""
    return list(BOOK_DATABASE.keys())
