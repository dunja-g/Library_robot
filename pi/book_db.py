"""Book catalogue with physical shelf and route metadata."""

from __future__ import annotations

from copy import deepcopy


BOOK_DATABASE = {
    "Deep Learning": {
        "book_id": "BK001",
        "box_id": "1A",
        "layer": 3,
        "position": 21,
        "location_code": "1A-L3-P21",
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
    location = {
        "zone": book["zone"],
        "shelf_number": book["shelf_number"],
        "shelf_code": book["shelf_code"],
        "level": book["level"],
        "slot": book["slot"],
        "destination_marker": book["destination_marker"],
    }
    if "box_id" in book:
        location.update(
            box_id=book["box_id"],
            layer=book["layer"],
            position=book["position"],
            location_code=book["location_code"],
        )
    return location


def get_aruco_id(title: str) -> int | None:
    """Compatibility helper returning the final destination marker."""
    book = get_book(title)
    return None if book is None else int(book["destination_marker"])


def get_all_books(*, grid_only: bool = False) -> list[str]:
    """Return all titles or only books assigned to a fixed-grid box."""
    if grid_only:
        return [
            title for title, book in BOOK_DATABASE.items() if "box_id" in book
        ]
    return list(BOOK_DATABASE.keys())


def search_books(query: str = "") -> list[dict]:
    """Search fixed-grid books by title, book ID, or location code."""
    needle = str(query).strip().casefold()
    results = []
    for title in get_all_books(grid_only=True):
        book = get_book(title)
        searchable = (
            title,
            book["book_id"],
            book["location_code"],
            book["box_id"],
        )
        if needle and not any(needle in value.casefold() for value in searchable):
            continue
        results.append(
            {
                "title": title,
                "book_id": book["book_id"],
                "location_code": book["location_code"],
                "box_id": book["box_id"],
                "layer": book["layer"],
                "position": book["position"],
            }
        )
    return results


def find_book(query: str) -> dict | None:
    """Resolve an exact title, book ID, or location code."""
    needle = str(query).strip().casefold()
    if not needle:
        return None
    for title in get_all_books(grid_only=True):
        book = get_book(title)
        if needle in {
            title.casefold(),
            book["book_id"].casefold(),
            book["location_code"].casefold(),
        }:
            return book
    return None
