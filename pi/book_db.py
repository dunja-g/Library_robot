"""Book catalogue with physical shelf and route metadata."""

from __future__ import annotations

from copy import deepcopy


BOOK_DATABASE = {
    # ---------------- 1A 号箱 (Row 1 Left - 现场编号 8712) ----------------
    "Deep Learning": {
        "book_id": "BK001",
        "subtitle": "A Modern Approach",
        "authors": ["Ian Goodfellow", "Yoshua Bengio", "Aaron Courville"],
        "rating": 4.8,
        "reviews": 324,
        "tags": ["AI", "Machine Learning", "Neural Networks"],
        "description": (
            "The definitive modern introduction to deep learning "
            "by the pioneers of the field."
        ),
        "box_id": "1A",
        "box_label": "8712",
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
    "Python Data Science Handbook": {
        "book_id": "BK002",
        "box_id": "1A",
        "box_label": "8712",
        "layer": 1,
        "position": 5,
        "location_code": "1A-L1-P05",
        "zone": "A",
        "shelf_number": 1,
        "shelf_code": "1A",
        "level": 1,
        "slot": 5,
        "destination_marker": 101,
        "outbound_route": "1A_OUTBOUND",
        "return_route": "1A_RETURN",
    },

    # ---------------- 1B 号箱 (Row 1 Right - 现场编号 8716) ----------------
    "Machine Learning Yearning": {
        "book_id": "BK003",
        "box_id": "1B",
        "box_label": "8716",
        "layer": 1,
        "position": 2,
        "location_code": "1B-L1-P02",
        "zone": "B",
        "shelf_number": 1,
        "shelf_code": "1B",
        "level": 1,
        "slot": 2,
        "destination_marker": 102,
        "outbound_route": "1B_OUTBOUND",
        "return_route": "1B_RETURN",
    },
    "Pattern Recognition and ML": {
        "book_id": "BK004",
        "box_id": "1B",
        "box_label": "8716",
        "layer": 2,
        "position": 8,
        "location_code": "1B-L2-P08",
        "zone": "B",
        "shelf_number": 1,
        "shelf_code": "1B",
        "level": 2,
        "slot": 8,
        "destination_marker": 102,
        "outbound_route": "1B_OUTBOUND",
        "return_route": "1B_RETURN",
    },

    # ---------------- 2A 号箱 (Row 2 Left - 现场编号 8712) ----------------
    "Clean Code": {
        "book_id": "BK005",
        "box_id": "2A",
        "box_label": "8712",
        "layer": 1,
        "position": 3,
        "location_code": "2A-L1-P03",
        "zone": "A",
        "shelf_number": 2,
        "shelf_code": "2A",
        "level": 1,
        "slot": 3,
        "destination_marker": 201,
        "outbound_route": "2A_OUTBOUND",
        "return_route": "2A_RETURN",
    },
    "Design Patterns": {
        "book_id": "BK006",
        "box_id": "2A",
        "box_label": "8712",
        "layer": 2,
        "position": 12,
        "location_code": "2A-L2-P12",
        "zone": "A",
        "shelf_number": 2,
        "shelf_code": "2A",
        "level": 2,
        "slot": 12,
        "destination_marker": 201,
        "outbound_route": "2A_OUTBOUND",
        "return_route": "2A_RETURN",
    },

    # ---------------- 2B 号箱 (Row 2 Right - 现场编号 8712) ----------------
    "Introduction to Algorithms": {
        "book_id": "BK007",
        "box_id": "2B",
        "box_label": "8712",
        "layer": 1,
        "position": 1,
        "location_code": "2B-L1-P01",
        "zone": "B",
        "shelf_number": 2,
        "shelf_code": "2B",
        "level": 1,
        "slot": 1,
        "destination_marker": 202,
        "outbound_route": "2B_OUTBOUND",
        "return_route": "2B_RETURN",
    },
    "Artificial Intelligence: A Modern Approach": {
        "book_id": "BK008",
        "box_id": "2B",
        "box_label": "8712",
        "layer": 2,
        "position": 10,
        "location_code": "2B-L2-P10",
        "zone": "B",
        "shelf_number": 2,
        "shelf_code": "2B",
        "level": 2,
        "slot": 10,
        "destination_marker": 202,
        "outbound_route": "2B_OUTBOUND",
        "return_route": "2B_RETURN",
    },

    # ---------------- 3A 号箱 (Row 3 Left) ----------------
    "Computer Networking": {
        "book_id": "BK009",
        "box_id": "3A",
        "layer": 1,
        "position": 4,
        "location_code": "3A-L1-P04",
        "zone": "A",
        "shelf_number": 3,
        "shelf_code": "3A",
        "level": 1,
        "slot": 4,
        "destination_marker": 301,
        "outbound_route": "3A_OUTBOUND",
        "return_route": "3A_RETURN",
    },

    # ---------------- 3B 号箱 (Row 3 Right) ----------------
    "Reinforcement Learning": {
        "book_id": "BK010",
        "box_id": "3B",
        "layer": 2,
        "position": 15,
        "location_code": "3B-L2-P15",
        "zone": "B",
        "shelf_number": 3,
        "shelf_code": "3B",
        "level": 2,
        "slot": 15,
        "destination_marker": 302,
        "outbound_route": "3B_OUTBOUND",
        "return_route": "3B_RETURN",
    },

    # The former row-4 titles remain available in the reduced six-box layout.
    "Computer Systems: CSAPP": {
        "book_id": "BK011",
        "box_id": "3A",
        "layer": 1,
        "position": 6,
        "location_code": "3A-L1-P06",
        "zone": "A",
        "shelf_number": 3,
        "shelf_code": "3A",
        "level": 1,
        "slot": 6,
        "destination_marker": 301,
        "outbound_route": "3A_OUTBOUND",
        "return_route": "3A_RETURN",
    },
    "Robotics, Vision and Control": {
        "book_id": "BK012",
        "box_id": "3B",
        "layer": 2,
        "position": 9,
        "location_code": "3B-L2-P09",
        "zone": "B",
        "shelf_number": 3,
        "shelf_code": "3B",
        "level": 2,
        "slot": 9,
        "destination_marker": 302,
        "outbound_route": "3B_OUTBOUND",
        "return_route": "3B_RETURN",
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
        if "box_label" in book:
            location["box_label"] = book["box_label"]
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
    """Search fixed-grid books by title, book ID, location code, or box label."""
    needle = str(query).strip().casefold()
    results = []
    for title in get_all_books(grid_only=True):
        book = get_book(title)
        searchable = [
            title,
            book["book_id"],
            book["location_code"],
            book["box_id"],
        ]
        if "box_label" in book:
            searchable.append(book["box_label"])
        if needle and not any(needle in value.casefold() for value in searchable):
            continue
        res = {
            "title": title,
            "book_id": book["book_id"],
            "location_code": book["location_code"],
            "box_id": book["box_id"],
            "layer": book["layer"],
            "position": book["position"],
        }
        for field in (
            "subtitle",
            "authors",
            "rating",
            "reviews",
            "tags",
            "description",
        ):
            if field in book:
                res[field] = deepcopy(book[field])
        if "box_label" in book:
            res["box_label"] = book["box_label"]
        results.append(res)
    return results


def find_book(query: str) -> dict | None:
    """Resolve an exact title, book ID, location code, or box label."""
    needle = str(query).strip().casefold()
    if not needle:
        return None
    for title in get_all_books(grid_only=True):
        book = get_book(title)
        match_set = {
            title.casefold(),
            book["book_id"].casefold(),
            book["location_code"].casefold(),
            book["box_id"].casefold(),
        }
        if "box_label" in book:
            match_set.add(book["box_label"].casefold())
        if needle in match_set:
            return book
    return None
