"""
book_db.py — Maps book titles to their ArUco marker IDs.
Update the titles and IDs to match your actual books and printed markers.
"""

# Maps book title (string) → ArUco marker ID (int, 0–4)
BOOK_DATABASE = {
    "The Great Gatsby":        0,
    "1984":                    1,
    "To Kill a Mockingbird":   2,
    "Harry Potter":            3,
    "The Hobbit":              4,
}


def get_aruco_id(title: str) -> int | None:
    """Returns the ArUco ID for a book title, or None if not found."""
    return BOOK_DATABASE.get(title, None)


def get_all_books() -> list[str]:
    """Returns a list of all book titles."""
    return list(BOOK_DATABASE.keys())
