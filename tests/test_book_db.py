from pi.book_db import get_all_books, get_aruco_id, get_book, get_book_location


def test_deep_learning_has_complete_b3_location():
    book = get_book("Deep Learning")
    location = get_book_location("Deep Learning")

    assert book["book_id"] == "BK001"
    assert location == {
        "zone": "B",
        "shelf_number": 3,
        "shelf_code": "B3",
        "level": 3,
        "slot": 12,
        "destination_marker": 203,
    }
    assert get_aruco_id("Deep Learning") == 203
    assert "Deep Learning" in get_all_books()


def test_legacy_book_lookup_remains_compatible():
    assert get_aruco_id("1984") == 1
    assert get_book("missing") is None
