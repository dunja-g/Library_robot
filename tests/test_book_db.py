from pi.book_db import (
    find_book,
    get_all_books,
    get_aruco_id,
    get_book,
    get_book_location,
    search_books,
)


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
        "box_id": "1A",
        "layer": 3,
        "position": 21,
        "location_code": "1A-L3-P21",
    }
    assert get_aruco_id("Deep Learning") == 203
    assert "Deep Learning" in get_all_books()
    assert get_all_books(grid_only=True) == ["Deep Learning"]
    assert book["location_code"] == "1A-L3-P21"


def test_legacy_book_lookup_remains_compatible():
    assert get_aruco_id("1984") == 1
    assert get_book("missing") is None


def test_fixed_grid_book_search_uses_title_id_and_location_code():
    assert find_book("deep learning")["book_id"] == "BK001"
    assert find_book("bk001")["location_code"] == "1A-L3-P21"
    assert find_book("1a-l3-p21")["title"] == "Deep Learning"
    assert search_books("learn")[0]["box_id"] == "1A"
    assert find_book("unknown") is None
