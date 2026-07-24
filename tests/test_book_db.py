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
    assert location["zone"] == "B"
    assert location["box_id"] == "1A"
    assert location["box_label"] == "8712"
    assert location["location_code"] == "1A-L3-P21"
    assert get_aruco_id("Deep Learning") == 203
    assert "Deep Learning" in get_all_books()
    assert "Deep Learning" in get_all_books(grid_only=True)
    assert len(get_all_books(grid_only=True)) == 12
    assert all(
        get_book(title)["box_id"] in {"1A", "1B", "2A", "2B", "3A", "3B"}
        for title in get_all_books(grid_only=True)
    )
    assert book["location_code"] == "1A-L3-P21"


def test_legacy_book_lookup_remains_compatible():
    assert get_aruco_id("1984") == 1
    assert get_book("missing") is None


def test_fixed_grid_book_search_uses_title_id_and_location_code():
    assert find_book("deep learning")["book_id"] == "BK001"
    assert find_book("bk001")["location_code"] == "1A-L3-P21"
    assert find_book("1a-l3-p21")["title"] == "Deep Learning"
    assert find_book("8716")["box_id"] == "1B"
    assert search_books("clean")[0]["box_id"] == "2A"
    assert find_book("unknown") is None
