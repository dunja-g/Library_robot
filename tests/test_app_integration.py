import importlib


def load_mock_app(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_USE_MOCK", "true")
    module = importlib.reload(importlib.import_module("pi.app"))
    module.controller.destination_dwell_seconds = 0
    module.controller.reset()
    return module, module.app.test_client()


def test_app_is_fixed_grid_only_and_lists_numbered_books(monkeypatch):
    _module, client = load_mock_app(monkeypatch)
    mode = client.get("/navigation_mode").get_json()
    assert mode["mode"] == "grid"
    assert mode["marker_scanning"] is False
    assert "Deep Learning" in client.get("/books").get_json()
    assert client.post("/request_box", json={"box_id": "1A"}).status_code == 404


def test_dashboard_contains_search_map_and_live_status_controls(monkeypatch):
    _module, client = load_mock_app(monkeypatch)
    page = client.get("/").get_data(as_text=True)
    assert "Smart Library" in page
    assert 'id="book-search"' in page
    assert 'data-box="4B"' in page
    assert 'id="encoder-health"' in page
    assert 'id="camera-connection"' in page
    assert 'id="camera-fps"' in page
    assert 'id="camera-latency"' in page
    assert 'id="reset-btn"' in page


def test_search_by_title_book_id_location_and_partial_text(monkeypatch):
    _module, client = load_mock_app(monkeypatch)
    for query in ("Deep Learning", "BK001", "1A-L3-P21", "deep"):
        results = client.get("/search_books", query_string={"q": query}).get_json()
        assert len(results) == 1
        assert results[0]["title"] == "Deep Learning"
        assert results[0]["location_code"] == "1A-L3-P21"
        assert results[0]["subtitle"] == "A Modern Approach"
        assert results[0]["rating"] == 4.8
        assert results[0]["tags"] == ["AI", "Machine Learning", "Neural Networks"]


def test_book_number_dispatches_without_marker_scan(monkeypatch):
    module, client = load_mock_app(monkeypatch)
    response = client.post("/request_book", json={"query": "BK001"})
    payload = response.get_json()
    assert response.status_code == 200
    assert payload["location_code"] == "1A-L3-P21"
    assert payload["destination"] == {
        "box_id": "1A",
        "layer": 3,
        "position": 21,
    }

    status = client.get("/status").get_json()
    assert status["state"] == "MOVING"
    assert status["camera"]["status"] == "MOCK"
    assert status["book"] == "Deep Learning"
    assert status["current_action"] == "FORWARD"
    assert "current_marker_id" not in status

    for _ in range(20):
        module.controller.step()
        if module.controller.get_state() == "DOCKED":
            break
    status = client.get("/status").get_json()
    assert status["state"] == "DOCKED"
    assert status["phase"] == "COMPLETE"


def test_partial_unique_book_query_can_start_and_reset(monkeypatch):
    _module, client = load_mock_app(monkeypatch)
    response = client.post("/request_book", json={"query": "CSAPP"})
    assert response.status_code == 200
    assert response.get_json()["title"] == "Computer Systems: CSAPP"
    assert client.post("/reset").status_code == 200
    assert client.get("/status").get_json()["state"] == "IDLE"


def test_unknown_book_does_not_move(monkeypatch):
    _module, client = load_mock_app(monkeypatch)
    response = client.post("/request_book", json={"query": "unknown"})
    assert response.status_code == 404
    assert client.get("/status").get_json()["state"] == "IDLE"
