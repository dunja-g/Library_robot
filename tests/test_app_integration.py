import importlib


def test_mock_app_book_request_status_and_reset(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_USE_MOCK", "true")
    module = importlib.import_module("pi.app")
    client = module.app.test_client()

    books = client.get("/books")
    assert books.status_code == 200
    assert "1984" in books.get_json()

    request = client.post("/request_book", json={"title": "1984"})
    assert request.status_code == 200
    assert request.get_json()["aruco_id"] == 1
    assert client.get("/status").get_json()["state"] == "SCANNING"

    reset = client.post("/reset")
    assert reset.status_code == 200
    assert client.get("/status").get_json()["state"] == "IDLE"


def test_mock_deep_learning_mission_runs_to_dock(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_USE_MOCK", "true")
    module = importlib.import_module("pi.app")
    module.controller.reset()
    client = module.app.test_client()

    response = client.post("/request_book", json={"title": "Deep Learning"})
    payload = response.get_json()
    assert response.status_code == 200
    assert payload["destination"] == {"zone": "B", "shelf": "B3", "level": 3, "slot": 12}
    assert payload["route"] == [101, 105, 203]

    for _ in range(100):
        module.controller.step()
        if module.controller.get_state() == "DOCKED":
            break
    status = client.get("/status").get_json()
    assert status["state"] == "DOCKED"
    assert status["phase"] == "COMPLETE"
    assert status["return_route"] == [105, 101, 0]


def test_mock_grid_mode_accepts_box_route(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_USE_MOCK", "true")
    monkeypatch.setenv("LIBRARY_ROBOT_NAVIGATION_MODE", "grid")
    module = importlib.reload(importlib.import_module("pi.app"))
    module.controller.destination_dwell_seconds = 0
    client = module.app.test_client()

    assert client.get("/boxes").get_json() == [
        "1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"
    ]
    response = client.post("/request_box", json={"box_id": "3b"})
    assert response.status_code == 200
    assert response.get_json()["box_id"] == "3B"

    for _ in range(20):
        module.controller.step()
        if module.controller.get_state() == "DOCKED":
            break
    status = client.get("/status").get_json()
    assert status["state"] == "DOCKED"
    assert status["phase"] == "COMPLETE"
    assert status["navigation_mode"] == "grid_encoder"
