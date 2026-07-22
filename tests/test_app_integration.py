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
