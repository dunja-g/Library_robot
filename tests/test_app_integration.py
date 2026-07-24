import importlib
import json


def load_mock_app(monkeypatch, tmp_path):
    monkeypatch.setenv("LIBRARY_ROBOT_USE_MOCK", "true")
    student_path = tmp_path / "students.json"
    student_path.write_text(
        json.dumps(
            [
                {
                    "id": "S001",
                    "name": "Alice Tan",
                    "qr_code": "LIBSTU-S001",
                    "borrowed_book_id": None,
                    "borrowed_at": None,
                },
                {
                    "id": "S002",
                    "name": "Bob Lim",
                    "qr_code": "LIBSTU-S002",
                    "borrowed_book_id": None,
                    "borrowed_at": None,
                }
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LIBRARY_ROBOT_STUDENT_DB_PATH", str(student_path))
    importlib.reload(importlib.import_module("pi.student_db"))
    module = importlib.reload(importlib.import_module("pi.app"))
    module.controller.destination_dwell_seconds = 10
    module.controller.reset()
    return module, module.app.test_client()


def check_in(client):
    response = client.post("/api/checkin", json={"qr_code": "LIBSTU-S001"})
    assert response.status_code == 200


def test_app_is_fixed_grid_only_and_lists_numbered_books(monkeypatch, tmp_path):
    _module, client = load_mock_app(monkeypatch, tmp_path)
    mode = client.get("/navigation_mode").get_json()
    assert mode["mode"] == "grid"
    assert mode["marker_scanning"] is False
    assert mode["linear_source"] == "encoder"
    assert mode["turn_source"] == "imu"
    assert mode["return_strategy"] == "direct_reverse"
    assert mode["active_controller"] == "GridController"
    assert mode["legacy_robot_controller_active"] is False
    assert "Deep Learning" in client.get("/books").get_json()
    assert client.post("/request_box", json={"box_id": "1A"}).status_code == 404


def test_auto_return_setting_controls_pickup_confirmation(monkeypatch, tmp_path):
    monkeypatch.setenv("LIBRARY_ROBOT_AUTO_RETURN", "false")
    module, client = load_mock_app(monkeypatch, tmp_path)

    mode = client.get("/navigation_mode").get_json()
    plan = module._build_borrowing_plan(module.get_book("Deep Learning"))

    assert mode["auto_return"] is False
    assert plan["pickup_confirmation_required"] is True


def test_dashboard_contains_search_map_and_live_status_controls(monkeypatch, tmp_path):
    _module, client = load_mock_app(monkeypatch, tmp_path)
    page = client.get("/").get_data(as_text=True)
    assert "Smart Library" in page
    assert 'id="book-search"' in page
    assert 'data-box="3B"' in page
    assert 'data-box="4A"' not in page
    assert 'data-box="4B"' not in page
    assert 'id="encoder-health"' in page
    assert 'id="pickup-confirmation"' in page
    assert 'id="confirm-pickup-btn"' in page
    assert 'id="cancel-mission-btn"' in page
    assert 'id="reset-btn"' in page
    assert "/api/return_book" not in page
    assert page.count('data-stream-src="/video_feed"') == 2
    assert ' src="/video_feed"' not in page


def test_camera_status_is_available_without_opening_another_stream(
    monkeypatch, tmp_path
):
    _module, client = load_mock_app(monkeypatch, tmp_path)

    response = client.get("/camera_status")

    assert response.status_code == 200
    assert response.get_json()["running"] is True
    assert response.get_json()["stream_clients"] == 0
    assert client.post("/api/return_book", json={}).status_code == 404


def test_search_by_title_book_id_location_and_partial_text(monkeypatch, tmp_path):
    _module, client = load_mock_app(monkeypatch, tmp_path)
    for query in ("Deep Learning", "BK001", "1A-L3-P21", "deep"):
        results = client.get("/search_books", query_string={"q": query}).get_json()
        assert len(results) == 1
        assert results[0]["title"] == "Deep Learning"
        assert results[0]["location_code"] == "1A-L3-P21"
        assert results[0]["subtitle"] == "A Modern Approach"
        assert results[0]["rating"] == 4.8
        assert results[0]["tags"] == ["AI", "Machine Learning", "Neural Networks"]


def test_book_number_dispatches_without_marker_scan(monkeypatch, tmp_path):
    module, client = load_mock_app(monkeypatch, tmp_path)
    check_in(client)
    response = client.post("/request_book", json={"query": "BK001"})
    payload = response.get_json()
    assert response.status_code == 202
    assert payload["book"]["location_code"] == "1A-L3-P21"
    assert payload["mission"]["state"] == "pending"

    status = client.get("/status").get_json()
    assert status["state"] == "MOVING"
    assert status["book"] == "Deep Learning"
    assert status["current_action"] == "FORWARD"
    assert "current_marker_id" not in status

    for _ in range(10):
        module.controller.step()
        if module.controller.get_state() == "ARRIVED":
            break
    assert module.controller.get_state() == "ARRIVED"
    
    # Run steps to simulate dwell time
    for _ in range(20):
        module.controller.step()
        module._reconcile_borrowing_state()
        if module.controller.get_state() == "MOVING":
            break
            
    # Then wait for DOCKED
    for _ in range(20):
        module.controller.step()
        if module.controller.get_state() == "DOCKED":
            break
    module._reconcile_borrowing_state()
    status = client.get("/status").get_json()
    assert status["state"] == "DOCKED"
    assert status["phase"] == "COMPLETE"
    assert status["current_student"] is None
    assert status["borrowing_mission"] is None


def test_partial_unique_book_query_can_start_and_reset(monkeypatch, tmp_path):
    _module, client = load_mock_app(monkeypatch, tmp_path)
    check_in(client)
    response = client.post("/request_book", json={"query": "CSAPP"})
    assert response.status_code == 202
    assert response.get_json()["book"]["title"] == "Computer Systems: CSAPP"
    assert client.post("/reset").status_code == 200
    assert client.get("/status").get_json()["state"] == "IDLE"


def test_unknown_book_does_not_move(monkeypatch, tmp_path):
    _module, client = load_mock_app(monkeypatch, tmp_path)
    check_in(client)
    response = client.post("/request_book", json={"query": "unknown"})
    assert response.status_code == 404
    assert client.get("/status").get_json()["state"] == "IDLE"


def start_pending_mission(module, client):
    check_in(client)
    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )
    assert response.status_code == 202
    return response.get_json()


def reach_destination(module):
    for _ in range(10):
        module.controller.step()
        if module.controller.get_state() == "ARRIVED":
            break
    assert module.controller.get_state() == "ARRIVED"


def test_book_is_recorded_automatically_after_arrival(monkeypatch, tmp_path):
    module, client = load_mock_app(monkeypatch, tmp_path)
    payload = start_pending_mission(module, client)
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None

    reach_destination(module)
    
    # Still not recorded before reconcile
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None
    
    # Step to trigger auto-confirm
    module.controller.step()
    module._reconcile_borrowing_state()
    
    assert module.get_student_by_id("S001")["borrowed_book_id"] == "BK001"
    assert module.get_student_by_id("S001")["borrowed_book_id"] == "BK001"
    assert module.controller.get_status()["phase"] == "AT_DESTINATION"
    
    # Step 10 more times to simulate dwell ending and return trip starting
    for _ in range(10):
        module.controller.step()
    
    assert module.controller.get_status()["phase"] in {"RETURNING", "COMPLETE"}
    assert module.controller.plan["return"][0]["action"] == "BACKWARD"



def test_duplicate_request_and_checkin_are_rejected_while_active(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    start_pending_mission(module, client)

    duplicate = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK002"},
    )
    second_checkin = client.post(
        "/api/checkin", json={"qr_code": "LIBSTU-S002"}
    )
    module._on_qr_detected("LIBSTU-S002")

    assert duplicate.status_code == 409
    assert second_checkin.status_code == 409
    assert module._get_current_student()["id"] == "S001"


def test_obstacle_stop_cancels_pending_without_database_write(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    start_pending_mission(module, client)
    module.controller.serial.get_ultrasonic = lambda: {
        "left": 100,
        "center": 5,
        "right": 100,
    }

    module.controller.step()
    module._reconcile_borrowing_state()

    status = client.get("/status").get_json()
    assert status["state"] == "STOPPED"
    assert status["borrowing_mission"]["state"] == "cancelled"
    assert status["borrowing_mission"]["cancel_reason"] == "center_obstacle"
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None


def test_encoder_stall_cancels_pending_without_database_write(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    start_pending_mission(module, client)
    module.controller.serial.get_encoders = lambda: {"left": 0, "right": 0}
    module.controller._last_progress_at = module.controller._clock() - 3

    module.controller._current_step = lambda: {"action": "FORWARD", "target_ticks": 1000, "target_seconds": 0.0, "label": "Mock Encoder Step"}
    module.controller.step()
    module._reconcile_borrowing_state()

    mission = client.get("/status").get_json()["borrowing_mission"]
    assert mission["state"] == "cancelled"
    assert mission["cancel_reason"] == "encoder_stall"
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None


def test_reset_and_timeout_cancel_pending_operation(monkeypatch, tmp_path):
    module, client = load_mock_app(monkeypatch, tmp_path)
    start_pending_mission(module, client)
    assert client.post("/reset").status_code == 200
    status = client.get("/status").get_json()
    assert status["borrowing_mission"]["state"] == "cancelled"
    assert status["current_student"] is None
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None

    check_in(client)
    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )
    assert response.status_code == 202
    module._get_current_mission().created_at -= (
        module.MISSION_TIMEOUT_SECONDS + 1
    )
    module._reconcile_borrowing_state()
    mission = client.get("/status").get_json()["borrowing_mission"]
    assert mission["state"] == "cancelled"
    assert mission["cancel_reason"] == "mission_timeout"
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None


def test_serial_and_imu_failures_cancel_pending_operation(monkeypatch, tmp_path):
    module, client = load_mock_app(monkeypatch, tmp_path)
    check_in(client)
    module.controller.serial.send_forward = lambda: False
    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )
    assert response.status_code == 503
    assert module._get_current_mission().state.value == "cancelled"
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None

    client.post("/reset")
    check_in(client)
    module.controller.serial.send_forward = lambda: True
    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )
    assert response.status_code == 202
    for _ in range(5):
        module.controller.step()
        if module.controller.get_state() == "TURNING":
            break
    assert module.controller.get_state() == "TURNING"
    module.controller.serial.turn_status = "ERROR"
    module.controller.step()
    module._reconcile_borrowing_state()
    assert module._get_current_mission().cancel_reason == "imu_turn_error"
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None


def test_calibration_failure_creates_no_mission_or_database_write(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    check_in(client)
    module.grid_geometry = module.GridGeometry()

    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )

    assert response.status_code == 503
    assert module._get_current_mission() is None
    assert module.get_student_by_id("S001")["borrowed_book_id"] is None


def test_unavailable_book_is_rejected_before_mission_creation(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    assert module.borrow_book("S002", "BK001") == {"ok": True}
    check_in(client)

    response = client.post(
        "/api/borrow",
        json={"student_id": "S001", "book_query": "BK001"},
    )

    assert response.status_code == 409
    assert module._get_current_mission() is None
    assert module.controller.get_state() == "IDLE"


def test_reset_after_confirmation_preserves_loan_and_session(
    monkeypatch, tmp_path
):
    module, client = load_mock_app(monkeypatch, tmp_path)
    start_pending_mission(module, client)
    reach_destination(module)
    
    # Step to trigger auto-confirm
    module.controller.step()
    module._reconcile_borrowing_state()

    assert client.post("/reset").status_code == 200

    status = client.get("/status").get_json()
    assert status["current_student"]["id"] == "S001"
    assert status["borrowing_mission"]["state"] == "confirmed"
    assert module.get_student_by_id("S001")["borrowed_book_id"] == "BK001"
