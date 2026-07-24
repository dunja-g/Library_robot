import json

from pi import student_db


def seed_students(monkeypatch, tmp_path):
    path = tmp_path / "students.json"
    path.write_text(
        json.dumps(
            [
                {
                    "id": "S001",
                    "name": "Alice",
                    "qr_code": "LIBSTU-S001",
                    "borrowed_book_id": None,
                    "borrowed_at": None,
                }
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(student_db, "DATA_FILE", str(path))


def test_borrow_and_matching_rollback_are_atomic(monkeypatch, tmp_path):
    seed_students(monkeypatch, tmp_path)
    assert student_db.borrow_book("S001", "BK001") == {"ok": True}
    assert student_db.get_student_by_id("S001")["borrowed_book_id"] == "BK001"

    assert student_db.rollback_borrow_book("S001", "BK001") == {"ok": True}
    assert student_db.get_student_by_id("S001")["borrowed_book_id"] is None


def test_rollback_never_clears_a_different_loan(monkeypatch, tmp_path):
    seed_students(monkeypatch, tmp_path)
    student_db.borrow_book("S001", "BK002")

    result = student_db.rollback_borrow_book("S001", "BK001")

    assert result == {"ok": False, "reason": "loan_mismatch"}
    assert student_db.get_student_by_id("S001")["borrowed_book_id"] == "BK002"
