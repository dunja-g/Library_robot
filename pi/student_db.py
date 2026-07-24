"""
Student database module for the Library Robot.
Manages reading and writing to data/students.json in a thread-safe manner.
"""
import json
import os
import threading
import tempfile
from datetime import datetime

# Path to the data directory (one level up from this file's directory)
DATA_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data', 'students.json')

# Module-level lock for thread safety
_db_lock = threading.Lock()

def get_all_students() -> list[dict]:
    """Reads and returns all students from the JSON file."""
    with _db_lock:
        return _get_all_students_no_lock()

def _get_all_students_no_lock() -> list[dict]:
    """Reads and returns all students from the JSON file without acquiring the lock."""
    if not os.path.exists(DATA_FILE):
        return []
    try:
        with open(DATA_FILE, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return []

def _save(students: list):
    """Private helper that writes back to the JSON file atomically."""
    # Assume the caller already holds _db_lock
    dir_name = os.path.dirname(DATA_FILE)
    if not os.path.exists(dir_name):
        os.makedirs(dir_name, exist_ok=True)
        
    fd, temp_path = tempfile.mkstemp(dir=dir_name, text=True)
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(students, f, indent=2)
        os.replace(temp_path, DATA_FILE)
    except Exception as e:
        os.remove(temp_path)
        raise e

def get_student_by_qr(qr_code: str) -> dict | None:
    """Find a student by their QR string."""
    students = get_all_students()
    for student in students:
        if student.get("qr_code") == qr_code:
            return student
    return None

def get_student_by_id(student_id: str) -> dict | None:
    """Find a student by their ID."""
    students = get_all_students()
    for student in students:
        if student.get("id") == student_id:
            return student
    return None

def borrow_book(student_id: str, book_id: str) -> dict:
    """
    Mark a student as having borrowed a book.
    Returns {"ok": True} or {"ok": False, "reason": "already_borrowed"}.
    Sets borrowed_at to ISO timestamp.
    """
    with _db_lock:
        students = _get_all_students_no_lock()
        for student in students:
            if student.get("id") == student_id:
                if student.get("borrowed_book_id"):
                    return {"ok": False, "reason": "already_borrowed"}
                student["borrowed_book_id"] = book_id
                student["borrowed_at"] = datetime.now().isoformat()
                _save(students)
                return {"ok": True}
        return {"ok": False, "reason": "student_not_found"}

def return_book(student_id: str) -> dict:
    """
    Mark a student's book as returned.
    Returns {"ok": True, "book_id": "..."} or {"ok": False, "reason": "no_active_loan"}.
    """
    with _db_lock:
        students = _get_all_students_no_lock()
        for student in students:
            if student.get("id") == student_id:
                book_id = student.get("borrowed_book_id")
                if not book_id:
                    return {"ok": False, "reason": "no_active_loan"}
                student["borrowed_book_id"] = None
                student["borrowed_at"] = None
                _save(students)
                return {"ok": True, "book_id": book_id}
        return {"ok": False, "reason": "student_not_found"}
