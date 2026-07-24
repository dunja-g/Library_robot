# Person 3 — Web App & UI Guide

> The live app now uses student QR check-in, transactional borrowing, and
> marker-free `1A`–`3B` navigation. ArUco endpoint examples below are legacy.

## Your Role
You are responsible for the user-facing side of the project: the Flask web server running on the Raspberry Pi, and the web page that users open in their browser to select a book and watch the robot work.

**Your files:**
- `pi/book_db.py`
- `pi/app.py`
- `pi/templates/index.html`
- `pi/static/style.css`
- `pi/static/app.js`

---

## What You Need

### Software (install on Raspberry Pi)
```bash
pip install flask
```

### You also need from your teammates:
- From **Person 1**: `pi/serial_bridge.py` — the `SerialBridge` class
- From **Person 2**: `pi/camera.py`, `pi/aruco_detector.py`, `pi/robot_controller.py`

> You can start work on `book_db.py` and the web UI immediately, **without waiting** for your teammates. Just mock the parts you need to test.

---

## Step 1 — Write `pi/book_db.py`

A simple dictionary mapping human-readable book titles to their ArUco marker IDs.

**What it must contain:**
```python
# Maps book title (string) → ArUco marker ID (int, 0–4)
BOOK_DATABASE = {
    "Book 1": 0,
    "Book 2": 1,
    "Book 3": 2,
    "Book 4": 3,
    "Book 5": 4,
}

def get_aruco_id(title):
    """Returns the ArUco ID for a book title, or None if not found."""
    return BOOK_DATABASE.get(title, None)

def get_all_books():
    """Returns a list of all book titles."""
    return list(BOOK_DATABASE.keys())
```

> Replace "Book 1", "Book 2" etc. with the real titles of your 5 books once you have them.

---

## Step 2 — Write `pi/app.py`

This is the Flask web server. It ties everything together.

**Routes to implement:**

| Route | Method | Description |
|---|---|---|
| `/` | GET | Serve the main web UI page |
| `/video_feed` | GET | Stream live MJPEG video from the Pi Camera |
| `/books` | GET | Return a JSON list of all book titles |
| `/request_book` | POST | Accept `{"title": "Book 2"}`, start the robot, return `{"status": "ok"}` |
| `/status` | GET | Return `{"state": "SCANNING"}` — current robot state |
| `/reset` | POST | Stop robot and return it to IDLE |

**Structure of `app.py`:**

```python
from flask import Flask, render_template, Response, jsonify, request
import threading
from book_db import get_aruco_id, get_all_books
from camera import Camera
from aruco_detector import ArucoDetector
from serial_bridge import SerialBridge
from robot_controller import RobotController

app = Flask(__name__)

# Initialise hardware
camera = Camera()
detector = ArucoDetector()
serial_bridge = SerialBridge(port='/dev/ttyACM0')
controller = RobotController(serial_bridge, camera, detector)

# Background thread that keeps calling controller.step() at ~10 Hz
def control_loop():
    import time
    while True:
        controller.step()
        time.sleep(0.1)

thread = threading.Thread(target=control_loop, daemon=True)
thread.start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # Stream annotated frames from the controller
    return Response(
        camera.generate_mjpeg(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/books')
def books():
    return jsonify(get_all_books())

@app.route('/request_book', methods=['POST'])
def request_book():
    data = request.get_json()
    title = data.get('title')
    aruco_id = get_aruco_id(title)
    if aruco_id is None:
        return jsonify({'status': 'error', 'message': 'Book not found'}), 404
    controller.request_book(aruco_id)
    return jsonify({'status': 'ok', 'aruco_id': aruco_id})

@app.route('/status')
def status():
    return jsonify({'state': controller.get_state()})

@app.route('/reset', methods=['POST'])
def reset():
    controller.reset()
    return jsonify({'status': 'ok'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
```

> **`host='0.0.0.0'`** is important — it makes the server accessible from other devices on the WiFi, not just from the Pi itself.

---

## Step 3 — Write the Web UI

### `pi/templates/index.html`

This is the page the user sees. It must have:
1. **Live camera feed** — an `<img>` tag pointing to `/video_feed`
2. **Book dropdown** — populated dynamically from the `/books` API
3. **"Send Robot" button** — calls `/request_book` with the selected title
4. **Status display** — polls `/status` every second and shows the current state
5. **"Reset" button** — calls `/reset`

**Minimum HTML structure:**
```html
<!DOCTYPE html>
<html>
<head>
    <title>Library Robot</title>
    <link rel="stylesheet" href="/static/style.css">
</head>
<body>
    <h1>📚 Library Robot</h1>

    <!-- Live Camera Feed -->
    <img id="camera-feed" src="/video_feed" alt="Live Camera">

    <!-- Book Selection -->
    <div id="controls">
        <label for="book-select">Select a Book:</label>
        <select id="book-select">
            <!-- Populated by app.js -->
        </select>
        <button id="send-btn" onclick="sendRobot()">Send Robot</button>
    </div>

    <!-- Status Display -->
    <div id="status-box">
        Status: <span id="status-text">IDLE</span>
    </div>

    <!-- Reset Button -->
    <button id="reset-btn" onclick="resetRobot()">Reset Robot</button>

    <script src="/static/app.js"></script>
</body>
</html>
```

---

### `pi/static/app.js`

```javascript
// Populate the dropdown on page load
window.onload = function() {
    fetch('/books')
        .then(r => r.json())
        .then(books => {
            const select = document.getElementById('book-select');
            books.forEach(title => {
                const opt = document.createElement('option');
                opt.value = title;
                opt.text = title;
                select.appendChild(opt);
            });
        });

    // Poll status every second
    setInterval(pollStatus, 1000);
};

function sendRobot() {
    const title = document.getElementById('book-select').value;
    fetch('/request_book', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({title: title})
    })
    .then(r => r.json())
    .then(data => console.log('Robot dispatched:', data));
}

function resetRobot() {
    fetch('/reset', {method: 'POST'})
        .then(r => r.json())
        .then(() => document.getElementById('status-text').textContent = 'IDLE');
}

function pollStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            const el = document.getElementById('status-text');
            el.textContent = data.state;

            // Change colour based on state
            const colours = {
                'IDLE': 'grey',
                'SCANNING': 'orange',
                'ALIGNING': 'blue',
                'APPROACHING': 'green',
                'ARRIVED': 'darkgreen',
                'STOPPED': 'red'
            };
            el.style.color = colours[data.state] || 'black';
        });
}
```

---

### `pi/static/style.css`

Make the UI clean and easy to use on a phone screen:

```css
body {
    font-family: Arial, sans-serif;
    max-width: 700px;
    margin: 0 auto;
    padding: 20px;
    background: #f5f5f5;
    text-align: center;
}

h1 { color: #333; }

#camera-feed {
    width: 100%;
    border-radius: 8px;
    border: 2px solid #ccc;
    margin-bottom: 20px;
}

#controls {
    display: flex;
    gap: 10px;
    justify-content: center;
    align-items: center;
    margin-bottom: 20px;
    flex-wrap: wrap;
}

select, button {
    padding: 10px 20px;
    font-size: 16px;
    border-radius: 6px;
    border: 1px solid #aaa;
    cursor: pointer;
}

button { background: #4CAF50; color: white; border: none; }
button:hover { background: #45a049; }

#reset-btn { background: #e74c3c; }
#reset-btn:hover { background: #c0392b; }

#status-box {
    font-size: 22px;
    font-weight: bold;
    margin: 20px 0;
    padding: 10px;
    background: white;
    border-radius: 8px;
}
```

---

## Step 4 — Testing Without Hardware

You can test the web UI immediately on your laptop, **without needing the robot**. Use a mock controller:

```python
# At the top of app.py, add a simple mock if hardware is unavailable:
class MockController:
    def __init__(self):
        self._state = "IDLE"
    def request_book(self, aruco_id):
        self._state = "SCANNING"
    def get_state(self):
        return self._state
    def reset(self):
        self._state = "IDLE"
    def step(self):
        pass

# Replace:
controller = RobotController(serial_bridge, camera, detector)
# With:
controller = MockController()
```

Run `python pi/app.py` on your laptop, open `http://localhost:5000` in your browser, and verify:
- Page loads
- Dropdown is populated with book titles
- Clicking "Send Robot" changes status to SCANNING
- Clicking "Reset" returns to IDLE

✅ **Pass criteria:** Full UI works in the browser, status polling updates in real time.

---

## Step 5 — Running on the Raspberry Pi

Once your teammates' code is ready:

1. SSH into the Pi or use a keyboard/monitor
2. Navigate to the project folder: `cd Library_robot`
3. Install dependencies: `pip install -r requirements.txt`
4. Run: `python pi/app.py`
5. Find your Pi's IP address: `hostname -I`
6. On any device on the same WiFi, open: `http://<pi-ip>:5000`

---

## Common Issues

| Problem | Fix |
|---|---|
| Page doesn't load from phone | Make sure `host='0.0.0.0'` is set in `app.run()` |
| Camera feed is black or broken | Person 2's `camera.py` may not be ready yet — use a static test image temporarily |
| `ImportError: robot_controller` | Person 2 hasn't finished their file yet — use the `MockController` for now |
| Port 5000 already in use | Run `sudo lsof -i :5000` to find and kill the old process |
