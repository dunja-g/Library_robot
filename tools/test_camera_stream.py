#!/usr/bin/env python3
"""Simple Standalone Camera Diagnostic Script for Raspberry Pi.

Run this script to test hardware camera capture speed and web streaming
independently of the main robot navigation system:

    python tools/test_camera_stream.py

Access http://<pi-ip>:5000 in your browser to view the real-time live feed.
"""

from __future__ import annotations

import os
import sys
import time
import threading
from flask import Flask, Response

import cv2
import numpy as np

# Ensure project root is in python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

app = Flask(__name__)

# Global frame buffer & lock
latest_jpeg: bytes | None = None
frame_lock = threading.Lock()
stats = {"frames_captured": 0, "fps": 0.0, "backend": "Unknown"}


def start_camera_capture():
    global latest_jpeg, stats

    backend_name = "Unknown"
    camera = None
    cap = None

    # 1. Try Picamera2 (Raspberry Pi CSI Camera)
    try:
        from picamera2 import Picamera2
        print("[Camera Test] Initializing Picamera2 (CSI Camera)...")
        camera = Picamera2()
        config = camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameRate": 20},
            buffer_count=1,
        )
        camera.configure(config)
        camera.start()
        backend_name = "Picamera2 (CSI)"
    except Exception as err:
        print(f"[Camera Test] Picamera2 not available ({err}). Falling back to OpenCV USB camera...")
        # 2. Fallback to OpenCV (USB Camera)
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[Camera Test] ERROR: Unable to open USB camera on index 0!")
            return
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 20)
        backend_name = "OpenCV (USB /dev/video0)"

    stats["backend"] = backend_name
    print(f"[Camera Test] Successfully started camera backend: {backend_name}")

    frame_count = 0
    start_time = time.monotonic()

    while True:
        try:
            if camera is not None:
                # Picamera2 RGB frame
                rgb = camera.capture_array("main")
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif cap is not None:
                # OpenCV BGR frame
                ret, bgr = cap.read()
                if not ret or bgr is None:
                    time.sleep(0.05)
                    continue

            # Overlay stats text on frame
            frame_count += 1
            now = time.monotonic()
            elapsed = now - start_time
            current_fps = frame_count / elapsed if elapsed > 0 else 0.0
            stats["fps"] = round(current_fps, 1)
            stats["frames_captured"] = frame_count

            timestamp_str = time.strftime("%H:%M:%S")
            overlay_text = f"{backend_name} | {stats['fps']} FPS | {timestamp_str}"
            cv2.putText(
                bgr,
                overlay_text,
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Encode to JPEG
            ok, jpeg_buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with frame_lock:
                    latest_jpeg = jpeg_buf.tobytes()

            time.sleep(0.01)
        except Exception as exc:
            print(f"[Camera Test] Capture error: {exc}")
            time.sleep(0.1)


@app.route("/")
def index_page():
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Camera Test Stream</title>
        <style>
            body {{ font-family: sans-serif; text-align: center; background: #0f172a; color: #fff; margin: 20px; }}
            h2 {{ color: #38bdf8; }}
            img {{ border: 3px solid #38bdf8; border-radius: 8px; max-width: 90%; height: auto; }}
            .info {{ margin: 15px; color: #94a3b8; font-size: 1.1em; }}
        </style>
    </head>
    <body>
        <h2>📷 Standalone Camera Test Feed</h2>
        <div class="info">
            Backend: <b style="color:#4ade80">{stats['backend']}</b> &nbsp;|&nbsp; 
            FPS: <b id="fps-val" style="color:#facc15">{stats['fps']}</b>
        </div>
        <img id="test-feed" src="/api/frame.jpg" alt="Camera Feed" />
        <script>
            // Zero-latency fast sequential loader
            const img = document.getElementById('test-feed');
            function loadFrame() {{
                const next = new Image();
                next.onload = () => {{
                    img.src = next.src;
                    setTimeout(loadFrame, 30); // ~30fps max refresh
                }};
                next.onerror = () => setTimeout(loadFrame, 200);
                next.src = '/api/frame.jpg?t=' + Date.now();
            }}
            loadFrame();
        </script>
    </body>
    </html>
    """


@app.route("/api/frame.jpg")
def single_frame():
    with frame_lock:
        data = latest_jpeg
    if data is None:
        return Response(status=204)
    res = Response(data, mimetype="image/jpeg")
    res.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return res


if __name__ == "__main__":
    # Start background capture thread
    t = threading.Thread(target=start_camera_capture, daemon=True)
    t.start()

    print("\n=======================================================")
    print(" 📷 Standalone Camera Test Server Starting...")
    print(" Access in browser: http://localhost:5000 or http://<pi-ip>:5000")
    print(" Press Ctrl+C to stop.")
    print("=======================================================\n")

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
