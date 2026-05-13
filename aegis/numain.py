#!/usr/bin/env python3
import cv2
import json
import time
import os
import sys
import subprocess
import tempfile
import threading
from datetime import datetime
from pathlib import Path
from queue import Queue
import base64
import io

def _ensure_deps():
    pkgs = {"ultralytics": "ultralytics", "supervision": "supervision", "flask": "flask", "flask-cors": "flask-cors"}
    for mod, pkg in pkgs.items():
        try:
            __import__(mod)
        except ImportError:
            print(f"[*] Installing {pkg}…")
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", pkg,
                 "--break-system-packages", "-q"]
            )

_ensure_deps()

from flask import Flask, render_template, request, jsonify, send_file
from flask_cors import CORS
from ultralytics import YOLO
import supervision as sv
import numpy as np


YOLO_MODEL = "model.pt"
CONF_THRESH = 0.30
IOU_THRESH = 0.45
TRACK_THRESH = 0.35
TRACK_BUFFER = 30
MATCH_THRESH = 0.80
FRAME_RATE = 30
REPORTS_DIR = ("scouting_reports")
TRAIL_LEN = 60

BALL_H_LO, BALL_H_HI = 18, 35
BALL_S_LO, BALL_S_HI = 80, 255
BALL_V_LO, BALL_V_HI = 80, 255
BALL_MIN_AREA = 60
BALL_MAX_AREA = 8000
BALL_MIN_CIRC = 0.55

# Colors (BGR)
ORANGE = (0, 165, 255)
GREEN = (0, 220, 80)
RED = (0, 60, 220)
CYAN = (255, 210, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (120, 120, 120)
BLUE = (220, 100, 0)
YELLOW = (0, 220, 220)

app = Flask(__name__)
CORS(app)


class ScoutingSession:
    def __init__(self, team_number: str, bumper_color: str = "red"):
        self.team_number = team_number
        self.bumper_color = bumper_color.lower()
        self.start_time = datetime.now().isoformat()
        self.events = []
        self.tracking_data = []
        self.lost_count = 0
        self.frames_tracked = 0
        self.shot_count = 0

    def log_position(self, frame_idx: int, fps: float, cx: int, cy: int, w: int, h: int):
        t_sec = frame_idx / fps if fps > 0 else 0
        self.tracking_data.append({
            "frame": frame_idx, "t": round(t_sec, 2),
            "cx": cx, "cy": cy, "w": w, "h": h,
        })
        self.frames_tracked += 1

    def log_event(self, frame_idx: int, fps: float, label: str):
        t_sec = frame_idx / fps if fps > 0 else 0
        self.events.append({"frame": frame_idx, "t": round(t_sec, 2), "event": label})

    def to_dict(self):
        return {
            "team_number": self.team_number,
            "bumper_color": self.bumper_color.upper(),
            "scouted_at": self.start_time,
            "frames_tracked": self.frames_tracked,
            "tracking_lost": self.lost_count,
            "shot_count": self.shot_count,
            "events": self.events,
        }

    def save(self) -> str:
        REPORTS_DIR.mkdir(exist_ok=True)
        safe = "".join(c for c in self.team_number if c.isalnum() or c in "_-")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = REPORTS_DIR / f"team{safe}_{stamp}.json"

        total_dist = 0.0
        prev = None
        for pt in self.tracking_data:
            if prev:
                dx = pt["cx"] - prev["cx"]
                dy = pt["cy"] - prev["cy"]
                total_dist += (dx**2 + dy**2) ** 0.5
            prev = pt

        report = {
            "team_number": self.team_number,
            "bumper_color": self.bumper_color.upper(),
            "scouted_at": self.start_time,
            "frames_tracked": self.frames_tracked,
            "tracking_lost": self.lost_count,
            "pixel_distance": round(total_dist, 1),
            "shot_count": self.shot_count,
            "events": self.events,
            "tracking_data": self.tracking_data,
        }
        path.write_text(json.dumps(report, indent=2))
        return str(path)

active_session = None
processing_thread = None
stop_processing = False
video_queue = Queue(maxsize=1)

def bumper_bgr(bumper_color: str) -> tuple:
    return BLUE if bumper_color.lower() == "blue" else RED

def detect_yellow_balls(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([BALL_H_LO, BALL_S_LO, BALL_V_LO])
    upper = np.array([BALL_H_HI, BALL_S_HI, BALL_V_HI])
    mask = cv2.inRange(hsv, lower, upper)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    balls = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if BALL_MIN_AREA <= area <= BALL_MAX_AREA:
            (cx, cy), r = cv2.minEnclosingCircle(cnt)
            circularity = 4 * np.pi * area / (cv2.arcLength(cnt, True) ** 2 + 1e-5)
            if circularity >= BALL_MIN_CIRC:
                balls.append((int(cx), int(cy), int(r)))
    
    return balls

def draw_box_tactical(frame, x1, y1, x2, y2, color, track_id=None, locked=False):
    w, h = x2 - x1, y2 - y1
    L = min(18, w // 3, h // 3)
    thickness = 3 if locked else 2
    corners = [(x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)]
    for (px, py, dx, dy) in corners:
        cv2.line(frame, (px, py), (px + dx*L, py), color, thickness)
        cv2.line(frame, (px, py), (px, py + dy*L), color, thickness)

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 14, 2)

    if track_id is not None:
        label = f"#{track_id}"
        cv2.putText(frame, label, (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

def frame_to_base64(frame):
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
    return base64.b64encode(buffer).decode('utf-8')

def process_video(video_path: str, team_number: str, bumper_color: str):
    global active_session, stop_processing

    print(f"[*] Loading YOLO model ({YOLO_MODEL})…")
    try:
        model = YOLO(YOLO_MODEL)
    except:
        print("[!] YOLO model not found. Using mock detections.")
        model = None

    tracker = sv.ByteTrack(
        track_activation_threshold=TRACK_THRESH,
        lost_track_buffer=TRACK_BUFFER,
        minimum_matching_threshold=MATCH_THRESH,
        frame_rate=FRAME_RATE,
    )

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[!] Cannot open video: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or float(FRAME_RATE)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    active_session = ScoutingSession(team_number, bumper_color)
    bcolor = bumper_bgr(bumper_color)
    locked_id = None
    frame_idx = 0
    status = "UNTRACKED"
    trail = []

    print(f"[*] Processing: {fps:.1f} fps, {total_frames} frames")

    while not stop_processing:
        ret, frame = cap.read()
        if not ret:
            print("[*] End of video.")
            break

        frame_idx += 1

        sv_dets = None
        if model:
            results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)[0]
            sv_dets = sv.Detections.from_ultralytics(results)
            sv_dets = tracker.update_with_detections(sv_dets)

        if sv_dets and len(sv_dets) > 0:
            for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                x1, y1, x2, y2 = map(int, bbox)
                is_locked = (tid == locked_id)
                color = bcolor if is_locked else GRAY
                draw_box_tactical(frame, x1, y1, x2, y2, color, track_id=tid, locked=is_locked)

        balls = detect_yellow_balls(frame)
        for cx, cy, r in balls:
            cv2.circle(frame, (cx, cy), r, YELLOW, 2)

        h_f, w_f = frame.shape[:2]
        t_sec = frame_idx / fps if fps > 0 else 0
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w_f, 52), BLACK, -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        cv2.putText(frame, f"TEAM  {team_number}", (12, 32),
                    cv2.FONT_HERSHEY_DUPLEX, 0.85, ORANGE, 2)
        cv2.circle(frame, (210, 20), 7, bcolor, -1)

        cv2.putText(frame, f"t={t_sec:.1f}s  f={frame_idx}/{total_frames}  shots={active_session.shot_count}",
                    (w_f - 350, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)

        frame_b64 = frame_to_base64(frame)
        try:
            if video_queue.full():
                video_queue.get_nowait()

                video_queue.put_nowait({
                    "frame": frame_b64,
                    "frame_idx": frame_idx,
                    "total_frames": total_frames,
                    "time": t_sec,
                    "status": "TRACKING" if locked_id else "SEARCHING",
                    "shot_count": active_session.shot_count,
                })
        except:
            pass

        time.sleep(0.001) 

    cap.release()
    print("[*] Video processing complete.")


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/start-session', methods=['POST'])
def start_session():
    global processing_thread, stop_processing
    
    data = request.json
    team_number = data.get('team_number', 'Unknown')
    bumper_color = data.get('bumper_color', 'red')
    video_path = data.get('video_path', '')

    if not video_path or not os.path.exists(video_path):
        return jsonify({"error": "Video file not found"}), 400

    stop_processing = False
    processing_thread = threading.Thread(
        target=process_video,
        args=(video_path, team_number, bumper_color),
        daemon=True
    )
    processing_thread.start()

    return jsonify({"status": "started", "team": team_number})

@app.route('/api/get-frame', methods=['GET'])
def get_frame():
    try:
        frame_data = video_queue.get(timeout=2)
        return jsonify(frame_data)
    except:
        return jsonify({"error": "No frame available"}), 204

@app.route('/api/log-shot', methods=['POST'])
def log_shot():
    global active_session
    if active_session:
        active_session.shot_count += 1
        return jsonify({"shot_count": active_session.shot_count})
    return jsonify({"error": "No active session"}), 400

@app.route('/api/log-event', methods=['POST'])
def log_event():
    global active_session
    data = request.json
    label = data.get('label', 'event')
    if active_session:
        active_session.log_event(0, 30, label)
        return jsonify({"events": len(active_session.events)})
    return jsonify({"error": "No active session"}), 400

@app.route('/api/save-session', methods=['POST'])
def save_session():
    global active_session, stop_processing
    stop_processing = True
    if active_session:
        path = active_session.save()
        return jsonify({
            "saved": True,
            "path": str(path),
            "session": active_session.to_dict()
        })
    return jsonify({"error": "No active session"}), 400

@app.route('/api/session-status', methods=['GET'])
def session_status():
    global active_session
    if active_session:
        return jsonify(active_session.to_dict())
    return jsonify({"error": "No active session"}), 400

@app.route('/api/list-reports', methods=['GET'])
def list_reports():
    if not REPORTS_DIR.exists():
        return jsonify([])
    reports = sorted(REPORTS_DIR.glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True)
    return jsonify([{"name": p.name, "path": str(p)} for p in reports[:20]])

@app.route('/api/upload-video', methods=['POST'])
def upload_video():
    if 'file' not in request.files:
        return jsonify({"error": "No file"}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No filename"}), 400
    
    uploads_dir = Path("uploads")
    uploads_dir.mkdir(exist_ok=True)
    
    filepath = uploads_dir / file.filename
    file.save(str(filepath))
    
    return jsonify({"path": str(filepath), "filename": file.filename})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=1768)