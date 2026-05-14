#!/usr/bin/env python3
"""
FRC Robot Scouting Backend (Flask)
Real-time video processing with YOLO + ByteTrack robot tracking
Streams frames to web UI with live statistics
"""

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
from robot_tracker import RobotTracker
from shot_tracker import BallTracker, OptimizedBallDetector

def _ensure_deps():
    pkgs = {
        "ultralytics": "ultralytics",
        "supervision": "supervision",
        "flask": "flask",
        "flask-cors": "flask-cors"
    }
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

# ===== CONFIGURATION =====
YOLO_MODEL = "model.pt"
CONF_THRESH = 0.30
IOU_THRESH = 0.45
TRACK_THRESH = 0.35
TRACK_BUFFER = 30
MATCH_THRESH = 0.80
FRAME_RATE = 30
REPORTS_DIR = Path("scouting_reports")
UPLOADS_DIR = Path("uploads")

# Ball detection config
BALL_H_LO, BALL_H_HI = 18, 35
BALL_S_LO, BALL_S_HI = 80, 255
BALL_V_LO, BALL_V_HI = 80, 255
BALL_MIN_AREA = 60
BALL_MAX_AREA = 8000
BALL_MIN_CIRC = 0.55
SHOT_ORIGIN_RADIUS = 60

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

app = Flask(__name__, template_folder="templates/", static_folder=".")
CORS(app)

# ===== BALL DETECTOR =====
ball_detector = OptimizedBallDetector(
    h_range=(BALL_H_LO, BALL_H_HI),
    s_range=(BALL_S_LO, BALL_S_HI),
    v_range=(BALL_V_LO, BALL_V_HI),
    min_area=BALL_MIN_AREA,
    max_area=BALL_MAX_AREA,
    min_circularity=BALL_MIN_CIRC,
)

def detect_yellow_balls(frame):
    return ball_detector.detect(frame)

# ===== SESSION MANAGEMENT =====
class ScoutingSession:
    def __init__(self, team_number: str, bumper_color: str = "red", video_path: str = ""):
        self.team_number = team_number
        self.bumper_color = bumper_color.lower()
        self.video_path = video_path
        self.start_time = datetime.now().isoformat()
        self.events = []
        self.tracking_data = []
        self.shot_count = 0
        self.frame_idx = 0
        self.fps = 30.0
        self.total_frames = 0

    def log_position(self, frame_idx: int, cx: int, cy: int, w: int, h: int):
        t_sec = frame_idx / self.fps if self.fps > 0 else 0
        self.tracking_data.append({
            "frame": frame_idx,
            "t": round(t_sec, 2),
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
        })

    def log_event(self, frame_idx: int, label: str):
        t_sec = frame_idx / self.fps if self.fps > 0 else 0
        self.events.append({
            "frame": frame_idx,
            "t": round(t_sec, 2),
            "event": label
        })

    def to_dict(self):
        return {
            "team_number": self.team_number,
            "bumper_color": self.bumper_color.upper(),
            "scouted_at": self.start_time,
            "frames_tracked": len(self.tracking_data),
            "shot_count": self.shot_count,
            "events": self.events,
        }

    def save(self, robot_stats: dict) -> str:
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
                total_dist += (dx ** 2 + dy ** 2) ** 0.5
            prev = pt

        report = {
            "team_number": self.team_number,
            "bumper_color": self.bumper_color.upper(),
            "video_source": self.video_path,
            "scouted_at": self.start_time,
            "frames_tracked": len(self.tracking_data),
            "tracking_lost": robot_stats.get("lost_count", 0),
            "pixel_distance": round(total_dist, 1),
            "shot_count": self.shot_count,
            "events": self.events,
            "tracking_data": self.tracking_data,
        }
        path.write_text(json.dumps(report, indent=2))
        return str(path)

# ===== GLOBAL STATE =====
active_session = None
processing_thread = None
stop_processing = False
is_paused = False          # NEW: pause flag
pause_condition = threading.Condition()  # NEW: condition variable for pausing
video_queue = Queue(maxsize=1)
robot_tracker = None
ball_tracker = None

# Last rendered frame dimensions for robot click selection
last_frame_dims = {"w": 1, "h": 1}

# Last raw (un-annotated) frame — shared with Flask routes so that
# lock_to_detection / lock_to_manual_roi can init CSRT even when paused
last_raw_frame = None
last_raw_frame_lock = threading.Lock()

# ===== DRAWING FUNCTIONS =====
def bumper_bgr(bumper_color: str) -> tuple:
    return BLUE if bumper_color.lower() == "blue" else RED

def draw_box_tactical(frame, x1, y1, x2, y2, color, track_id=None, locked=False):
    """Draw tactical corner-style bounding box."""
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
        prefix = "★ " if locked else ""
        cv2.putText(frame, prefix + label,
                    (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

def draw_all_detections(frame, detections, locked_id, bumper_col):
    """Draw all YOLO detections."""
    if detections is None or len(detections) == 0:
        return
    for bbox, tid in zip(detections.xyxy, detections.tracker_id):
        x1, y1, x2, y2 = map(int, bbox)
        is_locked = (tid == locked_id)
        color = bumper_col if is_locked else GRAY
        draw_box_tactical(frame, x1, y1, x2, y2, color, track_id=tid, locked=is_locked)

def draw_trail(frame, trail, color):
    """Draw robot movement trail."""
    for i in range(1, len(trail)):
        alpha = i / len(trail)
        col = (int(color[0]*alpha), int(color[1]*alpha), int(color[2]*alpha))
        cv2.line(frame, trail[i-1], trail[i], col, 2)

def draw_balls(frame, balls, owned_ids, robot_box):
    """Draw detected balls."""
    for tid, cx, cy, r in balls:
        owned = tid in owned_ids
        color = GREEN if owned else YELLOW
        cv2.circle(frame, (int(cx), int(cy)), int(r), color, 2)
        cv2.circle(frame, (int(cx), int(cy)), 2, WHITE, -1)
        cv2.putText(frame, f"B{tid}", (int(cx) + 5, int(cy) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        if owned:
            cv2.circle(frame, (int(cx), int(cy)), int(r + 4), CYAN, 1)

def draw_fuel_counter(frame, robot_box, shot_count, bumper_color):
    """Draw shot counter display."""
    if robot_box is None:
        return
    x1, y1, x2, y2 = robot_box
    h_f, w_f = frame.shape[:2]
    
    pad_x, pad_y = 10, 6
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.65
    font_thick = 2

    label = f"FUEL  {shot_count}"
    (lw, lh), _ = cv2.getTextSize(label, font, font_scale, font_thick)

    pill_w = lw + pad_x * 2
    pill_h = lh + pad_y * 2

    cx_box = (x1 + x2) // 2
    px1 = max(4, min(cx_box - pill_w // 2, w_f - pill_w - 4))
    py2 = max(pill_h + 4, y1 - 8)
    py1 = py2 - pill_h

    overlay = frame.copy()
    cv2.rectangle(overlay, (px1, py1), (px1 + pill_w, py2), BLACK, -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    border_col = BLUE if bumper_color.lower() == "blue" else RED
    cv2.rectangle(frame, (px1, py1), (px1 + pill_w, py2), border_col, 2)

    cv2.putText(frame, "FUEL", (px1 + pad_x, py1 + pad_y + lh - 2),
                font, font_scale * 0.7, YELLOW, font_thick - 1)
    count_str = str(shot_count)
    (cw, _), _ = cv2.getTextSize(count_str, font, font_scale, font_thick)
    cv2.putText(frame, count_str,
                (px1 + pill_w - pad_x - cw, py1 + pad_y + lh - 2),
                font, font_scale, WHITE, font_thick)

def draw_hud(frame, session, status, frame_idx, fps, bumper_color, paused=False):
    """Draw main HUD overlay."""
    h_f, w_f = frame.shape[:2]
    t_sec = frame_idx / fps if fps > 0 else 0
    overlay = frame.copy()

    # Top bar
    cv2.rectangle(overlay, (0, 0), (w_f, 52), BLACK, -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    bcolor = bumper_bgr(bumper_color)
    cv2.putText(frame, f"TEAM  {session.team_number}", (12, 32),
                cv2.FONT_HERSHEY_DUPLEX, 0.85, ORANGE, 2)
    cv2.circle(frame, (210, 20), 7, bcolor, -1)
    cv2.circle(frame, (210, 20), 7, WHITE, 1)

    # NEW: show PAUSED status if applicable
    if paused:
        pill_col = CYAN
        status_text = "PAUSED"
    else:
        pill_col = (GREEN if status == "TRACKING" else
                    ORANGE if status == "FALLBACK" else
                    YELLOW if status == "SEARCHING" else
                    RED if status == "LOST" else GRAY)
        status_text = status

    sx = w_f // 2 - 65
    cv2.rectangle(frame, (sx - 6, 8), (sx + 140, 44), pill_col, -1)
    cv2.putText(frame, status_text, (sx, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, BLACK, 2)

    info = f"t={t_sec:.1f}s  f={frame_idx}  shots={session.shot_count}"
    cv2.putText(frame, info, (w_f - 350, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)

    # NEW: "Click robot to track" hint when no robot is locked
    if robot_tracker and robot_tracker.locked_id is None and not paused:
        hint = "CLICK ROBOT TO TRACK"
        (hw, hh), _ = cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        hx = (w_f - hw) // 2
        hy = h_f - 20
        cv2.putText(frame, hint, (hx, hy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, CYAN, 2)

def frame_to_base64(frame):
    """Convert frame to base64 JPEG."""
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
    return base64.b64encode(buffer).decode('utf-8')

# ===== VIDEO PROCESSING =====
def process_video(video_path: str, team_number: str, bumper_color: str):
    global active_session, robot_tracker, ball_tracker, stop_processing, is_paused, last_frame_dims, last_raw_frame

    print(f"[*] Loading YOLO model ({YOLO_MODEL})…")
    try:
        model = YOLO(YOLO_MODEL)
        model.fuse()
    except Exception as e:
        print(f"[!] YOLO model error: {e}. Using mock mode.")
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

    active_session = ScoutingSession(team_number, bumper_color, video_path)
    active_session.fps = fps
    active_session.total_frames = total_frames

    # Initialize RobotTracker
    if model:
        robot_tracker = RobotTracker(model, tracker, fps=fps)
    else:
        robot_tracker = None

    # Initialize BallTracker
    ball_tracker = BallTracker(fps, robot_radius=SHOT_ORIGIN_RADIUS, height_threshold=0.3)

    bcolor = bumper_bgr(bumper_color)
    frame_idx = 0

    print(f"[*] Processing: {fps:.1f} fps, {total_frames} frames ({total_frames/fps:.1f}s)")

    # Keep the last rendered frame so we can re-serve it while paused
    last_annotated_frame = None

    while not stop_processing:
        # ── PAUSE HANDLING ──────────────────────────────────────────────────
        with pause_condition:
            while is_paused and not stop_processing:
                # Re-queue the last frame so the UI stays live while paused
                if last_annotated_frame is not None:
                    frame_b64 = frame_to_base64(last_annotated_frame)
                    try:
                        if video_queue.full():
                            video_queue.get_nowait()
                        video_queue.put_nowait({
                            "frame": frame_b64,
                            "frame_idx": frame_idx,
                            "total_frames": total_frames,
                            "time": frame_idx / fps if fps > 0 else 0,
                            "status": "PAUSED",
                            "shot_count": active_session.shot_count,
                            "lost_count": robot_tracker.lost_count if robot_tracker else 0,
                            "paused": True,
                            "detections": _get_detection_list(),
                        })
                    except Exception:
                        pass
                pause_condition.wait(timeout=0.1)
        # ────────────────────────────────────────────────────────────────────

        ret, frame = cap.read()
        if not ret:
            print("[*] End of video.")
            break

        frame_idx += 1

        # Store raw frame dimensions for click-to-select coordinate mapping
        fh, fw = frame.shape[:2]
        last_frame_dims["w"] = fw
        last_frame_dims["h"] = fh

        # Share the raw frame with Flask routes (for CSRT init on click/ROI)
        with last_raw_frame_lock:
            globals()['last_raw_frame'] = frame.copy()

        # Tell the tracker about this frame (needed for CSRT init inside _handle_yolo_lock)
        if robot_tracker:
            robot_tracker._store_frame(frame)

        # Run robot tracking
        if robot_tracker:
            status, robot_box, trail = robot_tracker.update(
                frame, frame_idx, CONF_THRESH, IOU_THRESH
            )

            # Log position if tracking
            if robot_box is not None and robot_tracker.locked_id is not None:
                x1, y1, x2, y2 = robot_box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w_b, h_b = x2 - x1, y2 - y1
                active_session.log_position(frame_idx, cx, cy, w_b, h_b)
        else:
            status = "UNTRACKED"
            robot_box = None
            trail = []

        # Detect and track balls
        raw_balls = detect_yellow_balls(frame)
        live_balls = []
        shot_now = False
        
        if ball_tracker:
            live_balls, shot_now = ball_tracker.update(raw_balls, robot_box, frame_idx)
            
            if shot_now:
                active_session.shot_count += 1
                active_session.log_event(frame_idx, f"AUTO shot (total: {active_session.shot_count})")
                print(f"  [FUEL AUTO] Shot detected — total: {active_session.shot_count}")

        # Draw frame
        if robot_tracker:
            draw_all_detections(frame, robot_tracker.get_detections(),
                              robot_tracker.locked_id, bcolor)
            if robot_tracker.using_csrt and robot_tracker.csrt_box is not None:
                cx1, cy1, cx2, cy2 = robot_tracker.csrt_box
                draw_box_tactical(frame, cx1, cy1, cx2, cy2, ORANGE,
                                track_id=robot_tracker.locked_id, locked=True)
            draw_trail(frame, robot_tracker.trail, bcolor)

        draw_balls(frame, live_balls, ball_tracker._owned if ball_tracker else set(), robot_box)
        draw_hud(frame, active_session, status, frame_idx, fps, bumper_color, paused=is_paused)

        if robot_box is not None and (robot_tracker and (robot_tracker.locked_id or robot_tracker.using_csrt)):
            draw_fuel_counter(frame, robot_box, active_session.shot_count, bumper_color)

        last_annotated_frame = frame.copy()

        # Queue frame for web UI
        frame_b64 = frame_to_base64(frame)
        try:
            if video_queue.full():
                video_queue.get_nowait()
            video_queue.put_nowait({
                "frame": frame_b64,
                "frame_idx": frame_idx,
                "total_frames": total_frames,
                "time": frame_idx / fps if fps > 0 else 0,
                "status": status,
                "shot_count": active_session.shot_count,
                "lost_count": robot_tracker.lost_count if robot_tracker else 0,
                "paused": False,
                "detections": _get_detection_list(),
            })
        except Exception as e:
            print(f"[QUEUE ERROR] {e}")

        time.sleep(0.001)

    cap.release()
    print("[*] Video processing complete.")


def _get_detection_list():
    """Return current detection boxes as a list for the frontend robot-picker."""
    if robot_tracker is None:
        return []
    dets = robot_tracker.get_detections()
    if dets is None or len(dets) == 0:
        return []
    result = []
    for bbox, tid in zip(dets.xyxy, dets.tracker_id):
        x1, y1, x2, y2 = map(int, bbox)
        result.append({
            "track_id": int(tid),
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "locked": (tid == robot_tracker.locked_id),
        })
    return result


# ===== FLASK ROUTES =====

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/start-session', methods=['POST'])
def start_session():
    global processing_thread, stop_processing, is_paused
    
    data = request.json
    team_number = data.get('team_number', 'Unknown')
    bumper_color = data.get('bumper_color', 'red')
    video_path = data.get('video_path', '')

    if not video_path or not os.path.exists(video_path):
        return jsonify({"error": "Video file not found"}), 400

    stop_processing = False
    is_paused = False
    processing_thread = threading.Thread(
        target=process_video,
        args=(video_path, team_number, bumper_color),
        daemon=True
    )
    processing_thread.start()

    return jsonify({"status": "started", "team": team_number})


# ── NEW: Pause / Resume ───────────────────────────────────────────────────────
@app.route('/api/pause', methods=['POST'])
def pause_video():
    global is_paused
    with pause_condition:
        is_paused = True
        pause_condition.notify_all()
    return jsonify({"paused": True})

@app.route('/api/resume', methods=['POST'])
def resume_video():
    global is_paused
    with pause_condition:
        is_paused = False
        pause_condition.notify_all()
    return jsonify({"paused": False})

@app.route('/api/pause-state', methods=['GET'])
def pause_state():
    return jsonify({"paused": is_paused})
# ─────────────────────────────────────────────────────────────────────────────


# ── Select robot to track ─────────────────────────────────────────────────────
@app.route('/api/select-robot', methods=['POST'])
def select_robot():
    """
    Accepts either:
      { "track_id": 3 }                        — select by track ID
      { "norm_x": 0.42, "norm_y": 0.61 }       — click coords (normalised 0-1)
    Uses lock_to_detection() so CSRT is initialised immediately.
    """
    global robot_tracker, last_raw_frame
    if robot_tracker is None:
        return jsonify({"error": "No active tracker"}), 400

    data = request.json or {}

    def _lock_id(tid, bbox):
        with last_raw_frame_lock:
            frame = last_raw_frame
        if frame is None:
            # No frame yet — just set the ID; CSRT will init next cycle
            robot_tracker.locked_id = tid
            robot_tracker.using_csrt = False
        else:
            robot_tracker.lock_to_detection(tid, frame, bbox)

    # ── Direct ID selection ───────────────────────────────────────────────
    if "track_id" in data:
        tid = int(data["track_id"])
        # Find bbox for this ID from current detections
        dets = robot_tracker.get_detections()
        bbox = None
        if dets is not None and len(dets) > 0:
            for b, t in zip(dets.xyxy, dets.tracker_id):
                if int(t) == tid:
                    bbox = tuple(map(int, b))
                    break
        if bbox is None:
            return jsonify({"error": f"Track #{tid} not in current detections"}), 404
        _lock_id(tid, bbox)
        return jsonify({"locked_id": tid})

    # ── Click-coordinate selection ────────────────────────────────────────
    if "norm_x" in data and "norm_y" in data:
        nx, ny = float(data["norm_x"]), float(data["norm_y"])
        fw, fh = last_frame_dims["w"], last_frame_dims["h"]
        px, py = nx * fw, ny * fh

        dets = robot_tracker.get_detections()
        if dets is None or len(dets) == 0:
            return jsonify({"error": "No detections in current frame"}), 404

        # Pick the smallest box whose area contains the click point
        best_id, best_bbox, best_area = None, None, float("inf")
        for bbox, tid in zip(dets.xyxy, dets.tracker_id):
            x1, y1, x2, y2 = bbox
            if x1 <= px <= x2 and y1 <= py <= y2:
                area = (x2 - x1) * (y2 - y1)
                if area < best_area:
                    best_area = area
                    best_id   = int(tid)
                    best_bbox = tuple(map(int, bbox))

        if best_id is None:
            return jsonify({"error": "No robot at that location"}), 404

        _lock_id(best_id, best_bbox)
        return jsonify({"locked_id": best_id})

    return jsonify({"error": "Provide track_id or norm_x/norm_y"}), 400


@app.route('/api/clear-lock', methods=['POST'])
def clear_lock():
    """Release all tracking locks — YOLO auto-tracking will resume."""
    global robot_tracker
    if robot_tracker:
        robot_tracker.reset()
    return jsonify({"locked_id": None})


# ── Manual ROI ────────────────────────────────────────────────────────────────
@app.route('/api/set-roi', methods=['POST'])
def set_roi():
    """
    Accept a normalised bounding box drawn by the user on the video frame.
    { "norm_x1": 0.1, "norm_y1": 0.2, "norm_x2": 0.4, "norm_y2": 0.6 }
    Converts to pixel coords then calls lock_to_manual_roi() on the tracker,
    which initialises CSRT immediately and lets it auto-promote to YOLO lock
    once there's a high-IoU match.
    """
    global robot_tracker, last_raw_frame
    if robot_tracker is None:
        return jsonify({"error": "No active tracker"}), 400

    data = request.json or {}
    required = ("norm_x1", "norm_y1", "norm_x2", "norm_y2")
    if not all(k in data for k in required):
        return jsonify({"error": "Provide norm_x1/y1/x2/y2"}), 400

    fw, fh = last_frame_dims["w"], last_frame_dims["h"]
    x1 = int(data["norm_x1"] * fw)
    y1 = int(data["norm_y1"] * fh)
    x2 = int(data["norm_x2"] * fw)
    y2 = int(data["norm_y2"] * fh)

    # Clamp to frame and normalise direction
    x1, x2 = sorted([max(0, min(x1, fw)), max(0, min(x2, fw))])
    y1, y2 = sorted([max(0, min(y1, fh)), max(0, min(y2, fh))])

    if (x2 - x1) < 5 or (y2 - y1) < 5:
        return jsonify({"error": "ROI too small (min 5px each side)"}), 400

    with last_raw_frame_lock:
        frame = last_raw_frame

    if frame is None:
        return jsonify({"error": "No frame available yet — start video first"}), 400

    robot_tracker.lock_to_manual_roi(frame, x1, y1, x2, y2)
    return jsonify({"roi": [x1, y1, x2, y2]})
# ─────────────────────────────────────────────────────────────────────────────


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
        active_session.log_event(active_session.frame_idx, f"Shot logged (total: {active_session.shot_count})")
        return jsonify({"shot_count": active_session.shot_count})
    return jsonify({"error": "No active session"}), 400

@app.route('/api/log-event', methods=['POST'])
def log_event():
    global active_session
    data = request.json
    label = data.get('label', 'event')
    
    if active_session:
        active_session.log_event(active_session.frame_idx, label)
        return jsonify({"events": len(active_session.events)})
    return jsonify({"error": "No active session"}), 400

@app.route('/api/save-session', methods=['POST'])
def save_session():
    global active_session, stop_processing
    
    stop_processing = True
    
    if active_session and robot_tracker:
        stats = robot_tracker.get_stats()
        path = active_session.save(stats)
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
        stats = {
            "frame_idx": active_session.frame_idx,
            "fps": active_session.fps,
            "total_frames": active_session.total_frames,
            "robot_stats": robot_tracker.get_stats() if robot_tracker else {},
        }
        return jsonify({**active_session.to_dict(), **stats})
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

    UPLOADS_DIR.mkdir(exist_ok=True)
    filepath = UPLOADS_DIR / file.filename
    file.save(str(filepath))

    return jsonify({"path": str(filepath), "filename": file.filename})

if __name__ == '__main__':
    REPORTS_DIR.mkdir(exist_ok=True)
    UPLOADS_DIR.mkdir(exist_ok=True)
    app.run(debug=True, host='0.0.0.0', port=1768)