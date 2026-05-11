#!/usr/bin/env python3
"""
Track a robot in FRC (or any) competition video using YOLOv8 + ByteTrack.
Supports local MP4 files and YouTube links.
Tracks bumper color (red or blue) for team alliance identification.

Dependencies:
    pip install ultralytics supervision yt-dlp opencv-python

Usage:
    python robot_scout.py

Controls (during tracking):
    SPACE        - Pause / Resume
    R            - Re-select robot (click a detected box to lock onto it)
    E            - Log a timestamped event (prompted in terminal)
    S            - Save current scouting report
    Q / ESC      - Quit
    +/-          - Speed up / slow down playback
    Click on box - Lock onto that detected robot
"""

import cv2
import json
import time
import os
import sys
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path

def _ensure_deps():
    pkgs = {"ultralytics": "ultralytics", "supervision": "supervision"}
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

from ultralytics import YOLO         
import supervision as sv            
import numpy as np                    

YOLO_MODEL   = "yolov8n.pt"   
CONF_THRESH  = 0.30
IOU_THRESH   = 0.45
TRACK_THRESH      = 0.35
TRACK_BUFFER      = 30   
MATCH_THRESH      = 0.80
FRAME_RATE        = 30   
REPORTS_DIR  = Path("scouting_reports")
TRAIL_LEN    = 60  
ORANGE = (0, 165, 255)
GREEN  = (0, 220, 80)
RED    = (0, 60, 220)
CYAN   = (255, 210, 0)
WHITE  = (255, 255, 255)
BLACK  = (0, 0, 0)
GRAY   = (120, 120, 120)
BLUE   = (220, 100, 0)
YELLOW = (0, 220, 220)

def download_youtube(url: str) -> str:
    try:
        import yt_dlp
    except ImportError:
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "yt-dlp",
             "--break-system-packages", "-q"]
        )
        import yt_dlp

    tmpdir   = tempfile.mkdtemp(prefix="scout_")
    out_tmpl = os.path.join(tmpdir, "%(title)s.%(ext)s")
    ydl_opts = {
        "format": "bestvideo[ext=mp4][height<=720]+bestaudio[ext=m4a]"
                  "/best[ext=mp4][height<=720]/best",
        "outtmpl": out_tmpl,
        "quiet": True,
        "no_warnings": True,
        "merge_output_format": "mp4",
    }
    print(f"[*] Downloading: {url}")
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        info     = ydl.extract_info(url, download=True)
        filename = ydl.prepare_filename(info)
        for ext in [".mp4", ".mkv", ".webm"]:
            candidate = Path(filename).with_suffix(ext)
            if candidate.exists():
                return str(candidate)
    return filename

class ScoutingSession:
    def __init__(self, video_source: str, team_number: str, bumper_color: str = "red"):
        self.video_source   = video_source
        self.team_number    = team_number
        self.bumper_color   = bumper_color.lower()
        self.start_time     = datetime.now().isoformat()
        self.events         = []
        self.tracking_data  = []
        self.lost_count     = 0
        self.frames_tracked = 0

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
        print(f"  [EVENT @ {t_sec:.1f}s] {label}")

    def save(self) -> str:
        REPORTS_DIR.mkdir(exist_ok=True)
        safe  = "".join(c for c in self.team_number if c.isalnum() or c in "_-")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path  = REPORTS_DIR / f"team{safe}_{stamp}.json"

        total_dist = 0.0
        prev = None
        for pt in self.tracking_data:
            if prev:
                dx = pt["cx"] - prev["cx"]
                dy = pt["cy"] - prev["cy"]
                total_dist += (dx**2 + dy**2) ** 0.5
            prev = pt

        report = {
            "team_number":    self.team_number,
            "bumper_color":   self.bumper_color.upper(),
            "video_source":   self.video_source,
            "scouted_at":     self.start_time,
            "frames_tracked": self.frames_tracked,
            "tracking_lost":  self.lost_count,
            "pixel_distance": round(total_dist, 1),
            "events":         self.events,
            "tracking_data":  self.tracking_data,
        }
        path.write_text(json.dumps(report, indent=2))
        return str(path)

def bumper_bgr(bumper_color: str) -> tuple:
    return BLUE if bumper_color.lower() == "blue" else RED

def draw_box_tactical(frame, x1, y1, x2, y2, color, track_id=None, locked=False):
    """Corner-bracket bounding box; double-line if locked."""
    w, h = x2 - x1, y2 - y1
    L    = min(18, w // 3, h // 3)
    thickness = 3 if locked else 2
    corners   = [(x1, y1, 1, 1), (x2, y1, -1, 1),
                 (x1, y2, 1, -1), (x2, y2, -1, -1)]
    for (px, py, dx, dy) in corners:
        cv2.line(frame, (px, py), (px + dx*L, py), color, thickness)
        cv2.line(frame, (px, py), (px, py + dy*L), color, thickness)

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 14, 2)

    if track_id is not None:
        label  = f"#{track_id}"
        prefix = "★ " if locked else ""
        cv2.putText(frame, prefix + label,
                    (x1 + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

def draw_all_detections(frame, detections, locked_id, bumper_col):
    """Draw all ByteTrack detections; highlight the locked one."""
    if detections is None or len(detections) == 0:
        return
    for i, (bbox, tid) in enumerate(
            zip(detections.xyxy, detections.tracker_id)):
        x1, y1, x2, y2 = map(int, bbox)
        is_locked = (tid == locked_id)
        color     = bumper_col if is_locked else GRAY
        draw_box_tactical(frame, x1, y1, x2, y2, color,
                          track_id=tid, locked=is_locked)

def draw_trail(frame, trail, color):
    for i in range(1, len(trail)):
        alpha = i / len(trail)
        col   = (int(color[0]*alpha), int(color[1]*alpha), int(color[2]*alpha))
        cv2.line(frame, trail[i-1], trail[i], col, 2)

def draw_hud(frame, session, status, frame_idx, fps, paused, speed):
    h_f, w_f = frame.shape[:2]
    t_sec     = frame_idx / fps if fps > 0 else 0
    overlay   = frame.copy()

    # Top bar
    cv2.rectangle(overlay, (0, 0), (w_f, 52), BLACK, -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    bcolor = bumper_bgr(session.bumper_color)
    cv2.putText(frame, f"TEAM  {session.team_number}", (12, 32),
                cv2.FONT_HERSHEY_DUPLEX, 0.85, ORANGE, 2)
    cv2.circle(frame, (210, 20), 7, bcolor, -1)
    cv2.circle(frame, (210, 20), 7, WHITE, 1)

    pill_col = GREEN if status == "TRACKING" else (YELLOW if status == "SEARCHING" else RED)
    sx = w_f // 2 - 65
    cv2.rectangle(frame, (sx - 6, 8), (sx + 140, 44), pill_col, -1)
    cv2.putText(frame, status, (sx, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, BLACK, 2)

    info = f"t={t_sec:.1f}s  f={frame_idx}  x{speed:.1f}"
    if paused:
        info = "[PAUSED]  " + info
    cv2.putText(frame, info, (w_f - 285, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)

    # Bottom bar
    overlay2 = frame.copy()
    cv2.rectangle(overlay2, (0, h_f - 36), (w_f, h_f), BLACK, -1)
    cv2.addWeighted(overlay2, 0.55, frame, 0.45, 0, frame)
    hint = "SPACE=pause  R=reselect  E=event  S=save  Q=quit  +/-=speed  click=lock"
    cv2.putText(frame, hint, (10, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, GRAY, 1)
    ev_txt = f"Events: {len(session.events)}  Lost: {session.lost_count}"
    cv2.putText(frame, ev_txt, (12, h_f - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, CYAN, 1)

_click_pos = None

def _mouse_cb(event, x, y, flags, param):
    global _click_pos
    if event == cv2.EVENT_LBUTTONDOWN:
        _click_pos = (x, y)

def _pick_track_from_click(detections, click_xy):
    """Return the track_id of the box containing the click, or None."""
    if detections is None or click_xy is None:
        return None
    cx, cy = click_xy
    for bbox, tid in zip(detections.xyxy, detections.tracker_id):
        x1, y1, x2, y2 = map(int, bbox)
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            return int(tid)
    return None

def run_scouting(video_path: str, team_number: str, bumper_color: str):
    global _click_pos

    # Load YOLO model
    print(f"[*] Loading YOLO model ({YOLO_MODEL})…")
    model = YOLO(YOLO_MODEL)
    model.fuse()

    # ByteTracker via supervision
    tracker = sv.ByteTracker(
        track_activation_threshold=TRACK_THRESH,
        lost_track_buffer=TRACK_BUFFER,
        minimum_matching_threshold=MATCH_THRESH,
        frame_rate=FRAME_RATE,
    )

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[!] Cannot open video: {video_path}")
        return

    fps   = cap.get(cv2.CAP_PROP_FPS) or float(FRAME_RATE)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[*] Video: {fps:.1f} fps, {total} frames ({total/fps:.1f}s)")

    # ── Read first frame so user can click to lock a robot ──
    ret, frame = cap.read()
    if not ret:
        print("[!] Could not read first frame.")
        return

    cv2.namedWindow("Robot Scout", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Scout",
                     min(frame.shape[1], 1280), min(frame.shape[0], 720))
    cv2.setMouseCallback("Robot Scout", _mouse_cb)

    # Run detection on frame 0 to give user visible boxes to click
    results0   = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)[0]
    sv_dets0   = sv.Detections.from_ultralytics(results0)
    sv_dets0   = tracker.update_with_detections(sv_dets0)
    bcolor     = bumper_bgr(bumper_color)
    display0   = frame.copy()

    if sv_dets0 is not None and len(sv_dets0) > 0:
        for bbox, tid in zip(sv_dets0.xyxy, sv_dets0.tracker_id):
            x1, y1, x2, y2 = map(int, bbox)
            draw_box_tactical(display0, x1, y1, x2, y2, GRAY, track_id=tid)

    cv2.putText(display0,
                "Click on the robot you want to track, then press ENTER or SPACE",
                (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, ORANGE, 2)
    cv2.imshow("Robot Scout", display0)

    locked_id  = None
    while locked_id is None:
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            cap.release()
            cv2.destroyAllWindows()
            return

        if _click_pos is not None:
            locked_id = _pick_track_from_click(sv_dets0, _click_pos)
            _click_pos = None
            if locked_id is None:
                print("[!] No detected robot at that position – try again.")

        # Enter/Space without click: lock first detection as fallback
        if key in (13, ord(' ')) and locked_id is None and sv_dets0 is not None and len(sv_dets0):
            locked_id = int(sv_dets0.tracker_id[0])
            print(f"[*] Auto-locked track #{locked_id} (first detection)")

    print(f"[✓] Locked onto track #{locked_id}")
    cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")

    session   = ScoutingSession(video_path, team_number, bumper_color)
    trail     = []
    paused    = False
    speed     = 1.0
    frame_idx = 1
    status    = "TRACKING"

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("[*] End of video.")
                break
            frame_idx += 1

            # YOLO detection
            results  = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)[0]
            sv_dets  = sv.Detections.from_ultralytics(results)
            sv_dets  = tracker.update_with_detections(sv_dets)

            # Find locked track
            found = False
            if sv_dets is not None and len(sv_dets) > 0:
                for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                    if int(tid) == locked_id:
                        x1, y1, x2, y2 = map(int, bbox)
                        cx  = (x1 + x2) // 2
                        cy  = (y1 + y2) // 2
                        w_b = x2 - x1
                        h_b = y2 - y1
                        trail.append((cx, cy))
                        if len(trail) > TRAIL_LEN:
                            trail.pop(0)
                        session.log_position(frame_idx, fps, cx, cy, w_b, h_b)
                        status = "TRACKING"
                        found  = True
                        break

            if not found:
                status = "SEARCHING"
                session.lost_count += 1

        draw_all_detections(frame, sv_dets if not paused else None,
                            locked_id, bcolor)
        draw_trail(frame, trail, bcolor)
        draw_hud(frame, session, status, frame_idx, fps, paused, speed)

        cv2.imshow("Robot Scout", frame)

        # Handle click-to-relock mid-session
        if _click_pos is not None and not paused:
            new_id = _pick_track_from_click(
                sv_dets if not paused else None, _click_pos)
            if new_id is not None and new_id != locked_id:
                locked_id = new_id
                trail.clear()
                session.log_event(frame_idx, fps, f"Switched lock → track #{locked_id}")
                print(f"[*] Switched lock to track #{locked_id}")
            _click_pos = None

        delay = max(1, int((1000 / fps) / speed))
        key   = cv2.waitKey(delay if not paused else 30) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key == ord(' '):
            paused = not paused
        elif key == ord('r'):
            # Re-enter select mode: pause, show all boxes, let user click
            paused = True
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  click to re-lock…")
            print("\n[*] Click on a detected robot to re-lock, then press SPACE.")
            relock_done = False
            while not relock_done:
                disp = frame.copy()
                if sv_dets is not None and len(sv_dets) > 0:
                    for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                        x1, y1, x2, y2 = map(int, bbox)
                        draw_box_tactical(disp, x1, y1, x2, y2, GRAY, track_id=tid)
                cv2.putText(disp, "Click robot to re-lock, SPACE=confirm",
                            (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, ORANGE, 2)
                cv2.imshow("Robot Scout", disp)
                k2 = cv2.waitKey(30) & 0xFF
                if _click_pos is not None:
                    new_id = _pick_track_from_click(sv_dets, _click_pos)
                    _click_pos = None
                    if new_id is not None:
                        locked_id = new_id
                        trail.clear()
                        session.log_event(frame_idx, fps,
                                          f"Manual re-lock → track #{locked_id}")
                        print(f"[✓] Re-locked to track #{locked_id}")
                        relock_done = True
                if k2 == ord(' ') or k2 in (ord('q'), 27):
                    relock_done = True
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")
            paused = False

        elif key == ord('e'):
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  type event in terminal…")
            paused = True
            print("\n  Enter event label (e.g. 'scored', 'defended', 'tipped'): ",
                  end="", flush=True)
            label = input().strip()
            if label:
                session.log_event(frame_idx, fps, label)
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")
            paused = False

        elif key == ord('s'):
            path = session.save()
            print(f"\n[✓] Report saved → {path}")
            flash = frame.copy()
            cv2.rectangle(flash, (0, 0), (frame.shape[1], frame.shape[0]), GREEN, 20)
            cv2.putText(flash, f"Saved: {Path(path).name}",
                        (20, frame.shape[0] // 2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, GREEN, 3)
            cv2.imshow("Robot Scout", flash)
            cv2.waitKey(800)

        elif key in (ord('+'), ord('=')):
            speed = min(speed + 0.25, 4.0)
            print(f"  Speed: {speed:.2f}x")
        elif key == ord('-'):
            speed = max(speed - 0.25, 0.25)
            print(f"  Speed: {speed:.2f}x")

    path = session.save()
    print(f"\n[✓] Session ended. Report saved → {path}")
    cap.release()
    cv2.destroyAllWindows()

def prompt_input(label: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    val = input(f"{label}{suffix}: ").strip()
    return val if val else default

def main():
    print("=" * 56)
    print("  🤖  ROBOT SCOUTING APP  –  YOLOv8 + ByteTrack")
    print("=" * 56)
    print()

    team = prompt_input("Team number to scout (e.g. 254)")
    if not team:
        print("[!] Team number is required.")
        sys.exit(1)

    print()
    print("Bumper color:")
    print("  1) Red alliance")
    print("  2) Blue alliance")
    color_choice = prompt_input("Choice", "1")
    bumper_color = "blue" if color_choice == "2" else "red"
    print(f"[✓] Bumper color: {bumper_color.upper()}")

    print()
    print("Video source:")
    print("  1) Local MP4 file path")
    print("  2) YouTube URL")
    choice = prompt_input("Choice", "1")

    video_path = None
    if choice == "2":
        url = prompt_input("YouTube URL")
        if not url:
            print("[!] No URL provided.")
            sys.exit(1)
        try:
            video_path = download_youtube(url)
            print(f"[✓] Downloaded to: {video_path}")
        except Exception as e:
            print(f"[!] Download failed: {e}")
            sys.exit(1)
    else:
        path = prompt_input("MP4 file path")
        if not path or not os.path.exists(path):
            print(f"[!] File not found: {path}")
            sys.exit(1)
        video_path = path

    print()
    print(f"[*] Starting scouting session for Team {team}…")
    print(f"[*] Alliance: {bumper_color.upper()}  |  Model: {YOLO_MODEL}  |  Reports → {REPORTS_DIR}/")
    print()
    run_scouting(video_path, team, bumper_color)

if __name__ == "__main__":
    main()