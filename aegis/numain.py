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
    R            - Select robot from YOLO detections (click a box)
    R R          - Draw a manual bounding box when YOLO can't find the robot
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

YOLO_MODEL   = "model.pt"   
CONF_THRESH  = 0.30
IOU_THRESH   = 0.45
TRACK_THRESH      = 0.35
TRACK_BUFFER      = 30   
MATCH_THRESH      = 0.80
FRAME_RATE        = 30   
REPORTS_DIR  = Path("scouting_reports")
TRAIL_LEN    = 60

# CSRT fallback: re-attempt YOLO hand-back after this many CSRT-only frames
CSRT_HANDBACK_INTERVAL = 10
# IoU overlap required between CSRT box and a YOLO detection to re-lock
CSRT_REJOIN_IOU        = 0.30
# Seconds a YOLO detection must consistently overlap CSRT before auto-promoting
YOLO_LOCK_CONFIRM_SECS = 1.0
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

    pill_col = (GREEN   if status == "TRACKING"  else
                ORANGE  if status == "FALLBACK"  else
                YELLOW  if status == "SEARCHING" else
                RED     if status == "LOST"      else GRAY)
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
    hint = "SPACE=pause  R=select  E=event  S=save  Q=quit  +/-=speed  click=lock"
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

def _iou(a, b):
    """IoU between two (x1,y1,x2,y2) boxes."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    area_a = (a[2]-a[0]) * (a[3]-a[1])
    area_b = (b[2]-b[0]) * (b[3]-b[1])
    return inter / (area_a + area_b - inter)

def _init_csrt(frame, x1, y1, x2, y2):
    """Create and initialise a CSRT tracker on the given bounding box."""
    tracker = cv2.TrackerCSRT_create()
    tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))
    return tracker

def run_scouting(video_path: str, team_number: str, bumper_color: str):
    global _click_pos

    # Load YOLO model
    print(f"[*] Loading YOLO model ({YOLO_MODEL})…")
    model = YOLO(YOLO_MODEL)
    model.fuse()

    # ByteTracker via supervision
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

    fps   = cap.get(cv2.CAP_PROP_FPS) or float(FRAME_RATE)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[*] Video: {fps:.1f} fps, {total} frames ({total/fps:.1f}s)")
    print(f"[*] Playing — press R at any time to select a robot to track.")

    cv2.namedWindow("Robot Scout", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Robot Scout", _mouse_cb)

    session   = ScoutingSession(video_path, team_number, bumper_color)
    bcolor    = bumper_bgr(bumper_color)
    trail     = []
    paused    = False
    speed     = 1.0
    frame_idx = 0
    status    = "UNTRACKED"
    locked_id = None
    sv_dets   = None
    frame     = None

    # CSRT fallback state
    csrt_tracker      = None   # active only when YOLO loses the robot
    csrt_box          = None   # last (x1,y1,x2,y2) from CSRT
    csrt_frames       = 0      # consecutive frames on CSRT-only mode
    using_csrt        = False  # True while YOLO can't find the locked robot

    # YOLO auto-promote state: track which YOLO id has been overlapping CSRT
    yolo_candidate_id     = None  # track id currently being evaluated
    yolo_candidate_frames = 0     # consecutive frames it has overlapped CSRT

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("[*] End of video.")
                break
            frame_idx += 1

            # Resize window to fit first frame
            if frame_idx == 1:
                cv2.resizeWindow("Robot Scout",
                                 min(frame.shape[1], 1280),
                                 min(frame.shape[0], 720))

            # ── YOLO + ByteTrack ──────────────────────────────────────
            results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)[0]
            sv_dets = sv.Detections.from_ultralytics(results)
            sv_dets = tracker.update_with_detections(sv_dets)

            # ── Try to find locked robot in YOLO detections ───────────
            found    = False
            yolo_box = None
            if locked_id is not None and sv_dets is not None and len(sv_dets) > 0:
                for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                    if int(tid) == locked_id:
                        yolo_box = tuple(map(int, bbox))
                        found = True
                        break

            if found:
                # ── YOLO primary tracking ─────────────────────────────
                x1, y1, x2, y2 = yolo_box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w_b, h_b = x2 - x1, y2 - y1
                trail.append((cx, cy))
                if len(trail) > TRAIL_LEN:
                    trail.pop(0)
                session.log_position(frame_idx, fps, cx, cy, w_b, h_b)
                status = "TRACKING"
                using_csrt = False
                csrt_frames = 0
                yolo_candidate_id = None
                yolo_candidate_frames = 0
                # Keep CSRT warm so it can take over instantly if needed
                csrt_tracker = _init_csrt(frame, x1, y1, x2, y2)
                csrt_box = yolo_box

            elif locked_id is not None or using_csrt:
                # ── CSRT fallback (covers both: lost YOLO lock AND
                #    manual ROI mode where locked_id is None) ──────────
                csrt_ok = False
                if csrt_tracker is not None:
                    ok, rect = csrt_tracker.update(frame)
                    if ok:
                        rx, ry, rw, rh = [int(v) for v in rect]
                        h_f, w_f = frame.shape[:2]
                        rx  = max(0, min(rx, w_f - 1))
                        ry  = max(0, min(ry, h_f - 1))
                        rw  = max(1, min(rw, w_f - rx))
                        rh  = max(1, min(rh, h_f - ry))
                        x1, y1, x2, y2 = rx, ry, rx + rw, ry + rh
                        csrt_box = (x1, y1, x2, y2)
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        w_b, h_b = rw, rh
                        trail.append((cx, cy))
                        if len(trail) > TRAIL_LEN:
                            trail.pop(0)
                        session.log_position(frame_idx, fps, cx, cy, w_b, h_b)
                        status = "FALLBACK"
                        using_csrt = True
                        csrt_frames += 1
                        csrt_ok = True

                        # ── YOLO auto-promote: find best overlapping detection ──
                        # Check every frame (not just on interval) so the 1-second
                        # consistency window is accurate.
                        if sv_dets is not None and len(sv_dets) > 0:
                            best_iou, best_tid = 0.0, None
                            for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                                iou_val = _iou(csrt_box, tuple(map(int, bbox)))
                                if iou_val > best_iou:
                                    best_iou, best_tid = iou_val, int(tid)

                            if best_iou >= CSRT_REJOIN_IOU and best_tid is not None:
                                # Same candidate as last frame → extend streak
                                if best_tid == yolo_candidate_id:
                                    yolo_candidate_frames += 1
                                else:
                                    # New candidate — start fresh streak
                                    yolo_candidate_id     = best_tid
                                    yolo_candidate_frames = 1

                                # Promote once streak exceeds 1 second
                                needed = max(1, int(fps * YOLO_LOCK_CONFIRM_SECS))
                                if yolo_candidate_frames >= needed:
                                    locked_id    = yolo_candidate_id
                                    using_csrt   = False
                                    csrt_frames  = 0
                                    yolo_candidate_id     = None
                                    yolo_candidate_frames = 0
                                    print(f"  [CSRT→YOLO] Auto-promoted to track "
                                          f"#{locked_id} after "
                                          f"{YOLO_LOCK_CONFIRM_SECS}s overlap "
                                          f"(IoU={best_iou:.2f})")
                            else:
                                # No good overlap — reset streak
                                yolo_candidate_id     = None
                                yolo_candidate_frames = 0

                if not csrt_ok:
                    status = "LOST"
                    if locked_id is not None:
                        session.lost_count += 1

            else:
                status = "UNTRACKED"

        # Draw
        draw_all_detections(frame, sv_dets if not paused else sv_dets,
                            locked_id, bcolor)
        # Draw CSRT fallback box in orange (dashed-style: just thicker corners)
        if using_csrt and csrt_box is not None:
            cx1, cy1, cx2, cy2 = csrt_box
            draw_box_tactical(frame, cx1, cy1, cx2, cy2, ORANGE,
                              track_id=locked_id, locked=True)
        draw_trail(frame, trail, bcolor)
        draw_hud(frame, session, status, frame_idx, fps, paused, speed)

        # Show YOLO auto-promote progress bar when a candidate is accumulating
        if using_csrt and yolo_candidate_id is not None and fps > 0:
            needed   = max(1, int(fps * YOLO_LOCK_CONFIRM_SECS))
            progress = min(yolo_candidate_frames / needed, 1.0)
            h_f, w_f = frame.shape[:2]
            bar_w    = 200
            bar_x    = w_f - bar_w - 12
            bar_y    = 60
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 16), GRAY, -1)
            cv2.rectangle(frame, (bar_x, bar_y),
                          (bar_x + int(bar_w * progress), bar_y + 16), CYAN, -1)
            cv2.putText(frame, f"YOLO lock #{yolo_candidate_id}…",
                        (bar_x, bar_y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, CYAN, 1)

        # Hint overlay when no robot is locked
        if locked_id is None:
            h_f, w_f = frame.shape[:2]
            cv2.putText(frame, "Press R to select a robot",
                        (w_f // 2 - 170, h_f // 2),
                        cv2.FONT_HERSHEY_DUPLEX, 0.9, ORANGE, 2)

        cv2.imshow("Robot Scout", frame)

        # Handle mid-playback click-to-lock
        if _click_pos is not None:
            new_id = _pick_track_from_click(sv_dets, _click_pos)
            if new_id is not None and new_id != locked_id:
                locked_id             = new_id
                trail.clear()
                using_csrt            = False
                csrt_frames           = 0
                csrt_box              = None
                csrt_tracker          = None
                yolo_candidate_id     = None
                yolo_candidate_frames = 0
                for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                    if int(tid) == new_id:
                        bx1, by1, bx2, by2 = map(int, bbox)
                        csrt_tracker = _init_csrt(frame, bx1, by1, bx2, by2)
                        break
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
            # ── Stage 1: click a YOLO-detected box ───────────────────
            paused = True
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  click to select robot…")
            print("\n[*] Click a detected robot to lock.  Press R again to draw a box manually.")
            relock_done = False
            while not relock_done:
                disp = frame.copy()
                if sv_dets is not None and len(sv_dets) > 0:
                    for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                        x1, y1, x2, y2 = map(int, bbox)
                        is_locked = (tid == locked_id)
                        color = bcolor if is_locked else GRAY
                        draw_box_tactical(disp, x1, y1, x2, y2, color, track_id=tid,
                                          locked=is_locked)
                cv2.putText(disp, "Click robot to lock  |  R=draw box  |  SPACE=cancel",
                            (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, ORANGE, 2)
                cv2.imshow("Robot Scout", disp)
                k2 = cv2.waitKey(30) & 0xFF

                # Click lands on a YOLO box → lock it
                if _click_pos is not None:
                    new_id = _pick_track_from_click(sv_dets, _click_pos)
                    _click_pos = None
                    if new_id is not None:
                        locked_id             = new_id
                        trail.clear()
                        using_csrt            = False
                        csrt_frames           = 0
                        csrt_box              = None
                        csrt_tracker          = None
                        yolo_candidate_id     = None
                        yolo_candidate_frames = 0
                        for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                            if int(tid) == new_id:
                                bx1, by1, bx2, by2 = map(int, bbox)
                                csrt_tracker = _init_csrt(frame, bx1, by1, bx2, by2)
                                break
                        session.log_event(frame_idx, fps,
                                          f"Locked → track #{locked_id}")
                        print(f"[✓] Locked to track #{locked_id}")
                        relock_done = True

                # R again → enter manual draw mode
                elif k2 == ord('r'):
                    relock_done = True   # exit stage-1 loop first
                    cv2.setWindowTitle("Robot Scout",
                                       "Robot Scout  –  draw box around robot…")
                    print("[*] Draw a box around the robot (click-drag), then SPACE to confirm.")

                    # ── Stage 2: freehand drag-to-draw ROI ───────────
                    draw_start = None
                    draw_end   = None
                    drawing    = False
                    draw_done  = False

                    def _draw_mouse_cb(event, x, y, flags, param):
                        nonlocal draw_start, draw_end, drawing
                        if event == cv2.EVENT_LBUTTONDOWN:
                            draw_start = (x, y)
                            draw_end   = (x, y)
                            drawing    = True
                        elif event == cv2.EVENT_MOUSEMOVE and drawing:
                            draw_end = (x, y)
                        elif event == cv2.EVENT_LBUTTONUP and drawing:
                            draw_end = (x, y)
                            drawing  = False

                    cv2.setMouseCallback("Robot Scout", _draw_mouse_cb)

                    while not draw_done:
                        disp2 = frame.copy()
                        # Draw all YOLO boxes faintly for context
                        if sv_dets is not None and len(sv_dets) > 0:
                            for bbox, tid in zip(sv_dets.xyxy, sv_dets.tracker_id):
                                bx1, by1, bx2, by2 = map(int, bbox)
                                draw_box_tactical(disp2, bx1, by1, bx2, by2, GRAY,
                                                  track_id=tid)
                        # Live rubber-band rect
                        if draw_start and draw_end:
                            rx1 = min(draw_start[0], draw_end[0])
                            ry1 = min(draw_start[1], draw_end[1])
                            rx2 = max(draw_start[0], draw_end[0])
                            ry2 = max(draw_start[1], draw_end[1])
                            cv2.rectangle(disp2, (rx1, ry1), (rx2, ry2), CYAN, 2)
                            cv2.putText(disp2, "MANUAL ROI",
                                        (rx1 + 4, ry1 - 6),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, CYAN, 2)
                        cv2.putText(disp2,
                                    "Drag to draw box  |  SPACE=confirm  |  ESC=cancel",
                                    (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, CYAN, 2)
                        cv2.imshow("Robot Scout", disp2)
                        k3 = cv2.waitKey(20) & 0xFF

                        if k3 == ord(' ') and draw_start and draw_end:
                            rx1 = min(draw_start[0], draw_end[0])
                            ry1 = min(draw_start[1], draw_end[1])
                            rx2 = max(draw_start[0], draw_end[0])
                            ry2 = max(draw_start[1], draw_end[1])
                            if rx2 - rx1 > 8 and ry2 - ry1 > 8:
                                # Initialise CSRT on the drawn region;
                                # clear YOLO lock so CSRT runs solo until
                                # it can hand back to ByteTrack
                                locked_id             = None
                                trail.clear()
                                csrt_tracker          = _init_csrt(frame, rx1, ry1, rx2, ry2)
                                csrt_box              = (rx1, ry1, rx2, ry2)
                                using_csrt            = True
                                csrt_frames           = 0
                                yolo_candidate_id     = None
                                yolo_candidate_frames = 0
                                session.log_event(frame_idx, fps,
                                                  "Manual ROI – CSRT tracking")
                                print(f"[✓] Manual ROI set – tracking via CSRT until "
                                      f"YOLO re-acquires.")
                            draw_done = True

                        elif k3 in (27, ord('q')):
                            draw_done = True   # cancel, keep previous state

                    # Restore normal mouse callback
                    cv2.setMouseCallback("Robot Scout", _mouse_cb)

                elif k2 == ord(' ') or k2 in (ord('q'), 27):
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
    print(f"[*] Press R to select a robot to track at any time.")
    print()
    run_scouting(video_path, team, bumper_color)

if __name__ == "__main__":
    main()