#!/usr/bin/env python3
"""
Track a robot in FRC (or any) competition video.
Supports local MP4 files and YouTube links.
Tracks bumper color (red or blue) for team alliance identification.

Usage:
    python robot_scout.py

Controls (during tracking):
    SPACE   - Pause / Resume
    R       - Re-select robot (draw new bounding box)
    S       - Save current scouting report
    Q / ESC - Quit
    +/-     - Speed up / slow down playback
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


TRACKER_ALGO = "CSRT"   # Best accuracy; swap to "KCF" for speed
REPORTS_DIR  = Path("scouting_reports")
TRAIL_LEN    = 60        # frames of motion trail to draw

def make_tracker(algo: str = TRACKER_ALGO):
    algo = algo.upper()
    if algo == "CSRT":
        return cv2.TrackerCSRT_create()
    if algo == "KCF":
        return cv2.TrackerKCF_create()
    if algo == "MIL":
        return cv2.TrackerMIL_create()
    raise ValueError(f"Unknown tracker: {algo}")

def download_youtube(url: str) -> str:
    """Download a YouTube video via yt-dlp and return the local path."""
    try:
        import yt_dlp
    except ImportError:
        print("[!] yt-dlp not found. Installing…")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "yt-dlp", "--break-system-packages", "-q"]
        )
        import yt_dlp

    tmpdir = tempfile.mkdtemp(prefix="scout_")
    out_tmpl = os.path.join(tmpdir, "%(title)s.%(ext)s")

    ydl_opts = {
        "format": "bestvideo[ext=mp4][height<=720]+bestaudio[ext=m4a]/best[ext=mp4][height<=720]/best",
        "outtmpl": out_tmpl,
        "quiet": True,
        "no_warnings": True,
        "merge_output_format": "mp4",
    }

    print(f"[*] Downloading: {url}")
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(url, download=True)
        filename = ydl.prepare_filename(info)
        # yt-dlp may change the extension after merging
        for ext in [".mp4", ".mkv", ".webm"]:
            candidate = Path(filename).with_suffix(ext)
            if candidate.exists():
                return str(candidate)
    return filename

class ScoutingSession:
    def __init__(self, video_source: str, team_number: str, bumper_color: str = "red"):
        self.video_source  = video_source
        self.team_number   = team_number
        self.bumper_color  = bumper_color.lower()  # "red" or "blue"
        self.start_time    = datetime.now().isoformat()
        self.events        = []          # list of timestamped events
        self.tracking_data = []          # (frame, x, y, w, h) centroid log
        self.lost_count    = 0
        self.frames_tracked = 0

    def log_position(self, frame_idx: int, fps: float, bbox):
        x, y, w, h = [int(v) for v in bbox]
        cx, cy = x + w // 2, y + h // 2
        t_sec = frame_idx / fps if fps > 0 else 0
        self.tracking_data.append({"frame": frame_idx, "t": round(t_sec, 2),
                                   "cx": cx, "cy": cy, "w": w, "h": h})
        self.frames_tracked += 1

    def log_event(self, frame_idx: int, fps: float, label: str):
        t_sec = frame_idx / fps if fps > 0 else 0
        self.events.append({"frame": frame_idx, "t": round(t_sec, 2), "event": label})
        print(f"  [EVENT @ {t_sec:.1f}s] {label}")

    def save(self) -> str:
        REPORTS_DIR.mkdir(exist_ok=True)
        safe_team = "".join(c for c in self.team_number if c.isalnum() or c in "_-")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path  = REPORTS_DIR / f"team{safe_team}_{stamp}.json"

        # Summary stats
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
            "pixel_distance":  round(total_dist, 1),
            "events":          self.events,
            "tracking_data":   self.tracking_data,
        }

        path.write_text(json.dumps(report, indent=2))
        return str(path)

ORANGE  = (0, 165, 255)
GREEN   = (0, 220, 80)
RED     = (0, 60, 220)
CYAN    = (255, 210, 0)
WHITE   = (255, 255, 255)
BLACK   = (0, 0, 0)
GRAY    = (120, 120, 120)
BLUE    = (220, 100, 0)

def get_bumper_color_display(bumper_color: str) -> tuple:
    """Return BGR color tuple for the bumper color."""
    if bumper_color.lower() == "blue":
        return BLUE
    elif bumper_color.lower() == "red":
        return RED
    return WHITE

def draw_hud(frame, session: ScoutingSession, status: str,
             frame_idx: int, fps: float, paused: bool, speed: float):
    h, w = frame.shape[:2]
    t_sec = frame_idx / fps if fps > 0 else 0

    # Semi-transparent top bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 52), BLACK, -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    # Team label with bumper color indicator
    bumper_col = get_bumper_color_display(session.bumper_color)
    cv2.putText(frame, f"TEAM  {session.team_number}", (12, 32),
                cv2.FONT_HERSHEY_DUPLEX, 0.85, ORANGE, 2)
    
    # Bumper color indicator dot
    cv2.circle(frame, (200, 20), 6, bumper_col, -1)
    cv2.circle(frame, (200, 20), 6, WHITE, 2)

    # Status pill
    pill_col = GREEN if status == "TRACKING" else RED
    status_x = w // 2 - 60
    cv2.rectangle(frame, (status_x - 6, 8), (status_x + 130, 44), pill_col, -1)
    cv2.putText(frame, status, (status_x, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, BLACK, 2)

    # Right side info
    info = f"t={t_sec:.1f}s  f={frame_idx}  x{speed:.1f}"
    if paused:
        info = "[PAUSED]  " + info
    cv2.putText(frame, info, (w - 280, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)

    # Bottom bar
    overlay2 = frame.copy()
    cv2.rectangle(overlay2, (0, h - 36), (w, h), BLACK, -1)
    cv2.addWeighted(overlay2, 0.55, frame, 0.45, 0, frame)
    hint = "SPACE=pause  R=reselect  E=log event  S=save  Q=quit  +/-=speed"
    cv2.putText(frame, hint, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, GRAY, 1)

    # Events counter
    ev_txt = f"Events: {len(session.events)}  Lost: {session.lost_count}"
    cv2.putText(frame, ev_txt, (12, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, CYAN, 1)


def draw_box(frame, bbox, bumper_color: str = "red"):
    """Draw bounding box with color based on bumper color."""
    color = get_bumper_color_display(bumper_color)
    x, y, w, h = [int(v) for v in bbox]
    # Corners only (tactical look)
    L = min(20, w // 3, h // 3)
    pts = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
    dirs = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    for (px, py), (dx, dy) in zip(pts, dirs):
        cv2.line(frame, (px, py), (px + dx*L, py), color, 3)
        cv2.line(frame, (px, py), (px, py + dy*L), color, 3)
    # Crosshair on centroid
    cx, cy = x + w // 2, y + h // 2
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 14, 2)


def draw_trail(frame, trail: list, bumper_color: str = "red"):
    """Draw motion trail with color based on bumper color."""
    base_color = get_bumper_color_display(bumper_color)
    for i in range(1, len(trail)):
        alpha = i / len(trail)
        col = (int(base_color[0] * alpha), int(base_color[1] * alpha), int(base_color[2] * alpha))
        cv2.line(frame, trail[i-1], trail[i], col, 2)

def run_scouting(video_path: str, team_number: str, bumper_color: str):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[!] Cannot open video: {video_path}")
        return

    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[*] Video: {fps:.1f} fps, {total} frames ({total/fps:.1f}s)")

    # ── Read first frame and let user select robot ──
    ret, frame = cap.read()
    if not ret:
        print("[!] Could not read first frame.")
        return

    print("\n[*] Draw a box around the robot you want to track.")
    print("    Drag to select, then press ENTER or SPACE to confirm.")
    cv2.namedWindow("Robot Scout", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Scout", min(frame.shape[1], 1280), min(frame.shape[0], 720))
    bbox = cv2.selectROI("Robot Scout", frame, fromCenter=False, showCrosshair=True)
    cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")

    if bbox == (0, 0, 0, 0):
        print("[!] No selection made. Exiting.")
        cap.release()
        cv2.destroyAllWindows()
        return

    # Initialise tracker
    tracker = make_tracker()
    tracker.init(frame, bbox)

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

            ok, bbox = tracker.update(frame)

            if ok:
                status = "TRACKING"
                draw_box(frame, bbox, session.bumper_color)
                cx, cy = int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]//2)
                trail.append((cx, cy))
                if len(trail) > TRAIL_LEN:
                    trail.pop(0)
                session.log_position(frame_idx, fps, bbox)
            else:
                status = "LOST"
                session.lost_count += 1
                trail.clear()

        draw_trail(frame, trail, session.bumper_color)
        draw_hud(frame, session, status, frame_idx, fps, paused, speed)

        cv2.imshow("Robot Scout", frame)

        # Delay adjusted for speed
        delay = max(1, int((1000 / fps) / speed))
        key   = cv2.waitKey(delay if not paused else 30) & 0xFF

        if key in (ord('q'), 27):   # Q or ESC
            break
        elif key == ord(' '):       # pause
            paused = not paused
        elif key == ord('r'):       # re-select
            paused = True
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  RE-SELECT robot…")
            bbox = cv2.selectROI("Robot Scout", frame, fromCenter=False, showCrosshair=True)
            if bbox != (0, 0, 0, 0):
                tracker = make_tracker()
                tracker.init(frame, bbox)
                trail.clear()
                status = "TRACKING"
                session.log_event(frame_idx, fps, "Manual re-selection")
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")
            paused = False
        elif key == ord('e'):       # log custom event
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  type event in terminal…")
            paused = True
            print("\n  Enter event label (e.g. 'scored', 'defended', 'tipped'): ", end="", flush=True)
            label = input().strip()
            if label:
                session.log_event(frame_idx, fps, label)
            cv2.setWindowTitle("Robot Scout", "Robot Scout  –  tracking…")
            paused = False
        elif key == ord('s'):       # save
            path = session.save()
            print(f"\n[✓] Report saved → {path}")
            # Flash green overlay briefly
            flash = frame.copy()
            cv2.rectangle(flash, (0, 0), (frame.shape[1], frame.shape[0]), GREEN, 20)
            cv2.putText(flash, f"Saved: {Path(path).name}", (20, frame.shape[0]//2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0, GREEN, 3)
            cv2.imshow("Robot Scout", flash)
            cv2.waitKey(800)
        elif key == ord('+') or key == ord('='):
            speed = min(speed + 0.25, 4.0)
            print(f"  Speed: {speed:.2f}x")
        elif key == ord('-'):
            speed = max(speed - 0.25, 0.25)
            print(f"  Speed: {speed:.2f}x")

    # ── Auto-save on quit ──
    path = session.save()
    print(f"\n[✓] Session ended. Report saved → {path}")
    cap.release()
    cv2.destroyAllWindows()

def prompt_input(label: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    val = input(f"{label}{suffix}: ").strip()
    return val if val else default


def main():
    print("=" * 54)
    print("  🤖  ROBOT SCOUTING APP  –  OpenCV Tracker")
    print("=" * 54)
    print()

    # ── Team number ──
    team = prompt_input("Team number to scout (e.g. 254)")
    if not team:
        print("[!] Team number is required.")
        sys.exit(1)

    # ── Bumper color ──
    print()
    print("Bumper color:")
    print("  1) Red alliance")
    print("  2) Blue alliance")
    color_choice = prompt_input("Choice", "1")
    bumper_color = "blue" if color_choice == "2" else "red"
    print(f"[✓] Bumper color set to: {bumper_color.upper()}")

    # ── Video source ──
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
    print(f"[*] Alliance: {bumper_color.upper()}  |  Tracker: {TRACKER_ALGO}  |  Reports → {REPORTS_DIR}/")
    print()
    run_scouting(video_path, team, bumper_color)


if __name__ == "__main__":
    main()