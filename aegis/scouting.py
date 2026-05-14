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