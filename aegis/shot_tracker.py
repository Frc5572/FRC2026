import cv2
import numpy as np
from collections import deque

class BallTracker:
    """
    Shot detector: counts yellow balls leaving the trapezoid shooting zone.
    Only triggers shots when the robot is stationary/slow AND the ball is
    moving faster than the robot (i.e. it was actually launched, not carried).

    Velocity thresholds (pixels/frame at native resolution):
        ROBOT_MAX_SPEED    robot centre must move <= this to be "still"
        BALL_MIN_REL_SPEED  ball must move >= this RELATIVE to the robot
    Both are tunable via constructor kwargs.
    """

   
    ROBOT_MAX_SPEED    = 2.0   
    BALL_MIN_REL_SPEED = 20.0  
    VELOCITY_WINDOW    = 5   

    def __init__(self,
                 fps: float,
                 robot_radius: int = 120,
                 height_threshold: float = 0.3,
                 robot_max_speed: float = ROBOT_MAX_SPEED,
                 ball_min_rel_speed: float = BALL_MIN_REL_SPEED,
                 velocity_window: int = VELOCITY_WINDOW):

        self.fps               = max(fps, 1.0)
        self.robot_radius      = robot_radius
        self.height_threshold  = height_threshold
        self.robot_max_speed   = robot_max_speed
        self.ball_min_rel_speed = ball_min_rel_speed
        self.vel_window        = velocity_window

        self._robot_center_hist: deque = deque(maxlen=velocity_window + 1)
        self._robot_speed = 0.0          

        self._ball_hist: dict = {}       
        self._ball_speeds: dict = {}     
        self._next_ball_id = 0

        self._last_near_count  = 0
        self._last_far_count   = 0
        self._owned            = set()  

    @staticmethod
    def _dist(a: tuple, b: tuple) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _update_robot_velocity(self, robot_box):
        if robot_box is None:
            self._robot_speed = 0.0
            return
        cx = (robot_box[0] + robot_box[2]) / 2.0
        cy = (robot_box[1] + robot_box[3]) / 2.0
        self._robot_center_hist.append((cx, cy))

        if len(self._robot_center_hist) >= 2:
            speeds = []
            hist = list(self._robot_center_hist)
            for i in range(1, len(hist)):
                speeds.append(self._dist(hist[i], hist[i - 1]))
            self._robot_speed = float(np.mean(speeds))
        else:
            self._robot_speed = 0.0

    def _match_balls(self, detections: list) -> dict:
        MAX_MATCH_DIST = 60   

        unmatched_dets = list(detections)   
        new_hist: dict = {}

        for bid, hist in self._ball_hist.items():
            if not hist or not unmatched_dets:
                break
            prev = hist[-1]
            dists = [self._dist(prev, (d[0], d[1])) for d in unmatched_dets]
            best_i = int(np.argmin(dists))
            if dists[best_i] <= MAX_MATCH_DIST:
                new_hist[bid] = hist        
                new_hist[bid].append((unmatched_dets[best_i][0],
                                      unmatched_dets[best_i][1]))
                unmatched_dets.pop(best_i)

        for d in unmatched_dets:
            bid = self._next_ball_id
            self._next_ball_id += 1
            q: deque = deque(maxlen=self.vel_window + 1)
            q.append((d[0], d[1]))
            new_hist[bid] = q

        self._ball_hist = new_hist
        
        result = {}
        for bid, hist in self._ball_hist.items():
            cx, cy = hist[-1]
            r = 8  
            for d in detections:
                if abs(d[0] - cx) < 2 and abs(d[1] - cy) < 2:
                    r = d[2]
                    break
            result[bid] = (cx, cy, r)
        return result

    def _ball_speed(self, bid: int) -> float:
        hist = self._ball_hist.get(bid)
        if hist is None or len(hist) < 2:
            return 0.0
        pts = list(hist)
        speeds = [self._dist(pts[i], pts[i - 1]) for i in range(1, len(pts))]
        return float(np.mean(speeds))

    def _robot_is_still(self) -> bool:
        return self._robot_speed <= self.robot_max_speed

    def _ball_is_fast_relative_to_robot(self, bid: int) -> bool:
        bs = self._ball_speed(bid)
        relative_speed = bs - self._robot_speed
        return relative_speed >= self.ball_min_rel_speed

    def _is_ball_above_robot(self, ball_y: float, robot_box: tuple) -> bool:
        if robot_box is None:
            return True
        rx1, ry1, rx2, ry2 = robot_box
        robot_height = ry2 - ry1
        above_threshold = ry1 - robot_height * self.height_threshold
        return ball_y < above_threshold

    def _point_in_trapezoid(self, px: float, py: float,
                             robot_box: tuple) -> bool:
        if robot_box is None:
            return False
        x1, y1, x2, y2 = robot_box
        robot_width = x2 - x1

        trap_bottom = y1
        trap_top    = y1 - self.robot_radius

        trap_bottom_width  = robot_width * 0.6
        trap_offset_bottom = (robot_width - trap_bottom_width) / 2

        trap_left_bottom  = x1 + trap_offset_bottom
        trap_right_bottom = x2 - trap_offset_bottom
        trap_left_top     = x1
        trap_right_top    = x2

        if py < trap_top or py > trap_bottom:
            return False

        t = (py - trap_top) / (trap_bottom - trap_top)
        left_x  = trap_left_top  + t * (trap_left_bottom  - trap_left_top)
        right_x = trap_right_top + t * (trap_right_bottom - trap_right_top)

        return left_x <= px <= right_x

    def update(self, detections: list, robot_box: tuple,
               frame_idx: int) -> tuple:
        shots_this_frame = 0

        self._update_robot_velocity(robot_box)

        if robot_box is None:
            self._last_near_count = 0
            self._last_far_count  = 0
            self._ball_hist.clear()
            return [], 0

        tracked = self._match_balls(detections)   

        balls_in_zone  = []   
        balls_out_zone = []

        for bid, (cx, cy, r) in tracked.items():
            if not self._is_ball_above_robot(cy, robot_box):
                continue  

            if self._point_in_trapezoid(cx, cy, robot_box):
                balls_in_zone.append((bid, cx, cy, r))
            else:
                balls_out_zone.append((bid, cx, cy, r))

        current_in_count  = len(balls_in_zone)
        current_out_count = len(balls_out_zone)

        robot_still = self._robot_is_still()

        if (self._last_near_count > 0
                and current_in_count == 0
                and robot_still):
            shots_this_frame = 1
            print(f"[SHOT] Balls left zone "
                  f"({self._last_near_count}→0) | "
                  f"robot_speed={self._robot_speed:.1f}px/f")

        if (shots_this_frame == 0
                and current_out_count > self._last_far_count
                and current_in_count <= self._last_near_count
                and robot_still):

            for bid, cx, cy, r in balls_out_zone:
                if self._ball_is_fast_relative_to_robot(bid):
                    shots_this_frame = 1
                    rel = self._ball_speed(bid) - self._robot_speed
                    print(f"[SHOT] Ball {bid} launched "
                          f"(rel_speed={rel:.1f}px/f, "
                          f"robot_speed={self._robot_speed:.1f}px/f)")
                    break

        if shots_this_frame and not robot_still:
            print(f"[SHOT SUPPRESSED] Robot moving "
                  f"({self._robot_speed:.1f}px/f > "
                  f"{self.robot_max_speed}px/f threshold)")
            shots_this_frame = 0

        self._last_near_count = current_in_count
        self._last_far_count  = current_out_count

        all_above = balls_in_zone + balls_out_zone
        display_balls = [(bid, cx, cy, r) for bid, cx, cy, r in all_above]

        # Update _owned for draw_balls highlight
        self._owned = {bid for bid, *_ in balls_in_zone}

        return display_balls, shots_this_frame



class OptimizedBallDetector:
    def __init__(self,
                 h_range=(18, 35),
                 s_range=(80, 255),
                 v_range=(80, 255),
                 min_area=60,
                 max_area=8000,
                 min_circularity=0.55):
        self.h_lo, self.h_hi = h_range
        self.s_lo, self.s_hi = s_range
        self.v_lo, self.v_hi = v_range
        self.min_area = min_area
        self.max_area = max_area
        self.min_circ = min_circularity

    def detect(self, frame) -> list:
        small = cv2.resize(frame, None, fx=0.75, fy=0.75)
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv,
                           (self.h_lo, self.s_lo, self.v_lo),
                           (self.h_hi, self.s_hi, self.v_hi))

        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        balls = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            perim = cv2.arcLength(cnt, True)
            if perim < 1:
                continue

            circ = (4 * np.pi * area) / (perim * perim)
            if circ < self.min_circ:
                continue

            (cx, cy), r = cv2.minEnclosingCircle(cnt)
            cx = int(cx / 0.75)
            cy = int(cy / 0.75)
            r  = int(r  / 0.75)

            balls.append((cx, cy, max(1, r)))

        return balls


if __name__ == "__main__":
    print(__doc__)