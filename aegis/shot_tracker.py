import cv2
import numpy as np
from collections import deque

class BallTracker:
    
    def __init__(self, fps: float, robot_radius: int = 120):
        self.fps = max(fps, 1.0)
        self.robot_radius = robot_radius
        
        self._next_id = 0
        self._tracks: dict[int, deque] = {}       
        self._missing: dict[int, int] = {}       
        self._near_count: dict[int, int] = {} 
        self._owned: set = set()                 
        self._shot_history: set = set()           
        self._shot_history_ttl: dict[int, int] = {} 
        
        self._velocity: dict[int, tuple] = {}     

    def _dist(self, a: tuple, b: tuple) -> float:
        return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5
    
    def _calc_velocity(self, tid: int) -> tuple:
        hist = self._tracks.get(tid, [])
        if len(hist) < 2:
            return (0, 0)
        if len(hist) >= 3:
            _, x1, y1 = hist[-3]
            _, x2, y2 = hist[-1]
            frames_between = 2
        else:
            _, x1, y1 = hist[-2]
            _, x2, y2 = hist[-1]
            frames_between = 1
        vx = (x2 - x1) / max(frames_between, 1)
        vy = (y2 - y1) / max(frames_between, 1)
        return (vx, vy)
    
    def update(self, detections: list, robot_box: tuple, frame_idx: int) -> tuple:
        
        if robot_box is not None:
            rx1, ry1, rx2, ry2 = robot_box
            rcx = (rx1 + rx2) / 2
            rcy = (ry1 + ry2) / 2
            frame_w = rx2 - rx1
            frame_h = ry2 - ry1
        else:
            rcx, rcy = None, None
            frame_w, frame_h = 640, 480  
        
        dead_shots = [tid for tid, ttl in self._shot_history_ttl.items() if ttl <= 0]
        for tid in dead_shots:
            self._shot_history.discard(tid)
            self._shot_history_ttl.pop(tid, None)
        for tid in self._shot_history_ttl:
            self._shot_history_ttl[tid] -= 1
        
        unmatched_dets = list(range(len(detections)))
        matched_track: dict[int, int] = {}
        
        for tid, hist in self._tracks.items():
            if not hist:
                continue
            _, prev_cx, prev_cy = hist[-1]
            best_dist, best_i = float('inf'), None
            
            max_jump = 100 
            for i in unmatched_dets:
                dcx, dcy, _ = detections[i]
                d = self._dist((prev_cx, prev_cy), (dcx, dcy))
                if d < best_dist and d < max_jump:
                    best_dist, best_i = d, i
            
            if best_i is not None:
                matched_track[tid] = best_i
                unmatched_dets.remove(best_i)
        
        shots_this_frame = 0
        
        for tid, det_i in matched_track.items():
            cx, cy, r = detections[det_i]
            self._tracks[tid].append((frame_idx, cx, cy))
            if len(self._tracks[tid]) > 60:
                self._tracks[tid].popleft()
            
            self._missing[tid] = 0
            vx, vy = self._calc_velocity(tid)
            self._velocity[tid] = (vx, vy)
            
            if rcx is not None:
                dist_to_robot = self._dist((cx, cy), (rcx, rcy))
                if dist_to_robot < self.robot_radius:
                    self._near_count[tid] = self._near_count.get(tid, 0) + 1
                    if self._near_count[tid] >= 2: 
                        self._owned.add(tid)
                else:
                    self._near_count[tid] = 0 
            if tid in self._owned and rcx is not None:
                dist_to_robot = self._dist((cx, cy), (rcx, rcy))
                speed = self._dist((0, 0), (vx, vy))
                
                if dist_to_robot > self.robot_radius:
                    to_robot = (rcx - cx, rcy - cy)
                    to_robot_dist = self._dist(to_robot, (0, 0))
                    
                    if to_robot_dist > 1:
                        dot = vx * to_robot[0] + vy * to_robot[1]
                        if speed >= 3.0 and dot < 0:
                            shots_this_frame += 1
                            self._owned.discard(tid)
                            self._shot_history.add(tid)
                            self._shot_history_ttl[tid] = 15  
        
        gone_ids = []
        for tid in list(self._tracks.keys()):
            if tid not in matched_track:
                self._missing[tid] = self._missing.get(tid, 0) + 1
                if self._missing[tid] >= 2:
                    if tid in self._owned and tid not in self._shot_history:
                        hist = self._tracks[tid]
                        if len(hist) >= 2:
                            _, x1, y1 = hist[-2]
                            _, x2, y2 = hist[-1]
                            dx, dy = x2 - x1, y2 - y1
                            speed = self._dist((0, 0), (dx, dy))
                            
                            if speed >= 3.0:
                                if tid not in self._shot_history:
                                    shots_this_frame += 1
                                    self._shot_history.add(tid)
                                    self._shot_history_ttl[tid] = 15
                    
                    gone_ids.append(tid)
        
        for tid in gone_ids:
            self._tracks.pop(tid, None)
            self._missing.pop(tid, None)
            self._near_count.pop(tid, None)
            self._velocity.pop(tid, None)
            self._owned.discard(tid)
        
        for i in unmatched_dets:
            cx, cy, r = detections[i]
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = deque([(frame_idx, cx, cy)], maxlen=60)
            self._missing[tid] = 0
            self._near_count[tid] = 0
            self._velocity[tid] = (0, 0)
        
        result = []
        for tid, hist in self._tracks.items():
            if hist:
                _, fcx, fcy = hist[-1]
                r = 8  # fallback
                for det in detections:
                    if abs(det[0]-fcx) < 3 and abs(det[1]-fcy) < 3:
                        r = det[2]
                        break
                result.append((tid, fcx, fcy, r))
        
        return result, shots_this_frame


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
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv,
                           (self.h_lo, self.s_lo, self.v_lo),
                           (self.h_hi, self.s_hi, self.v_hi))
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
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
            r = int(r / 0.75)
            
            balls.append((cx, cy, max(1, r)))
        
        return balls

if __name__ == "__main__":
    print(__doc__)
