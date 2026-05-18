"""
Robot tracking module for FRC scouting.
Handles YOLO detection and BoxMOT tracking (unified, no CSRT fallback).
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from pathlib import Path


class RobotTracker:
    """Manages robot detection and tracking using YOLO + BoxMOT."""
    
    # Tracking configuration
    TRAIL_LEN = 60
    
    def __init__(self, model, tracker, fps: float = 30.0):
        """
        Initialize the robot tracker.
        
        Args:
            model: YOLO model instance
            tracker: BoxMOT tracker instance (e.g., BoostTrack, ByteTrack)
            fps: Video frames per second
        """
        self.model = model
        self.tracker = tracker
        self.fps = fps
        
        # Tracking state
        self.locked_id: Optional[int] = None
        self.trail: List[Tuple[int, int]] = []
        self.status = "UNTRACKED"
        self.current_robot_box: Optional[Tuple[int, int, int, int]] = None
        
        # Latest detections
        self.current_detections = []  # List of (box, track_id)
        
        # Statistics
        self.lost_count = 0
        self.frames_tracked = 0
        self.frame_idx = 0
    
    def update(self, frame: np.ndarray, frame_idx: int, conf_thresh: float = 0.30,
               iou_thresh: float = 0.45) -> Tuple[str, Optional[Tuple[int, int, int, int]], List[Tuple[int, int]]]:
        """
        Update tracking state for a new frame.
        
        Args:
            frame: Current video frame
            frame_idx: Frame index
            conf_thresh: YOLO confidence threshold
            iou_thresh: YOLO IOU threshold
            
        Returns:
            Tuple of (status, robot_box, trail)
        """
        self.frame_idx = frame_idx
        
        # Run YOLO detection
        results = self.model(frame, conf=conf_thresh, iou=iou_thresh, verbose=False)[0]
        
        # Extract detections in format required by BoxMOT
        # BoxMOT expects: Nx6 array of [x1, y1, x2, y2, conf, cls]
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            detections.append([x1, y1, x2, y2, conf, cls])
        
        if detections:
            detections = np.array(detections)
        else:
            detections = np.empty((0, 6))
        
        # Update tracker with detections
        # BoxMOT tracker.update() returns Nx8 array: [x1, y1, x2, y2, id, conf, cls, ind]
        tracked = self.tracker.update(detections, frame)
        
        # Parse tracked results
        self.current_detections = []
        found = False
        
        if tracked is not None and len(tracked) > 0:
            for detection in tracked:
                x1, y1, x2, y2, track_id, conf, cls, _ = detection[:8]
                track_id = int(track_id)
                bbox = (int(x1), int(y1), int(x2), int(y2))
                self.current_detections.append((bbox, track_id))
                
                # Check if locked robot is detected
                if self.locked_id is not None and track_id == self.locked_id:
                    self.current_robot_box = bbox
                    self.status = "TRACKING"
                    found = True
                    self.frames_tracked += 1
        
        if self.locked_id is not None and not found:
            self.status = "LOST"
            self.lost_count += 1
            self.current_robot_box = None
        elif self.locked_id is None:
            self.status = "UNTRACKED"
        
        # Update trail
        if self.current_robot_box is not None:
            x1, y1, x2, y2 = self.current_robot_box
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            self.trail.append((cx, cy))
            if len(self.trail) > self.TRAIL_LEN:
                self.trail.pop(0)
        
        return self.status, self.current_robot_box, self.trail
    
    def lock_to_detection(self, detection_id: int, bbox: Tuple[int, int, int, int]):
        """Lock tracking to a specific detection by track ID."""
        self.locked_id = detection_id
        self.trail.clear()
        self.current_robot_box = bbox
        print(f"[*] Locked to track #{self.locked_id}")
    
    def lock_by_click(self, norm_x: float, norm_y: float, frame_w: int, frame_h: int):
        """
        Lock tracking by clicking on a robot in the frame.
        
        Args:
            norm_x: Normalized x coordinate (0-1)
            norm_y: Normalized y coordinate (0-1)
            frame_w: Frame width in pixels
            frame_h: Frame height in pixels
        """
        px = norm_x * frame_w
        py = norm_y * frame_h
        
        # Find the smallest box containing the click point
        best_id, best_bbox, best_area = None, None, float("inf")
        
        for bbox, track_id in self.current_detections:
            x1, y1, x2, y2 = bbox
            if x1 <= px <= x2 and y1 <= py <= y2:
                area = (x2 - x1) * (y2 - y1)
                if area < best_area:
                    best_area = area
                    best_id = track_id
                    best_bbox = bbox
        
        if best_id is not None:
            self.lock_to_detection(best_id, best_bbox)
            return best_id
        return None
    
    def reset(self):
        """Reset all tracking state."""
        self.locked_id = None
        self.trail.clear()
        self.status = "UNTRACKED"
        self.current_robot_box = None
        self.current_detections = []
        self.lost_count = 0
        self.frames_tracked = 0
    
    def get_detections(self) -> List[Tuple[Tuple[int, int, int, int], int]]:
        """Return current detections as list of (bbox, track_id)."""
        return self.current_detections
    
    def get_locked_id(self) -> Optional[int]:
        """Return currently locked track ID."""
        return self.locked_id
    
    def get_robot_box(self) -> Optional[Tuple[int, int, int, int]]:
        """Return current robot bounding box."""
        return self.current_robot_box
    
    def get_trail(self) -> List[Tuple[int, int]]:
        """Return position trail."""
        return self.trail
    
    def get_status(self) -> str:
        """Return current tracking status."""
        return self.status
    
    def get_stats(self) -> dict:
        """Return tracking statistics."""
        return {
            "frames_tracked": self.frames_tracked,
            "lost_count": self.lost_count,
            "status": self.status,
            "locked_id": self.locked_id,
        }