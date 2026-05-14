"""
Robot tracking module for FRC scouting.
Handles YOLO detection, ByteTrack integration, and CSRT fallback tracking.
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List


class RobotTracker:
    """Manages robot detection and tracking using YOLO + ByteTrack + CSRT."""
    
    # Tracking configuration
    CSRT_HANDBACK_INTERVAL = 10
    CSRT_REJOIN_IOU = 0.30
    YOLO_LOCK_CONFIRM_SECS = 1.0
    TRAIL_LEN = 60
    
    def __init__(self, model, tracker, fps: float = 30.0):
        """
        Initialize the robot tracker.
        
        Args:
            model: YOLO model instance
            tracker: ByteTrack instance
            fps: Video frames per second
        """
        self.model = model
        self.tracker = tracker
        self.fps = fps
        
        # Tracking state
        self.locked_id: Optional[int] = None
        self.trail: List[Tuple[int, int]] = []
        self.using_csrt = False
        self.status = "UNTRACKED"
        self.current_robot_box: Optional[Tuple[int, int, int, int]] = None
        
        # CSRT state
        self.csrt_tracker = None
        self.csrt_box: Optional[Tuple[int, int, int, int]] = None
        self.csrt_frames = 0
        
        # YOLO rejoin candidate
        self.yolo_candidate_id: Optional[int] = None
        self.yolo_candidate_frames = 0
        
        # Latest detections
        self.sv_dets = None
        
        # Statistics
        self.lost_count = 0
        self.frames_tracked = 0
    
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
        # Run YOLO detection
        results = self.model(frame, conf=conf_thresh, iou=iou_thresh, verbose=False)[0]
        import supervision as sv
        self.sv_dets = sv.Detections.from_ultralytics(results)
        self.sv_dets = self.tracker.update_with_detections(self.sv_dets)
        
        found = False
        yolo_box = None
        
        # Check if locked robot is detected
        if self.locked_id is not None and self.sv_dets is not None and len(self.sv_dets) > 0:
            for bbox, tid in zip(self.sv_dets.xyxy, self.sv_dets.tracker_id):
                if int(tid) == self.locked_id:
                    yolo_box = tuple(map(int, bbox))
                    found = True
                    break
        
        if found:
            self._handle_yolo_lock(yolo_box)
        elif self.locked_id is not None or self.using_csrt:
            self._handle_csrt_fallback(frame, yolo_box)
        else:
            self.status = "UNTRACKED"
        
        # Update trail
        if self.current_robot_box is not None:
            x1, y1, x2, y2 = self.current_robot_box
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            self.trail.append((cx, cy))
            if len(self.trail) > self.TRAIL_LEN:
                self.trail.pop(0)
        
        return self.status, self.current_robot_box, self.trail
    
    def _handle_yolo_lock(self, yolo_box: Tuple[int, int, int, int]):
        """Handle successful YOLO detection of locked robot."""
        x1, y1, x2, y2 = yolo_box
        self.current_robot_box = yolo_box
        self.status = "TRACKING"
        self.using_csrt = False
        self.csrt_frames = 0
        self.yolo_candidate_id = None
        self.yolo_candidate_frames = 0
        
        # Initialize CSRT for next frame
        frame_for_csrt = getattr(self, '_last_frame', None)
        if frame_for_csrt is not None:
            self.csrt_tracker = self._init_csrt(frame_for_csrt, x1, y1, x2, y2)
            self.csrt_box = yolo_box
        
        self.frames_tracked += 1
    
    def _handle_csrt_fallback(self, frame: np.ndarray, yolo_box: Optional[Tuple[int, int, int, int]]):
        """Handle CSRT fallback when YOLO loses track."""
        csrt_ok = False
        
        if self.csrt_tracker is not None:
            ok, rect = self.csrt_tracker.update(frame)
            if ok:
                rx, ry, rw, rh = [int(v) for v in rect]
                h_f, w_f = frame.shape[:2]
                
                # Clamp to frame boundaries
                rx = max(0, min(rx, w_f - 1))
                ry = max(0, min(ry, h_f - 1))
                rw = max(1, min(rw, w_f - rx))
                rh = max(1, min(rh, h_f - ry))
                
                x1, y1, x2, y2 = rx, ry, rx + rw, ry + rh
                self.csrt_box = (x1, y1, x2, y2)
                self.current_robot_box = self.csrt_box
                self.status = "FALLBACK"
                self.using_csrt = True
                self.csrt_frames += 1
                csrt_ok = True
                self.frames_tracked += 1
                
                # Try to rejoin YOLO
                self._try_yolo_rejoin(yolo_box)
        
        if not csrt_ok:
            self.status = "LOST"
            if self.locked_id is not None:
                self.lost_count += 1
    
    def _try_yolo_rejoin(self, yolo_box: Optional[Tuple[int, int, int, int]]):
        """Attempt to rejoin a YOLO detection that overlaps with CSRT."""
        if self.sv_dets is None or len(self.sv_dets) == 0 or self.csrt_box is None:
            return
        
        best_iou, best_tid = 0.0, None
        
        for bbox, tid in zip(self.sv_dets.xyxy, self.sv_dets.tracker_id):
            iou_val = self._iou(self.csrt_box, tuple(map(int, bbox)))
            if iou_val > best_iou:
                best_iou, best_tid = iou_val, int(tid)
        
        if best_iou >= self.CSRT_REJOIN_IOU and best_tid is not None:
            if best_tid == self.yolo_candidate_id:
                self.yolo_candidate_frames += 1
            else:
                self.yolo_candidate_id = best_tid
                self.yolo_candidate_frames = 1
            
            needed = max(1, int(self.fps * self.YOLO_LOCK_CONFIRM_SECS))
            if self.yolo_candidate_frames >= needed:
                self.locked_id = self.yolo_candidate_id
                self.using_csrt = False
                self.csrt_frames = 0
                self.yolo_candidate_id = None
                self.yolo_candidate_frames = 0
                print(f"  [CSRT→YOLO] Auto-promoted to track #{self.locked_id} "
                      f"after {self.YOLO_LOCK_CONFIRM_SECS}s overlap (IoU={best_iou:.2f})")
        else:
            self.yolo_candidate_id = None
            self.yolo_candidate_frames = 0
    
    def lock_to_detection(self, detection_id: int, frame: np.ndarray, bbox: Tuple[int, int, int, int]):
        """Lock tracking to a specific detection."""
        self.locked_id = detection_id
        self.trail.clear()
        self.using_csrt = False
        self.csrt_frames = 0
        self.csrt_box = None
        self.csrt_tracker = None
        self.yolo_candidate_id = None
        self.yolo_candidate_frames = 0
        
        x1, y1, x2, y2 = bbox
        self.csrt_tracker = self._init_csrt(frame, x1, y1, x2, y2)
        print(f"[*] Locked to track #{self.locked_id}")
    
    def lock_to_manual_roi(self, frame: np.ndarray, x1: int, y1: int, x2: int, y2: int):
        """Lock tracking to a manually-drawn region of interest."""
        self.locked_id = None
        self.trail.clear()
        self.using_csrt = True
        self.csrt_frames = 0
        self.csrt_tracker = self._init_csrt(frame, x1, y1, x2, y2)
        self.csrt_box = (x1, y1, x2, y2)
        self.yolo_candidate_id = None
        self.yolo_candidate_frames = 0
        print(f"[✓] Manual ROI set – tracking via CSRT until YOLO re-acquires.")
    
    def reset(self):
        """Reset all tracking state."""
        self.locked_id = None
        self.trail.clear()
        self.using_csrt = False
        self.status = "UNTRACKED"
        self.current_robot_box = None
        self.csrt_tracker = None
        self.csrt_box = None
        self.csrt_frames = 0
        self.yolo_candidate_id = None
        self.yolo_candidate_frames = 0
        self.sv_dets = None
        self.lost_count = 0
        self.frames_tracked = 0
    
    def get_detections(self):
        """Return current detections."""
        return self.sv_dets
    
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
    
    def is_using_csrt(self) -> bool:
        """Return True if currently using CSRT fallback."""
        return self.using_csrt
    
    def get_yolo_candidate(self) -> Tuple[Optional[int], int]:
        """Return (candidate_id, frames_matched)."""
        return self.yolo_candidate_id, self.yolo_candidate_frames
    
    def get_stats(self) -> dict:
        """Return tracking statistics."""
        return {
            "frames_tracked": self.frames_tracked,
            "lost_count": self.lost_count,
            "status": self.status,
            "locked_id": self.locked_id,
        }
    
    @staticmethod
    def _init_csrt(frame: np.ndarray, x1: int, y1: int, x2: int, y2: int):
        """Initialize CSRT tracker for a region."""
        tracker = cv2.TrackerCSRT_create()
        tracker.init(frame, (x1, y1, x2 - x1, y2 - y1))
        return tracker
    
    @staticmethod
    def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        """Calculate intersection-over-union between two boxes."""
        ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
        ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        if inter == 0:
            return 0.0
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])
        return inter / (area_a + area_b - inter)
    
    def _store_frame(self, frame: np.ndarray):
        """Store current frame for CSRT initialization (internal use)."""
        self._last_frame = frame