import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional dependency
    YOLO = None


@dataclass
class CameraCalibration:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class ScanFrame:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class Detection2D:
    label: str
    confidence: float
    x0: int
    y0: int
    x1: int
    y1: int
    detector: str

    @property
    def width(self) -> int:
        return max(0, self.x1 - self.x0)

    @property
    def height(self) -> int:
        return max(0, self.y1 - self.y0)

    @property
    def bottom_center(self) -> Tuple[float, float]:
        return (0.5 * (self.x0 + self.x1), float(self.y1))


class MultiTargetDetector:
    def __init__(
        self,
        enable_people_detection: bool,
        enable_robot_detection: bool,
        yolo_model_path: str,
        yolo_device: str,
        lower_body_crop_top_fraction: float,
        people_confidence_threshold: float,
        robot_min_area_px: int,
        max_detections: int,
        aruco_dictionary: str = "DICT_4X4_50",
    ):
        self.enable_people_detection = bool(enable_people_detection)
        self.enable_robot_detection = bool(enable_robot_detection)
        self.lower_body_crop_top_fraction = min(max(lower_body_crop_top_fraction, 0.0), 0.95)
        self.people_confidence_threshold = max(0.05, float(people_confidence_threshold))
        self.robot_min_area_px = max(16, int(robot_min_area_px))
        self.max_detections = max(1, int(max_detections))
        self.yolo_device = yolo_device.strip()

        self.yolo_model = None
        if yolo_model_path and YOLO is not None:
            try:
                self.yolo_model = YOLO(yolo_model_path)
            except Exception:
                self.yolo_model = None

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.aruco_dict = None
        self.aruco_params = None
        if hasattr(cv2, "aruco"):
            dict_id = getattr(cv2.aruco, aruco_dictionary, None)
            if dict_id is not None:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                if hasattr(cv2.aruco, "DetectorParameters"):
                    self.aruco_params = cv2.aruco.DetectorParameters()
                elif hasattr(cv2.aruco, "DetectorParameters_create"):
                    self.aruco_params = cv2.aruco.DetectorParameters_create()

    def describe_backends(self) -> str:
        parts = []
        if self.enable_people_detection:
            parts.append("people:yolo" if self.yolo_model is not None else "people:hog")
        if self.enable_robot_detection:
            robot_backend = "aruco+color" if self.aruco_dict is not None else "color"
            parts.append(f"robot:{robot_backend}")
        return ",".join(parts)

    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        detections: List[Detection2D] = []
        if self.enable_people_detection:
            detections.extend(self._detect_people(rgb))
        if self.enable_robot_detection:
            detections.extend(self._detect_robots(rgb))
        return self._nms(detections)[: self.max_detections]

    def _detect_people(self, rgb: np.ndarray) -> List[Detection2D]:
        if self.yolo_model is not None:
            return self._detect_people_yolo(rgb)
        return self._detect_people_hog(rgb)

    def _detect_people_yolo(self, rgb: np.ndarray) -> List[Detection2D]:
        frames = [(rgb, 0, "full")]
        crop_top = int(rgb.shape[0] * self.lower_body_crop_top_fraction)
        if crop_top > 0 and crop_top < rgb.shape[0] - 10:
            frames.append((rgb[crop_top:, :, :], crop_top, "lower"))

        detections: List[Detection2D] = []
        for frame, y_offset, suffix in frames:
            kwargs = {
                "source": frame,
                "verbose": False,
                "classes": [0],
                "conf": self.people_confidence_threshold,
            }
            if self.yolo_device:
                kwargs["device"] = self.yolo_device
            results = self.yolo_model.predict(**kwargs)
            if not results:
                continue
            boxes = getattr(results[0], "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                if conf < self.people_confidence_threshold:
                    continue
                xyxy = box.xyxy[0].tolist()
                x0, y0, x1, y1 = [int(round(v)) for v in xyxy]
                detections.append(
                    Detection2D(
                        label="person",
                        confidence=conf,
                        x0=max(0, x0),
                        y0=max(0, y0 + y_offset),
                        x1=min(rgb.shape[1] - 1, x1),
                        y1=min(rgb.shape[0] - 1, y1 + y_offset),
                        detector=f"yolo_person_{suffix}",
                    )
                )
        return detections

    def _detect_people_hog(self, rgb: np.ndarray) -> List[Detection2D]:
        frames = [(rgb, 0, "full")]
        crop_top = int(rgb.shape[0] * self.lower_body_crop_top_fraction)
        if crop_top > 0 and crop_top < rgb.shape[0] - 10:
            frames.append((rgb[crop_top:, :, :], crop_top, "lower"))

        detections: List[Detection2D] = []
        for frame, y_offset, suffix in frames:
            rects, weights = self.hog.detectMultiScale(
                frame,
                winStride=(8, 8),
                padding=(8, 8),
                scale=1.05,
            )
            for (x, y, w, h), weight in zip(rects, weights):
                conf = float(weight) if np.ndim(weight) == 0 else float(weight[0])
                if conf < self.people_confidence_threshold:
                    continue
                if h <= 0 or w <= 0:
                    continue
                aspect = w / float(h)
                if aspect < 0.2 or aspect > 1.2:
                    continue
                detections.append(
                    Detection2D(
                        label="person",
                        confidence=conf,
                        x0=int(x),
                        y0=int(y + y_offset),
                        x1=int(x + w),
                        y1=int(y + h + y_offset),
                        detector=f"hog_person_{suffix}",
                    )
                )
        return detections

    def _detect_robots(self, rgb: np.ndarray) -> List[Detection2D]:
        detections: List[Detection2D] = []
        if self.aruco_dict is not None:
            detections.extend(self._detect_robots_aruco(rgb))
        detections.extend(self._detect_robots_color(rgb))
        return detections

    def _detect_robots_aruco(self, rgb: np.ndarray) -> List[Detection2D]:
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners = []
        ids = None
        if hasattr(cv2.aruco, "ArucoDetector") and self.aruco_params is not None:
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        detections: List[Detection2D] = []
        if ids is None:
            return detections

        for pts in corners:
            flat = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
            x0 = int(np.min(flat[:, 0]))
            y0 = int(np.min(flat[:, 1]))
            x1 = int(np.max(flat[:, 0]))
            y1 = int(np.max(flat[:, 1]))
            detections.append(
                Detection2D(
                    label="robot",
                    confidence=0.99,
                    x0=x0,
                    y0=y0,
                    x1=x1,
                    y1=y1,
                    detector="aruco_robot",
                )
            )
        return detections

    def _detect_robots_color(self, rgb: np.ndarray) -> List[Detection2D]:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        lower_r1 = np.array([0, 90, 60], dtype=np.uint8)
        upper_r1 = np.array([12, 255, 255], dtype=np.uint8)
        lower_r2 = np.array([165, 90, 60], dtype=np.uint8)
        upper_r2 = np.array([179, 255, 255], dtype=np.uint8)
        mask1 = cv2.inRange(hsv, lower_r1, upper_r1)
        mask2 = cv2.inRange(hsv, lower_r2, upper_r2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections: List[Detection2D] = []
        frame_area = float(max(1, rgb.shape[0] * rgb.shape[1]))
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.robot_min_area_px:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            aspect = w / float(h)
            if aspect < 0.15 or aspect > 4.0:
                continue
            confidence = min(0.95, max(0.2, area / (0.02 * frame_area)))
            detections.append(
                Detection2D(
                    label="robot",
                    confidence=confidence,
                    x0=int(x),
                    y0=int(y),
                    x1=int(x + w),
                    y1=int(y + h),
                    detector="color_robot",
                )
            )
        return detections

    def _nms(self, detections: Sequence[Detection2D]) -> List[Detection2D]:
        ordered = sorted(detections, key=lambda det: det.confidence, reverse=True)
        kept: List[Detection2D] = []
        for det in ordered:
            if det.x1 <= det.x0 or det.y1 <= det.y0:
                continue
            duplicate = False
            for other in kept:
                if other.label != det.label:
                    continue
                if _bbox_iou(det, other) > 0.5:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(det)
        return kept


def project_detection_to_ground(
    detection: Detection2D,
    calibration: CameraCalibration,
    camera_height_m: float,
    camera_pitch_rad: float,
    target_ground_z_m: float,
    min_range_m: float,
    max_range_m: float,
    camera_forward_offset_m: float = 0.0,
    camera_lateral_offset_m: float = 0.0,
    scan: Optional[ScanFrame] = None,
    scan_window_half_width: int = 2,
    scan_min_valid_range_m: float = 0.20,
    scan_range_gate_m: float = 2.0,
) -> Optional[Tuple[float, float, float, float]]:
    u, v = detection.bottom_center
    if calibration.fx <= 1e-6 or calibration.fy <= 1e-6:
        return None

    x_opt = (u - calibration.cx) / calibration.fx
    y_opt = (v - calibration.cy) / calibration.fy
    ray_optical = np.array([x_opt, y_opt, 1.0], dtype=np.float64)
    ray_optical /= max(1e-9, np.linalg.norm(ray_optical))

    ray_base = np.array(
        [ray_optical[2], -ray_optical[0], -ray_optical[1]],
        dtype=np.float64,
    )

    c = math.cos(camera_pitch_rad)
    s = math.sin(camera_pitch_rad)
    rot_pitch = np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=np.float64,
    )
    ray_base = rot_pitch @ ray_base

    if ray_base[2] >= -1e-6:
        return None

    t = (target_ground_z_m - camera_height_m) / ray_base[2]
    if t <= 0.0:
        return None

    rel_x = camera_forward_offset_m + t * ray_base[0]
    rel_y = camera_lateral_offset_m + t * ray_base[1]
    bearing = math.atan2(rel_y, rel_x)
    range_m = math.hypot(rel_x, rel_y)

    if scan is not None:
        scan_range = scan_range_at_bearing(
            scan=scan,
            bearing=bearing,
            half_window=scan_window_half_width,
            min_valid_range_m=scan_min_valid_range_m,
        )
        if scan_range is not None and (abs(scan_range - range_m) <= scan_range_gate_m or range_m <= 0.0):
            range_m = scan_range
            rel_x = camera_forward_offset_m + range_m * math.cos(bearing)
            rel_y = camera_lateral_offset_m + range_m * math.sin(bearing)

    if range_m < min_range_m or range_m > max_range_m:
        return None

    return (rel_x, rel_y, bearing, range_m)


def scan_range_at_bearing(
    scan: ScanFrame,
    bearing: float,
    half_window: int,
    min_valid_range_m: float,
) -> Optional[float]:
    if scan.ranges.size == 0 or abs(scan.angle_increment) <= 1e-9:
        return None

    idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
    i0 = max(0, idx - half_window)
    i1 = min(scan.ranges.size, idx + half_window + 1)
    if i0 >= i1:
        return None

    window = scan.ranges[i0:i1]
    valid = np.isfinite(window)
    if not np.any(valid):
        return None

    min_valid = max(float(scan.range_min), float(min_valid_range_m))
    filtered = window[valid]
    filtered = filtered[(filtered >= min_valid) & (filtered <= float(scan.range_max))]
    if filtered.size == 0:
        return None
    return float(np.min(filtered))


def _bbox_iou(a: Detection2D, b: Detection2D) -> float:
    x0 = max(a.x0, b.x0)
    y0 = max(a.y0, b.y0)
    x1 = min(a.x1, b.x1)
    y1 = min(a.y1, b.y1)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    inter = float((x1 - x0) * (y1 - y0))
    union = float(a.width * a.height + b.width * b.height) - inter
    if union <= 0.0:
        return 0.0
    return inter / union
