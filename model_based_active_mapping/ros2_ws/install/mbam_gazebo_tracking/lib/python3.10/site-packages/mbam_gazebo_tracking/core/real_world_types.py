import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class TargetDetection:
    label: str
    confidence: float
    world_x: float
    world_y: float
    robot_x: float
    robot_y: float
    bearing_rad: float
    range_m: float
    detector: str
    bbox_xyxy: Optional[Tuple[int, int, int, int]] = None

    def to_dict(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {
            "label": self.label,
            "confidence": float(self.confidence),
            "world_x": float(self.world_x),
            "world_y": float(self.world_y),
            "robot_x": float(self.robot_x),
            "robot_y": float(self.robot_y),
            "bearing_rad": float(self.bearing_rad),
            "range_m": float(self.range_m),
            "detector": self.detector,
        }
        if self.bbox_xyxy is not None:
            data["bbox_xyxy"] = [int(v) for v in self.bbox_xyxy]
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TargetDetection":
        bbox = data.get("bbox_xyxy")
        bbox_xyxy = None
        if isinstance(bbox, Sequence) and len(bbox) == 4:
            bbox_xyxy = tuple(int(v) for v in bbox)  # type: ignore[assignment]
        return cls(
            label=str(data.get("label", "unknown")),
            confidence=float(data.get("confidence", 0.0)),
            world_x=float(data.get("world_x", 0.0)),
            world_y=float(data.get("world_y", 0.0)),
            robot_x=float(data.get("robot_x", 0.0)),
            robot_y=float(data.get("robot_y", 0.0)),
            bearing_rad=float(data.get("bearing_rad", 0.0)),
            range_m=float(data.get("range_m", 0.0)),
            detector=str(data.get("detector", "")),
            bbox_xyxy=bbox_xyxy,
        )


@dataclass
class RobotReport:
    robot_name: str
    stamp_sec: float
    world_frame: str
    pose_x: float
    pose_y: float
    pose_yaw: float
    detections: List[TargetDetection] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_name": self.robot_name,
            "stamp_sec": float(self.stamp_sec),
            "world_frame": self.world_frame,
            "pose": {
                "x": float(self.pose_x),
                "y": float(self.pose_y),
                "yaw": float(self.pose_yaw),
            },
            "detections": [d.to_dict() for d in self.detections],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RobotReport":
        pose = data.get("pose", {})
        detections_data = data.get("detections", [])
        detections = []
        if isinstance(detections_data, list):
            detections = [
                TargetDetection.from_dict(item)
                for item in detections_data
                if isinstance(item, dict)
            ]
        return cls(
            robot_name=str(data.get("robot_name", "")),
            stamp_sec=float(data.get("stamp_sec", 0.0)),
            world_frame=str(data.get("world_frame", "")),
            pose_x=float(pose.get("x", 0.0)),
            pose_y=float(pose.get("y", 0.0)),
            pose_yaw=float(pose.get("yaw", 0.0)),
            detections=detections,
        )

    @classmethod
    def from_json(cls, raw: str) -> "RobotReport":
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError("robot report payload must decode to a JSON object")
        return cls.from_dict(data)


@dataclass
class VelocityCommand:
    robot_name: str
    stamp_sec: float
    linear_x: float
    angular_z: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_name": self.robot_name,
            "stamp_sec": float(self.stamp_sec),
            "twist": {
                "linear_x": float(self.linear_x),
                "angular_z": float(self.angular_z),
            },
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "VelocityCommand":
        twist = data.get("twist", {})
        return cls(
            robot_name=str(data.get("robot_name", "")),
            stamp_sec=float(data.get("stamp_sec", 0.0)),
            linear_x=float(twist.get("linear_x", 0.0)),
            angular_z=float(twist.get("angular_z", 0.0)),
        )

    @classmethod
    def from_json(cls, raw: str) -> "VelocityCommand":
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError("velocity command payload must decode to a JSON object")
        return cls.from_dict(data)
