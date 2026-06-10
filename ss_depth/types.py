from dataclasses import dataclass, field
from enum import Enum


class Direction(str, Enum):
  GO = "GO"
  TURN_LEFT = "TURN_LEFT"
  TURN_RIGHT = "TURN_RIGHT"


@dataclass
class SectionAnalysis:
  index: int
  near_ratio: float = 0.0
  largest_blob_area: int = 0
  median_distance_m: float | None = None
  is_blocked: bool = False


@dataclass
class ObstacleBox:
  x: int
  y: int
  width: int
  height: int
  distance_m: float


@dataclass
class RobotPose:
  x: float = 0.0
  y: float = 0.0
  yaw_rad: float = 0.0


@dataclass
class PathPoint:
  x: float
  y: float


@dataclass
class DashboardState:
  direction: Direction = Direction.GO
  obstacle_boxes: list[ObstacleBox] = field(default_factory=list)
  section_analyses: list[SectionAnalysis] = field(default_factory=list)
  robot_pose: RobotPose = field(default_factory=RobotPose)
  path_points: list[PathPoint] = field(default_factory=list)
