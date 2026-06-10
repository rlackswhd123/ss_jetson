from dataclasses import dataclass, field
import math
from typing import Any

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from ss_depth import config
from ss_depth.types import RobotPose


@dataclass
class LidarMapState:
  occupancy_grid: Any = None
  robot_pose: RobotPose = field(default_factory=RobotPose)


class LidarMapSubscriber:
  def __init__(self, node: Any):
    self._node = node
    self._state = LidarMapState()
    self._map_subscription = node.create_subscription(
      OccupancyGrid,
      config.MAP_TOPIC,
      self._on_map,
      10,
    )
    self._pose_subscription = node.create_subscription(
      PoseStamped,
      config.POSE_TOPIC,
      self._on_pose,
      10,
    )

  @property
  def state(self) -> LidarMapState:
    return self._state

  def _on_map(self, msg: OccupancyGrid):
    self._state.occupancy_grid = msg

  def _on_pose(self, msg: PoseStamped):
    position = msg.pose.position
    orientation = msg.pose.orientation
    self._state.robot_pose = RobotPose(
      x=position.x,
      y=position.y,
      yaw_rad=self._quaternion_to_yaw(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
      ),
    )

  def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
