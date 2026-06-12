from dataclasses import dataclass, field
import math
import time
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

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
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, node)
    self._last_tf_lookup_at = 0.0
    self._map_subscription = node.create_subscription(
      OccupancyGrid,
      config.MAP_TOPIC,
      self._on_map,
      10,
    )
    self._pose_subscription = node.create_subscription(
      PoseWithCovarianceStamped,
      config.POSE_TOPIC,
      self._on_pose,
      10,
    )

  @property
  def state(self) -> LidarMapState:
    self._update_pose_from_tf()
    return self._state

  def _on_map(self, msg: OccupancyGrid):
    self._state.occupancy_grid = msg

  def _on_pose(self, msg: PoseWithCovarianceStamped):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
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

  def _update_pose_from_tf(self):
    now = time.perf_counter()
    if now - self._last_tf_lookup_at < config.TF_LOOKUP_INTERVAL_SEC:
      return
    self._last_tf_lookup_at = now
    try:
      transform = self._tf_buffer.lookup_transform(
        config.MAP_FRAME,
        config.BASE_FRAME,
        Time(),
      )
    except TransformException:
      return

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    self._state.robot_pose = RobotPose(
      x=translation.x,
      y=translation.y,
      yaw_rad=self._quaternion_to_yaw(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
      ),
    )

  def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
