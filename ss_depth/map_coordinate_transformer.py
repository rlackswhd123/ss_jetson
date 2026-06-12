import math
from typing import Any

from ss_depth import config
from ss_depth.types import PathPoint, RobotPose


class MapCoordinateTransformer:
  def heading_up_pixel_to_world(
    self,
    occupancy_grid: Any,
    robot_pose: RobotPose,
    pixel_x: int,
    pixel_y: int,
  ) -> PathPoint | None:
    if occupancy_grid is None:
      return None

    pixels_per_meter = config.MAP_VIEW_SCALE / occupancy_grid.info.resolution
    if pixels_per_meter <= 0.0:
      return None

    center_x = config.MAP_VIEW_WIDTH / 2.0
    center_y = config.MAP_VIEW_HEIGHT / 2.0
    local_forward = (center_y - pixel_y) / pixels_per_meter
    local_left = (center_x - pixel_x) / pixels_per_meter

    cos_yaw = math.cos(robot_pose.yaw_rad)
    sin_yaw = math.sin(robot_pose.yaw_rad)
    world_x = robot_pose.x + local_forward * cos_yaw - local_left * sin_yaw
    world_y = robot_pose.y + local_forward * sin_yaw + local_left * cos_yaw
    return PathPoint(x=world_x, y=world_y)
