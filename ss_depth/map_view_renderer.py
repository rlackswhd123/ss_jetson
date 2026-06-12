from typing import Any

import cv2
import numpy as np

from ss_depth import config
from ss_depth.types import Direction, PathPoint, RobotPose


class MapViewRenderer:
  def __init__(self):
    self._cached_occupancy_grid: Any = None
    self._cached_raw_map_image: Any = None

  def render(
    self,
    occupancy_grid: Any,
    robot_pose: RobotPose,
    path_points: list[PathPoint],
    direction: Direction,
  ) -> Any:
    if occupancy_grid is None:
      view = self._render_waiting_view(direction)
    else:
      view = self._render_heading_up_map(occupancy_grid, robot_pose)
      self._draw_path(view, occupancy_grid, robot_pose, path_points)
      self._draw_goal(view, occupancy_grid, robot_pose, path_points)
      self._draw_robot(view)

    self._draw_direction(view, direction)
    return view

  def _render_waiting_view(self, direction: Direction):
    _ = direction
    view = np.full(
      (config.MAP_VIEW_HEIGHT, config.MAP_VIEW_WIDTH, 3),
      36,
      dtype=np.uint8,
    )
    cv2.putText(
      view,
      "Waiting for LiDAR map",
      (150, config.MAP_VIEW_HEIGHT // 2),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.8,
      (190, 190, 190),
      2,
      cv2.LINE_AA,
    )
    return view

  def _render_heading_up_map(self, occupancy_grid: Any, robot_pose: RobotPose):
    raw_map = self._get_raw_map_image(occupancy_grid)
    robot_pixel = self._world_to_raw_pixel(occupancy_grid, robot_pose.x, robot_pose.y)

    center_x = config.MAP_VIEW_WIDTH / 2.0
    center_y = config.MAP_VIEW_HEIGHT / 2.0
    angle_deg = 90.0 - np.degrees(robot_pose.yaw_rad)

    matrix = cv2.getRotationMatrix2D(robot_pixel, angle_deg, config.MAP_VIEW_SCALE)
    matrix[0, 2] += center_x - robot_pixel[0]
    matrix[1, 2] += center_y - robot_pixel[1]

    return cv2.warpAffine(
      raw_map,
      matrix,
      (config.MAP_VIEW_WIDTH, config.MAP_VIEW_HEIGHT),
      flags=cv2.INTER_NEAREST,
      borderMode=cv2.BORDER_CONSTANT,
      borderValue=(36, 36, 36),
    )

  def _get_raw_map_image(self, occupancy_grid: Any):
    if occupancy_grid is not self._cached_occupancy_grid:
      self._cached_occupancy_grid = occupancy_grid
      self._cached_raw_map_image = self._build_raw_map_image(occupancy_grid)
    return self._cached_raw_map_image

  def _build_raw_map_image(self, occupancy_grid: Any):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.asarray(occupancy_grid.data, dtype=np.int16).reshape((height, width))

    gray = np.full((height, width), 150, dtype=np.uint8)
    gray[data == 0] = 245
    gray[data > 50] = 25
    gray = np.flipud(gray)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

  def _draw_path(
    self,
    view,
    occupancy_grid: Any,
    robot_pose: RobotPose,
    path_points: list[PathPoint],
  ):
    if len(path_points) < 2:
      return

    pixels = [
      self._world_to_heading_up_pixel(occupancy_grid, robot_pose, point.x, point.y)
      for point in path_points
    ]
    for start, end in zip(pixels, pixels[1:]):
      self._draw_dotted_line(view, start, end, (0, 170, 255), 2)

  def _draw_goal(
    self,
    view,
    occupancy_grid: Any,
    robot_pose: RobotPose,
    path_points: list[PathPoint],
  ):
    if not path_points:
      return

    goal = path_points[-1]
    center = self._world_to_heading_up_pixel(occupancy_grid, robot_pose, goal.x, goal.y)
    cv2.circle(view, center, 8, (0, 80, 255), -1)
    cv2.putText(
      view,
      "GOAL",
      (center[0] + 10, center[1] - 8),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.5,
      (0, 80, 255),
      2,
      cv2.LINE_AA,
    )

  def _draw_robot(self, view):
    center = np.array(
      [config.MAP_VIEW_WIDTH // 2, config.MAP_VIEW_HEIGHT // 2],
      dtype=np.int32,
    )
    points = np.array(
      [
        center + np.array([0, -20]),
        center + np.array([-12, 14]),
        center + np.array([12, 14]),
      ],
      dtype=np.int32,
    )
    cv2.fillConvexPoly(view, points, (30, 120, 255))
    cv2.circle(view, tuple(center), 3, (0, 0, 0), -1)

  def _draw_direction(self, view, direction: Direction):
    color = (0, 220, 0)
    cv2.putText(
      view,
      direction.value,
      (24, 42),
      cv2.FONT_HERSHEY_SIMPLEX,
      1.0,
      color,
      2,
      cv2.LINE_AA,
    )

  def _world_to_raw_pixel(self, occupancy_grid: Any, x_m: float, y_m: float) -> tuple[float, float]:
    info = occupancy_grid.info
    origin = info.origin.position
    map_x = (x_m - origin.x) / info.resolution
    map_y = (y_m - origin.y) / info.resolution
    return (float(map_x), float(info.height - map_y))

  def _world_to_heading_up_pixel(
    self,
    occupancy_grid: Any,
    robot_pose: RobotPose,
    x_m: float,
    y_m: float,
  ) -> tuple[int, int]:
    pixels_per_meter = config.MAP_VIEW_SCALE / occupancy_grid.info.resolution

    dx = x_m - robot_pose.x
    dy = y_m - robot_pose.y

    cos_yaw = np.cos(robot_pose.yaw_rad)
    sin_yaw = np.sin(robot_pose.yaw_rad)
    local_forward = cos_yaw * dx + sin_yaw * dy
    local_left = -sin_yaw * dx + cos_yaw * dy

    pixel_x = config.MAP_VIEW_WIDTH / 2.0 - local_left * pixels_per_meter
    pixel_y = config.MAP_VIEW_HEIGHT / 2.0 - local_forward * pixels_per_meter
    return (
      int(np.clip(pixel_x, 0, config.MAP_VIEW_WIDTH - 1)),
      int(np.clip(pixel_y, 0, config.MAP_VIEW_HEIGHT - 1)),
    )

  def _draw_dotted_line(self, view, start: tuple[int, int], end: tuple[int, int], color, thickness):
    start_point = np.array(start, dtype=np.float32)
    end_point = np.array(end, dtype=np.float32)
    distance = float(np.linalg.norm(end_point - start_point))
    if distance == 0:
      return

    dot_gap = 12
    steps = max(1, int(distance / dot_gap))
    for step in range(steps):
      if step % 2 != 0:
        continue
      t1 = step / steps
      t2 = min((step + 1) / steps, 1.0)
      p1 = start_point + (end_point - start_point) * t1
      p2 = start_point + (end_point - start_point) * t2
      cv2.line(view, tuple(p1.astype(int)), tuple(p2.astype(int)), color, thickness)
