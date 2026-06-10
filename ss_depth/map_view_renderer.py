from typing import Any

import cv2
import numpy as np

from ss_depth import config
from ss_depth.types import Direction, PathPoint, RobotPose


class MapViewRenderer:
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
      view = self._render_map(occupancy_grid)
      self._draw_path(view, occupancy_grid, path_points)
      self._draw_robot(view, occupancy_grid, robot_pose)
      self._draw_goal(view, occupancy_grid, path_points)

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

  def _render_map(self, occupancy_grid: Any):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.asarray(occupancy_grid.data, dtype=np.int16).reshape((height, width))

    gray = np.full((height, width), 150, dtype=np.uint8)
    gray[data == 0] = 245
    gray[data > 50] = 25
    gray = np.flipud(gray)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return cv2.resize(
      color,
      (config.MAP_VIEW_WIDTH, config.MAP_VIEW_HEIGHT),
      interpolation=cv2.INTER_NEAREST,
    )

  def _draw_path(self, view, occupancy_grid: Any, path_points: list[PathPoint]):
    if len(path_points) < 2:
      return

    pixels = [self._world_to_pixel(occupancy_grid, point.x, point.y) for point in path_points]
    for start, end in zip(pixels, pixels[1:]):
      self._draw_dotted_line(view, start, end, (0, 170, 255), 2)

  def _draw_goal(self, view, occupancy_grid: Any, path_points: list[PathPoint]):
    if not path_points:
      return

    goal = path_points[-1]
    center = self._world_to_pixel(occupancy_grid, goal.x, goal.y)
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

  def _draw_robot(self, view, occupancy_grid: Any, robot_pose: RobotPose):
    center = self._world_to_pixel(occupancy_grid, robot_pose.x, robot_pose.y)
    forward = np.array([np.cos(robot_pose.yaw_rad), -np.sin(robot_pose.yaw_rad)])
    side = np.array([-forward[1], forward[0]])
    points = np.array(
      [
        np.array(center) + forward * 18,
        np.array(center) - forward * 12 + side * 10,
        np.array(center) - forward * 12 - side * 10,
      ],
      dtype=np.int32,
    )
    cv2.fillConvexPoly(view, points, (30, 120, 255))
    cv2.circle(view, center, 3, (0, 0, 0), -1)

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

  def _world_to_pixel(self, occupancy_grid: Any, x_m: float, y_m: float) -> tuple[int, int]:
    info = occupancy_grid.info
    origin = info.origin.position
    map_x = (x_m - origin.x) / info.resolution
    map_y = (y_m - origin.y) / info.resolution
    pixel_x = int(map_x * config.MAP_VIEW_WIDTH / info.width)
    pixel_y = int((info.height - map_y) * config.MAP_VIEW_HEIGHT / info.height)
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
