from typing import Any

import cv2
import numpy as np

from ss_depth.types import Direction, ObstacleBox


class CameraViewRenderer:
  def render(
    self,
    color_image: Any,
    obstacle_boxes: list[ObstacleBox],
    direction: Direction,
  ) -> Any:
    if color_image is None:
      return self._render_waiting_view(direction)

    view = color_image.copy()
    self._draw_obstacle_boxes(view, obstacle_boxes)
    self._draw_direction(view, direction)
    return view

  def _render_waiting_view(self, direction: Direction):
    view = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(
      view,
      "Waiting for RealSense color image",
      (72, 240),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.8,
      (180, 180, 180),
      2,
      cv2.LINE_AA,
    )
    self._draw_direction(view, direction)
    return view

  def _draw_direction(self, view, direction: Direction):
    height, width = view.shape[:2]
    color = (0, 220, 0)
    thickness = 4

    if direction == Direction.TURN_LEFT:
      start = (width // 2, height // 2)
      end = (width // 4, height // 2)
    elif direction == Direction.TURN_RIGHT:
      start = (width // 2, height // 2)
      end = (width * 3 // 4, height // 2)
    else:
      start = (width // 2, height // 3)
      end = (width // 2, height // 6)

    cv2.arrowedLine(view, start, end, color, thickness, tipLength=0.25)
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

  def _draw_obstacle_boxes(self, view, obstacle_boxes: list[ObstacleBox]):
    for box in obstacle_boxes:
      start = (box.x, box.y)
      end = (box.x + box.width, box.y + box.height)
      color = (0, 180, 255)
      cv2.rectangle(view, start, end, color, 2)
      cv2.putText(
        view,
        f"Obstacle {box.distance_m:.2f}m",
        (box.x, max(24, box.y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
      )
