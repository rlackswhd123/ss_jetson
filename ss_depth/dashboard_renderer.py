from typing import Any

import cv2
import numpy as np

from ss_depth import config


class DashboardRenderer:
  def render(self, map_view: Any, camera_view: Any) -> Any:
    map_panel = self._resize_or_placeholder(
      map_view,
      config.MAP_VIEW_WIDTH,
      config.MAP_VIEW_HEIGHT,
      "Waiting for map",
    )
    camera_panel = self._resize_or_placeholder(
      camera_view,
      config.CAMERA_VIEW_WIDTH,
      config.CAMERA_VIEW_HEIGHT,
      "Waiting for camera",
    )
    return np.hstack((map_panel, camera_panel))

  def _resize_or_placeholder(self, image: Any, width: int, height: int, message: str):
    if image is None:
      placeholder = np.full((height, width, 3), 36, dtype=np.uint8)
      cv2.putText(
        placeholder,
        message,
        (120, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (190, 190, 190),
        2,
        cv2.LINE_AA,
      )
      return placeholder

    if len(image.shape) == 2:
      image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
