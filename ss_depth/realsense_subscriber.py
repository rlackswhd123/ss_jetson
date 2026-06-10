from dataclasses import dataclass
from typing import Any

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ss_depth import config


@dataclass
class RealsenseFrameSet:
  color_image: Any = None
  depth_image: Any = None


class RealsenseSubscriber:
  def __init__(self, node: Any):
    self._node = node
    self._frames = RealsenseFrameSet()
    self._bridge = CvBridge()
    self._color_subscription = node.create_subscription(
      Image,
      config.COLOR_TOPIC,
      self._on_color_image,
      10,
    )
    self._depth_subscription = node.create_subscription(
      Image,
      config.DEPTH_TOPIC,
      self._on_depth_image,
      10,
    )

  @property
  def frames(self) -> RealsenseFrameSet:
    return self._frames

  def _on_color_image(self, msg: Image):
    try:
      self._frames.color_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as exc:
      self._node.get_logger().warning(f"failed to convert color image: {exc}")

  def _on_depth_image(self, msg: Image):
    try:
      self._frames.depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as exc:
      self._node.get_logger().warning(f"failed to convert depth image: {exc}")
