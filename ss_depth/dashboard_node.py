import cv2
import rclpy
from rclpy.node import Node

from ss_depth import config
from ss_depth.camera_view_renderer import CameraViewRenderer
from ss_depth.dashboard_renderer import DashboardRenderer
from ss_depth.depth_obstacle_detector import DepthObstacleDetector
from ss_depth.direction_decider import DirectionDecider
from ss_depth.lidar_map_subscriber import LidarMapSubscriber
from ss_depth.map_view_renderer import MapViewRenderer
from ss_depth.path_planner import PathPlanner
from ss_depth.realsense_subscriber import RealsenseSubscriber
from ss_depth.types import PathPoint


class DashboardNode(Node):
  def __init__(self):
    super().__init__("dashboard_node")
    self._realsense_subscriber = RealsenseSubscriber(self)
    self._lidar_map_subscriber = LidarMapSubscriber(self)
    self._depth_obstacle_detector = DepthObstacleDetector()
    self._direction_decider = DirectionDecider()
    self._camera_view_renderer = CameraViewRenderer()
    self._map_view_renderer = MapViewRenderer()
    self._path_planner = PathPlanner()
    self._dashboard_renderer = DashboardRenderer()
    self._goal: PathPoint | None = None
    self._timer = self.create_timer(config.DASHBOARD_TIMER_SEC, self._on_timer)
    self.get_logger().info("ss_depth dashboard node started")

  def _on_timer(self):
    frames = self._realsense_subscriber.frames
    obstacle_boxes, section_analyses = self._depth_obstacle_detector.detect(frames.depth_image)
    direction = self._direction_decider.decide(section_analyses)
    map_state = self._lidar_map_subscriber.state
    path_points = self._path_planner.build_path(map_state.robot_pose, goal=self._goal)
    camera_view = self._camera_view_renderer.render(frames.color_image, obstacle_boxes, direction)
    map_view = self._map_view_renderer.render(
      map_state.occupancy_grid,
      map_state.robot_pose,
      path_points,
      direction,
    )
    dashboard_view = self._dashboard_renderer.render(map_view, camera_view)
    cv2.imshow(config.DASHBOARD_WINDOW_NAME, dashboard_view)
    cv2.waitKey(config.WAIT_KEY_DELAY_MS)


def main(args=None):
  rclpy.init(args=args)
  node = DashboardNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
