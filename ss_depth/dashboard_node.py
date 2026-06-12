import time

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
    self._last_perf_log_at = 0.0
    self._frame_index = 0
    self._last_obstacle_boxes = []
    self._last_section_analyses = []
    self._last_direction = self._direction_decider.decide([])
    self._last_map_view = None
    self._timer = self.create_timer(config.DASHBOARD_TIMER_SEC, self._on_timer)
    self.get_logger().info("ss_depth dashboard node started")

  def _on_timer(self):
    self._frame_index += 1
    total_started_at = time.perf_counter()
    frames = self._realsense_subscriber.frames

    started_at = time.perf_counter()
    obstacle_boxes, section_analyses = self._get_depth_analysis(frames.depth_image)
    depth_detect_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    direction = self._direction_decider.decide(section_analyses)
    self._last_direction = direction
    direction_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    map_state = self._lidar_map_subscriber.state
    path_points = self._path_planner.build_path(map_state.robot_pose, goal=self._goal)
    path_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    camera_view = self._camera_view_renderer.render(frames.color_image, obstacle_boxes, direction)
    camera_render_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    map_view = self._get_map_view(
      map_state.occupancy_grid,
      map_state.robot_pose,
      path_points,
      direction,
    )
    map_render_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    dashboard_view = self._dashboard_renderer.render(map_view, camera_view)
    dashboard_render_ms = (time.perf_counter() - started_at) * 1000.0

    started_at = time.perf_counter()
    cv2.imshow(config.DASHBOARD_WINDOW_NAME, dashboard_view)
    cv2.waitKey(config.WAIT_KEY_DELAY_MS)
    display_ms = (time.perf_counter() - started_at) * 1000.0
    total_ms = (time.perf_counter() - total_started_at) * 1000.0

    self._maybe_log_perf(
      depth_detect_ms=depth_detect_ms,
      direction_ms=direction_ms,
      path_ms=path_ms,
      camera_render_ms=camera_render_ms,
      map_render_ms=map_render_ms,
      dashboard_render_ms=dashboard_render_ms,
      display_ms=display_ms,
      total_ms=total_ms,
      has_map=map_state.occupancy_grid is not None,
      has_color=frames.color_image is not None,
      has_depth=frames.depth_image is not None,
    )

  def _maybe_log_perf(
    self,
    *,
    depth_detect_ms: float,
    direction_ms: float,
    path_ms: float,
    camera_render_ms: float,
    map_render_ms: float,
    dashboard_render_ms: float,
    display_ms: float,
    total_ms: float,
    has_map: bool,
    has_color: bool,
    has_depth: bool,
  ):
    now = time.perf_counter()
    if now - self._last_perf_log_at < config.PERF_LOG_INTERVAL_SEC:
      return

    self._last_perf_log_at = now
    self.get_logger().info(
      "perf "
      f"total={total_ms:.1f}ms "
      f"depth_detect={depth_detect_ms:.1f}ms "
      f"direction={direction_ms:.1f}ms "
      f"path={path_ms:.1f}ms "
      f"camera_render={camera_render_ms:.1f}ms "
      f"map_render={map_render_ms:.1f}ms "
      f"dashboard_render={dashboard_render_ms:.1f}ms "
      f"display={display_ms:.1f}ms "
      f"timer_target={config.DASHBOARD_TIMER_SEC * 1000.0:.1f}ms "
      f"has_map={has_map} has_color={has_color} has_depth={has_depth}"
    )

  def _get_depth_analysis(self, depth_image):
    if (
      self._frame_index % config.DEPTH_DETECT_INTERVAL_FRAMES == 1
      or not self._last_section_analyses
    ):
      self._last_obstacle_boxes, self._last_section_analyses = self._depth_obstacle_detector.detect(depth_image)
    return self._last_obstacle_boxes, self._last_section_analyses

  def _get_map_view(self, occupancy_grid, robot_pose, path_points, direction):
    if (
      self._frame_index % config.MAP_RENDER_INTERVAL_FRAMES == 1
      or self._last_map_view is None
    ):
      self._last_map_view = self._map_view_renderer.render(
        occupancy_grid,
        robot_pose,
        path_points,
        direction,
      )
    return self._last_map_view


def main(args=None):
  rclpy.init(args=args)
  node = DashboardNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
