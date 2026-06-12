import math
import unittest
from types import SimpleNamespace

from ss_depth import config
from ss_depth.goal_aware_direction_decider import GoalAwareDirectionDecider
from ss_depth.map_coordinate_transformer import MapCoordinateTransformer
from ss_depth.types import Direction, RobotPose, SectionAnalysis


class GoalNavigationTest(unittest.TestCase):
  def test_heading_error_positive_turns_left(self):
    decider = GoalAwareDirectionDecider()

    direction = decider.decide([], math.radians(80.0))

    self.assertEqual(Direction.TURN_LEFT, direction)

  def test_heading_error_negative_turns_right(self):
    decider = GoalAwareDirectionDecider()

    direction = decider.decide([], math.radians(-80.0))

    self.assertEqual(Direction.TURN_RIGHT, direction)

  def test_no_heading_error_uses_existing_obstacle_fallback(self):
    decider = GoalAwareDirectionDecider()
    center = config.SECTION_COUNT // 2
    section_analyses = [
      SectionAnalysis(index=index, is_blocked=index == center)
      for index in range(config.SECTION_COUNT)
    ]

    direction = Direction.GO
    for _ in range(config.STABLE_FRAME_COUNT):
      direction = decider.decide(section_analyses, None)

    self.assertEqual(Direction.TURN_RIGHT, direction)

  def test_heading_up_click_above_robot_sets_forward_goal(self):
    transformer = MapCoordinateTransformer()
    occupancy_grid = SimpleNamespace(info=SimpleNamespace(resolution=0.05))
    robot_pose = RobotPose(x=1.0, y=2.0, yaw_rad=0.0)

    goal = transformer.heading_up_pixel_to_world(
      occupancy_grid,
      robot_pose,
      config.MAP_VIEW_WIDTH // 2,
      config.MAP_VIEW_HEIGHT // 2 - 40,
    )

    self.assertIsNotNone(goal)
    self.assertAlmostEqual(2.0, goal.x)
    self.assertAlmostEqual(2.0, goal.y)

  def test_heading_up_click_left_of_robot_sets_left_goal(self):
    transformer = MapCoordinateTransformer()
    occupancy_grid = SimpleNamespace(info=SimpleNamespace(resolution=0.05))
    robot_pose = RobotPose(x=1.0, y=2.0, yaw_rad=0.0)

    goal = transformer.heading_up_pixel_to_world(
      occupancy_grid,
      robot_pose,
      config.MAP_VIEW_WIDTH // 2 - 40,
      config.MAP_VIEW_HEIGHT // 2,
    )

    self.assertIsNotNone(goal)
    self.assertAlmostEqual(1.0, goal.x)
    self.assertAlmostEqual(3.0, goal.y)


if __name__ == "__main__":
  unittest.main()
