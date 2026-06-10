from ss_depth.types import PathPoint, RobotPose


class PathPlanner:
  def build_path(self, robot_pose: RobotPose, goal: PathPoint | None) -> list[PathPoint]:
    if goal is None:
      return []

    points: list[PathPoint] = []
    segment_count = 24
    for index in range(segment_count + 1):
      ratio = index / segment_count
      points.append(
        PathPoint(
          x=robot_pose.x + (goal.x - robot_pose.x) * ratio,
          y=robot_pose.y + (goal.y - robot_pose.y) * ratio,
        )
      )
    return points
