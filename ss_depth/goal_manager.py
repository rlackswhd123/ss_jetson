from ss_depth.types import PathPoint


class GoalManager:
  def __init__(self):
    self._goal: PathPoint | None = None

  @property
  def goal(self) -> PathPoint | None:
    return self._goal

  def set_goal(self, x: float, y: float) -> PathPoint:
    self._goal = PathPoint(x=x, y=y)
    return self._goal

  def clear_goal(self):
    self._goal = None

  def has_goal(self) -> bool:
    return self._goal is not None
