import time
from enum import Enum

from ss_depth import config
from ss_depth.direction_decider import DirectionDecider
from ss_depth.types import Direction, SectionAnalysis


class GoalArrowState(Enum):
  GO_TO_GOAL = "go_to_goal"
  AVOID_OBSTACLE = "avoid_obstacle"


class GoalAwareDirectionDecider:
  def __init__(self):
    self._fallback_decider = DirectionDecider()
    self._state = GoalArrowState.GO_TO_GOAL
    self._locked_avoid_direction: Direction | None = None
    self._avoid_started_at: float | None = None

  def decide(
    self,
    section_analyses: list[SectionAnalysis],
    heading_error: float | None,
  ) -> Direction:
    if heading_error is None:
      self._reset_avoid_state()
      return self._fallback_decider.decide(section_analyses)

    now = time.monotonic()
    target_direction = self._direction_from_heading_error(heading_error)

    if self._state == GoalArrowState.AVOID_OBSTACLE:
      if self._can_return_to_goal(now, section_analyses, target_direction):
        self._reset_avoid_state()
        return target_direction

      if self._locked_avoid_direction is not None:
        return self._locked_avoid_direction

    if not self._is_direction_blocked(section_analyses, target_direction):
      return target_direction

    avoid_direction = self._choose_avoid_direction(section_analyses, target_direction)
    self._state = GoalArrowState.AVOID_OBSTACLE
    self._locked_avoid_direction = avoid_direction
    self._avoid_started_at = now
    return avoid_direction

  def _direction_from_heading_error(self, heading_error: float) -> Direction:
    if heading_error > config.HEADING_TOLERANCE_RAD:
      return Direction.TURN_LEFT
    if heading_error < -config.HEADING_TOLERANCE_RAD:
      return Direction.TURN_RIGHT
    return Direction.GO

  def _can_return_to_goal(
    self,
    now: float,
    section_analyses: list[SectionAnalysis],
    target_direction: Direction,
  ) -> bool:
    if self._avoid_started_at is None:
      return True
    if now - self._avoid_started_at < config.AVOID_MIN_HOLD_SECONDS:
      return False
    return not self._is_direction_blocked(section_analyses, target_direction)

  def _is_direction_blocked(
    self,
    section_analyses: list[SectionAnalysis],
    direction: Direction,
  ) -> bool:
    if direction == Direction.GO:
      return self._is_front_blocked(section_analyses)
    if direction == Direction.TURN_LEFT:
      return self._is_left_blocked(section_analyses)
    if direction == Direction.TURN_RIGHT:
      return self._is_right_blocked(section_analyses)
    return False

  def _choose_avoid_direction(
    self,
    section_analyses: list[SectionAnalysis],
    target_direction: Direction,
  ) -> Direction:
    if target_direction == Direction.TURN_LEFT:
      return Direction.TURN_RIGHT
    if target_direction == Direction.TURN_RIGHT:
      return Direction.TURN_LEFT

    left_score = self._blocked_score(section_analyses, is_left=True)
    right_score = self._blocked_score(section_analyses, is_left=False)
    if left_score > right_score:
      return Direction.TURN_RIGHT
    if right_score > left_score:
      return Direction.TURN_LEFT
    if self._locked_avoid_direction is not None:
      return self._locked_avoid_direction
    return Direction.TURN_LEFT

  def _is_front_blocked(self, section_analyses: list[SectionAnalysis]) -> bool:
    center_index = config.SECTION_COUNT // 2
    return any(analysis.index == center_index and analysis.is_blocked for analysis in section_analyses)

  def _is_left_blocked(self, section_analyses: list[SectionAnalysis]) -> bool:
    center_index = config.SECTION_COUNT // 2
    return any(analysis.index < center_index and analysis.is_blocked for analysis in section_analyses)

  def _is_right_blocked(self, section_analyses: list[SectionAnalysis]) -> bool:
    center_index = config.SECTION_COUNT // 2
    return any(analysis.index > center_index and analysis.is_blocked for analysis in section_analyses)

  def _blocked_score(self, section_analyses: list[SectionAnalysis], *, is_left: bool) -> float:
    center_index = config.SECTION_COUNT // 2
    score = 0.0
    for analysis in section_analyses:
      if is_left and analysis.index >= center_index:
        continue
      if not is_left and analysis.index <= center_index:
        continue
      if analysis.is_blocked:
        score += 1.0
      score += analysis.near_ratio
    return score

  def _reset_avoid_state(self):
    self._state = GoalArrowState.GO_TO_GOAL
    self._locked_avoid_direction = None
    self._avoid_started_at = None
