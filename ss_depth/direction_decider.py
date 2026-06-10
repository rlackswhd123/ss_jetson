from collections import deque

from ss_depth import config
from ss_depth.types import Direction, SectionAnalysis


class DirectionDecider:
  def __init__(self):
    self._recent_directions: deque[Direction] = deque(maxlen=config.STABLE_FRAME_COUNT)
    self._current_direction = Direction.GO

  def decide(self, section_analyses: list[SectionAnalysis]) -> Direction:
    candidate = self._choose_direction(section_analyses)
    self._recent_directions.append(candidate)
    if len(self._recent_directions) == config.STABLE_FRAME_COUNT:
      if len(set(self._recent_directions)) == 1:
        self._current_direction = candidate
    return self._current_direction

  def _choose_direction(self, section_analyses: list[SectionAnalysis]) -> Direction:
    if not section_analyses:
      return Direction.GO
    blocked_indices = {analysis.index for analysis in section_analyses if analysis.is_blocked}
    if not blocked_indices:
      return Direction.GO
    center_index = config.SECTION_COUNT // 2
    if center_index in blocked_indices:
      left_score = sum(1 for index in blocked_indices if index < center_index)
      right_score = sum(1 for index in blocked_indices if index > center_index)
      return Direction.TURN_LEFT if right_score > left_score else Direction.TURN_RIGHT
    if any(index < center_index for index in blocked_indices):
      return Direction.TURN_RIGHT
    return Direction.TURN_LEFT
