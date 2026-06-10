import cv2
import numpy as np

from ss_depth import config
from ss_depth.types import ObstacleBox, SectionAnalysis


class DepthObstacleDetector:
  def detect(self, depth_image) -> tuple[list[ObstacleBox], list[SectionAnalysis]]:
    if depth_image is None:
      return [], []

    depth_m = self._to_meter_depth(depth_image)
    roi_top = int(depth_m.shape[0] * (1.0 - config.ROI_HEIGHT_RATIO))
    roi_depth = depth_m[roi_top:, :]
    valid_mask = self._build_valid_mask(roi_depth)
    near_mask = self._build_near_mask(roi_depth, valid_mask)

    obstacle_boxes = self._find_obstacle_boxes(roi_depth, near_mask, roi_top)
    section_analyses = self._analyze_sections(roi_depth, valid_mask, near_mask)
    return obstacle_boxes, section_analyses

  def _to_meter_depth(self, depth_image) -> np.ndarray:
    depth = np.asarray(depth_image)
    if depth.ndim == 3:
      depth = depth[:, :, 0]

    depth = depth.astype(np.float32)
    finite_depth = depth[np.isfinite(depth)]
    if finite_depth.size > 0 and np.max(finite_depth) > config.VALID_MAX_M * 10:
      depth = depth / 1000.0
    return depth

  def _build_valid_mask(self, depth_m: np.ndarray) -> np.ndarray:
    finite_mask = np.isfinite(depth_m)
    return (
      finite_mask
      & (depth_m >= config.VALID_MIN_M)
      & (depth_m <= config.VALID_MAX_M)
    )

  def _build_near_mask(self, depth_m: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    return valid_mask & (depth_m <= config.AVOID_DISTANCE_M)

  def _find_obstacle_boxes(
    self,
    roi_depth: np.ndarray,
    near_mask: np.ndarray,
    roi_top: int,
  ) -> list[ObstacleBox]:
    component_mask = near_mask.astype(np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
      component_mask,
      connectivity=8,
    )

    obstacle_boxes: list[ObstacleBox] = []
    for label in range(1, component_count):
      area = int(stats[label, cv2.CC_STAT_AREA])
      if area < config.MIN_BLOB_AREA:
        continue

      x = int(stats[label, cv2.CC_STAT_LEFT])
      y = int(stats[label, cv2.CC_STAT_TOP])
      width = int(stats[label, cv2.CC_STAT_WIDTH])
      height = int(stats[label, cv2.CC_STAT_HEIGHT])
      component_depth = roi_depth[labels == label]
      median_distance_m = float(np.median(component_depth))
      obstacle_boxes.append(
        ObstacleBox(
          x=x,
          y=y + roi_top,
          width=width,
          height=height,
          distance_m=median_distance_m,
        )
      )

    return obstacle_boxes

  def _analyze_sections(
    self,
    roi_depth: np.ndarray,
    valid_mask: np.ndarray,
    near_mask: np.ndarray,
  ) -> list[SectionAnalysis]:
    analyses: list[SectionAnalysis] = []
    _, width = roi_depth.shape[:2]
    section_width = width / config.SECTION_COUNT

    for index in range(config.SECTION_COUNT):
      start_x = int(index * section_width)
      end_x = int((index + 1) * section_width) if index < config.SECTION_COUNT - 1 else width
      section_valid = valid_mask[:, start_x:end_x]
      section_near = near_mask[:, start_x:end_x]
      valid_count = int(np.count_nonzero(section_valid))
      near_count = int(np.count_nonzero(section_near))
      near_ratio = near_count / valid_count if valid_count > 0 else 0.0
      largest_blob_area = self._largest_blob_area(section_near)
      median_distance_m = self._median_near_distance(roi_depth[:, start_x:end_x], section_near)
      is_blocked = (
        near_ratio >= config.NEAR_RATIO_THRESHOLD
        and largest_blob_area >= config.MIN_BLOB_AREA
        and median_distance_m is not None
        and median_distance_m <= config.AVOID_DISTANCE_M
      )
      analyses.append(
        SectionAnalysis(
          index=index,
          near_ratio=near_ratio,
          largest_blob_area=largest_blob_area,
          median_distance_m=median_distance_m,
          is_blocked=is_blocked,
        )
      )

    return analyses

  def _largest_blob_area(self, section_near_mask: np.ndarray) -> int:
    component_mask = section_near_mask.astype(np.uint8)
    component_count, _, stats, _ = cv2.connectedComponentsWithStats(
      component_mask,
      connectivity=8,
    )
    if component_count <= 1:
      return 0
    return int(np.max(stats[1:, cv2.CC_STAT_AREA]))

  def _median_near_distance(
    self,
    section_depth: np.ndarray,
    section_near_mask: np.ndarray,
  ) -> float | None:
    near_depth = section_depth[section_near_mask]
    if near_depth.size == 0:
      return None
    return float(np.median(near_depth))
