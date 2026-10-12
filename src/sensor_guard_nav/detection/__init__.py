"""Lightweight sensor quality and cross-sensor consistency detectors."""

from sensor_guard_nav.detection.consistency import evaluate_consistency
from sensor_guard_nav.detection.lidar_quality import evaluate_lidar_quality

__all__ = ["evaluate_consistency", "evaluate_lidar_quality"]
