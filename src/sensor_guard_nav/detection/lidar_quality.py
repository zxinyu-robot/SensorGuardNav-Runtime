"""Auditable geometric/statistical LiDAR quality checks."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from sensor_guard_nav.contracts import QualityEvidence


def _validate(points: NDArray[np.floating], expected_points: int, angular_bins: int) -> None:
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError("point cloud must have shape (N, 4): x, y, z, intensity")
    if expected_points <= 0:
        raise ValueError("expected_points must be positive")
    if angular_bins < 4:
        raise ValueError("angular_bins must be at least 4")


def _angular_coverage(xyz: NDArray[np.float64], bins: int) -> float:
    if len(xyz) == 0:
        return 0.0
    angles = np.arctan2(xyz[:, 1], xyz[:, 0])
    occupied, _ = np.histogram(angles, bins=bins, range=(-math.pi, math.pi))
    return float(np.count_nonzero(occupied) / bins)


def evaluate_lidar_quality(
    points: NDArray[np.floating],
    *,
    expected_points: int,
    expected_intensity: float = 1.0,
    angular_bins: int = 36,
    min_range_spread_m: float = 2.0,
) -> QualityEvidence:
    """Score one Nx4 point cloud without modifying it.

    Invalid samples remain part of ``finite_ratio`` but are excluded from all
    geometric statistics. Scores are bounded in [0, 1].
    """
    array = np.asarray(points, dtype=np.float64)
    _validate(array, expected_points, angular_bins)
    if expected_intensity <= 0 or min_range_spread_m <= 0:
        raise ValueError("expected_intensity and min_range_spread_m must be positive")

    finite_mask = np.isfinite(array).all(axis=1)
    finite_ratio = float(finite_mask.mean()) if len(array) else 0.0
    valid = array[finite_mask]
    ranges = np.linalg.norm(valid[:, :3], axis=1) if len(valid) else np.empty(0)
    positive_mask = ranges > 1e-3
    valid = valid[positive_mask]
    ranges = ranges[positive_mask]

    count_ratio = min(1.0, len(valid) / expected_points)
    coverage = _angular_coverage(valid[:, :3], angular_bins)
    if len(ranges) >= 4:
        spread = float(np.percentile(ranges, 90) - np.percentile(ranges, 10))
        range_score = min(1.0, spread / min_range_spread_m)
    else:
        range_score = 0.0
    if len(valid):
        median_intensity = float(np.median(np.clip(valid[:, 3], 0.0, None)))
        intensity_score = min(1.0, median_intensity / expected_intensity)
    else:
        intensity_score = 0.0

    weights = (0.25, 0.25, 0.25, 0.10, 0.15)
    metrics = (finite_ratio, count_ratio, coverage, range_score, intensity_score)
    quality = float(sum(weight * metric for weight, metric in zip(weights, metrics, strict=True)))
    reasons: list[str] = []
    if finite_ratio < 0.98:
        reasons.append("invalid_points")
    if count_ratio < 0.75:
        reasons.append("point_dropout")
    if coverage < 0.70:
        reasons.append("angular_occlusion")
    if range_score < 0.50:
        reasons.append("collapsed_range_distribution")
    if intensity_score < 0.55:
        reasons.append("intensity_attenuation")

    return QualityEvidence(
        finite_ratio=finite_ratio,
        count_ratio=count_ratio,
        angular_coverage=coverage,
        range_score=range_score,
        intensity_score=intensity_score,
        quality_score=quality,
        point_count=len(array),
        reasons=tuple(reasons),
    )
