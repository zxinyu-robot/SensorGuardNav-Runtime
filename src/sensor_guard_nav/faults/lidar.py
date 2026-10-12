"""Deterministic LiDAR point-cloud fault injection.

All functions validate and copy the input Nx4 ``x, y, z, intensity`` array.
Random faults require an explicit seed so experiments are replayable.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

PointCloud = NDArray[np.float64]


def _cloud(points: NDArray[np.floating]) -> PointCloud:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 4:
        raise ValueError("point cloud must have shape (N, 4)")
    return cloud.copy()


def _fraction(value: float, name: str) -> None:
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be finite and in [0, 1]")


def point_dropout(points: NDArray[np.floating], fraction: float, *, seed: int = 0) -> PointCloud:
    """Remove an exact, seeded fraction of points."""
    cloud = _cloud(points)
    _fraction(fraction, "fraction")
    count = int(round(len(cloud) * fraction))
    if count == 0:
        return cloud
    keep = np.ones(len(cloud), dtype=bool)
    keep[np.random.default_rng(seed).choice(len(cloud), count, replace=False)] = False
    return cloud[keep]


def sector_occlusion(
    points: NDArray[np.floating], start_angle_rad: float, end_angle_rad: float
) -> PointCloud:
    """Remove points in a wrapped azimuth sector, inclusive of its edges."""
    cloud = _cloud(points)
    if not math.isfinite(start_angle_rad) or not math.isfinite(end_angle_rad):
        raise ValueError("sector angles must be finite")
    start = (start_angle_rad + math.pi) % (2 * math.pi) - math.pi
    end = (end_angle_rad + math.pi) % (2 * math.pi) - math.pi
    angles = np.arctan2(cloud[:, 1], cloud[:, 0])
    inside = (
        (angles >= start) & (angles <= end) if start <= end else (angles >= start) | (angles <= end)
    )
    return cloud[~inside]


def range_bias(points: NDArray[np.floating], bias_m: float) -> PointCloud:
    """Shift every finite point radially while preserving direction."""
    cloud = _cloud(points)
    if not math.isfinite(bias_m):
        raise ValueError("bias_m must be finite")
    xyz = cloud[:, :3]
    ranges = np.linalg.norm(xyz, axis=1)
    mask = np.isfinite(ranges) & (ranges > 1e-9)
    shifted = np.maximum(0.0, ranges[mask] + bias_m)
    xyz[mask] *= (shifted / ranges[mask])[:, None]
    return cloud


def gaussian_noise(points: NDArray[np.floating], sigma_m: float, *, seed: int = 0) -> PointCloud:
    """Add seeded zero-mean Gaussian noise to finite xyz coordinates."""
    cloud = _cloud(points)
    if not math.isfinite(sigma_m) or sigma_m < 0:
        raise ValueError("sigma_m must be finite and non-negative")
    finite = np.isfinite(cloud[:, :3])
    noise = np.random.default_rng(seed).normal(0.0, sigma_m, cloud[:, :3].shape)
    cloud[:, :3][finite] += noise[finite]
    return cloud


def intensity_attenuation(points: NDArray[np.floating], factor: float) -> PointCloud:
    """Multiply finite intensities by a factor in [0, 1]."""
    cloud = _cloud(points)
    _fraction(factor, "factor")
    finite = np.isfinite(cloud[:, 3])
    cloud[finite, 3] *= factor
    return cloud


def nan_corruption(points: NDArray[np.floating], fraction: float, *, seed: int = 0) -> PointCloud:
    """Replace all four fields for a seeded fraction of rows with NaN."""
    cloud = _cloud(points)
    _fraction(fraction, "fraction")
    count = int(round(len(cloud) * fraction))
    if count:
        rows = np.random.default_rng(seed).choice(len(cloud), count, replace=False)
        cloud[rows] = np.nan
    return cloud
