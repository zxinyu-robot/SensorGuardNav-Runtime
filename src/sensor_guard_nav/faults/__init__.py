"""Deterministic synthetic fault injectors."""

from sensor_guard_nav.faults.lidar import (
    gaussian_noise,
    intensity_attenuation,
    nan_corruption,
    point_dropout,
    range_bias,
    sector_occlusion,
)
from sensor_guard_nav.faults.stream import FrameDelay, FrameFreeze

__all__ = [
    "FrameDelay",
    "FrameFreeze",
    "gaussian_noise",
    "intensity_attenuation",
    "nan_corruption",
    "point_dropout",
    "range_bias",
    "sector_occlusion",
]
