"""Cross-check scan-matching motion against IMU and wheel odometry."""

from __future__ import annotations

import math

from sensor_guard_nav.contracts import LocalizationEvidence, PoseDelta


def wrap_angle(angle: float) -> float:
    """Wrap radians to [-pi, pi)."""
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _finite_pose(delta: PoseDelta) -> bool:
    return all(math.isfinite(value) for value in (delta.dx, delta.dy, delta.dyaw))


def evaluate_consistency(
    scan_delta: PoseDelta,
    imu_yaw_delta: float,
    wheel_delta: PoseDelta,
    *,
    translation_tolerance_m: float = 0.20,
    yaw_tolerance_rad: float = 0.12,
) -> LocalizationEvidence:
    """Return a normalized [0, 1] agreement score."""
    if not (_finite_pose(scan_delta) and _finite_pose(wheel_delta)):
        raise ValueError("pose deltas must be finite")
    if not math.isfinite(imu_yaw_delta):
        raise ValueError("IMU yaw delta must be finite")
    if translation_tolerance_m <= 0 or yaw_tolerance_rad <= 0:
        raise ValueError("tolerances must be positive")

    translation_error = math.hypot(
        scan_delta.dx - wheel_delta.dx,
        scan_delta.dy - wheel_delta.dy,
    )
    imu_error = abs(wrap_angle(scan_delta.dyaw - imu_yaw_delta))
    wheel_yaw_error = abs(wrap_angle(scan_delta.dyaw - wheel_delta.dyaw))
    yaw_error = 0.7 * imu_error + 0.3 * wheel_yaw_error
    normalized = math.hypot(
        translation_error / translation_tolerance_m,
        yaw_error / yaw_tolerance_rad,
    )
    score = 1.0 / (1.0 + normalized * normalized)
    reasons: list[str] = []
    if translation_error > translation_tolerance_m:
        reasons.append("scan_wheel_translation_mismatch")
    if yaw_error > yaw_tolerance_rad:
        reasons.append("scan_inertial_yaw_mismatch")
    return LocalizationEvidence(
        scan_delta=scan_delta,
        imu_yaw_delta=imu_yaw_delta,
        wheel_delta=wheel_delta,
        translation_error=translation_error,
        yaw_error=yaw_error,
        consistency_score=score,
        reasons=tuple(reasons),
    )
