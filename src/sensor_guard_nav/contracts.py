"""Immutable, JSON-friendly runtime contracts."""

from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum, StrEnum
from typing import Any


class SensorKind(StrEnum):
    LIDAR = "lidar"
    IMU = "imu"
    WHEEL_ODOMETRY = "wheel_odometry"


class FaultType(StrEnum):
    NONE = "none"
    DROPOUT = "dropout"
    OCCLUSION = "occlusion"
    INTENSITY_ATTENUATION = "intensity_attenuation"
    INVALID_DATA = "invalid_data"
    RANGE_BIAS = "range_bias"
    INCONSISTENT_MOTION = "inconsistent_motion"
    FROZEN = "frozen"
    DELAYED = "delayed"
    UNKNOWN = "unknown"


class NavigationMode(StrEnum):
    HEALTHY = "healthy"
    SUSPECT = "suspect"
    DEGRADED_ODOMETRY = "degraded_odometry"
    SAFE_STOP = "safe_stop"
    RECOVERING_LOCAL = "recovering_local"
    RECOVERING_GLOBAL = "recovering_global"
    MANUAL_REQUIRED = "manual_required"


class RecoveryAction(StrEnum):
    WAIT = "wait"
    LOCAL = "local"
    GLOBAL = "global"
    MANUAL = "manual"


@dataclass(frozen=True, slots=True)
class PoseDelta:
    dx: float
    dy: float
    dyaw: float


@dataclass(frozen=True, slots=True)
class QualityEvidence:
    finite_ratio: float
    count_ratio: float
    angular_coverage: float
    range_score: float
    intensity_score: float
    quality_score: float
    point_count: int
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class LocalizationEvidence:
    scan_delta: PoseDelta
    imu_yaw_delta: float
    wheel_delta: PoseDelta
    translation_error: float
    yaw_error: float
    consistency_score: float
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class SensorHealth:
    sensor: SensorKind
    score: float
    confidence: float
    fault: FaultType
    healthy: bool
    consecutive_bad: int
    consecutive_good: int
    evidence: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class SafetyContext:
    localization_uncertainty_m: float
    obstacle_fov_fraction: float
    braking_deceleration_mps2: float
    reaction_time_s: float
    nearest_obstacle_m: float | None
    fault_duration_s: float
    has_valid_obstacle_source: bool
    dead_reckoning_only: bool


@dataclass(frozen=True, slots=True)
class SafetyEnvelope:
    max_speed_mps: float
    stop_required: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class RecoveryEvidence:
    healthy_frames: int
    local_alignment_residual_m: float
    predicted_pose_error_m: float
    hypothesis_ambiguity: float


@dataclass(frozen=True, slots=True)
class RuntimeDecision:
    timestamp_s: float
    mode: NavigationMode
    previous_mode: NavigationMode
    health: SensorHealth
    safety: SafetyEnvelope
    recovery_action: RecoveryAction
    reason: str
    evidence: tuple[str, ...]
    synthetic: bool = False


def to_jsonable(value: Any) -> Any:
    """Recursively convert contracts, enums and tuples to JSON-compatible values."""
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        return {key: to_jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value
