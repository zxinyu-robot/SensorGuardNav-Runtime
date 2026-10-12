"""Gate sensor re-admission using temporal and geometric evidence."""

from __future__ import annotations

import math
from dataclasses import dataclass

from sensor_guard_nav.contracts import RecoveryAction, RecoveryEvidence, SensorHealth


@dataclass(frozen=True, slots=True)
class RecoveryGateConfig:
    healthy_frames_required: int = 5
    local_residual_max_m: float = 0.20
    local_prediction_error_max_m: float = 0.35
    local_ambiguity_max: float = 0.30
    global_prediction_error_max_m: float = 1.50
    global_ambiguity_max: float = 0.70

    def __post_init__(self) -> None:
        values = (
            self.local_residual_max_m,
            self.local_prediction_error_max_m,
            self.global_prediction_error_max_m,
        )
        if self.healthy_frames_required <= 0 or not all(
            math.isfinite(value) and value >= 0 for value in values
        ):
            raise ValueError("recovery frame count and error thresholds must be valid")
        ambiguities = (self.local_ambiguity_max, self.global_ambiguity_max)
        if not all(math.isfinite(value) and 0 <= value <= 1 for value in ambiguities):
            raise ValueError("recovery ambiguity thresholds must be in [0, 1]")


class RecoveryGate:
    def __init__(self, config: RecoveryGateConfig | None = None) -> None:
        self.config = config or RecoveryGateConfig()
        self._healthy_frames = 0

    @property
    def healthy_frames(self) -> int:
        return self._healthy_frames

    def decide(self, health: SensorHealth, evidence: RecoveryEvidence) -> RecoveryAction:
        values = (
            evidence.local_alignment_residual_m,
            evidence.predicted_pose_error_m,
            evidence.hypothesis_ambiguity,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("recovery evidence must be finite")
        if any(value < 0 for value in values) or evidence.hypothesis_ambiguity > 1:
            raise ValueError("recovery evidence must be non-negative and ambiguity in [0, 1]")
        if not health.healthy:
            self._healthy_frames = 0
            return RecoveryAction.WAIT
        self._healthy_frames += 1
        if self._healthy_frames < self.config.healthy_frames_required:
            return RecoveryAction.WAIT

        if (
            evidence.local_alignment_residual_m <= self.config.local_residual_max_m
            and evidence.predicted_pose_error_m <= self.config.local_prediction_error_max_m
            and evidence.hypothesis_ambiguity <= self.config.local_ambiguity_max
        ):
            return RecoveryAction.LOCAL
        if (
            evidence.predicted_pose_error_m <= self.config.global_prediction_error_max_m
            and evidence.hypothesis_ambiguity <= self.config.global_ambiguity_max
        ):
            return RecoveryAction.GLOBAL
        return RecoveryAction.MANUAL
