"""End-to-end SensorGuard-NAV decision pipeline."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from sensor_guard_nav.contracts import (
    PoseDelta,
    RecoveryEvidence,
    RuntimeDecision,
    SafetyContext,
)
from sensor_guard_nav.detection.consistency import evaluate_consistency
from sensor_guard_nav.detection.lidar_quality import evaluate_lidar_quality
from sensor_guard_nav.recovery.gate import RecoveryGate, RecoveryGateConfig
from sensor_guard_nav.safety.envelope import SafetyEnvelopeConfig, compute_safety_envelope
from sensor_guard_nav.state_machine import NavigationStateMachine, StateMachineConfig
from sensor_guard_nav.trust import TrustArbiter, TrustConfig


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    expected_points: int = 360
    expected_intensity: float = 1.0
    angular_bins: int = 36
    min_range_spread_m: float = 2.0
    translation_tolerance_m: float = 0.20
    yaw_tolerance_rad: float = 0.12
    trust: TrustConfig = TrustConfig()
    state_machine: StateMachineConfig = StateMachineConfig()
    safety: SafetyEnvelopeConfig = SafetyEnvelopeConfig()
    recovery: RecoveryGateConfig = RecoveryGateConfig()

    def __post_init__(self) -> None:
        if self.expected_points <= 0 or self.angular_bins < 4:
            raise ValueError("expected_points must be positive and angular_bins >= 4")
        positive = (
            self.expected_intensity,
            self.min_range_spread_m,
            self.translation_tolerance_m,
            self.yaw_tolerance_rad,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError("runtime scales and tolerances must be finite and positive")


class SensorGuardRuntime:
    """Stateful, middleware-independent localization safety runtime."""

    def __init__(self, config: RuntimeConfig | None = None) -> None:
        self.config = config or RuntimeConfig()
        self.trust = TrustArbiter(self.config.trust)
        self.state_machine = NavigationStateMachine(self.config.state_machine)
        self.recovery_gate = RecoveryGate(self.config.recovery)
        self._last_timestamp_s: float | None = None
        self._fault_started_s: float | None = None

    def process(
        self,
        *,
        timestamp_s: float,
        points: NDArray[np.floating],
        scan_delta: PoseDelta,
        imu_yaw_delta: float,
        wheel_delta: PoseDelta,
        safety_context: SafetyContext,
        recovery_evidence: RecoveryEvidence,
        synthetic: bool = False,
    ) -> RuntimeDecision:
        """Process one synchronized frame and return an auditable decision."""
        if not math.isfinite(timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if self._last_timestamp_s is not None and timestamp_s < self._last_timestamp_s:
            raise ValueError("timestamps must be monotonic")
        self._last_timestamp_s = timestamp_s

        quality = evaluate_lidar_quality(
            points,
            expected_points=self.config.expected_points,
            expected_intensity=self.config.expected_intensity,
            angular_bins=self.config.angular_bins,
            min_range_spread_m=self.config.min_range_spread_m,
        )
        consistency = evaluate_consistency(
            scan_delta,
            imu_yaw_delta,
            wheel_delta,
            translation_tolerance_m=self.config.translation_tolerance_m,
            yaw_tolerance_rad=self.config.yaw_tolerance_rad,
        )
        health = self.trust.update(quality, consistency)

        if health.healthy:
            self._fault_started_s = None
        elif self._fault_started_s is None:
            self._fault_started_s = timestamp_s
        fault_duration = (
            0.0 if self._fault_started_s is None else max(0.0, timestamp_s - self._fault_started_s)
        )
        context = replace(
            safety_context,
            fault_duration_s=max(safety_context.fault_duration_s, fault_duration),
            dead_reckoning_only=safety_context.dead_reckoning_only or not health.healthy,
        )
        safety = compute_safety_envelope(context, self.config.safety)
        recovery = self.recovery_gate.decide(health, recovery_evidence)
        transition = self.state_machine.update(timestamp_s, health, safety, recovery)

        evidence = tuple(
            dict.fromkeys(
                quality.reasons
                + consistency.reasons
                + safety.reasons
                + (
                    f"quality={quality.quality_score:.3f}",
                    f"consistency={consistency.consistency_score:.3f}",
                    f"trust={health.score:.3f}",
                )
            )
        )
        return RuntimeDecision(
            timestamp_s=timestamp_s,
            mode=transition.current,
            previous_mode=transition.previous,
            health=health,
            safety=safety,
            recovery_action=recovery,
            reason=transition.reason,
            evidence=evidence,
            synthetic=synthetic,
        )
