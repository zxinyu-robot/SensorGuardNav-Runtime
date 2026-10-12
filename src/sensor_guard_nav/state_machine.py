"""Explicit localization degradation and recovery state machine."""

from __future__ import annotations

from dataclasses import dataclass

from sensor_guard_nav.contracts import (
    NavigationMode,
    RecoveryAction,
    SafetyEnvelope,
    SensorHealth,
)


@dataclass(frozen=True, slots=True)
class StateMachineConfig:
    degraded_timeout_s: float = 8.0
    local_recovery_settle_s: float = 1.0
    global_recovery_timeout_s: float = 10.0


@dataclass(frozen=True, slots=True)
class Transition:
    previous: NavigationMode
    current: NavigationMode
    timestamp_s: float
    reason: str
    evidence: tuple[str, ...]


_ALLOWED: dict[NavigationMode, frozenset[NavigationMode]] = {
    NavigationMode.HEALTHY: frozenset({NavigationMode.SUSPECT}),
    NavigationMode.SUSPECT: frozenset(
        {
            NavigationMode.HEALTHY,
            NavigationMode.DEGRADED_ODOMETRY,
            NavigationMode.SAFE_STOP,
        }
    ),
    NavigationMode.DEGRADED_ODOMETRY: frozenset(
        {
            NavigationMode.SAFE_STOP,
            NavigationMode.RECOVERING_LOCAL,
            NavigationMode.RECOVERING_GLOBAL,
            NavigationMode.MANUAL_REQUIRED,
        }
    ),
    NavigationMode.SAFE_STOP: frozenset(
        {
            NavigationMode.RECOVERING_LOCAL,
            NavigationMode.RECOVERING_GLOBAL,
            NavigationMode.MANUAL_REQUIRED,
        }
    ),
    NavigationMode.RECOVERING_LOCAL: frozenset(
        {
            NavigationMode.HEALTHY,
            NavigationMode.DEGRADED_ODOMETRY,
            NavigationMode.SAFE_STOP,
            NavigationMode.RECOVERING_GLOBAL,
            NavigationMode.MANUAL_REQUIRED,
        }
    ),
    NavigationMode.RECOVERING_GLOBAL: frozenset(
        {
            NavigationMode.HEALTHY,
            NavigationMode.SAFE_STOP,
            NavigationMode.MANUAL_REQUIRED,
        }
    ),
    NavigationMode.MANUAL_REQUIRED: frozenset(),
}


class NavigationStateMachine:
    def __init__(self, config: StateMachineConfig | None = None) -> None:
        self.config = config or StateMachineConfig()
        self.mode = NavigationMode.HEALTHY
        self.entered_at_s = 0.0

    def transition(
        self,
        target: NavigationMode,
        timestamp_s: float,
        reason: str,
        evidence: tuple[str, ...] = (),
    ) -> Transition:
        if target == self.mode:
            return Transition(self.mode, self.mode, timestamp_s, reason, evidence)
        if target not in _ALLOWED[self.mode]:
            raise ValueError(f"illegal transition: {self.mode.value} -> {target.value}")
        previous = self.mode
        self.mode = target
        self.entered_at_s = timestamp_s
        return Transition(previous, target, timestamp_s, reason, evidence)

    def update(
        self,
        timestamp_s: float,
        health: SensorHealth,
        safety: SafetyEnvelope,
        recovery: RecoveryAction,
    ) -> Transition:
        elapsed = max(0.0, timestamp_s - self.entered_at_s)
        target = self.mode
        reason = "state held"

        if self.mode is NavigationMode.HEALTHY and health.consecutive_bad > 0:
            target, reason = NavigationMode.SUSPECT, "quality decline awaiting confirmation"
        elif self.mode is NavigationMode.SUSPECT:
            if health.healthy and health.consecutive_bad == 0:
                target, reason = NavigationMode.HEALTHY, "transient anomaly cleared"
            elif not health.healthy:
                target = (
                    NavigationMode.SAFE_STOP
                    if safety.stop_required
                    else NavigationMode.DEGRADED_ODOMETRY
                )
                reason = "LiDAR trust isolated after consecutive failures"
        elif self.mode is NavigationMode.DEGRADED_ODOMETRY:
            if safety.stop_required or elapsed >= self.config.degraded_timeout_s:
                target, reason = NavigationMode.SAFE_STOP, "degraded operation safety limit reached"
            elif health.healthy:
                target, reason = self._recovery_target(recovery)
        elif self.mode is NavigationMode.SAFE_STOP and health.healthy:
            target, reason = self._recovery_target(recovery)
        elif self.mode is NavigationMode.RECOVERING_LOCAL:
            if not health.healthy:
                target = (
                    NavigationMode.SAFE_STOP
                    if safety.stop_required
                    else NavigationMode.DEGRADED_ODOMETRY
                )
                reason = "sensor regressed during local recovery"
            elif recovery is RecoveryAction.GLOBAL:
                target, reason = NavigationMode.RECOVERING_GLOBAL, "local alignment remained ambiguous"
            elif recovery is RecoveryAction.MANUAL:
                target, reason = NavigationMode.MANUAL_REQUIRED, "recovery ambiguity requires operator"
            elif elapsed >= self.config.local_recovery_settle_s:
                target, reason = NavigationMode.HEALTHY, "local alignment remained stable"
        elif self.mode is NavigationMode.RECOVERING_GLOBAL:
            if recovery is RecoveryAction.MANUAL or elapsed >= self.config.global_recovery_timeout_s:
                target, reason = NavigationMode.MANUAL_REQUIRED, "global recovery failed or timed out"
            elif health.healthy and recovery is RecoveryAction.LOCAL:
                target, reason = NavigationMode.HEALTHY, "global localization converged"

        return self.transition(target, timestamp_s, reason, health.evidence)

    @staticmethod
    def _recovery_target(action: RecoveryAction) -> tuple[NavigationMode, str]:
        if action is RecoveryAction.LOCAL:
            return NavigationMode.RECOVERING_LOCAL, "recovery gate approved local alignment"
        if action is RecoveryAction.GLOBAL:
            return NavigationMode.RECOVERING_GLOBAL, "recovery gate requested global localization"
        if action is RecoveryAction.MANUAL:
            return NavigationMode.MANUAL_REQUIRED, "recovery gate requires operator"
        return NavigationMode.SAFE_STOP, "waiting for recovery evidence"
