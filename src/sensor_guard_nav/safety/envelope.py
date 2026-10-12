"""Conservative speed envelope for localization and obstacle-sensing loss."""

from __future__ import annotations

import math
from dataclasses import dataclass

from sensor_guard_nav.contracts import SafetyContext, SafetyEnvelope


@dataclass(frozen=True, slots=True)
class SafetyEnvelopeConfig:
    nominal_speed_mps: float = 1.0
    dead_reckoning_speed_mps: float = 0.25
    uncertainty_stop_m: float = 1.0
    minimum_fov_fraction: float = 0.25
    obstacle_margin_m: float = 0.35
    degraded_time_limit_s: float = 8.0
    minimum_command_speed_mps: float = 0.05


def _validate(context: SafetyContext) -> None:
    values = (
        context.localization_uncertainty_m,
        context.obstacle_fov_fraction,
        context.braking_deceleration_mps2,
        context.reaction_time_s,
        context.fault_duration_s,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("safety context values must be finite")
    if context.nearest_obstacle_m is not None and not math.isfinite(context.nearest_obstacle_m):
        raise ValueError("nearest obstacle distance must be finite")
    if context.localization_uncertainty_m < 0 or context.fault_duration_s < 0:
        raise ValueError("uncertainty and duration cannot be negative")
    if not 0 <= context.obstacle_fov_fraction <= 1:
        raise ValueError("obstacle_fov_fraction must be in [0, 1]")
    if context.braking_deceleration_mps2 <= 0 or context.reaction_time_s < 0:
        raise ValueError("braking deceleration must be positive and reaction time non-negative")


def _braking_speed(clearance: float, deceleration: float, reaction: float) -> float:
    if clearance <= 0:
        return 0.0
    return max(
        0.0,
        -deceleration * reaction
        + math.sqrt((deceleration * reaction) ** 2 + 2 * deceleration * clearance),
    )


def compute_safety_envelope(
    context: SafetyContext,
    config: SafetyEnvelopeConfig | None = None,
) -> SafetyEnvelope:
    """Compute a maximum command speed; dead reckoning is always capped."""
    _validate(context)
    cfg = config or SafetyEnvelopeConfig()
    reasons: list[str] = []
    speed = cfg.nominal_speed_mps

    if not context.has_valid_obstacle_source:
        return SafetyEnvelope(0.0, True, ("no valid obstacle source",))
    if context.obstacle_fov_fraction < cfg.minimum_fov_fraction:
        return SafetyEnvelope(0.0, True, ("obstacle field of view below hard minimum",))
    if context.localization_uncertainty_m >= cfg.uncertainty_stop_m:
        return SafetyEnvelope(0.0, True, ("localization uncertainty exceeds stop threshold",))
    if context.fault_duration_s >= cfg.degraded_time_limit_s:
        return SafetyEnvelope(0.0, True, ("degraded operation timeout",))

    fov_cap = cfg.nominal_speed_mps * math.sqrt(context.obstacle_fov_fraction)
    if fov_cap < speed:
        speed = fov_cap
        reasons.append("limited obstacle field of view")
    uncertainty_factor = 1.0 - context.localization_uncertainty_m / cfg.uncertainty_stop_m
    speed = min(speed, cfg.nominal_speed_mps * max(0.0, uncertainty_factor))
    if context.localization_uncertainty_m > 0:
        reasons.append("localization uncertainty")
    if context.dead_reckoning_only:
        speed = min(speed, cfg.dead_reckoning_speed_mps)
        reasons.append("dead reckoning speed cap")
    if context.nearest_obstacle_m is not None:
        clearance = (
            context.nearest_obstacle_m
            - cfg.obstacle_margin_m
            - context.localization_uncertainty_m
        )
        speed = min(
            speed,
            _braking_speed(
                clearance,
                context.braking_deceleration_mps2,
                context.reaction_time_s,
            ),
        )
        reasons.append("braking-distance cap")

    stop = speed < cfg.minimum_command_speed_mps
    return SafetyEnvelope(0.0 if stop else max(0.0, speed), stop, tuple(reasons))
