"""Command-line entry point for deterministic synthetic demonstrations."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from sensor_guard_nav.contracts import PoseDelta, RecoveryEvidence, SafetyContext
from sensor_guard_nav.faults.lidar import (
    intensity_attenuation,
    nan_corruption,
    point_dropout,
    sector_occlusion,
)
from sensor_guard_nav.recovery.gate import RecoveryGateConfig
from sensor_guard_nav.runtime import RuntimeConfig, SensorGuardRuntime
from sensor_guard_nav.safety.envelope import SafetyEnvelopeConfig
from sensor_guard_nav.state_machine import StateMachineConfig
from sensor_guard_nav.telemetry import JsonlTelemetry, write_summary
from sensor_guard_nav.trust import TrustConfig


def _runtime_config(data: dict[str, Any]) -> RuntimeConfig:
    runtime = data.get("runtime", {})
    return RuntimeConfig(
        expected_points=int(runtime.get("expected_points", 360)),
        expected_intensity=float(runtime.get("expected_intensity", 1.0)),
        angular_bins=int(runtime.get("angular_bins", 36)),
        min_range_spread_m=float(runtime.get("min_range_spread_m", 2.0)),
        translation_tolerance_m=float(runtime.get("translation_tolerance_m", 0.20)),
        yaw_tolerance_rad=float(runtime.get("yaw_tolerance_rad", 0.12)),
        trust=TrustConfig(**data.get("trust", {})),
        state_machine=StateMachineConfig(**data.get("state_machine", {})),
        safety=SafetyEnvelopeConfig(**data.get("safety", {})),
        recovery=RecoveryGateConfig(**data.get("recovery", {})),
    )


def _base_cloud(count: int) -> np.ndarray:
    angles = np.linspace(-math.pi, math.pi, count, endpoint=False)
    ranges = 5.0 + 2.0 * np.sin(3.0 * angles) + 0.5 * np.cos(7.0 * angles)
    return np.column_stack(
        (ranges * np.cos(angles), ranges * np.sin(angles), np.zeros(count), np.ones(count))
    )


def _context(phase: str, progress: float) -> SafetyContext:
    severe = phase == "severe_pollution"
    recovering = phase == "clean_recovery"
    uncertainty = 0.08 if phase == "normal" else 0.25 + 0.45 * progress
    if recovering:
        uncertainty = max(0.05, 0.35 * (1.0 - progress))
    return SafetyContext(
        localization_uncertainty_m=uncertainty,
        obstacle_fov_fraction=0.10 if severe else (0.65 if phase != "normal" else 1.0),
        braking_deceleration_mps2=1.2,
        reaction_time_s=0.20,
        nearest_obstacle_m=2.5,
        fault_duration_s=0.0,
        has_valid_obstacle_source=not severe,
        dead_reckoning_only=False,
    )


def run_demo(config_path: Path, output: Path) -> dict[str, object]:
    data = json.loads(config_path.read_text(encoding="utf-8"))
    runtime = SensorGuardRuntime(_runtime_config(data))
    expected = runtime.config.expected_points
    base = _base_cloud(expected)
    scenario = data.get("scenario", {})
    phases = (
        ("normal", int(scenario.get("normal_frames", 6))),
        ("progressive_pollution", int(scenario.get("progressive_frames", 8))),
        ("severe_pollution", int(scenario.get("severe_frames", 8))),
        ("clean_recovery", int(scenario.get("recovery_frames", 18))),
    )
    if any(count <= 0 for _, count in phases):
        raise ValueError("all scenario frame counts must be positive")

    output.mkdir(parents=True, exist_ok=True)
    timeline_path = output / "timeline.jsonl"
    timeline_path.unlink(missing_ok=True)
    telemetry = JsonlTelemetry(timeline_path)
    decisions = []
    timestamp = 0.0

    for phase, count in phases:
        for index in range(count):
            progress = index / max(1, count - 1)
            points = base.copy()
            scan = PoseDelta(0.10, 0.0, 0.01)
            if phase == "progressive_pollution":
                points = point_dropout(points, 0.10 + 0.45 * progress, seed=index)
                points = intensity_attenuation(points, 1.0 - 0.75 * progress)
                points = sector_occlusion(points, -0.15 - progress, 0.15 + progress)
                scan = PoseDelta(0.10 + 0.30 * progress, 0.0, 0.01 + 0.20 * progress)
            elif phase == "severe_pollution":
                points = point_dropout(points, 0.78, seed=100 + index)
                points = sector_occlusion(points, -1.3, 1.3)
                points = intensity_attenuation(points, 0.12)
                points = nan_corruption(points, 0.30, seed=200 + index)
                scan = PoseDelta(0.8, 0.2, 0.55)

            good_recovery = phase == "clean_recovery"
            recovery = RecoveryEvidence(
                healthy_frames=0,
                local_alignment_residual_m=0.08 if good_recovery else 0.8,
                predicted_pose_error_m=0.12 if good_recovery else 2.0,
                hypothesis_ambiguity=0.10 if good_recovery else 0.85,
            )
            decision = runtime.process(
                timestamp_s=timestamp,
                points=points,
                scan_delta=scan,
                imu_yaw_delta=0.01,
                wheel_delta=PoseDelta(0.10, 0.0, 0.01),
                safety_context=_context(phase, progress),
                recovery_evidence=recovery,
                synthetic=True,
            )
            telemetry.append(decision)
            decisions.append(decision)
            timestamp += 1.0

    summary = write_summary(output / "summary.json", decisions)
    result = {
        "synthetic": True,
        "config": str(config_path),
        "timeline": str(timeline_path),
        "summary": str(output / "summary.json"),
        **summary,
    }
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="sensor-guard-nav")
    subparsers = parser.add_subparsers(dest="command", required=True)
    demo = subparsers.add_parser("demo", help="run a deterministic synthetic scenario")
    demo.add_argument("--config", type=Path, required=True)
    demo.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "demo":
        result = run_demo(args.config, args.output)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
