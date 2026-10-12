import json
import math

import numpy as np
import pytest

from sensor_guard_nav.contracts import (
    FaultType,
    NavigationMode,
    PoseDelta,
    RecoveryAction,
    RuntimeDecision,
    SafetyEnvelope,
    SensorHealth,
    SensorKind,
    to_jsonable,
)
from sensor_guard_nav.detection.consistency import evaluate_consistency, wrap_angle
from sensor_guard_nav.detection.lidar_quality import evaluate_lidar_quality
from sensor_guard_nav.faults.lidar import (
    intensity_attenuation,
    point_dropout,
    sector_occlusion,
)


def test_contract_is_json_serializable() -> None:
    decision = RuntimeDecision(
        timestamp_s=1.0,
        mode=NavigationMode.HEALTHY,
        previous_mode=NavigationMode.HEALTHY,
        health=SensorHealth(SensorKind.LIDAR, 1.0, 1.0, FaultType.NONE, True, 0, 3),
        safety=SafetyEnvelope(0.8, False, ("nominal",)),
        recovery_action=RecoveryAction.WAIT,
        reason="stable",
        evidence=("quality=1",),
        synthetic=True,
    )
    payload = json.loads(json.dumps(to_jsonable(decision), allow_nan=False))
    assert payload["mode"] == "healthy"
    assert payload["health"]["sensor"] == "lidar"


def test_healthy_cloud_scores_high(healthy_cloud: np.ndarray) -> None:
    evidence = evaluate_lidar_quality(healthy_cloud, expected_points=360)
    assert evidence.quality_score > 0.9
    assert evidence.angular_coverage == 1.0


def test_pollution_reduces_quality(healthy_cloud: np.ndarray) -> None:
    polluted = point_dropout(healthy_cloud, 0.7, seed=4)
    polluted = sector_occlusion(polluted, -1.2, 1.2)
    polluted = intensity_attenuation(polluted, 0.1)
    clean = evaluate_lidar_quality(healthy_cloud, expected_points=360)
    dirty = evaluate_lidar_quality(polluted, expected_points=360)
    assert dirty.quality_score < clean.quality_score - 0.35
    assert {"point_dropout", "angular_occlusion", "intensity_attenuation"} <= set(dirty.reasons)


def test_invalid_cloud_shape_is_rejected() -> None:
    with pytest.raises(ValueError, match="shape"):
        evaluate_lidar_quality(np.ones((10, 3)), expected_points=10)


def test_consistency_handles_angle_wrap() -> None:
    evidence = evaluate_consistency(
        PoseDelta(0.1, 0.0, math.pi - 0.01),
        -math.pi + 0.01,
        PoseDelta(0.1, 0.0, -math.pi + 0.01),
    )
    assert evidence.yaw_error < 0.03
    assert evidence.consistency_score > 0.9
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)


def test_motion_mismatch_has_reason() -> None:
    evidence = evaluate_consistency(
        PoseDelta(1.0, 0.2, 0.8),
        0.0,
        PoseDelta(0.1, 0.0, 0.0),
    )
    assert evidence.consistency_score < 0.1
    assert len(evidence.reasons) == 2
