"""Run one healthy frame and print the auditable runtime decision."""

import json
import math

import numpy as np

from sensor_guard_nav.contracts import (
    PoseDelta,
    RecoveryEvidence,
    SafetyContext,
    to_jsonable,
)
from sensor_guard_nav.runtime import SensorGuardRuntime

angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
ranges = 5.0 + 2.0 * np.sin(3.0 * angles)
cloud = np.column_stack(
    (ranges * np.cos(angles), ranges * np.sin(angles), np.zeros(360), np.ones(360))
)

decision = SensorGuardRuntime().process(
    timestamp_s=0.0,
    points=cloud,
    scan_delta=PoseDelta(0.1, 0.0, 0.01),
    imu_yaw_delta=0.01,
    wheel_delta=PoseDelta(0.1, 0.0, 0.01),
    safety_context=SafetyContext(0.05, 1.0, 1.2, 0.2, 3.0, 0.0, True, False),
    recovery_evidence=RecoveryEvidence(0, 0.05, 0.05, 0.1),
    synthetic=True,
)

print(json.dumps(to_jsonable(decision), ensure_ascii=False, indent=2))
