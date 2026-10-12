"""Transport-neutral adapter interfaces.

Implementations may bridge ROS 2 or another middleware, but this module has no
ROS dependency and keeps the safety core independently testable.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray

from sensor_guard_nav.contracts import PoseDelta, RuntimeDecision, SafetyContext


@runtime_checkable
class SensorFrameAdapter(Protocol):
    def point_cloud(self) -> NDArray[np.float64]: ...

    def scan_delta(self) -> PoseDelta: ...

    def imu_yaw_delta(self) -> float: ...

    def wheel_delta(self) -> PoseDelta: ...

    def safety_context(self) -> SafetyContext: ...


@runtime_checkable
class DecisionSink(Protocol):
    def publish(self, decision: RuntimeDecision) -> None: ...
