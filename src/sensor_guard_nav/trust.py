"""Temporal trust arbitration with smoothing, hysteresis and confirmation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from sensor_guard_nav.contracts import (
    FaultType,
    LocalizationEvidence,
    QualityEvidence,
    SensorHealth,
    SensorKind,
)


@dataclass(frozen=True, slots=True)
class TrustConfig:
    quality_weight: float = 0.65
    consistency_weight: float = 0.35
    ema_alpha: float = 0.40
    window_size: int = 5
    bad_threshold: float = 0.55
    good_threshold: float = 0.75
    bad_frames_required: int = 3
    good_frames_required: int = 4

    def __post_init__(self) -> None:
        if abs(self.quality_weight + self.consistency_weight - 1.0) > 1e-9:
            raise ValueError("trust weights must sum to 1")
        if not 0 < self.ema_alpha <= 1:
            raise ValueError("ema_alpha must be in (0, 1]")
        if not 0 <= self.bad_threshold < self.good_threshold <= 1:
            raise ValueError("thresholds must satisfy 0 <= bad < good <= 1")
        if min(self.window_size, self.bad_frames_required, self.good_frames_required) <= 0:
            raise ValueError("window and confirmation lengths must be positive")


def _classify_fault(reasons: tuple[str, ...]) -> FaultType:
    mappings = (
        ("invalid_points", FaultType.INVALID_DATA),
        ("point_dropout", FaultType.DROPOUT),
        ("angular_occlusion", FaultType.OCCLUSION),
        ("intensity_attenuation", FaultType.INTENSITY_ATTENUATION),
        ("mismatch", FaultType.INCONSISTENT_MOTION),
    )
    for marker, fault in mappings:
        if any(marker in reason for reason in reasons):
            return fault
    return FaultType.UNKNOWN


class TrustArbiter:
    """Stateful LiDAR trust estimator; create one instance per sensor stream."""

    def __init__(self, config: TrustConfig | None = None) -> None:
        self.config = config or TrustConfig()
        self._samples: deque[float] = deque(maxlen=self.config.window_size)
        self._ema: float | None = None
        self._healthy = True
        self._bad = 0
        self._good = 0

    def update(
        self,
        quality: QualityEvidence,
        consistency: LocalizationEvidence,
    ) -> SensorHealth:
        raw = (
            self.config.quality_weight * quality.quality_score
            + self.config.consistency_weight * consistency.consistency_score
        )
        if not 0 <= raw <= 1:
            raise ValueError("input evidence scores must be in [0, 1]")
        self._samples.append(raw)
        self._ema = (
            raw
            if self._ema is None
            else (self.config.ema_alpha * raw + (1 - self.config.ema_alpha) * self._ema)
        )
        score = 0.5 * self._ema + 0.5 * (sum(self._samples) / len(self._samples))

        if score < self.config.bad_threshold:
            self._bad += 1
            self._good = 0
        elif score > self.config.good_threshold:
            self._good += 1
            self._bad = 0
        else:
            self._bad = 0
            self._good = 0

        if self._healthy and self._bad >= self.config.bad_frames_required:
            self._healthy = False
        elif not self._healthy and self._good >= self.config.good_frames_required:
            self._healthy = True

        reasons = quality.reasons + consistency.reasons
        fault = FaultType.NONE if self._healthy else _classify_fault(reasons)
        confidence = min(1.0, len(self._samples) / self.config.window_size)
        return SensorHealth(
            sensor=SensorKind.LIDAR,
            score=score,
            confidence=confidence,
            fault=fault,
            healthy=self._healthy,
            consecutive_bad=self._bad,
            consecutive_good=self._good,
            evidence=reasons,
        )
