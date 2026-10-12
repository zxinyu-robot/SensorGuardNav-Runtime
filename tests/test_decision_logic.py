import math

import pytest

from sensor_guard_nav.contracts import (
    FaultType,
    LocalizationEvidence,
    NavigationMode,
    PoseDelta,
    QualityEvidence,
    RecoveryAction,
    RecoveryEvidence,
    SafetyContext,
    SafetyEnvelope,
    SensorHealth,
    SensorKind,
)
from sensor_guard_nav.recovery.gate import RecoveryGate, RecoveryGateConfig
from sensor_guard_nav.safety.envelope import compute_safety_envelope
from sensor_guard_nav.state_machine import NavigationStateMachine
from sensor_guard_nav.trust import TrustArbiter, TrustConfig


def _quality(score: float, *reasons: str) -> QualityEvidence:
    return QualityEvidence(score, score, score, score, score, score, 100, reasons)


def _consistency(score: float) -> LocalizationEvidence:
    pose = PoseDelta(0.0, 0.0, 0.0)
    return LocalizationEvidence(pose, 0.0, pose, 0.0, 0.0, score)


def _health(healthy: bool, bad: int = 0) -> SensorHealth:
    return SensorHealth(
        SensorKind.LIDAR,
        0.9 if healthy else 0.1,
        1.0,
        FaultType.NONE if healthy else FaultType.DROPOUT,
        healthy,
        bad,
        5 if healthy else 0,
        ("test",),
    )


def test_trust_requires_consecutive_frames_and_hysteresis() -> None:
    arbiter = TrustArbiter(
        TrustConfig(
            ema_alpha=1.0,
            window_size=1,
            bad_frames_required=2,
            good_frames_required=3,
        )
    )
    assert arbiter.update(_quality(0.0, "point_dropout"), _consistency(0.0)).healthy
    bad = arbiter.update(_quality(0.0, "point_dropout"), _consistency(0.0))
    assert not bad.healthy and bad.fault is FaultType.DROPOUT
    assert not arbiter.update(_quality(1.0), _consistency(1.0)).healthy
    assert not arbiter.update(_quality(1.0), _consistency(1.0)).healthy
    assert arbiter.update(_quality(1.0), _consistency(1.0)).healthy


def test_state_machine_rejects_illegal_jump() -> None:
    machine = NavigationStateMachine()
    with pytest.raises(ValueError, match="illegal transition"):
        machine.transition(NavigationMode.SAFE_STOP, 1.0, "skip suspect")


def test_state_machine_degrades_then_stops() -> None:
    machine = NavigationStateMachine()
    moving = SafetyEnvelope(0.2, False, ())
    stopped = SafetyEnvelope(0.0, True, ("blind",))
    transition = machine.update(1.0, _health(True, bad=1), moving, RecoveryAction.WAIT)
    assert transition.current is NavigationMode.SUSPECT
    transition = machine.update(2.0, _health(False, bad=3), moving, RecoveryAction.WAIT)
    assert transition.current is NavigationMode.DEGRADED_ODOMETRY
    transition = machine.update(3.0, _health(False, bad=4), stopped, RecoveryAction.WAIT)
    assert transition.current is NavigationMode.SAFE_STOP


def test_safety_stops_without_obstacle_source() -> None:
    envelope = compute_safety_envelope(SafetyContext(0.1, 1.0, 1.0, 0.2, 4.0, 0.0, False, True))
    assert envelope.stop_required
    assert envelope.max_speed_mps == 0.0


def test_dead_reckoning_is_speed_capped() -> None:
    envelope = compute_safety_envelope(SafetyContext(0.1, 0.8, 1.0, 0.2, 4.0, 1.0, True, True))
    assert not envelope.stop_required
    assert envelope.max_speed_mps <= 0.25


def test_near_obstacle_reduces_speed() -> None:
    far = compute_safety_envelope(SafetyContext(0.05, 1.0, 1.0, 0.2, 5.0, 0.0, True, False))
    near = compute_safety_envelope(SafetyContext(0.05, 1.0, 1.0, 0.2, 0.5, 0.0, True, False))
    assert near.max_speed_mps < far.max_speed_mps


def test_recovery_waits_then_allows_local() -> None:
    gate = RecoveryGate(RecoveryGateConfig(healthy_frames_required=3))
    evidence = RecoveryEvidence(0, 0.05, 0.1, 0.1)
    assert gate.decide(_health(True), evidence) is RecoveryAction.WAIT
    assert gate.decide(_health(True), evidence) is RecoveryAction.WAIT
    assert gate.decide(_health(True), evidence) is RecoveryAction.LOCAL


def test_recovery_rejects_nan_evidence() -> None:
    gate = RecoveryGate(RecoveryGateConfig(healthy_frames_required=1))
    with pytest.raises(ValueError, match="finite"):
        gate.decide(_health(True), RecoveryEvidence(0, math.nan, 0.1, 0.1))


@pytest.mark.parametrize(
    ("evidence", "expected"),
    [
        (RecoveryEvidence(0, 0.8, 0.8, 0.5), RecoveryAction.GLOBAL),
        (RecoveryEvidence(0, 0.8, 2.0, 0.9), RecoveryAction.MANUAL),
    ],
)
def test_recovery_selects_global_or_manual(
    evidence: RecoveryEvidence, expected: RecoveryAction
) -> None:
    gate = RecoveryGate(RecoveryGateConfig(healthy_frames_required=1))
    assert gate.decide(_health(True), evidence) is expected
