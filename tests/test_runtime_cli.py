import json
from pathlib import Path

import numpy as np
import pytest

from sensor_guard_nav.cli import main, run_demo
from sensor_guard_nav.contracts import PoseDelta, RecoveryEvidence, SafetyContext
from sensor_guard_nav.runtime import SensorGuardRuntime


def _process(runtime: SensorGuardRuntime, timestamp: float, cloud: np.ndarray) -> None:
    runtime.process(
        timestamp_s=timestamp,
        points=cloud,
        scan_delta=PoseDelta(0.1, 0.0, 0.01),
        imu_yaw_delta=0.01,
        wheel_delta=PoseDelta(0.1, 0.0, 0.01),
        safety_context=SafetyContext(0.05, 1.0, 1.0, 0.2, 3.0, 0.0, True, False),
        recovery_evidence=RecoveryEvidence(0, 0.05, 0.1, 0.1),
        synthetic=True,
    )


def test_runtime_rejects_non_monotonic_time(healthy_cloud: np.ndarray) -> None:
    runtime = SensorGuardRuntime()
    _process(runtime, 1.0, healthy_cloud)
    with pytest.raises(ValueError, match="monotonic"):
        _process(runtime, 0.5, healthy_cloud)


def test_demo_generates_auditable_timeline(tmp_path: Path) -> None:
    output = tmp_path / "demo"
    result = run_demo(Path("configs/lidar_mud.json"), output)
    summary = json.loads((output / "summary.json").read_text())
    records = [json.loads(line) for line in (output / "timeline.jsonl").read_text().splitlines()]
    modes = {record["mode"] for record in records}
    assert result["synthetic"] is True
    assert summary["frames"] == 40
    assert {"healthy", "suspect", "safe_stop", "recovering_local"} <= modes
    assert all(record["reason"] and record["evidence"] for record in records)
    assert summary["final_mode"] == "healthy"


def test_cli_smoke_prints_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    exit_code = main(
        [
            "demo",
            "--config",
            "configs/lidar_mud.json",
            "--output",
            str(tmp_path),
        ]
    )
    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["synthetic"] is True
    assert Path(payload["timeline"]).exists()
