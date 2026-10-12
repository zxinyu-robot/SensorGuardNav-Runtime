"""Append-only JSONL telemetry and compact replay summaries."""

from __future__ import annotations

import json
from collections import Counter
from collections.abc import Iterable
from pathlib import Path

from sensor_guard_nav.contracts import RuntimeDecision, to_jsonable


class JsonlTelemetry:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, decision: RuntimeDecision) -> None:
        record = json.dumps(
            to_jsonable(decision), ensure_ascii=False, sort_keys=True, allow_nan=False
        )
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(record + "\n")


def summarize(decisions: Iterable[RuntimeDecision]) -> dict[str, object]:
    records = list(decisions)
    modes = Counter(item.mode.value for item in records)
    faults = Counter(
        item.health.fault.value for item in records if item.health.fault.value != "none"
    )
    transitions = sum(item.mode is not item.previous_mode for item in records)
    stops = sum(item.safety.stop_required for item in records)
    return {
        "frames": len(records),
        "duration_s": (
            max(0.0, records[-1].timestamp_s - records[0].timestamp_s) if records else 0.0
        ),
        "mode_counts": dict(sorted(modes.items())),
        "fault_counts": dict(sorted(faults.items())),
        "transitions": transitions,
        "stop_frames": stops,
        "final_mode": records[-1].mode.value if records else None,
        "synthetic": bool(records) and all(item.synthetic for item in records),
    }


def write_summary(path: str | Path, decisions: Iterable[RuntimeDecision]) -> dict[str, object]:
    summary = summarize(decisions)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return summary
