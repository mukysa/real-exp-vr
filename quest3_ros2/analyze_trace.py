#!/usr/bin/env python3
"""Validate a Quest 3 JSONL trace and summarize its event coverage."""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path


REQUIRED_EVENTS = {
    "requested",
    "requested_invalid",
    "decision",
    "raw",
    "accepted",
    "measured",
    "status",
    "fault_mode",
    "reanchor",
    "initial_anchor",
}


def analyze(trace_path: Path) -> dict[str, object]:
    counts: Counter[str] = Counter()
    line_count = 0
    position_record_count = 0
    for line_count, line in enumerate(trace_path.open(encoding="utf-8"), start=1):
        record = json.loads(line)
        event = str(record["event"])
        counts[event] += 1
        if "positions" in record:
            positions = record["positions"]
            if not isinstance(positions, list) or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in positions
            ):
                raise ValueError(f"invalid positions at line {line_count}")
            position_record_count += 1

    missing = sorted(REQUIRED_EVENTS - counts.keys())
    return {
        "trace": str(trace_path),
        "line_count": line_count,
        "position_record_count": position_record_count,
        "event_counts": dict(sorted(counts.items())),
        "missing_required_events": missing,
        "strict_json_and_finite_positions": True,
        "passed": line_count > 0 and not missing,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = analyze(args.trace)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
