#!/usr/bin/env python3
"""Run the no-ROS A/B pipeline and emit only JSONL/JSON/Markdown evidence."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from data_collection.offline.ab_pipeline import validate_ab_pipeline


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--initial-snapshot", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--mount-calibration", type=Path)
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--pose-shaper-bandwidth", type=float, default=25.0)
    args = parser.parse_args()
    result = validate_ab_pipeline(
        args.recording, args.initial_snapshot, args.output,
        side=args.side,
        mount_path=args.mount_calibration,
        position_scale=args.position_scale,
        pose_shaper_bandwidth=args.pose_shaper_bandwidth,
    )
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
