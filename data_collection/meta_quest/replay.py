"""Validate and replay full-rate Meta Quest JSONL recordings."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Iterator

from .protocol import PoseSample
from .raw_recorder import SCHEMA_VERSION
from .receiver import ReceivedPose


class RecordingFormatError(ValueError):
    """A JSONL recording is incomplete, corrupt, or violates its schema."""


@dataclass(frozen=True, slots=True)
class PoseRecording:
    metadata: dict[str, Any]
    poses: tuple[ReceivedPose, ...]
    summary: dict[str, Any]

    @property
    def lossless(self) -> bool:
        return self.summary["subscriber_dropped_records"] == 0


def _finite_vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise RecordingFormatError(f"{field} must contain {size} values")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise RecordingFormatError(f"{field} must be numeric") from exc
    if not all(math.isfinite(item) for item in result):
        raise RecordingFormatError(f"{field} must contain finite values")
    return result


def _optional_bool(value: Any, field: str) -> bool | None:
    if value is None or isinstance(value, bool):
        return value
    raise RecordingFormatError(f"{field} must be bool or null")


def _pose_from_record(record: dict[str, Any], line_number: int) -> ReceivedPose:
    try:
        sequence = record["sequence"]
        connection_id = record["connection_id"]
        received_at = float(record["received_monotonic_s"])
        side = record["side"]
        source = record["source"]
        tracked = record["tracked"]
        layout = record["layout"]
    except (KeyError, TypeError, ValueError) as exc:
        raise RecordingFormatError(f"line {line_number}: missing or invalid pose field") from exc
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
        raise RecordingFormatError(f"line {line_number}: sequence must be a positive integer")
    if not isinstance(connection_id, int) or isinstance(connection_id, bool) or connection_id < 0:
        raise RecordingFormatError(f"line {line_number}: connection_id must be non-negative")
    if not math.isfinite(received_at) or received_at < 0.0:
        raise RecordingFormatError(f"line {line_number}: received_monotonic_s is invalid")
    if not isinstance(side, str) or not side:
        raise RecordingFormatError(f"line {line_number}: side must be non-empty")
    if not isinstance(source, str) or not source:
        raise RecordingFormatError(f"line {line_number}: source must be non-empty")
    if not isinstance(tracked, bool):
        raise RecordingFormatError(f"line {line_number}: tracked must be bool")
    if not isinstance(layout, str) or not layout:
        raise RecordingFormatError(f"line {line_number}: layout must be non-empty")
    position = _finite_vector(record.get("position"), 3, "position")
    quaternion = _finite_vector(record.get("quaternion_xyzw"), 4, "quaternion_xyzw")
    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm < 1e-8 or abs(norm - 1.0) > 1e-5:
        raise RecordingFormatError(f"line {line_number}: quaternion must be normalized")
    grasp = record.get("grasp")
    if grasp is not None:
        try:
            grasp = float(grasp)
        except (TypeError, ValueError) as exc:
            raise RecordingFormatError(f"line {line_number}: grasp must be numeric or null") from exc
        if not math.isfinite(grasp):
            raise RecordingFormatError(f"line {line_number}: grasp must be finite")
    frame_id = record.get("frame_id")
    device_timestamp_ns = record.get("device_timestamp_ns")
    for value, name in ((frame_id, "frame_id"), (device_timestamp_ns, "device_timestamp_ns")):
        if value is not None and (not isinstance(value, int) or isinstance(value, bool) or value < 0):
            raise RecordingFormatError(f"line {line_number}: {name} must be non-negative or null")
    sample = PoseSample(
        side=side,
        source=source,
        position=position,  # type: ignore[arg-type]
        quaternion_xyzw=quaternion,  # type: ignore[arg-type]
        tracked=tracked,
        grasp=grasp,
        clutch=_optional_bool(record.get("clutch"), "clutch"),
        record=_optional_bool(record.get("record"), "record"),
        frame_id=frame_id,
        device_timestamp_ns=device_timestamp_ns,
        layout=layout,
    )
    return ReceivedPose(sample, received_at, sequence, connection_id)


def load_recording(path: str | Path) -> PoseRecording:
    path = Path(path)
    metadata = None
    summary = None
    poses: list[ReceivedPose] = []
    last_sequence = 0
    last_received_at = -1.0
    try:
        lines = path.open("r", encoding="utf-8")
    except OSError as exc:
        raise RecordingFormatError(f"cannot open recording {path}: {exc}") from exc
    with lines:
        for line_number, line in enumerate(lines, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RecordingFormatError(f"line {line_number}: invalid JSON") from exc
            if not isinstance(record, dict):
                raise RecordingFormatError(f"line {line_number}: record must be an object")
            record_type = record.get("record_type")
            if line_number == 1:
                if record_type != "metadata":
                    raise RecordingFormatError("first record must be metadata")
                if record.get("schema_version") != SCHEMA_VERSION:
                    raise RecordingFormatError("unsupported schema_version")
                metadata = record
                continue
            if summary is not None:
                raise RecordingFormatError(f"line {line_number}: data follows summary")
            if record_type == "pose":
                pose = _pose_from_record(record, line_number)
                if pose.sequence <= last_sequence:
                    raise RecordingFormatError(f"line {line_number}: sequence is not increasing")
                if pose.received_monotonic_s < last_received_at:
                    raise RecordingFormatError(f"line {line_number}: receive time moved backwards")
                poses.append(pose)
                last_sequence = pose.sequence
                last_received_at = pose.received_monotonic_s
            elif record_type == "summary":
                summary = record
            else:
                raise RecordingFormatError(f"line {line_number}: unsupported record_type")
    if metadata is None:
        raise RecordingFormatError("recording has no metadata")
    if summary is None or summary.get("completed") is not True:
        raise RecordingFormatError("recording has no completed summary")
    if summary.get("pose_records") != len(poses):
        raise RecordingFormatError("summary pose_records does not match file")
    dropped = summary.get("subscriber_dropped_records")
    if not isinstance(dropped, int) or isinstance(dropped, bool) or dropped < 0:
        raise RecordingFormatError("summary subscriber_dropped_records is invalid")
    receiver_records = summary.get("receiver_pose_records")
    if (
        not isinstance(receiver_records, int)
        or isinstance(receiver_records, bool)
        or receiver_records < 0
        or receiver_records != len(poses) + dropped
    ):
        raise RecordingFormatError("summary receiver_pose_records is inconsistent")
    return PoseRecording(metadata, tuple(poses), summary)


def replay_poses(
    recording: PoseRecording,
    speed: float = 1.0,
    no_sleep: bool = False,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> Iterator[ReceivedPose]:
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("speed must be positive and finite")
    previous_time = None
    for pose in recording.poses:
        if previous_time is not None and not no_sleep:
            sleep_fn(max(0.0, pose.received_monotonic_s - previous_time) / speed)
        previous_time = pose.received_monotonic_s
        yield pose


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate and replay a Quest JSONL recording.")
    parser.add_argument("recording", type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-sleep", action="store_true")
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        parser.error("--speed must be positive and finite")
    return args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    recording = load_recording(args.recording)
    status = "lossless" if recording.lossless else (
        f"LOSSY ({recording.summary['subscriber_dropped_records']} subscriber drops)"
    )
    print(f"Loaded {len(recording.poses)} poses, schema={SCHEMA_VERSION}, {status}")
    if args.summary_only:
        return
    for pose in replay_poses(recording, speed=args.speed, no_sleep=args.no_sleep):
        sample = pose.sample
        print(
            f"seq={pose.sequence} conn={pose.connection_id} "
            f"{sample.source}/{sample.side} p={sample.position} "
            f"q_xyzw={sample.quaternion_xyzw}",
            flush=True,
        )


if __name__ == "__main__":
    main()
