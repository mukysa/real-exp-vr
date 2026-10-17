"""Record every validated Meta Quest pose to an atomic JSONL capture."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from queue import Empty
import socket
import subprocess
import time
from typing import Any

from .power import QuestKeepAwake
from .receiver import PoseReceiver, PoseSubscription, ReceivedPose


SCHEMA_VERSION = 1
TESTED_APK_SHA256 = "e0b41ab36e0bcbbc1a1d616bd659bad6d93c02efcb332c1d87c2bee1c6e04879"


def pose_record(pose: ReceivedPose) -> dict[str, Any]:
    sample = pose.sample
    return {
        "record_type": "pose",
        "received_monotonic_s": pose.received_monotonic_s,
        "sequence": pose.sequence,
        "connection_id": pose.connection_id,
        "side": sample.side,
        "source": sample.source,
        "position": list(sample.position),
        "quaternion_xyzw": list(sample.quaternion_xyzw),
        "tracked": sample.tracked,
        "grasp": sample.grasp,
        "clutch": sample.clutch,
        "record": sample.record,
        "frame_id": sample.frame_id,
        "device_timestamp_ns": sample.device_timestamp_ns,
        "layout": sample.layout,
    }


class PoseRecordingWriter:
    """Write to ``.partial`` and publish the capture only after a clean close."""

    def __init__(self, output_path: str | Path, metadata: dict[str, Any]) -> None:
        self.output_path = Path(output_path)
        self.partial_path = self.output_path.with_name(self.output_path.name + ".partial")
        self.metadata = dict(metadata)
        self.pose_records = 0
        self._file = None
        self._completed = False

    def __enter__(self) -> "PoseRecordingWriter":
        if self.output_path.exists():
            raise FileExistsError(f"refusing to overwrite {self.output_path}")
        if self.partial_path.exists():
            raise FileExistsError(f"partial recording already exists: {self.partial_path}")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.partial_path.open("x", encoding="utf-8", newline="\n")
        self._write({
            **self.metadata,
            "record_type": "metadata",
            "schema_version": SCHEMA_VERSION,
        })
        return self

    def write_pose(self, pose: ReceivedPose) -> None:
        if self._file is None or self._completed:
            raise RuntimeError("recording writer is not active")
        self._write(pose_record(pose))
        self.pose_records += 1
        if self.pose_records % 100 == 0:
            self._file.flush()

    def complete(self, summary: dict[str, Any]) -> Path:
        if self._file is None or self._completed:
            raise RuntimeError("recording writer is not active")
        self._write({
            **summary,
            "record_type": "summary",
            "completed": True,
            "pose_records": self.pose_records,
        })
        self._file.flush()
        os.fsync(self._file.fileno())
        self._file.close()
        self._file = None
        os.replace(self.partial_path, self.output_path)
        self._completed = True
        return self.output_path

    def _write(self, value: dict[str, Any]) -> None:
        assert self._file is not None
        self._file.write(
            json.dumps(value, separators=(",", ":"), sort_keys=True, allow_nan=False)
        )
        self._file.write("\n")

    def __exit__(self, *_: object) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None


def _git_commit() -> str | None:
    repository_root = Path(__file__).resolve().parents[2]
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repository_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _drain(subscription: PoseSubscription, writer: PoseRecordingWriter) -> None:
    while True:
        try:
            writer.write_pose(subscription.get_nowait())
        except Empty:
            return


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record every validated Meta Quest pose; this never connects to a robot."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--queue-size", type=int, default=4096)
    parser.add_argument("--print-hz", type=float, default=1.0)
    parser.add_argument("--keep-awake-grace-s", type=float, default=2.0)
    parser.add_argument("--no-keep-awake", action="store_true")
    args = parser.parse_args(argv)
    if not 0 < args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.duration_s is not None and args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if args.queue_size <= 0 or args.print_hz <= 0.0 or args.keep_awake_grace_s <= 0.0:
        parser.error("--queue-size, --print-hz, and --keep-awake-grace-s must be positive")
    return args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    receiver = PoseReceiver(args.host, args.port)
    subscription = receiver.subscribe(args.queue_size)
    power = QuestKeepAwake(enabled=not args.no_keep_awake)
    started_monotonic = time.monotonic()
    metadata = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "host": socket.gethostname(),
        "project_commit": _git_commit(),
        "quest_apk_sha256": TESTED_APK_SHA256,
        "source": "meta_quest_tcp",
        "listen_host": args.host,
        "port": args.port,
    }

    with PoseRecordingWriter(args.output, metadata) as writer:
        receiver.start()
        print(f"Recording every accepted Quest pose on {args.host}:{receiver.port}")
        print(f"Partial output: {writer.partial_path}")
        next_print = time.monotonic()
        try:
            while args.duration_s is None or time.monotonic() - started_monotonic < args.duration_s:
                try:
                    writer.write_pose(subscription.get(timeout=0.1))
                except Empty:
                    pass
                now = time.monotonic()
                snapshot = receiver.snapshot()
                stream_active = any(
                    source == "controller"
                    and now - pose.received_monotonic_s <= args.keep_awake_grace_s
                    for (source, _side), pose in snapshot.poses.items()
                )
                power.update(stream_active)
                if now >= next_print:
                    print(
                        f"poses={snapshot.pose_records} written={writer.pose_records} "
                        f"rejected={snapshot.rejected_pose_records} "
                        f"dropped={subscription.dropped_records}",
                        flush=True,
                    )
                    next_print = now + 1.0 / args.print_hz
        except KeyboardInterrupt:
            print("Stopping raw Quest recording...", flush=True)
        finally:
            receiver.close()
            _drain(subscription, writer)
            snapshot = receiver.snapshot()
            receiver.unsubscribe(subscription)
            power.close()

        duration_s = time.monotonic() - started_monotonic
        output = writer.complete({
            "duration_s": duration_s,
            "receiver_pose_records": snapshot.pose_records,
            "rejected_pose_records": snapshot.rejected_pose_records,
            "subscriber_dropped_records": subscription.dropped_records,
            "accepted_connections": snapshot.accepted_connections,
        })
    print(
        f"Saved {writer.pose_records} poses to {output}; "
        f"dropped={subscription.dropped_records}",
        flush=True,
    )


if __name__ == "__main__":
    main()
