"""Generate robot-disconnected end-effector target previews from Quest JSONL."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Sequence

from .calibration import PoseCalibration, RigidPose
from .coordinates import IDENTITY_POSE, UNITY_TO_ROBOT_BASIS, unity_to_robot_pose
from .models import TargetPose
from .receiver import ReceivedPose
from .replay import PoseRecording, load_recording


PREVIEW_SCHEMA_VERSION = 1


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class PreviewResult:
    targets: tuple[TargetPose, ...]
    selected_pose_records: int
    ignored_pose_records: int
    anchor_sequences: tuple[int, ...]

    @property
    def valid_targets(self) -> int:
        return sum(target.valid for target in self.targets)

    @property
    def held_targets(self) -> int:
        return len(self.targets) - self.valid_targets


class TargetPreviewGenerator:
    """Pure offline mapper with explicit anchoring and reconnect holds.

    ``first`` anchoring exists only for inspecting a known recording.  A live
    control state machine must use a deliberate button edge.  Once a TCP
    connection changes, this class never silently continues the old mapping.
    """

    def __init__(
        self,
        target_anchor: RigidPose,
        *,
        side: str = "right",
        anchor: str = "first",
        frame_id: str = "right_teleop_preview",
        position_scale: float = 1.0,
        controller_to_tool: RigidPose = IDENTITY_POSE,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if anchor not in {"first", "clutch", "record"}:
            raise ValueError("anchor must be first, clutch, or record")
        if not frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(position_scale) or position_scale <= 0.0:
            raise ValueError("position_scale must be positive and finite")
        self.target_anchor = target_anchor
        self.side = side
        self.anchor = anchor
        self.frame_id = frame_id
        self.position_scale = position_scale
        self.controller_to_tool = controller_to_tool
        self._calibration: PoseCalibration | None = None
        self._connection_id: int | None = None
        self._last_target = target_anchor
        self._previous_button = False
        self._button_released = False
        self._reconnected = False
        self.anchor_sequences: list[int] = []

    def _button_pressed(self, pose: ReceivedPose) -> bool:
        if self.anchor == "clutch":
            return bool(pose.sample.clutch)
        if self.anchor == "record":
            return bool(pose.sample.record)
        return False

    def _anchor_requested(self, pose: ReceivedPose) -> bool:
        if self.anchor == "first":
            return self._connection_id is None and self._calibration is None
        pressed = self._button_pressed(pose)
        if not pressed:
            self._button_released = True
        rising_edge = pressed and not self._previous_button and self._button_released
        self._previous_button = pressed
        return rising_edge

    def update(self, received: ReceivedPose) -> TargetPose | None:
        sample = received.sample
        if sample.source != "controller" or sample.side != self.side:
            return None
        converted = unity_to_robot_pose(
            RigidPose(sample.position, sample.quaternion_xyzw),
            position_scale=self.position_scale,
            controller_to_tool=self.controller_to_tool,
        )
        if self._connection_id is not None and received.connection_id != self._connection_id:
            self._calibration = None
            self._reconnected = True
            self._previous_button = bool(self._button_pressed(received))
            self._button_released = not self._previous_button
        anchor_requested = self._anchor_requested(received)
        if anchor_requested:
            # Re-anchoring must preserve the last target exactly.  Mapping a
            # later button edge back to the original target anchor would make
            # clutch toggles discontinuous.
            anchor_target = self._last_target if self.anchor_sequences else self.target_anchor
            self._calibration = PoseCalibration.from_anchor(converted, anchor_target)
            self._connection_id = received.connection_id
            self._reconnected = False
            self.anchor_sequences.append(received.sequence)
        if self._calibration is None:
            reason = "reconnect_requires_button_anchor" if self._reconnected else "waiting_for_anchor"
            return TargetPose(
                pose=self._last_target,
                frame_id=self.frame_id,
                source_sequence=received.sequence,
                source_connection_id=received.connection_id,
                generated_monotonic_s=received.received_monotonic_s,
                state="HOLD",
                valid=False,
                hold_reason=reason,
            )
        mapped = self._calibration.apply(converted)
        self._last_target = mapped
        return TargetPose(
            pose=mapped,
            frame_id=self.frame_id,
            source_sequence=received.sequence,
            source_connection_id=received.connection_id,
            generated_monotonic_s=received.received_monotonic_s,
            state="PREVIEW",
            valid=True,
        )


def generate_targets(
    recording: PoseRecording,
    generator: TargetPreviewGenerator,
) -> PreviewResult:
    targets: list[TargetPose] = []
    ignored = 0
    for received in recording.poses:
        target = generator.update(received)
        if target is None:
            ignored += 1
        else:
            targets.append(target)
    return PreviewResult(
        tuple(targets),
        selected_pose_records=len(targets),
        ignored_pose_records=ignored,
        anchor_sequences=tuple(generator.anchor_sequences),
    )


def _target_record(target: TargetPose) -> dict[str, Any]:
    return {
        "record_type": "target",
        "source_sequence": target.source_sequence,
        "source_connection_id": target.source_connection_id,
        "generated_monotonic_s": target.generated_monotonic_s,
        "frame_id": target.frame_id,
        "state": target.state,
        "valid": target.valid,
        "hold_reason": target.hold_reason,
        "position": list(target.pose.position),
        "quaternion_xyzw": list(target.pose.quaternion_xyzw),
    }


def write_preview(
    output_path: str | Path,
    *,
    source_path: str | Path,
    source_sha256: str,
    result: PreviewResult,
    metadata: dict[str, Any],
    summary_metadata: dict[str, Any] | None = None,
) -> Path:
    output = Path(output_path)
    source = Path(source_path)
    if output.resolve() == source.resolve():
        raise ValueError("preview output must not overwrite the source recording")
    partial = output.with_name(output.name + ".partial")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    if partial.exists():
        raise FileExistsError(f"partial preview already exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with partial.open("x", encoding="utf-8", newline="\n") as target_file:
        records: Iterable[dict[str, Any]] = (
            {
                **metadata,
                "record_type": "metadata",
                "schema_version": PREVIEW_SCHEMA_VERSION,
                "source_recording": str(source),
                "source_sha256": source_sha256,
                "robot_output_enabled": False,
                "recenter_detection": "unsupported_by_controller_compact_controls_v1",
            },
            *(_target_record(target) for target in result.targets),
            {
                **(summary_metadata or {}),
                "record_type": "summary",
                "completed": True,
                "selected_pose_records": result.selected_pose_records,
                "ignored_pose_records": result.ignored_pose_records,
                "valid_targets": result.valid_targets,
                "held_targets": result.held_targets,
                "anchor_sequences": list(result.anchor_sequences),
            },
        )
        for record in records:
            target_file.write(json.dumps(record, separators=(",", ":"), sort_keys=True, allow_nan=False))
            target_file.write("\n")
        target_file.flush()
        os.fsync(target_file.fileno())
    if sha256_file(source) != source_sha256:
        raise RuntimeError("source recording changed while generating preview; output not published")
    os.replace(partial, output)
    return output


def _plot_targets(path: Path, targets: Sequence[TargetPose]) -> None:
    valid = [target.pose.position for target in targets if target.valid]
    if not valid:
        raise ValueError("cannot plot a preview with no valid targets")
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("--plot requires matplotlib") from exc
    figure = plt.figure(figsize=(8, 7))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(
        [position[0] for position in valid],
        [position[1] for position in valid],
        [position[2] for position in valid],
        linewidth=1.0,
    )
    axes.scatter(*valid[0], color="green", label="anchor")
    axes.scatter(*valid[-1], color="red", label="end")
    axes.set_xlabel("robot x / forward [m]")
    axes.set_ylabel("robot y / left [m]")
    axes.set_zlabel("robot z / up [m]")
    axes.set_title("Quest offline target preview (no robot output)")
    axes.legend()
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a Quest recording into robot-disconnected target previews."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--anchor", choices=("first", "clutch", "record"), default="first")
    parser.add_argument("--frame-id", default="right_teleop_preview")
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--target-position", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--target-quaternion", type=float, nargs=4, default=(0.0, 0.0, 0.0, 1.0))
    parser.add_argument("--tool-position", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--tool-quaternion", type=float, nargs=4, default=(0.0, 0.0, 0.0, 1.0))
    args = parser.parse_args(argv)
    if not math.isfinite(args.position_scale) or args.position_scale <= 0.0:
        parser.error("--position-scale must be positive and finite")
    return args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    source_hash = sha256_file(args.recording)
    recording = load_recording(args.recording)
    target_anchor = RigidPose(args.target_position, args.target_quaternion)
    controller_to_tool = RigidPose(args.tool_position, args.tool_quaternion)
    generator = TargetPreviewGenerator(
        target_anchor,
        side=args.side,
        anchor=args.anchor,
        frame_id=args.frame_id,
        position_scale=args.position_scale,
        controller_to_tool=controller_to_tool,
    )
    result = generate_targets(recording, generator)
    output = write_preview(
        args.output,
        source_path=args.recording,
        source_sha256=source_hash,
        result=result,
        metadata={
            "source_schema_version": recording.metadata.get("schema_version"),
            "side": args.side,
            "anchor_mode": args.anchor,
            "frame_id": args.frame_id,
            "position_scale": args.position_scale,
            "unity_to_robot_basis": UNITY_TO_ROBOT_BASIS,
            "target_anchor_position": list(target_anchor.position),
            "target_anchor_quaternion_xyzw": list(target_anchor.quaternion_xyzw),
            "controller_to_tool_position": list(controller_to_tool.position),
            "controller_to_tool_quaternion_xyzw": list(controller_to_tool.quaternion_xyzw),
        },
    )
    if args.plot is not None:
        _plot_targets(args.plot, result.targets)
    print(
        f"Saved {result.valid_targets} valid and {result.held_targets} held targets to {output}; "
        f"anchors={list(result.anchor_sequences)}; robot output disabled",
        flush=True,
    )


if __name__ == "__main__":
    main()
