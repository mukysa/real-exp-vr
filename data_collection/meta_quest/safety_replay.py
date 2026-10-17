"""Replay Quest captures through the offline teleoperation safety state machine."""

from __future__ import annotations

import argparse
from collections import Counter
import math
from pathlib import Path

from .calibration import RigidPose
from .coordinates import IDENTITY_POSE, UNITY_TO_ROBOT_BASIS
from .replay import PoseRecording, load_recording
from .safety import SafetyLimits
from .state_machine import TeleoperationStateMachine
from .target_preview import PreviewResult, sha256_file, write_preview


def replay_safety(
    recording: PoseRecording,
    machine: TeleoperationStateMachine,
) -> PreviewResult:
    targets = []
    ignored = 0
    for received in recording.poses:
        target = machine.update(received)
        if target is None:
            ignored += 1
        else:
            targets.append(target)
    return PreviewResult(
        targets=tuple(targets),
        selected_pose_records=len(targets),
        ignored_pose_records=ignored,
        anchor_sequences=tuple(machine.anchor_sequences),
    )


def _plot_states(path: Path, result: PreviewResult) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("--plot requires matplotlib") from exc
    colors = {
        "WAIT_INPUT": "gray",
        "READY": "gold",
        "FOLLOWING": "tab:blue",
        "MANUAL_HOLD": "tab:orange",
        "FAULT_HOLD": "tab:red",
    }
    figure = plt.figure(figsize=(8, 7))
    axes = figure.add_subplot(111, projection="3d")
    following = [target.pose.position for target in result.targets if target.valid]
    if following:
        axes.plot(
            [position[0] for position in following],
            [position[1] for position in following],
            [position[2] for position in following],
            color=colors["FOLLOWING"],
            linewidth=1.0,
            label="FOLLOWING",
        )
    for state, color in colors.items():
        held = [
            target.pose.position
            for target in result.targets
            if not target.valid and target.state == state
        ]
        if held:
            axes.scatter(
                [position[0] for position in held],
                [position[1] for position in held],
                [position[2] for position in held],
                color=color,
                s=10,
                label=state,
            )
    axes.set_xlabel("robot x / forward [m]")
    axes.set_ylabel("robot y / left [m]")
    axes.set_zlabel("robot z / up [m]")
    axes.set_title("Quest safety state replay (no robot output)")
    axes.legend()
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    defaults = SafetyLimits()
    parser = argparse.ArgumentParser(
        description="Replay Quest input through the robot-disconnected safety state machine."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--engage-button", choices=("clutch", "record"), default="clutch")
    parser.add_argument("--frame-id", default="right_teleop_preview")
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--target-position", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--target-quaternion", type=float, nargs=4, default=(0.0, 0.0, 0.0, 1.0))
    parser.add_argument("--tool-position", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--tool-quaternion", type=float, nargs=4, default=(0.0, 0.0, 0.0, 1.0))
    parser.add_argument("--min-ready-samples", type=int, default=3)
    parser.add_argument("--button-debounce-s", type=float, default=0.15)
    parser.add_argument("--stale-timeout-s", type=float, default=defaults.stale_timeout_s)
    parser.add_argument("--max-source-step-m", type=float, default=defaults.max_source_translation_step_m)
    parser.add_argument("--max-source-rotation-rad", type=float, default=defaults.max_source_rotation_step_rad)
    parser.add_argument("--max-target-speed-m-s", type=float, default=defaults.max_target_linear_speed_m_s)
    parser.add_argument("--max-target-angular-speed-rad-s", type=float, default=defaults.max_target_angular_speed_rad_s)
    parser.add_argument("--workspace-radius-m", type=float, default=defaults.workspace_radius_m)
    parser.add_argument("--workspace-rotation-rad", type=float, default=defaults.workspace_rotation_rad)
    args = parser.parse_args(argv)
    if args.min_ready_samples < 1:
        parser.error("--min-ready-samples must be positive")
    if not math.isfinite(args.button_debounce_s) or args.button_debounce_s < 0.0:
        parser.error("--button-debounce-s must be non-negative and finite")
    return args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    source_hash = sha256_file(args.recording)
    recording = load_recording(args.recording)
    limits = SafetyLimits(
        stale_timeout_s=args.stale_timeout_s,
        max_source_translation_step_m=args.max_source_step_m,
        max_source_rotation_step_rad=args.max_source_rotation_rad,
        max_target_linear_speed_m_s=args.max_target_speed_m_s,
        max_target_angular_speed_rad_s=args.max_target_angular_speed_rad_s,
        workspace_radius_m=args.workspace_radius_m,
        workspace_rotation_rad=args.workspace_rotation_rad,
    )
    target_anchor = RigidPose(args.target_position, args.target_quaternion)
    controller_to_tool = RigidPose(args.tool_position, args.tool_quaternion)
    machine = TeleoperationStateMachine(
        target_anchor,
        side=args.side,
        engage_button=args.engage_button,
        frame_id=args.frame_id,
        position_scale=args.position_scale,
        controller_to_tool=controller_to_tool,
        limits=limits,
        min_ready_samples=args.min_ready_samples,
        button_debounce_s=args.button_debounce_s,
    )
    result = replay_safety(recording, machine)
    state_counts = Counter(target.state for target in result.targets)
    hold_reason_counts = Counter(
        target.hold_reason for target in result.targets if target.hold_reason is not None
    )
    output = write_preview(
        args.output,
        source_path=args.recording,
        source_sha256=source_hash,
        result=result,
        metadata={
            "preview_type": "teleoperation_safety_state_machine",
            "source_schema_version": recording.metadata.get("schema_version"),
            "side": args.side,
            "engage_button": args.engage_button,
            "frame_id": args.frame_id,
            "position_scale": args.position_scale,
            "unity_to_robot_basis": UNITY_TO_ROBOT_BASIS,
            "target_anchor_position": list(target_anchor.position),
            "target_anchor_quaternion_xyzw": list(target_anchor.quaternion_xyzw),
            "controller_to_tool_position": list(controller_to_tool.position),
            "controller_to_tool_quaternion_xyzw": list(controller_to_tool.quaternion_xyzw),
            "safety_limits": {
                "stale_timeout_s": limits.stale_timeout_s,
                "max_source_translation_step_m": limits.max_source_translation_step_m,
                "max_source_rotation_step_rad": limits.max_source_rotation_step_rad,
                "max_target_linear_speed_m_s": limits.max_target_linear_speed_m_s,
                "max_target_angular_speed_rad_s": limits.max_target_angular_speed_rad_s,
                "workspace_radius_m": limits.workspace_radius_m,
                "workspace_rotation_rad": limits.workspace_rotation_rad,
            },
        },
        summary_metadata={
            "state_counts": dict(sorted(state_counts.items())),
            "hold_reason_counts": dict(sorted(hold_reason_counts.items())),
        },
    )
    if args.plot is not None:
        _plot_states(args.plot, result)
    print(
        f"Saved safety replay to {output}; states={dict(state_counts)}; "
        f"holds={dict(hold_reason_counts)}; anchors={list(result.anchor_sequences)}; "
        "robot output disabled",
        flush=True,
    )


if __name__ == "__main__":
    main()
