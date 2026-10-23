"""Quest JSONL -> state machine -> strict IK -> AdapterCore offline validation."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

from data_collection.fr3_ik.a3_replay import ArmReplay, load_inputs, check_source
from data_collection.fr3_ik.interfaces import JointVector, MeasuredJointState
from data_collection.meta_quest.adapter_core import AdapterCore


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def validate_ab_pipeline(
    recording_path: Path,
    snapshot_path: Path,
    output_path: Path,
    *,
    side: str = "left",
    mount_path: Path | None = None,
    position_scale: float = 1.0,
    pose_shaper_bandwidth: float = 25.0,
    motion_mapping: str = "tabletop-relative",
) -> dict:
    """Run the real A-line processors and B-line gate without ROS or publication."""
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    root = Path(__file__).resolve().parents[2]
    recording_path, output_path, mount_path, source_hash, recording, snapshot_bytes = load_inputs(
        recording_path, snapshot_path, output_path, mount_path)
    arm = ArmReplay(
        recording, snapshot_bytes, source_hash, side, mount_path, position_scale,
        None, pose_shaper_bandwidth, "strict", motion_mapping,
    )
    gate = AdapterCore(side=side, expected_sequence_step=None)
    measured_q = JointVector(tuple(arm.q0))
    frames = []
    for received in arm.poses:
        a_row = arm.step(received)
        result = arm.last_result
        measured = MeasuredJointState(
            measured_q,
            result.request.target_stamp,
            f"offline_initial_snapshot:{snapshot_path.name}",
            True,
        )
        decision = gate.assess(result, measured, result.computed_stamp)
        frames.append({
            "quest_timestamp": result.request.source_stamp.seconds,
            "sequence": result.request.target.source_sequence,
            "connection_id": result.request.target.source_connection_id,
            "tracking_state": "valid" if received.sample.tracked else "lost",
            "target_pose": {
                "frame_id": result.request.target.frame_id,
                "position": list(result.request.target.pose.position),
                "quaternion_xyzw": list(result.request.target.pose.quaternion_xyzw),
            },
            "measured_q": list(measured_q.positions_rad),
            "measured_source": measured.source,
            "ik_state": result.state.value,
            "ik_success": result.success,
            "ik_reason": result.reason.value,
            "position_residual": result.position_residual_m,
            "rotation_residual": result.rotation_residual_rad,
            "gate_state": decision.state.value,
            "gate_accepted": decision.would_publish,
            "would_publish": decision.would_publish,
            "reject_reason": decision.reject_reason,
            "joint_target": (None if decision.joint_target is None else
                             list(decision.joint_target.joints.positions_rad)),
            "solve_time_s": result.solve_time_s,
            "pipeline_time_s": a_row["pipeline_time_s"],
        })
    check_source(recording_path, source_hash)
    accepted = [frame for frame in frames if frame["would_publish"]]
    reject_counts = Counter(frame["reject_reason"] for frame in frames if frame["reject_reason"])
    state_counts = Counter(frame["ik_state"] for frame in frames)
    summary = {
        "schema": "fr3_ab_offline_summary_v1",
        "side": side,
        "total_frames": len(frames),
        "ik_state_counts": dict(state_counts),
        "ik_success_rate": sum(frame["ik_success"] for frame in frames) / len(frames) if frames else 0.0,
        "gate_accept_count": len(accepted),
        "gate_accept_rate": len(accepted) / len(frames) if frames else 0.0,
        "reject_reason_counts": dict(reject_counts),
        "max_position_residual_m": max((frame["position_residual"] or 0.0 for frame in frames), default=0.0),
        "max_rotation_residual_rad": max((frame["rotation_residual"] or 0.0 for frame in frames), default=0.0),
        "invalid_result_accepted": sum(frame["would_publish"] and not frame["ik_success"] for frame in frames),
        "measured_feedback_mode": "initial_snapshot_only",
        "measured_freshness_proven": False,
        "hardware_command_published": False,
        "go_no_go": "NO_GO_NEEDS_DEDICATED_SMALL_MOTION_CAPTURE",
    }
    manifest = {
        "schema": "fr3_ab_offline_manifest_v1",
        "git_commit": _git_head(root),
        "python": sys.version,
        "platform": platform.platform(),
        "recording": str(recording_path),
        "recording_sha256": source_hash,
        "initial_snapshot": str(snapshot_path),
        "initial_snapshot_sha256": _sha256(snapshot_path),
        "mount_calibration": str(mount_path),
        "mount_calibration_sha256": _sha256(mount_path),
        "parameters": {
            "side": side,
            "position_scale": position_scale,
            "pose_shaper_bandwidth": pose_shaper_bandwidth,
            "motion_mapping": motion_mapping,
            "ik_tracking_mode": "strict",
        },
    }
    output_path.mkdir(parents=True)
    (output_path / "frames.jsonl").write_text(
        "".join(json.dumps(frame, allow_nan=False, sort_keys=True) + "\n" for frame in frames),
        encoding="utf-8",
    )
    (output_path / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False, sort_keys=True) + "\n", encoding="utf-8")
    (output_path / "manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False, sort_keys=True) + "\n", encoding="utf-8")
    (output_path / "README.md").write_text(
        "# A/B offline validation\n\n"
        "This directory was generated without ROS, fake hardware, or robot command publication.\n\n"
        "The measured joint vector is the initial read-only snapshot only; this run cannot prove "
        "live feedback freshness. `summary.json` remains NO-GO until the dedicated small-motion "
        "Quest capture and later read-only hardware checks are supplied.\n",
        encoding="utf-8",
    )
    return {"summary": summary, "manifest": manifest}
