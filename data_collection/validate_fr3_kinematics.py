"""Offline FR3/Franka Hand model audit; never creates a robot or ROS node."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import time

import numpy as np
import pinocchio as pin
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from data_collection.move_to_target_ee import (
    ARM_POSITION_LOWER_RAD,
    ARM_POSITION_UPPER_RAD,
    build_fr3_model,
    forward_end_effector_pose,
    pose_error,
    solve_fr3_ik,
)


ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = ROOT / "gello_software/ros2/src/franka_fr3_arm_controllers"


def frame_pose(model, q, name):
    frame = model.getFrameId(name)
    if frame >= len(model.frames):
        raise ValueError(f"Missing model frame: {name}")
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return np.array(data.oMf[frame].homogeneous)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def audit(q):
    config_path = CONTROLLERS / "config/example_fr3_right_config.yaml"
    config = yaml.safe_load(config_path.read_text())["RIGHT"]
    require(config["arm_id"] == "fr3" and config["arm_prefix"] == "right"
            and config["namespace"] == "right"
            and str(config["load_gripper"]).lower() == "true", "Unexpected right arm configuration")
    q = np.asarray(q, dtype=float)
    require(q.shape == (7,) and np.all(np.isfinite(q)), "q must contain seven finite radians")
    require(np.all(q >= ARM_POSITION_LOWER_RAD) and np.all(q <= ARM_POSITION_UPPER_RAD),
            "q is outside controller operational limits")

    header = (CONTROLLERS / "include/franka_fr3_arm_controllers/joint_reference_generator.hpp").read_text()
    limits = {}
    for key in ("kPositionLower", "kPositionUpper", "kMaxVelocity", "kMaxAcceleration"):
        match = re.search(rf"{key}\s*=\s*\{{([^}}]+)\}}", header)
        require(match is not None, f"Cannot read controller limit {key}")
        limits[key] = [float(value.strip()) for value in match.group(1).split(",")]
    require(np.array_equal(limits["kPositionLower"], ARM_POSITION_LOWER_RAD)
            and np.array_equal(limits["kPositionUpper"], ARM_POSITION_UPPER_RAD),
            "Existing IK limits differ from controller limits")

    description = Path(get_package_share_directory("franka_description"))
    xacro_path = description / "robots" / config["urdf_file"]
    xml = xacro.process_file(str(xacro_path), mappings={
        "hand": "true", "arm_prefix": "right", "ros2_control": "false",
        "robot_ip": "", "use_fake_hardware": "false", "fake_sensor_commands": "false",
    }).toxml()
    full = pin.buildModelFromXML(xml)
    arm, flange_frame = build_fr3_model()
    require(arm.nq == 7 and list(arm.names)[1:] == [f"fr3_joint{i}" for i in range(1, 8)],
            "Unexpected arm model joint order")
    require(list(full.names)[1:8] == [f"right_fr3_joint{i}" for i in range(1, 8)],
            "Unexpected prefixed arm joint order")
    full_q = pin.neutral(full)  # Fixed zero finger displacement for this offline audit.
    full_q[:7] = q
    flange = frame_pose(full, full_q, "right_fr3_link8")
    tcp = frame_pose(full, full_q, "right_fr3_hand_tcp")
    flange_to_tcp = np.linalg.inv(flange) @ tcp
    nominal_tcp = forward_end_effector_pose(arm, flange_frame, q, flange_to_tcp)
    require(np.allclose(nominal_tcp, tcp, atol=1e-10, rtol=0),
            "Arm FK plus tool transform differs from full hand model")

    # Perturb the seed: solving at the original exact q alone would be trivial.
    seed = q + np.array([0.02, -0.015, 0.01, 0.015, -0.01, 0.02, -0.015])
    seed = np.clip(seed, ARM_POSITION_LOWER_RAD + 1e-6, ARM_POSITION_UPPER_RAD - 1e-6)
    started = time.perf_counter()
    result = solve_fr3_ik(seed, nominal_tcp, flange_to_tcp, arm, flange_frame,
                          try_alternative_seeds=False, max_function_evaluations=200)
    elapsed = time.perf_counter() - started
    recovered = forward_end_effector_pose(arm, flange_frame, result.q, flange_to_tcp)
    position_error, orientation_error = pose_error(recovered, nominal_tcp)
    require(position_error <= 1e-5 and orientation_error <= 1e-5,
            "FK-IK-FK residual exceeds offline self-consistency tolerance")
    # Also check the independent full model at the solved configuration.
    full_q[:7] = result.q
    require(np.allclose(frame_pose(full, full_q, "right_fr3_hand_tcp"), recovered,
                        atol=1e-10, rtol=0), "Solved full-model FK differs")
    return {
        "passed": True, "robot_connected": False, "robot_output_enabled": False,
        "actual_F_T_EE_verified": False,
        "tool_transform_source": "nominal URDF, not measured robot F_T_EE",
        "robot_config": str(config_path), "description_path": str(description),
        "expanded_hand_urdf_sha256": hashlib.sha256(xml.encode()).hexdigest(),
        "base_frame": "right_fr3_link0", "flange_frame": "right_fr3_link8",
        "target_frame": "right_fr3_hand_tcp",
        "command_joint_names": [f"fr3_joint{i}" for i in range(1, 8)],
        "hardware_joint_names": [f"right_fr3_joint{i}" for i in range(1, 8)],
        "controller_limits": limits, "q0_rad": q.tolist(),
        "flange_to_nominal_tcp": flange_to_tcp.tolist(), "T0": nominal_tcp.tolist(),
        "ik_seed_rad": seed.tolist(), "ik_result_rad": result.q.tolist(),
        "position_error_m": position_error, "orientation_error_rad": orientation_error,
        "max_joint_difference_from_q0_rad": float(np.max(np.abs(result.q - q))),
        "ik_elapsed_s": elapsed,
        "timing_note": "Single solve including lazy imports; not a steady-state benchmark",
        "function_evaluations": result.function_evaluations,
        "scope": "Model consistency only; not continuous IK, collision or hardware validation",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q", nargs=7, type=float,
                        default=[0, -0.4, 0, -1.8, 0, 1.5, 0.7])
    parser.add_argument("--output", type=Path, help="New JSON report; refuses overwrite")
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        parser.error(f"Refusing to overwrite {args.output}")
    report = audit(args.q)
    payload = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output is not None:
        with args.output.open("x") as destination:
            destination.write(payload)
    print(payload, end="")


if __name__ == "__main__":
    main()
