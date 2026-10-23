#!/usr/bin/env python3
"""Run Step 6 GELLO/Quest regression checks without starting ROS or hardware."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path


def _contains(path: Path, *fragments: str) -> bool:
    text = path.read_text(encoding="utf-8")
    return all(fragment in text for fragment in fragments)


def run(repository_root: Path) -> dict[str, object]:
    gello_config = repository_root / (
        "gello_software/ros2/src/franka_fr3_arm_controllers/config/controllers.yaml"
    )
    gello_defaults = repository_root / (
        "gello_software/ros2/src/franka_fr3_arm_controllers/include/"
        "franka_fr3_arm_controllers/joint_impedance_controller.hpp"
    )
    gello_publisher = repository_root / (
        "gello_software/ros2/src/franka_gello_state_publisher/"
        "franka_gello_state_publisher/gello_publisher.py"
    )
    gello_launcher = repository_root / "scripts/start_teleoperation.sh"
    quest_launcher = repository_root / "scripts/start_quest3_teleoperation_client.sh"
    quest_override = repository_root / (
        "quest3_ros2/src/franka_vr_adapter/config/vr_controller.yaml"
    )
    quest_launch = repository_root / (
        "quest3_ros2/src/franka_vr_adapter/launch/right_fake.launch.py"
    )

    paths = (
        gello_config,
        gello_defaults,
        gello_publisher,
        gello_launcher,
        quest_launcher,
        quest_override,
        quest_launch,
    )
    checks = {
        "required_files_present": all(path.is_file() for path in paths),
        "gello_config_topics_unchanged": _contains(
            gello_config,
            "command_topic: gello/raw_joint_states",
            "accepted_target_topic: gello/accepted_joint_states",
            "activation_mode: always",
            "command_timeout_sec: 0.5",
        ),
        "gello_compiled_defaults_unchanged": _contains(
            gello_defaults,
            'return "gello/raw_joint_states";',
            'return "gello/accepted_joint_states";',
            'return "always";',
        ),
        "gello_publisher_topic_unchanged": _contains(
            gello_publisher, 'JointState, "gello/raw_joint_states", 10'
        ),
        "quest_topics_are_separate": _contains(
            quest_override,
            "command_topic: vr/raw_joint_states",
            "accepted_target_topic: vr/accepted_joint_states",
            "command_timeout_sec: 0.5",
        ),
        "shared_exclusive_lock": _contains(
            gello_launcher,
            'launcher_lock_path="${runtime_dir}/real-exp-teleoperation-${UID}.lock"',
        )
        and _contains(
            quest_launcher,
            'launcher_lock_path="${runtime_dir}/real-exp-teleoperation-${UID}.lock"',
        ),
        "quest_checks_live_gello_publisher": _contains(
            quest_launcher,
            "/right/gello/raw_joint_states",
            "GELLO is already publishing",
        ),
        "quest_launch_is_fake_only": _contains(
            quest_launch,
            '"use_fake_hardware": "true"',
            '"robot_ip": "fake-hardware.invalid"',
            '"load_gripper": "false"',
        ),
        "quest_and_gello_builds_are_separate": _contains(
            quest_launcher,
            'quest_install="${QUEST3_ROS_INSTALL_PREFIX:',
            'controller_install="${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX:',
        ),
    }

    git_result = subprocess.run(
        [
            "git",
            "-C",
            str(repository_root),
            "status",
            "--porcelain",
            "--",
            "gello_software/ros2",
            "scripts/start_teleoperation.sh",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    checks["gello_tracked_files_unmodified"] = not git_result.stdout.strip()
    return {
        "generated_at_unix_s": time.time(),
        "mode": "offline_no_hardware",
        "all_passed": all(checks.values()),
        "checks": checks,
        "gello_git_status": git_result.stdout.splitlines(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = run(args.repository_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    raise SystemExit(0 if payload["all_passed"] else 1)


if __name__ == "__main__":
    main()
