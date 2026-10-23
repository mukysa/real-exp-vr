from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_vr_controller_override_changes_only_transport_parameters() -> None:
    config = yaml.safe_load((PACKAGE_ROOT / "config" / "vr_controller.yaml").read_text())
    parameters = config["/**"]["joint_impedance_controller"]["ros__parameters"]

    assert parameters == {
        "command_topic": "vr/raw_joint_states",
        "accepted_target_topic": "vr/accepted_joint_states",
        "activation_mode": "always",
        "command_timeout_sec": 0.5,
    }


def test_adapter_launch_has_no_robot_controller_or_gello_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "adapter.launch.py"
    source = launch_path.read_text()

    ast.parse(source)
    assert 'executable="adapter_node"' in source
    assert "controller_manager" not in source
    assert "fake_hardware" not in source
    assert "synthetic" not in source
    assert "172.16." not in source
    assert "franka_gello_state_publisher" not in source


def test_left_live_launch_allows_inter_episode_sequence_gaps() -> None:
    source = (PACKAGE_ROOT / "launch" / "left_live_control.launch.py").read_text()

    ast.parse(source)
    assert '"expected_sequence_step": 0' in source
