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


def test_fake_launch_has_no_real_robot_address_or_gello_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "right_fake.launch.py"
    source = launch_path.read_text()

    ast.parse(source)
    assert '"use_fake_hardware": "true"' in source
    assert '"robot_ip": "fake-hardware.invalid"' in source
    assert 'default_value="false"' in source
    assert '"enable_fault_injection"' in source
    assert "172.16." not in source
    assert "franka_gello_state_publisher" not in source


def test_left_fake_launch_is_left_only_and_parameterizes_topics() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "left_fake.launch.py"
    source = launch_path.read_text()
    ast.parse(source)
    assert '"namespace": "left"' in source
    assert '"arm_prefix": "left"' in source
    assert '"robot_ip": "fake-hardware.invalid"' in source
    assert '"measured_topic": "/left/joint_states"' in source
    assert '"raw_topic": "/left/vr/raw_joint_states"' in source
    assert '"accepted_topic": "/left/vr/accepted_joint_states"' in source
    assert '"gello_topic": "/left/gello/raw_joint_states"' in source
    assert "/right/" not in source
