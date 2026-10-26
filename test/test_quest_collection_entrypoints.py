"""Offline routing checks: copied launchers cannot reach real hardware."""
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("action", ["--check-only", "--enable-hardware"])
def test_quest_client_routes_without_gello_build(tmp_path, action):
    shutil.copy(ROOT / "scripts/start_data_collection_client.sh", tmp_path)
    (tmp_path / "start_quest3_hardware.sh").write_text('printf "%s\\n" "$@"\n')
    result = subprocess.run(
        ["bash", str(tmp_path / "start_data_collection_client.sh"),
         "--teleop-source", "quest", "--left", "--arm", action],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["--left", action]


@pytest.mark.parametrize("launcher", ["client", "server"])
@pytest.mark.parametrize("modes", [["--right", "--arm"], ["--duo", "--arm"],
                                   ["--left", "--gripper"], ["--left", "--hand"]])
def test_unsupported_quest_modes_fail_before_environment_setup(launcher, modes):
    result = subprocess.run(
        ["bash", str(ROOT / f"scripts/start_data_collection_{launcher}.sh"),
         "--teleop-source", "quest", *modes], capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "only --left --arm" in result.stderr


def test_quest_requires_explicit_hardware_action():
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/start_data_collection_client.sh"),
         "--teleop-source", "quest", "--left", "--arm"], capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "explicit --enable-hardware" in result.stderr


def test_quest_left_bridge_contract():
    params = yaml.safe_load((ROOT / "config/data_collection/quest_left.yaml").read_text())["lerobot_data_bridge"]["ros__parameters"]
    assert params["arm_mode"] == "left"
    assert params["teleop_source"] == "quest"
    assert params["left_arm_action_topic"] == "/left/vr/accepted_joint_states"
    assert not params["include_right_arm"]
    assert not params["include_gripper"]
    assert not params["include_hand"]
    assert params["camera_1_enabled"] and params["camera_2_enabled"]
    assert not params["camera_3_enabled"]


def test_quest_source_is_persisted_without_changing_legacy_gello_metadata():
    from data_collection.lerobot_collection import action_config_from_packet

    packet = {
        "arm_mode": "left", "include_right_arm": False,
        "include_gripper": False, "include_hand": False,
        "state_action_mode": "joint", "robot_state_dim": 7, "action_dim": 7,
        "arm_action_representation": "delta_joint_position",
    }
    legacy = action_config_from_packet(packet)
    assert "teleop_source" not in legacy
    assert action_config_from_packet(packet | {"teleop_source": "gello"}) == legacy
    quest = action_config_from_packet(packet | {"teleop_source": "quest"})
    assert quest["teleop_source"] == "quest"
    assert quest != legacy
