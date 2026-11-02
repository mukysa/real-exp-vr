"""Offline routing checks: copied launchers cannot reach real hardware."""
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("action", ["--check-only", "--enable-hardware"])
@pytest.mark.parametrize("data_mode", ["--arm", "--gripper", "--hand-arm"])
def test_quest_client_routes_without_gello_build(tmp_path, action, data_mode):
    shutil.copy(ROOT / "scripts/start_data_collection_client.sh", tmp_path)
    config_dir = tmp_path.parent / "config" / "data_collection"
    config_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(ROOT / "config/data_collection/cyclonedds-client.xml", config_dir)
    (tmp_path / "start_quest3_hardware.sh").write_text('printf "%s\\n" "$@"\n')
    result = subprocess.run(
        ["bash", str(tmp_path / "start_data_collection_client.sh"),
         "--teleop-source", "quest", "--left", data_mode, action],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "Quest collection DDS: wired client to server" in result.stdout
    expected = ["--left"] + ([data_mode] if data_mode in {"--gripper", "--hand-arm"} else []) + [action]
    assert result.stdout.splitlines()[-len(expected):] == expected


@pytest.mark.parametrize("launcher", ["client", "server"])
@pytest.mark.parametrize("modes", [
    ["--teleop-source", "quest", "--left", "--gripper"],
    ["--teleop-source", "quest", "--left", "--hand"],
    ["--teleop-source", "quest", "--right"],
    ["--teleop-source", "quest", "--duo"],
    ["--left"],
])
def test_hand_arm_rejects_incompatible_modes_before_setup(launcher, modes):
    result = subprocess.run(
        ["bash", str(ROOT / f"scripts/start_data_collection_{launcher}.sh"),
         "--hand-arm", *modes], capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "--hand-arm requires" in result.stderr


def test_hand_arm_automatic_recorder_preserves_keyboard_stdin():
    source = (ROOT / "scripts/start_data_collection_server.sh").read_text()
    function = source[source.index("start_process() {"):source.index("signal_running_groups() {")]
    result = subprocess.run(
        ["bash", "-c", 'set -eu\nhand_arm=true\ndeclare -a child_pids=()\n'
         'declare -A child_names=()\n' + function +
         '\nstart_process "LeRobot recorder" bash -c \'read -r command; test "$command" = q\'\n'
         'wait "${child_pids[0]}"\n'],
        input="q\n", capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("launcher", ["client", "server"])
@pytest.mark.parametrize("modes", [["--right", "--arm"], ["--duo", "--arm"],
                                   ["--right", "--gripper"], ["--left", "--hand"]])
def test_unsupported_quest_modes_fail_before_environment_setup(launcher, modes):
    result = subprocess.run(
        ["bash", str(ROOT / f"scripts/start_data_collection_{launcher}.sh"),
         "--teleop-source", "quest", *modes], capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "only --left --arm or --left --gripper" in result.stderr


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
    assert params["left_gripper_action_topic"] == "/left/vr/gripper_open_fraction"
    assert not params["include_hand"]
    assert params["camera_1_enabled"]
    assert params["camera_1_name"] == "cam_left"
    assert not params["camera_2_enabled"]
    assert not params["camera_3_enabled"]

    camera = yaml.safe_load((ROOT / "config/data_collection/quest_left_camera.yaml").read_text())["realsense_camera_publisher"]["ros__parameters"]
    assert camera["camera_1_enabled"] and camera["camera_1_name"] == "cam_left"
    assert camera["camera_1_serial"] == "260322270745"
    assert not camera["camera_2_enabled"] and not camera["camera_3_enabled"]


@pytest.mark.parametrize("trigger,opening", [(0.0, 1.0), (0.25, 0.75), (1.0, 0.0), (-1.0, 1.0), (2.0, 0.0)])
def test_left_trigger_maps_continuously_to_gripper_opening(trigger, opening):
    from data_collection.meta_quest.gripper_mapping import trigger_to_open_fraction

    assert trigger_to_open_fraction(trigger) == opening


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
