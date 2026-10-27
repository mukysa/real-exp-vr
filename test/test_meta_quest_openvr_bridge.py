from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

import pytest

from data_collection.meta_quest.openvr_bridge import (
    _AsyncKeepAwake,
    _controller_controls,
    _physical_controller_id,
    _rotation_matrix_to_quaternion,
    openvr_matrix_to_unity_pose,
    parse_args,
)


def test_slow_adb_keep_awake_does_not_block_pose_polling() -> None:
    entered = threading.Event()
    release = threading.Event()
    attempts: list[bool] = []

    def slow_update(active: bool) -> bool:
        attempts.append(active)
        entered.set()
        release.wait(timeout=2.0)
        return False

    power = SimpleNamespace(active=False, update=slow_update, close=lambda: None)
    keep_awake = _AsyncKeepAwake(power)
    try:
        start = time.monotonic()
        keep_awake.update()
        assert entered.wait(timeout=1.0)
        for _ in range(30):
            keep_awake.update()
        assert time.monotonic() - start < 0.5
        assert attempts == [True]
    finally:
        release.set()
        keep_awake.close()


def test_openvr_identity_pose_flips_forward_axis_for_unity_contract() -> None:
    pose = openvr_matrix_to_unity_pose(
        (
            (1.0, 0.0, 0.0, 0.2),
            (0.0, 1.0, 0.0, 0.7),
            (0.0, 0.0, 1.0, -0.4),
        )
    )

    assert pose.position == pytest.approx((0.2, 0.7, 0.4))
    assert pose.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_openvr_rotation_conversion_accepts_tracking_float_noise() -> None:
    angle = math.radians(35.0)
    noisy = (
        (math.cos(angle) + 2e-7, -math.sin(angle), 0.0),
        (math.sin(angle), math.cos(angle) - 2e-7, 0.0),
        (0.0, 0.0, 1.0 + 1e-7),
    )

    quaternion = _rotation_matrix_to_quaternion(noisy)

    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_controller_controls_maps_trigger_and_stage2_buttons() -> None:
    state = SimpleNamespace(
        ulButtonPressed=(1 << 7) | (1 << 1),
        rAxis=[SimpleNamespace(x=0.0), SimpleNamespace(x=0.73)],
    )
    system = SimpleNamespace(getControllerState=lambda _: (True, state))

    trigger, clutch, record = _controller_controls(system, 2)
    assert trigger == pytest.approx(0.73)
    assert clutch is True
    assert record is True


def test_single_left_maps_y_and_dual_mode_keeps_b_button_bit() -> None:
    state = SimpleNamespace(
        # SteamVR Touch legacy: left Y shares the B-button bit (id 1).
        ulButtonPressed=(1 << 1),
        rAxis=[SimpleNamespace(x=0.0), SimpleNamespace(x=0.0)],
    )
    system = SimpleNamespace(getControllerState=lambda _: (True, state))
    assert _controller_controls(system, 1, single_left=True)[1] is True
    assert _controller_controls(system, 1, single_left=False)[1] is True
    assert parse_args(["--single-left"]).side == "left"


def test_physical_left_controller_wins_over_virtual_hand_role() -> None:
    openvr = SimpleNamespace(
        TrackedControllerRole_LeftHand=1,
        TrackedControllerRole_RightHand=2,
        TrackedDeviceClass_Controller=2,
        Prop_SerialNumber_String=100,
        k_unTrackedDeviceIndexInvalid=4294967295,
        k_unMaxTrackedDeviceCount=5,
    )
    serials = {
        1: "1WMHH000X00000_Controller_Left",
        3: "VRLINKQ_Hand_Left",
    }
    system = SimpleNamespace(
        getTrackedDeviceIndexForControllerRole=lambda role: 3,
        getTrackedDeviceClass=lambda device_id: 2 if device_id in serials else 0,
        getStringTrackedDeviceProperty=lambda device_id, prop: serials[device_id],
    )

    assert _physical_controller_id(system, openvr, "left") == 1

    del serials[1]
    assert _physical_controller_id(system, openvr, "left") == openvr.k_unTrackedDeviceIndexInvalid


@pytest.mark.parametrize('mask', [0, 1 << 7, 1 << 3, 1 << 2])
def test_single_left_other_buttons_do_not_engage(mask: int) -> None:
    state = SimpleNamespace(ulButtonPressed=mask, rAxis=[])
    system = SimpleNamespace(getControllerState=lambda _: (True, state))
    assert _controller_controls(system, 1, single_left=True)[1] is False


def test_openvr_cli_validates_rates() -> None:
    assert parse_args(["--poll-hz", "60"]).poll_hz == 60.0
    assert parse_args(["--check-left-touch"]).check_left_touch is True
    both = parse_args(["--global-left-y", "--check-both-touch"])
    assert both.side == "both"
    assert both.global_left_y is True
    assert both.check_both_touch is True
    assert parse_args(["--check-both-touch", "--touch-wait-s", "300"]).touch_wait_s == 300.0
    with pytest.raises(SystemExit):
        parse_args(["--poll-hz", "0"])
    with pytest.raises(SystemExit):
        parse_args(["--touch-wait-s", "-1"])
