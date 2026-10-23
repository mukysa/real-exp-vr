from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from data_collection.meta_quest.openvr_bridge import (
    _controller_controls,
    _rotation_matrix_to_quaternion,
    openvr_matrix_to_unity_pose,
    parse_args,
)


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


@pytest.mark.parametrize('mask', [0, 1 << 7, 1 << 3, 1 << 2])
def test_single_left_other_buttons_do_not_engage(mask: int) -> None:
    state = SimpleNamespace(ulButtonPressed=mask, rAxis=[])
    system = SimpleNamespace(getControllerState=lambda _: (True, state))
    assert _controller_controls(system, 1, single_left=True)[1] is False


def test_openvr_cli_validates_rates() -> None:
    assert parse_args(["--poll-hz", "60"]).poll_hz == 60.0
    with pytest.raises(SystemExit):
        parse_args(["--poll-hz", "0"])
