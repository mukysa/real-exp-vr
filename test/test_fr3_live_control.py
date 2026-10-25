from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.live_control import LeftLiveControl
from data_collection.fr3_ik.preview_mapping import converted_matrix
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose


Q = (-0.11657, 0.42515, -0.66825, -1.97659, 0.82153, 1.84735, -1.95020)


def sample(
    sequence: int,
    timestamp: float,
    *,
    y: bool = False,
    x: float = 0.0,
    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
) -> ReceivedPose:
    pose = PoseSample(
        "left",
        "controller",
        (x, 1.0, 0.0),
        quaternion_xyzw,
        True,
        clutch=y,
    )
    return ReceivedPose(pose, timestamp, sequence, 1)


def ready(core: LeftLiveControl) -> None:
    for sequence, timestamp in enumerate((1.00, 1.04, 1.08), 1):
        core.update_measured(Q, (0.0,) * 7, timestamp)
        core.process(sample(sequence, timestamp))
    assert core.state == "READY"


def test_y_captures_measured_anchor_and_second_y_returns() -> None:
    core = LeftLiveControl(Kinematics("left"), clock_id="test")
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    engaged = core.process(sample(4, 1.20, y=True))
    assert core.state == "FOLLOWING"
    assert engaged.result is not None and engaged.result.success
    assert engaged.result.command.joints.positions_rad == Q

    core.update_measured(Q, (0.0,) * 7, 1.24)
    following = core.process(sample(5, 1.24))
    assert following.result is not None and following.result.success
    assert core.solver.diagnostics["stationary_target_reused"] is True
    assert core.solver.diagnostics["evaluations"] == 0

    core.update_measured(Q, (0.0,) * 7, 1.40)
    returning = core.process(sample(6, 1.40, y=True))
    assert returning.result is not None and returning.result.success
    assert core.state == "READY"
    assert "return complete" in core.reason

    core.update_measured(Q, (0.0,) * 7, 1.44)
    core.process(sample(7, 1.44, y=False))
    core.update_measured(Q, (0.0,) * 7, 1.64)
    reengaged = core.process(sample(8, 1.64, y=True))
    assert core.state == "FOLLOWING"
    assert reengaged.result is not None and reengaged.result.success


def test_scaled_workspace_is_hard_capped_at_five_mm() -> None:
    core = LeftLiveControl(Kinematics("left"), clock_id="test")
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))
    core.update_measured(Q, (0.0,) * 7, 1.24)
    inside = core.process(sample(5, 1.24, x=0.10))
    assert inside.result is not None and inside.result.success
    core.update_measured(Q, (0.0,) * 7, 1.28)
    update = core.process(sample(6, 1.28, x=0.18))
    assert update.result is None
    assert core.state == "FAULT"
    assert "5 mm" in core.reason


def test_half_scale_workspace_is_hard_capped_at_three_hundred_mm() -> None:
    core = LeftLiveControl(
        Kinematics("left"),
        clock_id="test",
        position_scale=0.5,
        max_tcp_offset_m=0.30,
    )
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))
    # Model a continuous approach to the boundary without asking the IK solver
    # to track the intermediate points in this workspace-limit unit test.
    core.last_source = converted_matrix(sample(5, 1.24, x=0.50).sample)
    core.last_pose_stamp = 1.24
    core.update_measured(Q, (0.0,) * 7, 1.28)
    update = core.process(sample(6, 1.28, x=0.61))
    assert update.result is None
    assert core.state == "FAULT"
    assert "300 mm" in core.reason


def test_zero_workspace_cap_disables_radial_limit() -> None:
    core = LeftLiveControl(
        Kinematics("left"),
        clock_id="test",
        position_scale=1.0,
        max_tcp_offset_m=0.0,
    )
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))
    core.last_source = converted_matrix(sample(5, 1.24, x=0.50).sample)
    core.last_pose_stamp = 1.24
    core.update_measured(Q, (0.0,) * 7, 1.28)
    core.process(sample(6, 1.28, x=0.61))
    assert "TCP workspace exceeded" not in core.reason


def test_y_after_fault_reanchors_at_measured_state_and_returns() -> None:
    core = LeftLiveControl(Kinematics("left"), clock_id="test")
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))
    core.update_measured(Q, (0.0,) * 7, 1.24)
    core.process(sample(5, 1.24, y=False))
    core._fault("test recoverable stop")

    core.update_measured(Q, (0.0,) * 7, 1.28)
    core.process(sample(6, 1.28, y=False))
    core.update_measured(Q, (0.0,) * 7, 1.48)
    recovery = core.process(sample(7, 1.48, y=True))

    assert core.state == "RETURNING"
    assert recovery.result is not None and recovery.result.success
    assert recovery.result.command.joints.positions_rad == Q
    assert "returning from measured state" in core.reason


def test_newer_measured_callback_does_not_false_trip_stale_feedback() -> None:
    core = LeftLiveControl(Kinematics("left"), clock_id="test")
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))

    # ROS may dispatch a joint-state callback received just after the queued
    # controller pose before it dispatches that pose callback.
    core.update_measured(Q, (0.0,) * 7, 1.245)
    update = core.process(sample(5, 1.24, x=0.01))

    assert update.result is not None and update.result.success
    assert core.state == "FOLLOWING"


def test_relaxed_live_profile_uses_hardware_controller_as_motion_limiter() -> None:
    core = LeftLiveControl(
        Kinematics("left"),
        clock_id="test",
        input_timeout_s=2.0,
        relaxed_motion_gates=True,
        max_source_translation_step_m=2.0,
        max_source_rotation_step_rad=3.141592653589793,
    )

    assert core.limits.max_step_rad == 6.0
    assert core.limits.input_gap_s == 2.0
    assert min(core.limits.velocity) == 1000.0
    assert min(core.limits.acceleration) == 100000.0
    assert core.max_source_translation_step_m == 2.0


def test_tabletop_translation_is_rotated_into_tilted_left_base() -> None:
    world_from_base = np.eye(4)
    world_from_base[:3, :3] = Rotation.from_euler(
        "xyz", [0.8, 0.06, -0.04]
    ).as_matrix()
    core = LeftLiveControl(
        Kinematics("left"),
        clock_id="test",
        world_from_base=world_from_base,
    )

    delta_world = np.array([0.0, 0.0, 0.20])
    delta_base = core.base_from_world_rotation @ delta_world

    np.testing.assert_allclose(
        world_from_base[:3, :3] @ delta_base,
        delta_world,
        atol=1e-14,
    )
    assert core.translation_frame == "tabletop_world_calibrated"
    assert abs(delta_base[1]) > 0.1
    assert abs(delta_base[2]) > 0.1


def test_controller_rotation_maps_one_to_one_about_calibrated_world_axes() -> None:
    world_from_base = np.eye(4)
    world_from_base[:3, :3] = Rotation.from_euler(
        "xyz", [0.8, 0.06, -0.04]
    ).as_matrix()
    core = LeftLiveControl(
        Kinematics("left"), clock_id="test", world_from_base=world_from_base
    )
    core.home_pose = core.kin.fk(Q)
    core.source_anchor = np.eye(4)
    core.source_anchor[:3, :3] = Rotation.from_euler(
        "xyz", [0.15, -0.1, 0.2]
    ).as_matrix()
    base_rotation = world_from_base[:3, :3]

    for axis in np.eye(3):
        angle = 0.12
        delta_world = Rotation.from_rotvec(angle * axis).as_matrix()
        source = core.source_anchor.copy()
        source[:3, :3] = delta_world @ core.source_anchor[:3, :3]
        target = core._target_from_source(source)
        np.testing.assert_allclose(target[:3, 3], core.home_pose[:3, 3])
        np.testing.assert_allclose(
            base_rotation @ target[:3, :3],
            delta_world @ base_rotation @ core.home_pose[:3, :3],
            atol=1e-12,
        )
        assert np.isclose(
            Rotation.from_matrix(
                target[:3, :3] @ core.home_pose[:3, :3].T
            ).magnitude(),
            angle,
        )


def test_each_left_controller_rotation_axis_reaches_ik_without_translation() -> None:
    for unity_axis in np.eye(3):
        core = LeftLiveControl(
            Kinematics("left"), clock_id="test", pose_shaper_bandwidth=100.0,
            relaxed_motion_gates=True,
        )
        ready(core)
        core.update_measured(Q, (0.0,) * 7, 1.20)
        core.process(sample(4, 1.20, y=True))
        core.update_measured(Q, (0.0,) * 7, 1.24)
        rotated = sample(
            5, 1.24,
            quaternion_xyzw=tuple(Rotation.from_rotvec(0.05 * unity_axis).as_quat()),
        )
        following = core.process(rotated)
        assert following.result is not None and following.result.success
        request_pose = following.result.request.target.pose
        np.testing.assert_allclose(request_pose.position, core.home_pose[:3, 3], atol=1e-12)
        desired_rotation = Rotation.from_quat(request_pose.quaternion_xyzw).as_matrix()
        delta_base = desired_rotation @ core.home_pose[:3, :3].T
        actual_rotvec = Rotation.from_matrix(delta_base).as_rotvec()
        converted_rotvec = Rotation.from_matrix(
            converted_matrix(rotated.sample)[:3, :3]
        ).as_rotvec()
        np.testing.assert_allclose(
            actual_rotvec / np.linalg.norm(actual_rotvec),
            converted_rotvec / np.linalg.norm(converted_rotvec),
            atol=1e-8,
        )
        assert 0.03 < np.linalg.norm(actual_rotvec) < 0.05
