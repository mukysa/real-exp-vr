from __future__ import annotations

from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.live_control import LeftLiveControl
from data_collection.fr3_ik.preview_mapping import converted_matrix
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose


Q = (-0.11657, 0.42515, -0.66825, -1.97659, 0.82153, 1.84735, -1.95020)


def sample(sequence: int, timestamp: float, *, y: bool = False, x: float = 0.0) -> ReceivedPose:
    pose = PoseSample(
        "left",
        "controller",
        (x, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
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


def test_half_scale_workspace_is_hard_capped_at_two_hundred_mm() -> None:
    core = LeftLiveControl(
        Kinematics("left"),
        clock_id="test",
        position_scale=0.5,
        max_tcp_offset_m=0.20,
    )
    ready(core)
    core.update_measured(Q, (0.0,) * 7, 1.20)
    core.process(sample(4, 1.20, y=True))
    # Model a continuous approach to the boundary without asking the IK solver
    # to track the intermediate points in this workspace-limit unit test.
    core.last_source = converted_matrix(sample(5, 1.24, x=0.30).sample)
    core.last_pose_stamp = 1.24
    core.update_measured(Q, (0.0,) * 7, 1.28)
    update = core.process(sample(6, 1.28, x=0.41))
    assert update.result is None
    assert core.state == "FAULT"
    assert "200 mm" in core.reason


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
