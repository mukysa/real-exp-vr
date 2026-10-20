from __future__ import annotations

import json
import math

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.coordinates import unity_to_robot_pose
from data_collection.meta_quest.protocol import parse_pose_line
from data_collection.meta_quest.steamvr_bridge import (
    alvr_is_streaming,
    compact_controller_line,
    sapien_pose_to_unity,
)


@pytest.mark.parametrize(
    ("position", "quaternion_wxyz"),
    [
        ((0.3, -0.2, 0.8), (1.0, 0.0, 0.0, 0.0)),
        (
            (-0.1, 0.4, 0.7),
            (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
        ),
    ],
)
def test_sapien_pose_round_trips_through_existing_unity_contract(
    position,
    quaternion_wxyz,
) -> None:
    unity_pose = sapien_pose_to_unity(position, quaternion_wxyz)
    recovered = unity_to_robot_pose(unity_pose)

    assert recovered.position == pytest.approx(position)
    expected_xyzw = (
        quaternion_wxyz[1],
        quaternion_wxyz[2],
        quaternion_wxyz[3],
        quaternion_wxyz[0],
    )
    alignment = sum(
        actual * expected
        for actual, expected in zip(recovered.quaternion_xyzw, expected_xyzw)
    )
    assert abs(alignment) == pytest.approx(1.0)


def test_compact_line_reuses_validated_controller_protocol() -> None:
    line = compact_controller_line(
        side="right",
        pose=RigidPose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),
        grasp=0.75,
        clutch=True,
        record=False,
        frame_id=42,
        timestamp_ns=123456789,
    )

    sample = parse_pose_line(line)
    assert sample is not None
    assert sample.source == "controller"
    assert sample.side == "right"
    assert sample.layout == "controller_compact_controls_v1"
    assert sample.position == (1.0, 2.0, 3.0)
    assert sample.grasp == 0.75
    assert sample.clutch is True
    assert sample.record is False
    assert sample.frame_id == 42
    assert sample.device_timestamp_ns == 123456789


def test_alvr_streaming_state_is_explicit_and_fail_closed(tmp_path) -> None:
    session = tmp_path / "session.json"
    assert alvr_is_streaming(session) is False

    session.write_text("not json", encoding="utf-8")
    assert alvr_is_streaming(session) is False

    session.write_text(
        json.dumps(
            {
                "client_connections": {
                    "quest": {"connection_state": "Disconnected"}
                }
            }
        ),
        encoding="utf-8",
    )
    assert alvr_is_streaming(session) is False

    session.write_text(
        json.dumps(
            {
                "client_connections": {
                    "quest": {"connection_state": "Streaming"}
                }
            }
        ),
        encoding="utf-8",
    )
    assert alvr_is_streaming(session) is True
