from __future__ import annotations

import math

import pytest

from data_collection.meta_quest.calibration import RigidPose
from data_collection.meta_quest.coordinates import (
    UNITY_TO_ROBOT_BASIS,
    change_rotation_basis,
    quaternion_to_matrix,
    transform_vector,
    unity_to_robot_pose,
)


@pytest.mark.parametrize(
    ("unity", "robot"),
    [
        ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    ],
)
def test_unity_axes_map_to_documented_robot_axes(unity, robot) -> None:
    assert transform_vector(unity) == pytest.approx(robot)


def test_rotation_basis_change_preserves_rotated_vector_relationship() -> None:
    half_sqrt_two = math.sqrt(0.5)
    unity_rotation = (half_sqrt_two, 0.0, 0.0, half_sqrt_two)
    robot_rotation = change_rotation_basis(unity_rotation)
    unity_vector = (0.0, 1.0, 0.0)

    unity_matrix = quaternion_to_matrix(unity_rotation)
    rotated_unity = tuple(
        sum(unity_matrix[row][column] * unity_vector[column] for column in range(3))
        for row in range(3)
    )
    robot_vector = transform_vector(unity_vector)
    robot_matrix = quaternion_to_matrix(robot_rotation)
    rotated_robot = tuple(
        sum(robot_matrix[row][column] * robot_vector[column] for column in range(3))
        for row in range(3)
    )

    assert transform_vector(rotated_unity) == pytest.approx(rotated_robot)
    assert sum(value * value for value in robot_rotation) == pytest.approx(1.0)


def test_unity_pose_conversion_applies_scale_and_tool_offset() -> None:
    pose = RigidPose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    tool = RigidPose((0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    converted = unity_to_robot_pose(pose, position_scale=0.5, controller_to_tool=tool)

    assert converted.position == pytest.approx((1.6, -0.5, 1.0))
    assert converted.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_coordinate_conversion_rejects_bad_scale_and_basis() -> None:
    pose = RigidPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="position_scale"):
        unity_to_robot_pose(pose, position_scale=0.0)
    with pytest.raises(ValueError, match="orthonormal"):
        transform_vector((1.0, 0.0, 0.0), ((1.0, 0.0, 0.0),) * 3)


def test_basis_constant_is_a_handedness_change() -> None:
    # det(B) == -1 documents the intentional left- to right-handed conversion.
    basis = UNITY_TO_ROBOT_BASIS
    determinant = (
        basis[0][0] * (basis[1][1] * basis[2][2] - basis[1][2] * basis[2][1])
        - basis[0][1] * (basis[1][0] * basis[2][2] - basis[1][2] * basis[2][0])
        + basis[0][2] * (basis[1][0] * basis[2][1] - basis[1][1] * basis[2][0])
    )
    assert determinant == pytest.approx(-1.0)
