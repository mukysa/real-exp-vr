"""Numerical constraints for offline joint references, not collision checking."""
from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from .interfaces import ConstraintChecks, FailureReason


@dataclass(frozen=True)
class Limits:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple
    max_step_rad: float = 0.08
    position_error_m: float = 0.001
    rotation_error_rad: float = 0.002
    input_gap_s: float = 0.2
    source_jump_m: float = 0.08
    source_jump_rad: float = 0.35

    @classmethod
    def from_controller(cls):
        path = Path(__file__).resolve().parents[2] / 'gello_software/ros2/src/franka_fr3_arm_controllers/include/franka_fr3_arm_controllers/joint_reference_generator.hpp'
        text = path.read_text()
        def values(name):
            match = re.search(rf'{name}\s*=\s*\{{([^}}]+)\}}', text)
            if match is None:
                raise ValueError(f'Missing controller constraint {name}')
            result = tuple(float(x) for x in match[1].split(','))
            if len(result) != 7 or not np.isfinite(result).all():
                raise ValueError(f'Invalid controller constraint {name}')
            return result
        return cls(*(values(name) for name in ('kPositionLower', 'kPositionUpper', 'kMaxVelocity', 'kMaxAcceleration')))


def residual(actual, target):
    return (float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
            float(Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude()))


def check_candidate(q, previous, previous_velocity, dt, position_error, rotation_error, limits):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Finite seven-joint candidate and positive dt required')
    delta = q - np.asarray(previous)
    velocity = delta / dt
    acceleration = (velocity - np.asarray(previous_velocity)) / dt
    checks = ConstraintChecks(
        residual=bool(np.isfinite(position_error) and np.isfinite(rotation_error)
                      and 0 <= position_error <= limits.position_error_m
                      and 0 <= rotation_error <= limits.rotation_error_rad),
        joint_limits=bool(np.all(q >= limits.lower) and np.all(q <= limits.upper)),
        step=bool(np.max(np.abs(delta)) <= limits.max_step_rad),
        velocity=bool(np.all(np.abs(velocity) <= np.asarray(limits.velocity) + 1e-9)),
        acceleration=bool(np.all(np.abs(acceleration) <= np.asarray(limits.acceleration) + 1e-9)),
    )
    reason = FailureReason.NONE
    for passed, failure in ((checks.residual, FailureReason.RESIDUAL),
                            (checks.joint_limits, FailureReason.JOINT_LIMIT),
                            (checks.step, FailureReason.STEP_LIMIT),
                            (checks.velocity, FailureReason.VELOCITY_LIMIT),
                            (checks.acceleration, FailureReason.ACCELERATION_LIMIT)):
        if not passed:
            reason = failure
            break
    return checks, reason, velocity, acceleration
