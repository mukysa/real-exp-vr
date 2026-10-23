"""Compatibility imports for the ROS-independent adapter core.

New code should import :mod:`data_collection.meta_quest.adapter_core` directly.
The old B-line gate implementation was removed so there is only one safety
decision path and one ``IKResult`` type in the project.
"""

from data_collection.fr3_ik.interfaces import JOINT_NAMES as COMMAND_JOINT_NAMES

from .adapter_core import (
    AdapterCore,
    AdapterState,
    GateDecision,
    MAX_VELOCITY,
    POSITION_LOWER,
    POSITION_UPPER,
)


JointCommandGate = AdapterCore

__all__ = [
    "AdapterCore",
    "AdapterState",
    "COMMAND_JOINT_NAMES",
    "GateDecision",
    "JointCommandGate",
    "MAX_VELOCITY",
    "POSITION_LOWER",
    "POSITION_UPPER",
]
