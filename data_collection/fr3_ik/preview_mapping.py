"""Preview-only decoupled mapping; does not modify shared control calibration."""
import numpy as np
from scipy.spatial.transform import Rotation
from data_collection.meta_quest.coordinates import unity_to_robot_pose
from data_collection.meta_quest.calibration import RigidPose


def converted_matrix(sample):
    pose=unity_to_robot_pose(RigidPose(sample.position,sample.quaternion_xyzw))
    result=np.eye(4)
    result[:3,3]=pose.position
    result[:3,:3]=Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
    return result


def base_relative_target(source, source_anchor, robot_anchor):
    """Base-axis translation; spatial relative rotation left-multiplies robot R0."""
    target=np.eye(4)
    target[:3,3]=robot_anchor[:3,3]+source[:3,3]-source_anchor[:3,3]
    target[:3,:3]=source[:3,:3]@source_anchor[:3,:3].T@robot_anchor[:3,:3]
    return target


def world_relative_target(source, source_anchor, robot_anchor, world_from_base):
    """Apply hand deltas along tabletop axes, then return a base-frame IK target."""
    world_anchor = world_from_base @ robot_anchor
    world_target = base_relative_target(source, source_anchor, world_anchor)
    return np.linalg.inv(world_from_base) @ world_target
