"""Cached existing FR3 arm model plus nominal hand model for cross-check/display."""
import hashlib
from pathlib import Path

import numpy as np
import pinocchio as pin
import xacro
from ament_index_python.packages import get_package_share_directory

from data_collection.move_to_target_ee import build_fr3_model


class Kinematics:
    def __init__(self, side='right'):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        self.side = side
        self.model, self.flange_id = build_fr3_model()
        self.data = self.model.createData()
        path = Path(get_package_share_directory('franka_description')) / 'robots/fr3/fr3.urdf.xacro'
        xml = xacro.process_file(str(path), mappings={
            'hand': 'true', 'arm_prefix': side, 'ros2_control': 'false',
            'robot_ip': '', 'use_fake_hardware': 'false', 'fake_sensor_commands': 'false',
        }).toxml()
        self.full_model = pin.buildModelFromXML(xml)
        self.full_data = self.full_model.createData()
        self.full_q = pin.neutral(self.full_model)
        self.tcp_id = self.full_model.getFrameId(f'{side}_fr3_hand_tcp')
        self.full_flange_id = self.full_model.getFrameId(f'{side}_fr3_link8')
        self._full_fk(np.zeros(7))
        self.tool = np.linalg.inv(self.full_data.oMf[self.full_flange_id].homogeneous) @ self.full_data.oMf[self.tcp_id].homogeneous
        self.tool = np.array(self.tool)
        self.provenance = {'description': str(path), 'hand_urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
                           'side': side, 'flange_to_nominal_tcp': self.tool.tolist(), 'actual_F_T_EE_verified': False}

    def flange_pose(self, q):
        pin.forwardKinematics(self.model, self.data, np.asarray(q))
        pin.updateFramePlacements(self.model, self.data)
        return np.array(self.data.oMf[self.flange_id].homogeneous)

    def end_effector_pose(self, q, flange_to_ee):
        return self.flange_pose(q) @ flange_to_ee

    def fk(self, q):
        return self.end_effector_pose(q, self.tool)

    def tcp_jacobian(self, q):
        """TCP linear/angular velocity in base axes, columns in joint order."""
        self.full_q[:7] = q
        return np.array(pin.computeFrameJacobian(
            self.full_model, self.full_data, self.full_q, self.tcp_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED))[:, :7]

    def _full_fk(self, q):
        self.full_q[:7] = q
        pin.forwardKinematics(self.full_model, self.full_data, self.full_q)
        pin.updateFramePlacements(self.full_model, self.full_data)

    def geometry(self, q):
        self._full_fk(q)
        points = [np.array(self.full_data.oMf[self.full_model.getFrameId(f'{self.side}_fr3_link{i}')].translation).tolist()
                  for i in range(9)]
        tcp = np.array(self.full_data.oMf[self.tcp_id].homogeneous)
        return points + [tcp[:3, 3].tolist()], tcp
