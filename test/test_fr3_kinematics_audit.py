"""Run with the same system ROS dependencies as validate_fr3_kinematics.sh."""
import unittest

import numpy as np

from data_collection.validate_fr3_kinematics import audit


class KinematicsAuditTest(unittest.TestCase):
    def test_nominal_tcp_and_round_trip_at_two_configurations(self):
        reports = [audit(q) for q in (
            [0, -0.4, 0, -1.8, 0, 1.5, 0.7],
            [0.3, -0.6, 0.2, -2.0, 0.1, 1.8, -0.2],
        )]
        expected = np.eye(4)
        c = np.sqrt(0.5)
        expected[:2, :2] = [[c, c], [-c, c]]
        expected[2, 3] = 0.1034
        for report in reports:
            self.assertTrue(report["passed"])
            self.assertFalse(report["actual_F_T_EE_verified"])
            np.testing.assert_allclose(report["flange_to_nominal_tcp"], expected, atol=1e-12)
            self.assertLess(report["position_error_m"], 1e-5)
            self.assertLess(report["orientation_error_rad"], 1e-5)

    def test_invalid_joint_inputs_rejected(self):
        for q in ([0] * 7, [float("nan")] * 7, [0] * 6):
            with self.subTest(q=q), self.assertRaises(ValueError):
                audit(q)


if __name__ == "__main__":
    unittest.main()
