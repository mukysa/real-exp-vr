from dataclasses import replace
import unittest
import numpy as np
from data_collection.fr3_ik.constraints import Limits, check_candidate
from data_collection.fr3_ik.interfaces import FailureReason
from data_collection.fr3_ik.trajectories import Q0


class ConstraintTests(unittest.TestCase):
    def setUp(self):
        self.limits = Limits.from_controller()

    def check(self, q, dt=.02, velocity=None, error=0, limits=None):
        return check_candidate(q, Q0, np.zeros(7) if velocity is None else velocity,
                               dt, error, 0, limits or self.limits)

    def test_stationary_and_bad_values(self):
        self.assertTrue(self.check(Q0)[0].all_passed)
        for q, dt in (([float('nan')]*7,.02), (Q0,0), (Q0,-1), ([0]*6,.02)):
            with self.assertRaises(ValueError):
                self.check(q,dt)

    def test_reject_each_physical_constraint(self):
        q = np.array(Q0)
        q[0] = 3
        self.assertEqual(self.check(q)[1], FailureReason.JOINT_LIMIT)
        q[0] = .1
        self.assertEqual(self.check(q)[1], FailureReason.STEP_LIMIT)
        q[0] = .02
        self.assertEqual(self.check(q)[1], FailureReason.VELOCITY_LIMIT)
        q[0] = .001
        self.assertEqual(self.check(q)[1], FailureReason.ACCELERATION_LIMIT)
        self.assertEqual(self.check(Q0,error=.01)[1], FailureReason.RESIDUAL)

    def test_time_step_changes_velocity_and_acceleration(self):
        q = np.array(Q0); q[0] += .001
        a = self.check(q,dt=.02)
        b = self.check(q,dt=.04)
        np.testing.assert_allclose(a[2], b[2]*2)
        np.testing.assert_allclose(a[3], b[3]*4)


if __name__ == '__main__':
    unittest.main()
