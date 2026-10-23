from dataclasses import replace
import unittest
import numpy as np

from data_collection.fr3_ik.constraints import Limits
from data_collection.fr3_ik.interfaces import ClockDomain, FailureReason, JointTarget, JointVector, Timestamp, MeasuredJointState
from data_collection.fr3_ik.kinematics import Kinematics
from data_collection.fr3_ik.offline_validator import request_for, run_scenario, trajectory_digest, assess
from data_collection.fr3_ik.solver import ContinuousIK
from data_collection.fr3_ik.trajectories import Q0, Sample, scenarios


class OfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = Kinematics()
        cls.limits = Limits.from_controller()

    def setup_solver(self, **kwargs):
        solver = ContinuousIK(self.kin, **kwargs)
        anchor = JointTarget(JointVector(Q0), Timestamp(0, ClockDomain.REPLAY, 'test'))
        solver.reset(anchor, 1)
        request = request_for(Sample(.02,self.kin.fk(Q0)),1,'test')
        return solver, request

    def test_fresh_full_model_fk_matches_arm_model(self):
        for q in (Q0, (.3,-.6,.2,-2,.1,1.8,-.2)):
            _, full = self.kin.geometry(q)
            np.testing.assert_allclose(self.kin.fk(q),full,atol=1e-12)

    def test_clock_order_feedback_and_connection_rejection(self):
        for failure in ('clock','sequence','feedback','connection','previous'):
            solver, request = self.setup_solver()
            previous = solver.last
            measured = None
            if failure=='clock':
                request=replace(request,target_stamp=Timestamp(.02,ClockDomain.REPLAY,'other'))
            if failure=='sequence':
                solver.last_sequence=1
            if failure=='feedback':
                measured=MeasuredJointState(previous.joints,Timestamp(1,ClockDomain.REPLAY,'test'),'test',True)
            if failure=='connection':
                request=replace(request,target=replace(request.target,source_connection_id=2))
            if failure=='previous':
                previous=replace(previous,stamp=Timestamp(.001,ClockDomain.REPLAY,'test'))
            result=solver.solve(request,previous,measured)
            self.assertFalse(result.success)
            self.assertIsNone(result.command)
            self.assertEqual(result.last_safe_target.joints.positions_rad,Q0)

    def test_fault_stays_latched_until_explicit_reset(self):
        solver, request = self.setup_solver(timeout_s=1e-12)
        result=solver.solve(request,solver.last)
        self.assertEqual(result.reason,FailureReason.SOLVE_TIMEOUT)
        next_request=request_for(Sample(.04,self.kin.fk(Q0)),2,'test')
        self.assertFalse(solver.solve(next_request,solver.last).success)
        self.assertEqual(solver.last.joints.positions_rad,Q0)
        solver.timeout_s=None
        solver.reset(JointTarget(solver.last.joints,Timestamp(.02,ClockDomain.REPLAY,'test')),1)
        self.assertTrue(solver.solve(next_request,solver.last).success)

    def test_normal_hold_requires_reanchor(self):
        solver, _ = self.setup_solver()
        held=request_for(Sample(.02,self.kin.fk(Q0),'hold'),1,'test')
        self.assertFalse(solver.solve(held,solver.last).success)
        following=request_for(Sample(.04,self.kin.fk(Q0)),2,'test')
        self.assertEqual(solver.solve(following,solver.last).reason,FailureReason.REANCHOR_REQUIRED)

    def test_moving_trajectory_is_deterministic_and_not_all_hold(self):
        samples=dict(scenarios(self.kin.fk(Q0),hz=20,duration=4))['x']
        a=run_scenario('x',samples,self.kin,self.limits,20,80)
        b=run_scenario('x',samples,self.kin,self.limits,20,80)
        self.assertTrue(assess('x',a,self.limits)['passed'])
        self.assertEqual(trajectory_digest(a),trajectory_digest(b))


if __name__=='__main__':
    unittest.main()
