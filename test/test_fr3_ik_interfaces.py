import hashlib
import json
import re
from dataclasses import replace
from pathlib import Path
import unittest

from data_collection.fr3_ik.examples import contract_examples
from data_collection.fr3_ik.interfaces import (
    ClockDomain, ConstraintChecks, JointVector, Timestamp,
)
from data_collection.meta_quest.replay import load_recording
from data_collection.meta_quest.state_machine import TeleoperationState

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "recordings/quest_raw/stage7-openvr-guided-20261020.jsonl"
FIXTURE_SHA = "024488228e430dd0cd94a7e34756ded1d4f53782a9e30525e2b84e7c2f6cdbba"


class InterfaceTests(unittest.TestCase):
    def setUp(self):
        self.examples = contract_examples()

    def test_python310_state_enum_preserves_string_contract(self):
        state = TeleoperationState.FOLLOWING
        self.assertEqual(str(state), "FOLLOWING")
        self.assertEqual(state, "FOLLOWING")
        self.assertEqual(hash(state), hash("FOLLOWING"))
        self.assertEqual(json.dumps(state), '"FOLLOWING"')

    def test_only_following_has_adapter_command(self):
        for name, result in self.examples.items():
            if name == "following":
                self.assertTrue(result.success)
                self.assertEqual(result.command_for_adapter(), result.last_safe_target)
            else:
                self.assertFalse(result.success)
                self.assertIsNone(result.command)
                with self.assertRaises(ValueError):
                    result.command_for_adapter()

    def test_non_success_cannot_expose_command(self):
        command = self.examples["following"].command
        for name in ("normal_hold", "unreachable", "reconnect"):
            with self.assertRaises(ValueError):
                replace(self.examples[name], command=command)

    def test_success_requires_every_check_and_residual(self):
        result = self.examples["following"]
        for field in ("residual", "joint_limits", "step", "velocity", "acceleration"):
            for value in (False, None):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    replace(result, checks=replace(result.checks, **{field: value}))
        with self.assertRaises(ValueError):
            replace(result, position_residual_m=None)

    def test_invalid_vectors_clocks_and_metrics(self):
        for q in ((0,) * 6, (float("nan"),) * 7, (float("inf"),) * 7):
            with self.assertRaises(ValueError):
                JointVector(q)
        with self.assertRaises(ValueError):
            JointVector((0,) * 7, tuple(reversed(self.examples["following"].command.joints.names)))
        now = Timestamp(2, ClockDomain.HOST_MONOTONIC, "host-a")
        self.assertEqual(now.elapsed_since(Timestamp(1, now.domain, now.clock_id)), 1)
        for earlier in (Timestamp(1, ClockDomain.REPLAY, "host-a"),
                        Timestamp(1, ClockDomain.HOST_MONOTONIC, "host-b")):
            with self.assertRaises(ValueError):
                now.elapsed_since(earlier)
        with self.assertRaises(ValueError):
            replace(self.examples["following"], solve_time_s=float("nan"))
        with self.assertRaises(ValueError):
            ConstraintChecks(step="true")

    def test_reject_preview_wrong_frame_and_timestamp(self):
        request = self.examples["following"].request
        for target in (replace(request.target, state="PREVIEW"),
                       replace(request.target, frame_id="unity"),
                       replace(request.target, generated_monotonic_s=123)):
            with self.assertRaises(ValueError):
                replace(request, target=target)

    def test_ik_request_selects_left_or_right_frames_explicitly(self):
        request = self.examples["following"].request
        left_target = replace(request.target, frame_id="left_fr3_link0")
        left_request = replace(request, target=left_target, tool_frame="left_fr3_hand_tcp", side="left")
        self.assertEqual(left_request.side, "left")
        self.assertEqual(left_request.target.frame_id, "left_fr3_link0")
        with self.assertRaises(ValueError):
            replace(request, side="left")

    def test_connection_and_upstream_reason_preserved(self):
        result = self.examples["reconnect"]
        self.assertEqual(result.request.target.source_connection_id, 2)
        self.assertEqual(result.request.target.source_sequence, 2)
        self.assertEqual(result.request.target.hold_reason, "connection_changed")
        self.assertEqual(result.request.source_stamp.clock_id, "example-host-session")

    def test_stage7_fixture_is_lossless_and_not_modified(self):
        if not FIXTURE.exists():
            self.skipTest("Local stage7 recording is not distributed with interface code")
        before = hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
        self.assertEqual(before, FIXTURE_SHA)
        recording = load_recording(FIXTURE)
        right = tuple(p for p in recording.poses if p.sample.side == "right")
        self.assertTrue(recording.lossless)
        self.assertEqual(len(recording.poses), 904)
        self.assertEqual(len(right), 452)
        self.assertEqual({p.connection_id for p in right}, {1})
        self.assertEqual((right[0].sequence, right[-1].sequence), (2, 904))
        self.assertEqual(hashlib.sha256(FIXTURE.read_bytes()).hexdigest(), before)

    def test_sample_joints_within_gello_controller_envelope(self):
        header = (ROOT / "gello_software/ros2/src/franka_fr3_arm_controllers/include/"
                  "franka_fr3_arm_controllers/joint_reference_generator.hpp").read_text()
        def values(key):
            return tuple(float(x) for x in re.search(rf"{key}\s*=\s*\{{([^}}]+)\}}", header)[1].split(","))
        q = self.examples["following"].command.joints.positions_rad
        self.assertTrue(all(lo <= x <= hi for lo, x, hi in
                            zip(values("kPositionLower"), q, values("kPositionUpper"))))


if __name__ == "__main__":
    unittest.main()
