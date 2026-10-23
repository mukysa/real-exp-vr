# Quest 3 ROS 2 workspace

This workspace contains only the Quest 3 / VR command path.  It is kept
separate from `gello_software/ros2` so the two teleoperation inputs can be
built and started independently.

The current adapter defaults to `enable_synthetic_input:=false`.  In that
mode it reports status but never publishes a joint target.

Build this workspace independently:

```bash
scripts/build_quest3_ros2.sh
```

Start the right-arm fake-hardware stack independently:

```bash
scripts/start_quest3_teleoperation_client.sh --fake
```

The launch reuses the validated `franka_fr3_arm_controllers` package as a
shared robot-side controller.  It does not launch the GELLO publisher and it
overrides the controller transport topics to `vr/raw_joint_states` and
`vr/accepted_joint_states` inside the `right` namespace.
