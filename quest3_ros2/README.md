# Quest 3 ROS 2 workspace

This workspace contains only the Quest 3 / VR command path.  It is kept
separate from `gello_software/ros2` so the two teleoperation inputs can be
built and started independently.

The adapter consumes `fr3_ik_result_v1` JSON messages produced from the sole
strict A-line `IKResult` contract.  There is no synthetic target source in the
runtime package.

Build this workspace independently:

```bash
scripts/build_quest3_ros2.sh
```

Start only the adapter (during the later, supervised hardware stage):

```bash
scripts/start_quest3_teleoperation_client.sh --side right
```

The launch does not start a robot controller or GELLO.  The Quest and GELLO
workspaces and startup entries remain independent.  The adapter refuses to
publish when a GELLO raw-command publisher is present for the selected arm.
