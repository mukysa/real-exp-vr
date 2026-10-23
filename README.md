# Real Experiments

This repository groups the code and notes used for real-world Franka FR3 experiments with GELLO and Meta Quest teleoperation, LeRobot dataset collection, and training workflows.

## Current VR / IK work

Start with [A线-当前状态与下一步.md](A线-当前状态与下一步.md) for the current dual-arm IK status. The latest offline review, test evidence, and blockers are in [docs/A-Line-Offline-Feasibility-Review-20261023.md](docs/A-Line-Offline-Feasibility-Review-20261023.md). Historical plans and completed validation reports are indexed in [z_histroy_plan/README.md](z_histroy_plan/README.md).

## Repository Layout

- `data_collection/`: scripts and documentation for recording, replaying, and editing local LeRobot datasets.
- `train/`: training-related notes and experiment-specific training assets.
- `deploy/`: policy inspection, serving, fetching, and real-robot execution tools.
- `utils/`: shared dataset-statistics and image-preprocessing helpers.
- `lerobot/`: vendored LeRobot codebase used by the local collection and training workflow.
- `gello_software/`: GELLO and ROS 2 integration code, tracked as a git submodule.
- `libs/hand-tracking-streamer/`: upstream Meta Quest tracking application, tracked as a git submodule.
- `data_collection/meta_quest/`: standalone, robot-disconnected Quest pose transport and monitor.
- `data/`: local datasets and experiment outputs.

## Submodules

The repo expects its submodules, including `lerobot/`, `gello_software/`, and
`libs/hand-tracking-streamer/`, to be available locally. If a submodule is
empty after cloning, initialize all submodules:

```bash
git submodule update --init --recursive
```

## Environment Split

This repo uses two different Python environments:

- Use the system ROS 2 / Franka environment for `gello_software`, ROS 2 launch files, and `data_collection/replay_lerobot_episode.py`.
- Use the `lerobot` Conda environment for `data_collection/lerobot_collection.py`, dataset inspection, dataset editing, and training under `lerobot/`.
- Use the environment loaded by `scripts/validate_fr3_ik_offline.sh` for FR3 IK. The checked-in `ms3-enviroment.yaml` is a snapshot from another host; client currently has no installed `ms3` Conda environment.

## Common Workflows

### Data Collection

Data collection and replay instructions live in [data_collection/DATA_COLLECTION_README.md](data_collection/DATA_COLLECTION_README.md).
- [`scripts/start_teleoperation.sh`](scripts/start_teleoperation.sh): start left-, right-, or dual-arm GELLO/FR3 teleoperation with matching Franka grippers or Wuji hands.
- [`scripts/record_meta_quest_openvr.sh`](scripts/record_meta_quest_openvr.sh): record Quest Touch controller input through the validated ALVR/OpenVR path without connecting to a robot. See [`Quest-ALVR-OpenVR-Reproduction.md`](Quest-ALVR-OpenVR-Reproduction.md).
- `start_teleoperation.sh` accepts exactly one end-effector mode (`--arm`, `--gripper`, or `--hand`) and one arm selection (`--left`, `--right`, or `--duo`).

All launchers can be invoked from a bare shell; manually activating Conda is
not required. ROS-only processes use the system ROS Python. LeRobot, policy,
and Wuji processes use the `lerobot` Conda environment automatically without
changing the calling shell. Set `LEROBOT_CONDA_ENV`, `WUJI_CONDA_ENV`, or
`DEPLOYMENT_CONDA_ENV` to select a differently named environment.
- [`scripts/start_data_collection_client.sh`](scripts/start_data_collection_client.sh): start control-host GELLO/Franka teleoperation for split collection.
- [`scripts/start_data_collection_server.sh`](scripts/start_data_collection_server.sh): start the data-server RealSense publishers, ROS bridge, and optional recorder.
- [`scripts/replay.sh`](scripts/replay.sh): start the dual-arm FR3 replay controllers and Franka-hand managers, then replay a recorded LeRobot episode.

### Meta Quest input and offline IK status (client, 2026-10-23)

The supported live input path is Quest Touch controllers through ALVR/SteamVR
and the direct OpenVR bridge into the existing lossless JSONL recorder. The
older Hand Tracking Streamer/ADB launcher and the SAPIEN SteamVR bridge are
historical diagnostics, not current operator entry points.

Use the robot-disconnected launcher after ALVR reaches `Streaming`:

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/stage8-openvr-guided-20261020.jsonl
```

Press `Ctrl+C` once and wait for `Saved ... poses`; the recorder writes a
completion summary and atomically publishes the final JSONL. Use a new filename
after stage 8 exists. The full startup, validation, and shutdown procedure is
maintained only in
[Quest-ALVR-OpenVR-Reproduction.md](Quest-ALVR-OpenVR-Reproduction.md).
Dual-arm offline IK and the read-only preview are implemented. The selected A-line suite passes 109 tests in the project ROS/Franka environment, but the result remains diagnostic and has not been jointly validated through the ROS adapter and fake controller. See [A线-当前状态与下一步.md](A线-当前状态与下一步.md) for current progress and
[data_collection/meta_quest/README.md](data_collection/meta_quest/README.md) for
the protocol and safety boundary.

### Train

- Training instructions live in [train/TRAIN_README.md](train/TRAIN_README.md).

### Deploy

- Deployment instructions live in [deploy/DEPLOY_README.md](deploy/DEPLOY_README.md).
- [`scripts/start_deployment_server.sh`](scripts/start_deployment_server.sh): configure cameras and the deployment bridge from checkpoint-embedded metadata, then start the policy server on the inference server.
- [`scripts/start_deployment_client.sh`](scripts/start_deployment_client.sh): query the policy server for its checkpoint contract, then select left/right/duo FR3 hardware plus the matching gripper or Wuji worker. The client does not need the dataset or checkpoint.

## Notes

- Do not change the GELLO USB connections. If the topology changes, reinstall or update the USB rules and verify both aliases before launching ROS 2 nodes.
- Check the GELLO offsets whenever the teleoperator joints do not align with the robot or after a hardware reset. Recompute the offsets with the procedure in [data_collection/DATA_COLLECTION_README.md](data_collection/DATA_COLLECTION_README.md) before collecting data.
