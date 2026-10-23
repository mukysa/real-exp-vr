#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p outputs/fr3_ik/live-tools
g++ -std=c++17 -O2 scripts/franka_readonly_preview.cpp \
 -I/home/landau/franka_ros2_ws/src/libfranka/include \
 -I/home/landau/franka_ros2_ws/src/libfranka/common/include \
 -L/home/landau/franka_ros2_ws/build/libfranka \
 -Wl,-rpath,/home/landau/franka_ros2_ws/build/libfranka -lfranka \
 -o outputs/fr3_ik/live-tools/franka_readonly
