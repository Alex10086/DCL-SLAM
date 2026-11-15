#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/results/home/log" "$root/results/home/dcl_output"
sudo -n docker run -d --name dcl-slam-noetic \
  --network host --ipc host \
  --user "$(id -u):$(id -g)" \
  --env HOME=/results/home --env ROS_HOME=/results/home/.ros \
  --mount "type=bind,source=$root/ws,target=/ws" \
  --mount "type=bind,source=$root/data,target=/data,readonly" \
  --mount "type=bind,source=$root/results,target=/results" \
  --mount "type=bind,source=$root/docker,target=/repro,readonly" \
  --workdir /ws dcl-slam:noetic-arm64 sleep infinity
