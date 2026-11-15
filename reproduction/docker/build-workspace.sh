#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
cd /ws
catkin init
catkin config --merge-devel --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
catkin build dcl_fast_lio -j2 -p1 --no-status
