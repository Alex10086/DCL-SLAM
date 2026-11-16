# DCL-SLAM

A ROS package of DCL-SLAM: Distributed Collaborative LiDAR SLAM Framework for a Robotic Swarm. 

https://user-images.githubusercontent.com/41199568/213071890-679025cf-23f5-48f0-a2ef-d9b00911f926.mp4

The HD video of the demonstration of DCL-SLAM is avaliable at [BiliBili](https://www.bilibili.com/video/BV12G4y187mw/?spm_id_from=333.337.search-card.all.click).

## Author
[Shipeng Zhong](https://github.com/zhongshp) & [Dapeng Feng](https://github.com/DapengFeng) & [Zhiqiang Chen](https://github.com/thisparticle)

## Prerequisites
  - [Ubuntu ROS](http://wiki.ros.org/ROS/Installation) (Robot Operating System on Ubuntu 18.04 or 20.04)
  - Python (For wstool and catkin tool)
  - CMake (Compilation Configuration Tool)
  - [Boost](http://www.boost.org/) (portable C++ source libraries)
  - [PCL](https://pointclouds.org/downloads/linux.html) (Default Point Cloud Library on Ubuntu work normally)
  - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (Default Eigen library on Ubuntu work normally)
  ```
  sudo apt-get install cmake libboost-all-dev python-wstool python-catkin-tools
  ```
  - [GTSAM](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)
  - [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) (The driver for Livox LiDAR)

  These prerequisites will be installed during the compilation.

## Compilation
  Set up the workspace configuration:
  ```
  mkdir -p ~/cslam_ws/src
  cd ~/cslam_ws
  catkin init
  catkin config --merge-devel
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
  
  Then use wstool for fetching catkin dependencies:
  ```
  cd src
  git clone https://github.com/Alex10086/DCL-SLAM.git
  git clone https://github.com/zhongshp/DCL-LIO-SAM.git
  git clone https://github.com/zhongshp/DCL-FAST-LIO.git
  wstool init
  wstool merge DCL-SLAM/dependencies.rosinstall
  wstool update
  ```

  Build DCL-SLAM
  ```
  catkin build dcl_lio_sam
  catkin build dcl_fast_lio
  ```
  
## Run with Dataset
  - [S3E dataset](https://github.com/DapengFeng/S3E). The datasets are configured to run with default parameter.
  ```
  roslaunch dcl_slam run.launch
  rosbag play *your-bag-path*.bag
  ```
  
  - Other dataset. Please follow [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) and [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) to set your own config file for the dataset in "config/*your-config-file*.yaml", and change the path in "launch/single_ugv.launch".

### Local Jetson4 Docker reproduction

This fork is maintained at https://github.com/Alex10086/DCL-SLAM with
https://github.com/zhongshp/DCL-SLAM retained as the upstream project.
The isolated workspace is `~/dcl_slam_docker`; the ARM64 Ubuntu 20.04 /
ROS Noetic container is `dcl-slam-noetic`. The host ROS 2 Jazzy is unchanged.
The reproduction uses the supported FAST-LIO2 frontend (`lioType=2`).

**Status:** three-robot short-sequence acceptance has not passed yet.
The latest backend-finalization change received Python syntax checking only.
No additional tests or replays were started after the request to fix and fork.
Earlier runs proved full sensor coverage and A/B optimization, not complete
three-robot fusion. Historical measurements and patch status are recorded in
`reproduction/versions.json`; do not describe these changes as runtime-verified.

`reproduction/` contains the Docker scripts, conversion tools, published
calibration overlays, pinned source manifest, and companion repository patches.
These assets use the isolated workspace layout rather than the catkin package root:

```bash
workspace="$HOME/dcl_slam_docker"
cp -a reproduction/. "$workspace/"
git -C "$workspace/ws/src/DCL-FAST-LIO" apply "$workspace/patches/dcl-fast-lio.patch"
git -C "$workspace/ws/src/gtsam_catkin" apply "$workspace/patches/gtsam-catkin.patch"
```

Apply patches only to the clean revisions listed in the manifest. They are
already applied in the existing Jetson workspace and must not be applied twice.
Build with `bash "$workspace/docker/build-workspace.sh"`. Creating this fork does
not build, start, or test the environment.

When testing is explicitly resumed, use only the prepared 121-second prefix:

```bash
sudo docker exec dcl-slam-noetic bash -c 'source /ws/devel/setup.bash; python3 /repro/run-dataset.py /results/loop_diagnosis/prefix121.bag --output /results/short_$(date +%Y%m%d_%H%M%S)'
```

The runner uses actual slowest-stream odometry for rosbag feedback limiting.
Native `--keep-alive` continues only `/clock` after the last recorded sensor input,
allowing queued loop detection and optimization to finish. It stops only after
input coverage, a connected A/B/C loop graph, and a committed, nonaborted
optimization round with identical coordinator, participants (0,1,2), order,
and committed PCM separator edges reported by all three robots. The committed
separator graph must itself connect all three; pre-PCM loop connectivity alone
is insufficient. Actual End (6), with both rotation and pose stages done, is
retained through later Idle/PostEnd statuses; historical A/B completion cannot
combine with a newer C
completion, and a newer incomplete or aborted round blocks older success.
Every map must be received after the joint completion observation (ROS clock),
not merely carry an old keyframe header stamp. Backend finalization is bounded
by `--drain-timeout` (600 wall seconds); no later sensor data is added.
`validate_results.py` applies identical live/final criteria. Both `progress.json`
and `summary.json` save numeric state history, the newest 32 rounds with per-robot
metadata, latest status, latched abort/completion evidence and receiver times,
plus separate map reception and header times.

The lowest-ID coordinator drives PREPARE/READY/COMMIT: each participant freezes
its own graph/values and runs unchanged local PCM, then the coordinator intersects
endpoint acceptance, checks connectivity and commits a common connected greedy
order. All participants acknowledge before rotation starts. Each round uses a
fresh optimizer; round/iteration-tagged incremental Gauss-Seidel retains early
estimates, requires final rotation payloads before pose conversion, and distributes
the final anchor even to non-neighbors. Aborts propagate without shrinking the
participant set. `dcl_slam/optimization_round_timeout` defaults to 120 wall seconds
without semantic progress. The loop worker releases its shared lock and yields
1 ms after each batch to let callbacks run even when ROS Rate overruns.

**Rebuild required:** `optimizationState` now uses `dcl_slam/optimization_state`,
not `std_msgs/Int8`, with round metadata, proposal/committed separator keys and
stage flags. Neighbor estimates carry round/iteration IDs; the separate
`rotationEstimateState`/`poseEstimateState` topics are removed. Rebuild the whole
workspace and every frontend/custom-message caller together; old binaries and
Int8 subscribers are incompatible. No PCM/ICP thresholds or solver mathematics
are relaxed by this round protocol.

After rebuilding, run the focused checks:

```bash
sudo docker exec dcl-slam-noetic bash -c 'source /ws/devel/setup.bash; cmake --build /ws/build/dcl_slam --target dcl_round_protocol_test && /ws/devel/lib/dcl_slam/dcl_round_protocol_test'
sudo docker exec dcl-slam-noetic bash -c 'source /ws/devel/setup.bash; python3 -m unittest discover -s /ws/src/DCL-SLAM/reproduction/tests -p test_validate_results.py -v'
```

The C++ regression lives in `reproduction/tests/round_protocol_test.cpp`; Python
tests exercise real live/final snapshot validation without starting ROS nodes.

The coarse rowkey candidate width is 40 instead of 6: timestamp-aligned saved
queries placed valid C candidates at ranks 13-35. ICP, RANSAC, Hamming and PCM
acceptance thresholds were not relaxed. Necessary source corrections also cover
KD-column/keyframe correspondence, aligned descriptor lookup, scoped KD trees,
missing-neighbor handling, descriptor yaw units, periodic image shifts, IMU-frame
keyframes, synchronized graph access, post-reordering PCM trajectory keys, and
sequential-odometry merging after a frozen round.

The original [S3E dataset](https://pengyu-team.github.io/S3E/) is retained outside
Git. Conversion uses uncompressed ROS 1 chunks and preserves measured firing times.
Calibration overlays are derived from the publisher's S3Ev1 camera-chain files.
PCM requires `$HOME/dcl_output`, created by `docker/start-container.sh`.
TUM trajectories, PCD map views and recorded output bags remain local; map views
are not a complete dense map. Datasets, build products, credentials and experiment
logs are not included in this fork.

## Citation
This work is published in IEEE Sensors Journal 2023, and please cite related papers:

```
@ARTICLE{10375928,
  author={Zhong, Shipeng and Qi, Yuhua and Chen, Zhiqiang and Wu, Jin and Chen, Hongbo and Liu, Ming},
  journal={IEEE Sensors Journal}, 
  title={DCL-SLAM: A Distributed Collaborative LiDAR SLAM Framework for a Robotic Swarm}, 
  year={2024},
  volume={24},
  number={4},
  pages={4786-4797},
  keywords={Simultaneous localization and mapping;Laser radar;Sensors;Optimization;Feature extraction;Trajectory;Odometry;Collaborative localization;distributed framework;place recognition;range sensor},
  doi={10.1109/JSEN.2023.3345541}}
```

```
@ARTICLE{10740801,
  author={Feng, Dapeng and Qi, Yuhua and Zhong, Shipeng and Chen, Zhiqiang and Chen, Qiming and Chen, Hongbo and Wu, Jin and Ma, Jun},
  journal={IEEE Robotics and Automation Letters}, 
  title={S3E: A Multi-Robot Multimodal Dataset for Collaborative SLAM}, 
  year={2024},
  volume={9},
  number={12},
  pages={11401-11408},
  keywords={Simultaneous localization and mapping;Robot sensing systems;Synchronization;Trajectory;Global navigation satellite system;Collaboration;Accuracy;Motion capture;Robot localization;Multi-robot systems;Multi-robot SLAM;data sets for SLAM;SLAM},
  doi={10.1109/LRA.2024.3490402}}
```

## Acknowledgement

  - DCL-LIO-SAM adopt [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) as front end (Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping).

  - DCL-FAST-LIO adopt [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) as front end (Xu, Wei, Yixi Cai, Dongjiao He, Jiarong Lin, and Fu Zhang. Fast-lio2: Fast direct lidar-inertial odometry).

  - DCL-SLAM is based on a two-stage distributed Gauss-Seidel approach (Siddharth Choudhary and Luca Carlone and Carlos Nieto and John Rogers and Henrik I. Christensen and Frank Dellaert. Distributed Trajectory Estimation with Privacy and Communication Constraints: a Two-Stage Distributed Gauss-Seidel Approach).

  - DCL-SLAM is based on outlier rejection of [DOOR-SLAM](https://github.com/lajoiepy/robust_distributed_mapper) (Lajoie, Pierre-Yves and Ramtoula, Benjamin and Chang, Yun and Carlone, Luca and Beltrame, Giovanni. DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams).
