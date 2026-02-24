# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DDDMR Navigation (3D Mobile Robot Navigation) is a ROS 2 Jazzy navigation stack for mapping, localization, and autonomous navigation in 3D environments. It handles scenarios that Nav2 cannot: multi-layer floor mapping, ramp navigation, stereo structure path planning, and 3D point cloud perception. The stack targets C++17 and runs on Ubuntu 24.04 with PCL 1.15 and GTSAM 4.2a9.

## Build Commands

```bash
# Full build (run from workspace root)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace after building
source install/setup.bash

# Build a single package
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>

# Build a package and its dependencies
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <package_name>

# Run tests
colcon test
colcon test-result --verbose
```

## Docker Development

Docker images are built via `dddmr_docker/docker_file/build.bash` with variants:
- `x64` - Ubuntu 24.04 without GPU
- `x64_cuda` - With CUDA 12.6, TensorRT 10.7, cuDNN 9
- `x64_gz` - With Gazebo simulation
- `x64_l4t_r36` - NVIDIA Jetson L4T

### Cross-Machine Setup (Robot + Laptop)

The stack is split across two machines: robot runs SLAM/navigation, laptop runs rviz2.
Use the `justfile` at the repo root:

```bash
# SLAM bag playback (original demo)
just start-dddmr   # robot PC: compose down + up (launches lego_loam_bag)
just start-rviz    # laptop:   xhost + compose down + up (launches rviz2)

# Navigation with mapping (online SLAM + navigation, Ouster OS1-32)
just start-mapping        # robot PC: lego_loam + global_planner + p2p_move_base
just start-rviz-mapping   # laptop:   rviz2 with p2p_move_base_mapping.rviz

# Navigation with localization (MCL on pre-built map + navigation, Ouster OS1-32)
just start-localization        # robot PC: mcl_feature + mcl_3dl + global_planner + p2p_move_base
just start-rviz-localization   # laptop:   rviz2 with p2p_move_base_localization.rviz
```

**Compose files** in `dddmr_docker/`:
- `compose.robot.yaml` - robot container (no display, lego_loam_bag with use_rviz:=false)
- `compose.laptop.yaml` - laptop container (X11 + rviz2 with lego_loam.rviz)
- `compose.robot.mapping.yaml` - robot container for mapping+navigation (headless)
- `compose.robot.localization.yaml` - robot container for localization+navigation (headless)
- `compose.laptop.mapping.yaml` - laptop rviz2 for mapping mode
- `compose.laptop.localization.yaml` - laptop rviz2 for localization mode

**CycloneDDS** is required on both sides (`ros-jazzy-rmw-cyclonedds-cpp`). Config files:
- `dddmr_docker/config/cyclonedds_robot.xml` - uses `eth0`, peers: 10.15.20.{2,3,5,209}
- `dddmr_docker/config/cyclonedds_laptop.xml` - uses `wlx14ebb6a8f246`, same peers
- Env files `net_robot.env` / `net_laptop.env` set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **Any external service** (lidar driver, etc.) must also add the laptop IP (10.15.20.209) to its CycloneDDS peers list

**UDP buffer size**: large PointCloud2 messages (~3.6MB) require a larger receive buffer than the Linux default (212KB). Both compose files run `sysctl -w net.core.rmem_max=2147483647` before starting ROS nodes. Without this, rviz receives small topics (odometry, markers) but silently drops large point clouds.

**Network**: robot at 10.15.20.3 (eth0), laptop at 10.15.20.209 (wlx14ebb6a8f246), lidar at 10.15.20.5.

### Real Robot Setup (Lynx + Ouster OS1-32)

The navigation configs with `_os1` suffix are tuned for the real robot:
- **Lidar**: Ouster OS1-32, 512x10 config (32 beams, 512 horizontal, 10 Hz, ±22.5° vertical FOV)
- **Lidar topic**: `/ouster/points` (frame: `os_lidar`)
- **Odometry**: `/lynx/odometry/filtered` (EKF-fused wheel odometry)
- **TF frames**: `lynx/odom` → `lynx/base_link` → ... → `os_lidar` (published by robot, not by dddmr)
- **No static TF** in headless launch files — the robot's driver publishes base_link→lidar TF

**Config files** (in `dddmr_p2p_move_base/config/`):
- `p2p_move_base_mapping_os1.yaml` - mapping mode (uses `lego_loam_map`/`lego_loam_ground`)
- `p2p_move_base_localization_os1.yaml` - localization mode (uses `mapcloud`/`mapground` from MCL, requires `pose_graph_dir` to be set)
- `lego_loam_bor/config/loam_os1_config.yaml` - LeGO-LOAM params for OS1-32

**Headless launch files** (in `dddmr_p2p_move_base/launch/`):
- `p2p_move_base_mapping_headless.launch` - lego_loam + global_planner + p2p_move_base
- `p2p_move_base_localization_headless.launch` - mcl_feature + mcl_3dl + global_planner + p2p_move_base

**Workflow**: map first with `just start-mapping`, save the pose graph, then set `pose_graph_dir` in the localization config and use `just start-localization`.

## Architecture

### Navigation Pipeline

```
p2p_move_base (FSM navigation controller)
  ├── global_planner (A* on graph-based 3D map)
  ├── local_planner (MPC-based trajectory evaluation)
  │     ├── trajectory_generators (plugin-based trajectory generation)
  │     ├── mpc_critics (plugin-based cost evaluation)
  │     └── recovery_behaviors (plugin-based recovery actions)
  ├── perception_3d (3D obstacle detection, marking/clearing)
  └── mcl_3dl (3D Monte Carlo Localization with submaps)
```

### ROS 2 Packages (under `src/`)

| Package | Role |
|---------|------|
| `dddmr_sys_core` | Core enums, base classes, action definitions (GetPlan, PToPMoveBase, RecoveryBehaviors, TagDocking) |
| `dddmr_perception_3d` | Pluggable perception layers: multi-layer lidar, depth cameras, speed/no-entry zones |
| `dddmr_global_planner` | A* path planning on static ground graphs + dynamic sensor graphs |
| `dddmr_local_planner/` | MPC trajectory planning (split into `base_trajectory`, `local_planner`, `mpc_critics`, `trajectory_generators`, `recovery_behaviors`) |
| `dddmr_p2p_move_base` | FSM controller integrating global planner, local planner, and recovery |
| `dddmr_mcl_3dl` | 3D particle filter localization with submap support and pose graph integration |
| `dddmr_lego_loam/` | SLAM system (LeGO-LOAM variant) with loop closure; contains `lego_loam_bor` and `cloud_msgs` |
| `dddmr_odom_3d` | 3D odometry estimation |
| `dddmr_rviz_tools/` | RViz plugins: 3D pose tools, mapping panel, map editor, pose graph editors |
| `dddmr_semantic_segmentation` | DDRNet semantic segmentation via TensorRT on RGB-D |
| `dddmr_trt` | Shared TensorRT/YOLO inference library |

### Key Design Patterns

- **Plugin architecture** via `pluginlib` for perception layers, trajectory generators, MPC critics, and recovery behaviors.
- **All planning operates in 3D point cloud space** (PCL). Ground maps are graph-based, not 2D occupancy grids.
- **Submap-based localization** uses rolling windows for scalability on large maps.
- **GTSAM** is used for pose graph optimization in both SLAM and localization.
- **Action-based interfaces**: long-running tasks use ROS 2 actions defined in `dddmr_sys_core/action/`.

### Key Launch Files

- `ros2 launch lego_loam_bor lego_loam.launch` - Online SLAM
- `ros2 launch lego_loam_bor lego_loam_bag.launch` - Offline SLAM from bag
- `ros2 launch mcl_3dl mcl_3dl.launch` - 3D localization
- `ros2 launch p2p_move_base p2p_move_base_mapping_headless.launch` - Mapping + navigation (OS1-32, headless)
- `ros2 launch p2p_move_base p2p_move_base_localization_headless.launch` - Localization + navigation (OS1-32, headless)
- `ros2 launch p2p_move_base go2_localization.launch` - Full navigation (Gazebo Go2)
- `ros2 launch perception_3d perception_3d_ros.launch` - Perception only
- `ros2 launch local_planner local_planner_play_ground.launch` - Local planner test

### Configuration

YAML parameter files live alongside each package in `config/` directories. Key configs:
- `lego_loam_bor/config/` - Lidar specs, loop closure parameters
- `dddmr_mcl_3dl/config/` - Localization particle filter tuning
- `dddmr_perception_3d/config/` - Perception layer definitions
- `dddmr_p2p_move_base/config/` - Navigation system parameters

### Robot Requirements

The stack expects these ROS 2 interfaces from the robot:
- `/cmd_vel` (geometry_msgs/msg/Twist) - velocity commands
- Odometry (nav_msgs/msg/Odometry) - odometry with TF (default `/odom`, Lynx uses `/lynx/odometry/filtered`)
- Point cloud topic (sensor_msgs/msg/PointCloud2) - from multi-layer lidar (default `/lslidar_point_cloud`, Lynx uses `/ouster/points`)
- TF tree: odom_frame -> base_link_frame -> sensor frames (Lynx: `lynx/odom` -> `lynx/base_link` -> `os_lidar`)
