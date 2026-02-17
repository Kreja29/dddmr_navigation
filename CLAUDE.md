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
- `/odom` (nav_msgs/msg/Odometry) - odometry with TF
- Point cloud topic (sensor_msgs/msg/PointCloud2) - from multi-layer lidar
- TF tree: map -> odom -> base_link -> sensor frames
