# Navigation Stack Docker Launch Plan (Mapping + Localization Modes)

## Context

Run the full dddmr navigation stack on the real robot (Ouster OS1-32 lidar) via simple `just` commands. Two modes:
1. **Mapping mode** — online SLAM + navigation (build a map)
2. **Localization mode** — MCL localization on a pre-built map + navigation

Robot runs headless; rviz runs on a separate laptop container. Ouster driver and robot odometry run in a separate container — the navigation container only subscribes via CycloneDDS.

### Robot Setup
- **Lidar**: Ouster OS1-32, 512×10 config (32 beams, 512 horizontal, 10 Hz, ±22.5° vertical FOV)
- **Topic**: `/ouster/points` (frame: `os_lidar`)
- **TF**: Published externally. Frame chain: `lynx/odom → lynx/base_link → ... → os_lidar`
- **Odometry**: `/lynx/odometry/filtered` (EKF-fused wheel odometry)
- **Base frame**: `lynx/base_link`, **odom frame**: `lynx/odom`, **map frame**: `map`

---

## Files to Create

### 1. Headless launch files (no rviz, no static TF)

**`src/dddmr_p2p_move_base/launch/p2p_move_base_mapping_headless.launch`**
- Based on `p2p_move_base_mapping.launch` but:
  - Remove rviz2 node entirely
  - Remove static_transform_publisher (TF comes from robot externally)
  - Use new config: `p2p_move_base_mapping_os1.yaml`
  - Add remap: `/lslidar_point_cloud` → `/ouster/points`

**`src/dddmr_p2p_move_base/launch/p2p_move_base_localization_headless.launch`**
- Based on `p2p_move_base_localization.launch` but:
  - Remove rviz2 node entirely
  - Remove static_transform_publisher
  - Use new config: `p2p_move_base_localization_os1.yaml`
  - Add remap: `/lslidar_point_cloud` → `/ouster/points`
  - Add remap: `/odom` → `/lynx/odometry/filtered`

### 2. Config files (tuned for Ouster OS1-32 + lynx frame names)

**`src/dddmr_p2p_move_base/config/p2p_move_base_mapping_os1.yaml`**
- Based on `p2p_move_base_mapping.yaml` with:
  - All `base_link` → `lynx/base_link`
  - Lidar params: `num_vertical_scans: 32`, `num_horizontal_scans: 512`, `scan_period: 0.1`
  - Vertical FOV: `vertical_angle_bottom: -22.5`, `vertical_angle_top: 22.5`
  - Perception vertical FOV: `vertical_FOV_top: 22.5`, `vertical_FOV_bottom: -22.5`
  - Ground FOV: update `ground_fov_bottom` and `ground_fov_top` for OS1-32 geometry

**`src/dddmr_p2p_move_base/config/p2p_move_base_localization_os1.yaml`**
- Based on `p2p_move_base_localization.yaml` with:
  - All `base_link` → `lynx/base_link`
  - All `odom` frame references → `lynx/odom`
  - MCL odom_type: `"wheel_odometry"` (using robot's filtered EKF odometry)
  - Same lidar params as mapping config above
  - `pose_graph_dir`: placeholder path (user sets after first SLAM run)

**`src/dddmr_lego_loam/lego_loam_bor/config/loam_os1_config.yaml`**
- Based on `loam_c16_config.yaml` with OS1-32 lidar parameters:
  - `num_vertical_scans: 32`, `num_horizontal_scans: 512`, `scan_period: 0.1`
  - Vertical angles: `-22.5` to `22.5`
  - Ground FOV adjusted for wider vertical spread
  - `baselink_frame: "lynx/base_link"`

### 3. Compose files

**`dddmr_docker/compose.robot.mapping.yaml`**
- Based on `compose.robot.yaml`
- Command: `ros2 launch p2p_move_base p2p_move_base_mapping_headless.launch`
- Same volumes, env, network, sysctl settings

**`dddmr_docker/compose.robot.localization.yaml`**
- Based on `compose.robot.yaml`
- Command: `ros2 launch p2p_move_base p2p_move_base_localization_headless.launch`

**`dddmr_docker/compose.laptop.mapping.yaml`**
- Based on `compose.laptop.yaml`
- Command: `ros2 run rviz2 rviz2 -d /root/dddmr_navigation/src/dddmr_p2p_move_base/rviz/p2p_move_base_mapping.rviz`

**`dddmr_docker/compose.laptop.localization.yaml`**
- Based on `compose.laptop.yaml`
- Command: `ros2 run rviz2 rviz2 -d /root/dddmr_navigation/src/dddmr_p2p_move_base/rviz/p2p_move_base_localization.rviz`

### 4. Updated justfile

New commands:
```
start-mapping:
    docker compose -f {{compose_dir}}/compose.robot.mapping.yaml down
    docker compose -f {{compose_dir}}/compose.robot.mapping.yaml up

start-localization:
    docker compose -f {{compose_dir}}/compose.robot.localization.yaml down
    docker compose -f {{compose_dir}}/compose.robot.localization.yaml up

start-rviz-mapping:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.mapping.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.mapping.yaml up

start-rviz-localization:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.localization.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.localization.yaml up
```

---

## Summary of All Files

| Action | File |
|--------|------|
| **Create** | `src/dddmr_p2p_move_base/launch/p2p_move_base_mapping_headless.launch` |
| **Create** | `src/dddmr_p2p_move_base/launch/p2p_move_base_localization_headless.launch` |
| **Create** | `src/dddmr_p2p_move_base/config/p2p_move_base_mapping_os1.yaml` |
| **Create** | `src/dddmr_p2p_move_base/config/p2p_move_base_localization_os1.yaml` |
| **Create** | `src/dddmr_lego_loam/lego_loam_bor/config/loam_os1_config.yaml` |
| **Create** | `dddmr_docker/compose.robot.mapping.yaml` |
| **Create** | `dddmr_docker/compose.robot.localization.yaml` |
| **Create** | `dddmr_docker/compose.laptop.mapping.yaml` |
| **Create** | `dddmr_docker/compose.laptop.localization.yaml` |
| **Modify** | `justfile` |

## Key Config Tuning Notes

- **Cuboid collision model** — keep existing dimensions for now (tune to actual robot size later)
- **Velocity limits** — keep existing (max_vel_x: 1.0, max_vel_theta: 0.6) as starting point
- **pose_graph_dir** — localization config will have a placeholder; update after first SLAM run
- **Remaps** — `/lslidar_point_cloud` → `/ouster/points` and `/odom` → `/lynx/odometry/filtered` in launch files

## Things to Watch Out For

1. **CycloneDDS peers**: The Ouster driver container must also have the navigation container's IP in its CycloneDDS peers list (and vice versa)
2. **Large point cloud UDP buffers**: Already handled by sysctl in compose files
3. **LeGO-LOAM ground segmentation**: OS1-32 has wider vertical spread (45° vs 30° for C16), `ground_fov_bottom`/`ground_fov_top` need careful tuning — ground rings typically in lower ~5-15° range
4. **Frame `os_lidar`**: LeGO-LOAM reads `baselink_frame` from config and uses TF to find the sensor frame from the point cloud's `frame_id`. As long as `os_lidar → lynx/base_link` TF exists (published by robot), it will work

## Verification

1. **Build**: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select p2p_move_base lego_loam_bor`
2. **Mapping mode test**:
   - Start Ouster driver + robot odom in their own container
   - `just start-mapping` on robot PC
   - `just start-rviz-mapping` on laptop
   - Verify: point cloud visible, SLAM map building, can click goals and robot navigates
3. **Localization mode test**:
   - Set `pose_graph_dir` in localization config to saved map
   - `just start-localization` on robot PC
   - `just start-rviz-localization` on laptop
   - Verify: map loaded, robot localized, can navigate to goals
