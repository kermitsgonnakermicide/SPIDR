# Spooder: OAK-D Lite Hexapod Robot — Full Simulation + Hardware

A ROS 2 Jazzy research platform for terrain-aware hexapod locomotion using the OAK-D Lite stereo depth camera. The robot navigates complex terrain by building a 3D OctoMap from depth data, computing per-cell terrain cost (slope, roughness, clearance), and planning adaptive footholds in real-time.

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Robot | Spooder (custom 6-legged hexapod) | Locomotion platform |
| Depth Camera | Luxonis OAK-D Lite | Stereo depth perception (no LiDAR) |
| IMU | BMI270 (on OAK-D Lite) | Orientation + motion |
| Compute | Jetson/PC with USB 3.0 | ROS 2 processing |
| Actuators | 18× ST3215 servos (3 per leg) | Joint control |

### OAK-D Lite Specifications
- **Stereo baseline:** 7.5 cm
- **Depth range:** 0.2 – 10.0 m
- **Resolution:** 640×400 (depth), 1280×720 (RGB)
- **FOV:** ~71° horizontal
- **Frame rate:** 30 FPS (depth), 30 FPS (RGB)
- **Onboard processing:** VPU for stereo matching

## Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Nav2 (spooder_navigation)                                  │
│  ├── Path planning through costmap                          │
│  ├── Trajectory rollout                                     │
│  └── OctoMap goal/trajectory planners                        │
└──────────────────────┬──────────────────────────────────────┘
                       │ /cmd_vel
┌──────────────────────┴──────────────────────────────────────┐
│  Gait Controller (hexapod_nav)                              │
│  └── Tripod/wave gaits, adaptive step height                │
└──────────────────────┬──────────────────────────────────────┘
                       │ Joint commands
┌──────────────────────┴──────────────────────────────────────┐
│  ros2_control + Gazebo Sim                                  │
└──────────────────────┬──────────────────────────────────────┘
                       │ /tf, /joint_states
┌──────────────────────┴──────────────────────────────────────┐
│  Perception Pipeline (spooder_perception + hexapod_nav)     │
│  ├── Pointcloud Optimizer  (voxel filter, scene change)     │
│  ├── OctoMap Server        (3D occupancy mapping)           │
│  ├── Terrain Extractor     (floor/ceiling from voxels)     │
│  ├── Cost Computation      (slope, roughness, clearance)   │
│  └── Foothold Planner      (IK reachability + cost query)  │
└──────────────────────┬──────────────────────────────────────┘
                       │ /oak_d/points
┌──────────────────────┴──────────────────────────────────────┐
│  OAK-D Lite (depthai_ros_driver)                            │
│  ├── Stereo depth → PointCloud2                             │
│  ├── RGB camera → Image                                     │
│  └── IMU → Imu messages                                     │
└─────────────────────────────────────────────────────────────┘
```

## Packages

| Package | Role |
|---------|------|
| `spooder_description` | URDF xacros, sensor macros, RViz config |
| `spooder_gazebo` | Gazebo worlds, robot spawn, ROS-Gazebo bridge |
| `spooder_perception` | OAK-D Lite driver, point cloud processing, terrain analysis |
| `spooder_navigation` | Nav2 bringup, OctoMap-based goal/trajectory planners |
| `hexapod_nav` | Terrain pipeline (cost, foothold planning, gait control) |
| `spooder_control` | ros2_control hardware interface for ST3215 servos |
| `spooder_hardware` | Hardware interface for real servos |

## Quick Start

### Prerequisites
- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy (`/opt/ros/jazzy/setup.bash`)
- Gazebo Harmonic
- `depthai_ros_driver` (apt: `ros-jazzy-depthai-ros-driver`)
- `octomap_server`, `nav2_bringup`, `grid_map_*`

### Build
```bash
cd ~/spooder_ws
colcon build --symlink-install \
  --packages-select \
    spooder_description \
    spooder_gazebo \
    spooder_perception \
    spooder_navigation \
    spooder_control \
    hexapod_nav
source install/setup.bash
```

### Run Full Spooder Simulation
```bash
./scripts/run_oakd_lite.sh sim
# Or with bag recording:
./scripts/run_oakd_lite.sh sim --record
```
This starts: Gazebo + Spooder hexapod + OAK-D Lite Gazebo plugin + OctoMap + terrain pipeline + Nav2.

### Just Gazebo + Robot (no perception/Nav2)
```bash
./scripts/run_oakd_lite.sh sim_only
```

### Run on Hardware
```bash
# One-time: setup USB permissions
sudo ./scripts/setup_oakd_permissions.sh

# Run driver
./scripts/run_oakd_lite.sh hardware
```

## Topic Reference

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/oak_d/points` | PointCloud2 | 30 Hz | depthai_ros_driver |
| `/oak_d/stereo/image` | Image (depth) | 30 Hz | depthai_ros_driver |
| `/oak_d/rgb/image` | Image (RGB) | 30 Hz | depthai_ros_driver |
| `/oak_d/imu` | Imu | 100 Hz | depthai_ros_driver |
| `/oak_d/points/optimized` | PointCloud2 | 2–10 Hz | spooder_perception |
| `/octomap_binary` | Octomap | 1 Hz | octomap_server |
| `/projected_map` | OccupancyGrid | 1 Hz | octomap_server |
| `/terrain_cost_map` | GridMap | 1–5 Hz | hexapod_nav |
| `/foothold_plan` | visualization_msgs/Marker | 5 Hz | hexapod_nav |
| `/odom` | Odometry | 50 Hz | Gazebo/EKF |
| `/tf`, `/tf_static` | TFMessage | varies | robot_state_publisher |
| `/cmd_vel` | Twist | varies | Nav2 |
| `/joint_states` | JointState | 100 Hz | ros2_control |

## Research Workflow

1. **Develop in simulation** — Test new algorithms with `mode:=simulation`
2. **Record data** — Use `--record` flag or `./scripts/record_experiment.sh -d 60`
3. **Analyze** — `python3 ./scripts/analyze_bag.py ~/bags/exp_*/`
4. **Validate on hardware** — Deploy to real OAK-D Lite with `mode:=hardware`
5. **Compare sim/hardware** — Same topic interface ensures direct comparison

See `documentation/EXPERIMENTS.md` for detailed experimental design.

## Sim → Hardware Transfer

The OAK-D Lite topic interface is identical in simulation and hardware:
- `/oak_d/points` (PointCloud2)
- `/oak_d/depth/image` (Image)
- `/oak_d/rgb/image` (Image)
- `/oak_d/imu` (Imu)
- `/oak_d/camera_info` (CameraInfo)

The TF tree is also identical:
```
spooder/base_link
  └─ oak_d_base_link
       ├─ oak_d_left_camera_link → oak_d_left_camera_optical
       ├─ oak_d_right_camera_link → oak_d_right_camera_optical
       ├─ oak_d_rgb_camera_link → oak_d_rgb_camera_optical
       └─ oak_d_imu_link
```

## Troubleshooting

### OAK-D not detected on Linux
```bash
lsusb | grep 03e7   # Luxonis vendor ID
sudo ./scripts/setup_oakd_permissions.sh
```

### Gazebo black screen
```bash
# Force software rendering
export LIBGL_ALWAYS_SOFTWARE=1
# Or update GPU drivers
```

### Topics not publishing
```bash
ros2 topic list
ros2 node list
ros2 run rqt_tf_tree rqt_tf_tree
```

### Build errors
```bash
rm -rf build install log
colcon build --symlink-install --packages-select <package>
```

## License

MIT

## Maintainer

Daksh Vohra <daksh.vohra1@gmail.com>

## Citation

If you use this codebase in research, please cite the relevant publications (see `documentation/` for specific paper references).
