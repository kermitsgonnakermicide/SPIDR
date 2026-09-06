# spooder_perception

Perception package for the Spooder hexapod robot. Owns the OAK-D Lite camera driver, point cloud processing, and terrain analysis.

## Contents

- `launch/` — Launch files for camera bringup
- `config/` — OAK-D Lite configuration files
- `spooder_perception/` — Python node implementations
- `urdf/` — URDF macros (if needed)

## Nodes

| Executable | Purpose |
|------------|---------|
| `pointcloud_optimizer` | Voxel-based scene change detection, downsampling |
| `pointcloud_saver` | Persistent map saving (PCD format) |
| `terrain_analyzer` | Real-time terrain classification from point cloud |
| `octomap_terrain_adapter` | Bridge between OctoMap and hexapod_nav terrain pipeline |

## Launch Files

### `oakd_bringup.launch.py` (Canonical Entry Point)

Unified launch that handles both simulation and hardware:

```bash
# Simulation
ros2 launch spooder_perception oakd_bringup.launch.py mode:=simulation

# Hardware
ros2 launch spooder_perception oakd_bringup.launch.py mode:=hardware

# With bag recording
ros2 launch spooder_perception oakd_bringup.launch.py mode:=simulation record:=true

# Without OctoMap pipeline
ros2 launch spooder_perception oakd_bringup.launch.py mode:=hardware use_octomap:=false
```

**Arguments:**
- `mode` — `simulation` | `hardware` | `simulation_only` (default: simulation)
- `record` — Enable ROS 2 bag recording (default: false)
- `use_octomap` — Run OctoMap terrain pipeline (default: true)
- `world` — Gazebo world name (simulation only)
- `spawn_x/y/z` — Robot spawn position (simulation only)

### `oakd_driver.launch.py`

Hardware-only launch for the OAK-D Lite driver:
```bash
ros2 launch spooder_perception oakd_driver.launch.py
```

### `oakd_sim.launch.py`

Simulation-only launch (perception pipeline for Gazebo output):
```bash
ros2 launch spooder_perception oakd_sim.launch.py
```

## Configuration Files

### `oakd_lite.yaml`

Main configuration for the OAK-D Lite driver. Contains:
- Camera intrinsics
- Stereo depth parameters
- IMU configuration
- TF frame setup

### `oakd_sim.yaml`

Simulation-specific parameters. Maps Gazebo topics to match the real hardware interface.

### `oakd_calibration.yaml`

Calibration parameters (intrinsics, extrinsics). Default values are from OAK-D Lite factory calibration. For custom calibration, use the `depthai-calibration` tool.

## OAK-D Lite Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak_d/points` | PointCloud2 | Stereo depth point cloud |
| `/oak_d/left/image_raw` | Image | Left grayscale image |
| `/oak_d/right/image_raw` | Image | Right grayscale image |
| `/oak_d/rgb/image_raw` | Image | RGB image (1280×720) |
| `/oak_d/stereo/image_raw` | Image | Depth image (640×400) |
| `/oak_d/imu` | Imu | 6-DOF IMU data |
| `/oak_d/left/camera_info` | CameraInfo | Left camera intrinsics |
| `/oak_d/right/camera_info` | CameraInfo | Right camera intrinsics |
| `/oak_d/rgb/camera_info` | CameraInfo | RGB camera intrinsics |

## Perception Pipeline

The package implements a multi-stage perception pipeline:

1. **Capture** — depthai_ros_driver publishes raw depth/RGB/IMU
2. **Optimize** — `pointcloud_optimizer` reduces noise and detects scene changes
3. **Map** — `octomap_server` builds 3D occupancy grid
4. **Analyze** — `octomap_terrain_adapter` extracts floor/ceiling + classifies
5. **Output** — `/projected_map` (2D occupancy) + `/terrain_cost_map` (heightmap)

## Hardware Setup

### USB Permissions

The OAK-D Lite requires USB 3.0 access. Run once:
```bash
sudo ./scripts/setup_oakd_permissions.sh
```

### Verify Hardware Detection
```bash
lsusb | grep 03e7   # Should show "Luxonis"
ros2 launch spooder_perception oakd_driver.launch.py
ros2 topic list | grep oak_d   # Should show OAK-D topics
```

## Hardware vs. Simulation

| Aspect | Hardware | Simulation |
|--------|----------|------------|
| Topic names | Identical | Identical |
| Frame names | Identical | Identical |
| Data source | depthai_ros_driver (real) | Gazebo depth_camera plugin |
| TF tree | Published by depthai_ros_driver | Published by robot_state_publisher |
| IMU | BMI270 from OAK-D | Gazebo IMU sensor |
| Latency | ~30ms | ~1ms |

The topic interface is identical, so the rest of the perception/navigation stack is unaware of whether data is from real hardware or simulation.

## Dependencies

- `depthai_ros_driver` (ros-jazzy-depthai-ros-driver)
- `octomap_server` (ros-jazzy-octomap-server)
- `octomap_msgs`
- `grid_map_msgs`
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `tf2_ros`, `tf2_sensor_msgs`
- `rclpy`, `std_srvs`

## Troubleshooting

### Camera not detected
```bash
lsusb
sudo ./scripts/setup_oakd_permissions.sh
# Try a different USB 3.0 port
```

### Topics not publishing
```bash
ros2 node info /oak_d
# Check depthai_ros_driver logs
DEPTHAI_DEBUG=1 ros2 launch spooder_perception oakd_driver.launch.py
```

### Low framerate
```bash
# Check USB speed
lsusb -t
# Should show "5000M" for USB 3.0
```

### Point cloud noisy
```yaml
# In oakd_lite.yaml, increase depth quality
stereo:
  i_depth_preset: 'HIGH_ACCURACY'  # or 'HIGH_DENSITY'
  i_subpixel: true
  i_lr_check: true
```
