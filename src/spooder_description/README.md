# spooder_description

Robot description package for the Spooder hexapod robot. Contains URDF xacros, sensor macros, and Gazebo configurations.

## Contents

- `urdf/` — Xacro files defining the robot geometry and sensors
- `launch/` — Launch files for robot description and RViz visualization
- `config/` — Configuration files for sensors and hardware

## URDF Structure

```
spooder.xacro           # Main robot URDF (assembles all components)
├── gazebo_oakd.plugin.xacro  # OAK-D Lite Gazebo simulation plugin
├── lidar.xacro          # LiDAR sensor (gpu_lidar)
├── imu.xacro            # IMU sensor (gz_imu)
└── oakd_lite.urdf.xacro # OAK-D Lite base frame definitions
```

## Sensors

### OAK-D Lite Camera
The OAK-D Lite stereo depth camera is the primary perception sensor:
- **URDF:** `gazebo_oakd.plugin.xacro` (simulation) / `oakd_lite.urdf.xacro` (hardware base frames)
- **Parameters:**
  - Baseline: 7.5 cm
  - FOV: ~71° horizontal
  - Depth range: 0.2–10 m
- **Topics (simulation):**
  - `/oak_d/points` — PointCloud2 from depth camera
  - `/oak_d/rgb/image` — RGB camera image
  - `/oak_d/camera_info` — Camera intrinsics
- **TF tree:** `spooder/base_link → oak_d_base_link → oak_d_*_camera_link → oak_d_*_camera_optical`

### LiDAR (Optional)
- **URDF:** `lidar.xacro`
- **Type:** gpu_lidar (360° scan)
- **Range:** 0.12–10 m
- **Topic:** `/scan`

### IMU
- **URDF:** `imu.xacro`
- **Plugin:** Gazebo IMU sensor
- **Topic:** `/imu`

## Usage

### Visualize in RViz
```bash
ros2 launch spooder_description display.launch.py
```

### With OAK-D URDF
```bash
ros2 launch spooder_description oakd_lite.launch.py
```

### Spawn in Gazebo (via spooder_gazebo)
```bash
ros2 launch spooder_gazebo 02_robot_spawn.launch.py
```

## Configuration Files

- `config/oak_run.yaml` — OAK-D run parameters (legacy)
- `rviz/config.rviz` — Default RViz configuration

## Modification Guide

### Adding a New Sensor

1. Create a new `sensor_name.xacro` file in `urdf/`
2. Define a xacro macro with the sensor geometry and Gazebo plugin
3. Include the xacro in `spooder.xacro`
4. Instantiate the macro with appropriate origin parameters
5. Update the TF tree in launch files if needed

### Changing OAK-D Position

Edit the origin parameters in `spooder.xacro`:
```xml
<xacro:gazebo_oakd_sensor parent_link="base_link"
    origin_xyz="0.095 0 0.08"
    origin_rpy="0 0 0"/>
```

## Dependencies

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `depthai_descriptions` (for hardware URDF)
