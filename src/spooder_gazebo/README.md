# spooder_gazebo

Gazebo simulation environment for the Spooder hexapod. Contains world files, robot spawn scripts, and ROS-Gazebo bridge configurations.

## Contents

- `worlds/` — Gazebo SDF world files
- `launch/` — Launch files for spawning robot, starting simulation
- `config/` — Bridge configuration files (ROS ↔ Gazebo)

## World Files

- `test_world.sdf` — Default test environment with simple obstacles
- `plain_world.sdf` — Flat plane (good for initial tests)
- `rough_terrain.sdf` — Hilly terrain (terrain navigation tests)
- `cave_world.sdf` — Cave-like environment (perception tests)
- `maze_world.sdf` — Maze with walls (path planning tests)
- `foothold_terrain.sdf` — Stepped terrain (foothold planning tests)

## Launch Files

| Launch | Purpose |
|--------|---------|
| `01_sim_world.launch.py` | Start Gazebo + bridge with selected world |
| `02_robot_spawn.launch.py` | Spawn robot + EKF + controllers |
| `03_navigation.launch.py` | Start Nav2 (custom variant) |
| `03_custom_nav.launch.py` | Custom Nav2 configuration |
| `04_viz.launch.py` | Start RViz with default configuration |
| `05_unified_bringup.launch.py` | All-in-one simulation bringup |
| `simulation.launch.py` | Full simulation with hexapod_nav |

## Bridge Configuration

The `config/bridge_config.yaml` defines ROS ↔ Gazebo topic mapping:

**Default topics (fallback config):**
- `/clock` — Simulation time
- `/imu` — IMU data
- `/odom` — Odometry
- `/cmd_vel` — Velocity commands
- `/camera/image` — RGB image
- `/camera/depth_image` — Depth image
- `/camera/points` — Point cloud
- `/camera/camera_info` — Camera info

**OAK-D Lite topics (simulation):**
- `/oak_d/points` — Stereo depth point cloud
- `/oak_d/rgb/image` — RGB image
- `/oak_d/depth/image` — Depth image
- `/oak_d/camera_info` — Camera info

The `01_sim_world.launch.py` generates a world-specific bridge config at runtime.

## Usage

### Start a Basic Simulation
```bash
ros2 launch spooder_gazebo 01_sim_world.launch.py world:=plain_world
```

### Spawn Robot
```bash
ros2 launch spooder_gazebo 02_robot_spawn.launch.py
```

### Full Simulation
```bash
ros2 launch spooder_gazebo simulation.launch.py
# Or use the unified script:
./scripts/run_oakd_lite.sh sim
```

## Configuration

### Spawn Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spawn_x` | 0.0 | X spawn position |
| `spawn_y` | 0.6 | Y spawn position |
| `spawn_z` | 3.0 | Z spawn position (high enough to drop onto terrain) |
| `spawn_yaw` | 0.0 | Yaw rotation |
| `use_hexapod_nav` | false | Use hexapod_nav gait controller instead of spooder_control |
| `use_sim_time` | true | Use simulation clock |

### Environment Variables

- `GZ_IP=127.0.0.1` — Force localhost Gazebo discovery
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` — Use FastDDS

## Headless Mode

For running without GUI (e.g., in CI or remote server):
```bash
ros2 launch spooder_gazebo 01_sim_world.launch.py headless:=true
```

## Troubleshooting

### Gazebo won't start
```bash
# Check Gazebo installation
gz sim --version
# Check ROS-Gazebo bridge
ros2 pkg list | grep ros_gz
```

### Robot doesn't spawn
```bash
# Check if URDF can be parsed
xacro src/spooder_description/urdf/spooder.xacro > /tmp/spooder.urdf
# Check Gazebo logs
ls ~/.gz/sim/logs/
```

### Bridge topics not publishing
```bash
ros2 topic list
# Check bridge config
ros2 launch spooder_gazebo 01_sim_world.launch.py world:=test_world --debug
```

## Dependencies

- `gazebo_ros` (ros-jazzy-ros-gz-bridge)
- `nav2_bringup`
- `robot_state_publisher`
- `xacro`
- `spooder_description`
