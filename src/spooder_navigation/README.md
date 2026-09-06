# spooder_navigation

Nav2 bringup and custom planners for the Spooder hexapod robot.

## Contents

- `launch/` — Nav2 launch files
- `config/` — Nav2 and SLAM configuration
- `spooder_navigation/` — Custom Nav2 plugins

## Nodes / Plugins

| Node / Plugin | Purpose |
|---------------|---------|
| `navigation.launch.py` | Nav2 stack bringup |
| `slam.launch.py` | SLAM Toolbox for mapping |
| `octomap_goal_planner` | Navigate to goals in 3D terrain |
| `octomap_trajectory_planner` | Plan trajectories avoiding obstacles |
| `unstuck_monitor` | Detect and recover from stuck states |

## Launch Files

| Launch | Purpose |
|--------|---------|
| `navigation.launch.py` | Nav2 bringup with OctoMap costmaps |
| `slam.launch.py` | SLAM Toolbox (used when LiDAR is present) |

## Configuration

### `config/nav2_params.yaml`
Nav2 parameters tuned for hexapod terrain navigation:
- **Costmaps:** Use `/projected_map` from OctoMap as static layer
- **Planner:** RPP (Rrt*, Potential Field) for terrain avoidance
- **Controller:** TEB or DWB adapted for slow hexapod speeds

### `config/slam_toolbox_params.yaml`
SLAM Toolbox configuration (requires LiDAR).

## Usage

```bash
# Start Nav2 with OctoMap costmaps
ros2 launch spooder_navigation navigation.launch.py use_sim_time:=true

# With custom params
ros2 launch spooder_navigation navigation.launch.py \
  params_file:=config/custom_nav2_params.yaml
```

## Dependencies

- `nav2_bringup`
- `nav2_msgs`
- `octomap_server`
- `grid_map_msgs`
- `spooder_description`
