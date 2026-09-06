# hexapod_nav

OctoMap-first hexapod 3D navigation with dynamic foothold placement. The terrain-aware pipeline that makes the hexapod walk over complex terrain.

## Contents

- `launch/` — Pipeline launch files
- `config/` — OctoMap, terrain cost, and foothold parameters
- `hexapod_nav/` — Python node implementations
- `rviz/` — RViz visualization configurations

## Nodes

| Executable | Purpose |
|------------|---------|
| `octomap_terrain_node` | Extract floor/ceiling from OctoMap voxels |
| `terrain_cost_node` | Compute terrain cost (slope, roughness, clearance) |
| `foothold_planner_node` | IK-based foothold selection with terrain cost |
| `gait_controller_node` | Tripod/wave gait execution |
| `robot_state_viz` | RViz visualization of terrain state |
| `rerun_bridge` | Bridge to Rerun.io for visualization |

## Launch Files

| Launch | Purpose |
|--------|---------|
| `simulation.launch.py` | Run hexapod_nav on top of existing sim |
| `full_simulation.launch.py` | Self-contained sim with Gazebo + hexapod_nav |
| `full_pipeline.launch.py` | Hardware: OAK-D → terrain → Nav2 (delegated to spooder_perception) |

## Terrain Pipeline Flow

```
/oak_d/points (PointCloud2)
    │
    ▼
octomap_server (octomap_server)
    │ /octomap_binary (full 3D OctoMap)
    ▼
octomap_terrain_node (hexapod_nav)
    │ /octomap_point_cloud_centers
    ▼
terrain_cost_node (hexapod_nav)
    │ /terrain_cost_map (GridMap: height, slope, roughness, clearance)
    ▼
foothold_planner_node (hexapod_nav)
    │ /foothold_plan (MarkerArray)
    ▼
gait_controller_node (hexapod_nav)
    │ /cmd_vel (Twist)
    ▼
ros2_control (Gazebo/hardware)
```

## Configuration Files

### `config/octomap_params.yaml`
- **Resolution:** 5 cm voxels
- **Max range:** 5 m sensor range
- **Ground filter:** OFF (floor voxels kept for terrain analysis)
- **Occupancy filters:** 0.08–8.0 m height band

### `config/foothold_params.yaml`
- **Cost weights:** slope, roughness, clearance, IK reachability
- **Leg kinematics:** coxa/femur/tibia lengths
- **Gait parameters:** step height, step length, swing duration

### `config/nav2_params.yaml`
Nav2 parameters consuming `/projected_map` from octomap_server.

## Usage

```bash
# Run on existing simulation
ros2 launch hexapod_nav simulation.launch.py

# Full self-contained simulation
ros2 launch hexapod_nav full_simulation.launch.py world:=rough_terrain
```

## Parameters

### Terrain Cost Weights
| Parameter | Default | Description |
|-----------|---------|-------------|
| `slope_weight` | 2.0 | Cost weight for slope |
| `roughness_weight` | 1.5 | Cost weight for roughness |
| `clearance_weight` | 1.0 | Cost weight for obstacle clearance |
| `reachability_weight` | 3.0 | Cost weight for IK validity |

### Foothold Planner
| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_step_height` | 0.15 | Max climbable step (m) |
| `max_slope` | 0.35 | Max traversable slope (rad) |
| `search_radius` | 0.15 | Foot search radius (m) |

## Dependencies

- `octomap_msgs`, `octomap_server`
- `grid_map_msgs`, `grid_map_ros`
- `nav2_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `visualization_msgs`
- `spooder_perception` (for OAK-D driver)
