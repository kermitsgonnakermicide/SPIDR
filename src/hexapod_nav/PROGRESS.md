# Progress — hexapod_nav Pipeline Implementation

## Status: COMPLETE + integration fixes (updated 2026-07-22)

All 4 pipeline nodes, kinematics, config files, launch files, and tests are implemented and passing.
Build: clean. Tests: 87/87 passing.

---

## What Was Built

### The Guide's Novel Contribution (Implemented)

**Dual-layer terrain cost coupling OctoMap 3D occupancy with foothold cost:**

1. **`unknown_above` layer** (`octomap_terrain_node.py`):
   - For each occupied floor voxel, count occupied voxels above it within `robot_body_height + 0.1m`
   - `unknown_above = 1.0 - min(1.0, overhead_count / column_voxels)` → continuous [0.0, 1.0]
   - 0.0 = column fully occupied (well-observed, likely a ceiling)
   - 1.0 = no overhead voxels (fully unknown above)
   - Published as 4th layer on `terrain_grid_map` topic

2. **`unknown_above` consumption** (`terrain_cost_node.py`):
   - **Strict mode** (`strict_unknown: true`): any `unknown_above > 0` → INF cost (conservative)
   - **Non-strict** (`strict_unknown: false`): `cost += w_u * unknown_above_fraction` (continuous penalty)
   - Configurable via `foothold_params.yaml` → `strict_unknown` and `w_unknown`

3. **AEP/PEP workspace restriction** (`foothold_planner_node.py`):
   - `_aep_pep_filter()`: transforms world candidates to coxa frame via leg angle rotation
   - Rejects if `coxa_x < pep_limit` (behind coxa joint) or `coxa_x > aep_limit` (beyond reach)
   - `pep_limit = COXA_LEN + pep_offset`, `aep_limit = COXA_LEN + FEMUR_LEN + TIBIA_LEN - aep_offset`
   - `pep_offset` and `aep_offset` are now configurable via `foothold_params.yaml`

4. **Swing phase coordination** (`gait_controller_node.py` → `foothold_planner_node.py`):
   - Gait controller publishes `/leg_phase` (UInt8MultiArray, 6 values: 0=STANCE, 1=SWING) every control tick
   - Foothold planner subscribes to `/leg_phase`, detects STANCE→SWING transitions (triggers foothold selection) and SWING→STANCE transitions (clears committed targets)
   - Enables dynamic replanning: `costmap_callback` checks committed target cost for SWING legs and replans if delta exceeds threshold

### Nodes

| Node | File | Purpose |
|------|------|---------|
| OctoMap Terrain | `octomap_terrain_node.py` | OctoMap → 4-layer GridMap (floor/ceiling/clearance/unknown_above) |
| Terrain Cost | `terrain_cost_node.py` | GridMap → cost layer (slope + roughness + clearance + unknown_above penalty) |
| Foothold Planner | `foothold_planner_node.py` | Body-path + cost + terrain → 6 foothold targets (with AEP/PEP filter) |
| Gait Controller | `gait_controller_node.py` | Footholds → joint commands (bezier swing arcs, tripod gait, ros2_control) |

### Kinematics (`kinematics.py`)
- 3-DOF IK (coxa-femur-tibia) with Diddler-specific values
- `COXA_LEN=0.043`, `FEMUR_LEN=0.060`, `TIBIA_LEN=0.104`
- Frame ID: `spooder/base_footprint`
- Joint command: `Float64MultiArray` → `/spooder_controller/commands` (ros2_control)

### Config Files
- `config/octomap_params.yaml` — OctoMap server: ground filtering enabled, 3D obstacles
- `config/foothold_params.yaml` — cost weights, gait timing, AEP/PEP, swing height
- `config/nav2_params.yaml` — Nav2: static + obstacle + inflation layers on global costmap
- `config/robot_description.yaml` — robot name, link lengths, leg origins, joint limits

### Launch Files
- `launch/full_pipeline.launch.py` — Real robot: OctoMap + Terrain + Planner + Gait + Nav2
- `launch/simulation.launch.py` — Gazebo overlay: adds OctoMap + terrain pipeline
- `launch/full_simulation.launch.py` — Full sim: spawns robot at z=3.0, loads OctoMap server

### Tests (84 total)
- `test_kinematics.py` — 25 tests (IK, transforms, reachable zones, bezier arcs)
- `test_terrain_cost.py` — 23 tests (variance, cost layers, unknown_above strict/continuous, continuous range)
- `test_foothold_planner.py` — 17 tests (cost extraction, replan threshold, AEP/PEP filter, phase transitions)
- `test_gait_controller.py` — 17 tests (bezier arcs, tripod groups, swing progress, joint names, leg phase)

---

## Build & Test Commands

```bash
# Build
source /opt/ros/jazzy/setup.bash && source ~/spooder_ws/install/setup.bash
colcon build --symlink-install --packages-select hexapod_nav

# Test
cd src/hexapod_nav && python3 -m pytest test/ -v
```

---

## Known Adaptations from Guide

| Guide Says | Diddler Uses | Why |
|------------|-------------|-----|
| `base_link` | `spooder/base_footprint` | Diddler frame naming |
| `JointState` to `/joint_commands` | `Float64MultiArray` to `/spooder_controller/commands` | ros2_control interface |
| Generic kinematics values | COXA=0.043, FEMUR=0.060, TIBIA=0.104 | Diddler hardware |
| `rospy` | `rclpy` | ROS2 Jazzy |
| OctoMap + costmap_2d | Custom terrain GridMap | Guide's novel approach |
| Fixed AEP/PEP limits | Configurable via `aep_forward_offset`/`pep_backward_offset` params | Tunable per robot |
| Binary unknown_above | Continuous [0,1] fraction | Better cost gradient for optimization |

---

## Bugs Fixed (2026-07-18)

1. **OctoMap ground filtering OFF** — `filter_ground: false` caused the floor plane to be marked as obstacle in the 2D projected map, flooding the Nav2 costmap with occupied cells on the ground. Nav2 could not find any free space to plan paths. Fixed: `filter_ground: true` with `max_distance: 0.15`.

2. **Robot spawning inside ground / too low** — `start_spooder.sh` did not pass a useful `spawn_z` to `02_robot_spawn.launch.py`, and the shared spawn launch still defaulted to ground level. Robot spawned too low for rough terrain. Fixed: canonical `spawn_z:=3.0` in `start_spooder.sh`, `02_robot_spawn.launch.py`, `full_simulation.launch.py`, `spooder_gazebo/launch/simulation.launch.py`, and `05_unified_bringup.launch.py`.

3. **Nav2 global costmap missing obstacle_layer** — Only had `static_layer` (from OctoMap 2D projection) + `inflation_layer`. No real-time obstacle sensing. Fixed: added `obstacle_layer` using `/scan` data.

4. **RViz 3D visualization broken** — TF display was disabled, camera PointCloud2 was disabled. Users couldn't see the robot frame tree, 3D markers, or point clouds. Fixed: enabled TF display, enabled camera PointCloud2, added Nav2 costmap displays.

5. **Missing tf2_ros dependency** — `octomap_terrain_node.py` imports `tf2_ros` but `package.xml` did not list it as a dependency. Could cause import failures. Fixed: added `tf2_ros` and `tf2_geometry_msgs`.

6. **Nav2 launch/config drift** — `start_spooder.sh` started Nav2 before the hexapod OctoMap pipeline and used the legacy Nav2 params by default. Fixed: hexapod pipeline is default, starts before Nav2, and Nav2 receives `hexapod_nav/config/nav2_params.yaml`.

7. **RViz RobotModel and OctoMap point clouds invisible** — RViz needed `TF Prefix: spooder` for prefixed robot_state_publisher frames, sensor-data QoS for Gazebo point clouds, and non-namespaced OctoMap topics. Fixed: `sim.rviz` now points at `/camera/points`, `/octomap_point_cloud_centers`, `/occupied_cells_vis_array`, and `/projected_map`.

8. **OctoMap Z clipping at high spawn** — `point_cloud_max_z` / `occupancy_max_z` of `2.0` discarded geometry when the robot spawned at ~3 m. Fixed: cave-safe band `-2.0` … `8.0` in `octomap_params.yaml` (ground filter kept on for Nav2).

9. **Spawn ordering / hung `ros_gz_sim create`** — Controllers started before the Gazebo entity existed; `create` with `use_sim_time` hung on world discovery. Fixed: spawn→controllers→EKF order, no sim-time on `create`, `GZ_IP=127.0.0.1`.

10. **RViz `isRowMajor` spam** — GridMap layers used labels `column`/`row`; `grid_map_ros` requires `column_index`/`row_index`. Fixed via `grid_map_utils.make_layer()`.

11. **Nav2 goal set but robot idle** — Gait ignored `angular.z` while RPP was rotate-to-heading (yaw-only cmd_vel), and `/leg_phase` was not published while walking so footholds never selected. Fixed: gait handles yaw rate, always publishes phase, RPP `use_rotate_to_heading: false`.

12. **Foothold visualization** — `/foothold_markers` (target spheres + rays + labels) and `/gait_foot_markers` (live foot tips); enabled in `sim.rviz`.

---

## What's Left (Potential Future Work)

- [ ] Integration test with real OctoMap data (bag file playback)
- [ ] Tune `foothold_params.yaml` weights on real robot
- [ ] Add diagnostic topic publishing for AEP/PEP rejection rate
- [ ] Consider adding footprint inflation for body collision checking (currently Nav2 does body-path-only)
