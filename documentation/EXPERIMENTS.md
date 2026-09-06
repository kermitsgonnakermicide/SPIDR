# Experiment Design Guide

## Overview

This guide helps you design and run experiments using the Spooder hexapod with OAK-D Lite camera.

## Experiment Workflow

```
1. Define Hypothesis
       │
       ▼
2. Design Experiment
       │
       ▼
3. Configure Parameters
       │
       ▼
4. Run in Simulation
       │
       ▼
5. Record Data
       │
       ▼
6. Analyze Results
       │
       ▼
7. Validate on Hardware
       │
       ▼
8. Document Findings
```

## Common Experiment Types

### 1. Terrain Traversability

**Hypothesis:** The hexapod can traverse terrain with slopes up to X degrees.

**Setup:**
```bash
ros2 launch spooder_gazebo 01_sim_world.launch.py world:=rough_terrain
ros2 launch spooder_navigation navigation.launch.py
```

**Metrics:**
- Success rate per slope angle
- Path length
- Time to traverse
- Number of falls/stumbles

**Data to record:**
- `/terrain_cost_map`
- `/foothold_plan`
- `/odom`
- `/tf`

### 2. Foothold Planning

**Hypothesis:** Terrain-aware foothold planning reduces energy consumption vs. fixed grid.

**Setup:**
- Enable terrain cost in `foothold_params.yaml`
- Compare with `foothold_planner.terrain_aware:=false`

**Metrics:**
- Energy consumption (joint torques integrated over time)
- Path efficiency
- Foot slippage

**Data to record:**
- `/joint_states` (torques if available)
- `/foothold_plan`
- `/cmd_vel`

### 3. Perception Range

**Hypothesis:** Increasing perception range improves navigation speed.

**Setup:**
- Vary `octomap_params.yaml` → `sensor_model.max_range`: [2, 3, 4, 5] m

**Metrics:**
- Navigation speed
- Path smoothness (acceleration)
- Obstacle clearance

### 4. Sim-to-Hardware Transfer

**Hypothesis:** Algorithms developed in simulation transfer to hardware without modification.

**Setup:**
1. Run experiment in simulation with `mode:=simulation`
2. Run same experiment on hardware with `mode:=hardware`
3. Compare metrics

**Metrics:**
- Topic timing/latency
- Navigation success rate
- Perception accuracy

## Configuration Template

Create a config file for each experiment:

```yaml
# experiment_config.yaml
experiment:
  name: "terrain_traversability_slope_30"
  date: "2024-01-15"
  hypothesis: "Hexapod can traverse 30° slopes"

hardware:
  camera: "OAK-D-LITE"
  robot: "SPOODER-v1"

simulation:
  world: "rough_terrain"
  use_sim_time: true

parameters:
  terrain_cost:
    slope_weight: 2.0
    roughness_weight: 1.5
  octomap:
    max_range: 4.0
    resolution: 0.05

metrics:
  - success_rate
  - path_length
  - traversal_time
  - energy_consumption

data_recording:
  topics:
    - /oak_d/points
    - /terrain_cost_map
    - /foothold_plan
    - /odom
    - /joint_states
    - /tf
```

## Running Experiments

### Quick Recording

```bash
./scripts/run_oakd_lite.sh sim --record
```

### Custom Recording

```bash
# Create topic list
echo "/oak_d/points" > my_topics.txt
echo "/odom" >> my_topics.txt

# Record
./scripts/record_experiment.sh -t my_topics.txt -d 300 -o ~/bags/exp_terrain_test
```

### Analysis

```bash
python3 ./scripts/analyze_bag.py ~/bags/exp_*/ -o results.txt
```

## Results Template

See `documentation/RESULTS_TEMPLATE.md` for a table format to log results.

## Tips for Reproducibility

1. **Version control parameters:** Store configs in git
2. **Record everything:** Always record bags, even if you think you don't need them
3. **Note environment:** Gazebo world version, ROS packages installed
4. **Multiple runs:** Run each experiment at least 5 times for statistical significance
5. **Control variables:** Change one thing at a time

## Common Pitfalls

- **Not recording TF:** Always record TF — it's needed for replay and analysis
- **Short recordings:** Record longer than you think you need
- **Wrong sim time:** Ensure `use_sim_time:=true` in simulation
- **Missing dependencies:** Verify all packages are built before running
