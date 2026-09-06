# spooder_control

ros2_control hardware interface and joint controllers for the Spooder hexapod.

## Contents

- `launch/` — Controller launch files
- `config/` — Controller and joint definitions
- `spooder_control/` — Python controller implementations
- `scripts/` — Calibration and testing scripts

## Controllers

| Controller | Type | Purpose |
|------------|-------|---------|
| `joint_state_broadcaster` | broadcaster | Publish joint states |
| `position_trajectory_controller` | joint_trajectory | Position control for gait |
| `gait_controller` | custom | Hexapod gait pattern generation |

## Launch Files

```bash
# Start controllers
ros2 launch spooder_control control.launch.py
```

## Configuration

### `config/controllers.yaml`
- Joint limits for coxa/femur/tibia
- PID gains for position control
- Gait timing parameters

## Usage

```bash
# Send position commands
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, -1.0]}"
```

## Dependencies

- `controller_manager`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `spooder_description`
