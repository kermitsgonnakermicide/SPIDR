"""spooder_robot: onboard-robot node package.

Exposes:
  - ``robot_node``  - coordinator with services and diagnostics
  - ``imu_node``    - OAK-D Lite IMU republisher (handles upside-down mount)
  - ``gait_node``   - open-loop tripod gait with Bezier swing arcs and
                      IMU-driven body leveling
  - ``kinematics``  - 3-DOF per-leg IK / FK matching the URDF geometry
  - ``calibration`` - load per-joint offsets/limits/invert from a YAML

The package assumes ros2_control's ST3215System is already running and
``/spooder_controller/commands`` is the standard
``Float64MultiArray`` input expected by the
``position_controllers/JointGroupPositionController``.
"""
