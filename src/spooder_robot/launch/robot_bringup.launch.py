"""Single-file bringup for the physical spooder robot.

Run from the robot onboard computer *or* from a laptop that has UDP
multicast back to it (ROS_DOMAIN_ID must match between the two or the
graph will partition):

    # on the robot
    ros2 launch spooder_robot robot_bringup.launch.py \
        usb_port:=/dev/ttyUSB0 camera_ip:='' use_oakd:=true

    # on a laptop, run only the planning side
    ros2 launch hexapod_nav simulation.launch.py use_sim_time:=false

Stops as soon as it cannot reach required packages or the OAK-D
device, with a one-line message instead of a stack trace.

The order is:

  t=0.0s   robot_state_publisher + ros2_control_node
  t=2.0s   joint_state_broadcaster + spooder_controller (load via spawner)
  t=4.0s   depthai_ros_driver (OAK-D, optional)
  t=6.0s   spooder_imu_node (subscribes /oak_d/imu -> /imu/data)
  t=7.0s   spooder_gait_node + spooder_robot_node
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

import launch_ros.parameter_descriptions


def _require(pkg, apt):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Missing ROS package '{pkg}'. Install with: "
            f"sudo apt-get install -y {apt}"
        ) from exc


def generate_launch_description():
    pkg_spooder_robot = _require("spooder_robot", "ros-jazzy-spooder-robot")
    pkg_spooder_hardware = _require("spooder_hardware",
                                     "ros-jazzy-spooder-hardware")
    pkg_spooder_description = _require("spooder_description",
                                       "ros-jazzy-spooder-description")
    pkg_spooder_perception = _require("spooder_perception",
                                       "ros-jazzy-spooder-perception")

    # --------- arguments ---------
    use_oakd = LaunchConfiguration("use_oakd", default="true")
    oakd_yaml = LaunchConfiguration(
        "oakd_yaml",
        default=os.path.join(pkg_spooder_perception,
                              "config", "oakd_lite.yaml"),
    )
    usb_port = LaunchConfiguration("usb_port", default="/dev/ttyUSB0")
    calibration_yaml = LaunchConfiguration(
        "calibration_yaml",
        default=os.path.join(pkg_spooder_robot, "config",
                              "robot_calibration.yaml"),
    )
    calibration_json_path = LaunchConfiguration(
        "calibration_json_path", default="",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # --------- URDF ---------
    xacro_file = os.path.join(pkg_spooder_description, "urdf", "spooder.xacro")
    robot_description = Command(["xacro ", xacro_file, " config_file:=",
                                  os.path.join(pkg_spooder_hardware, "config", "controllers.yaml")])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description":
                launch_ros.parameter_descriptions.ParameterValue(
                    robot_description, value_type=str),
            "use_sim_time": use_sim_time,
            # Don't prefix frames; the physical robot's TF tree should
            # use plain /base_link rather than /spooder/base_link like
            # the sim does.
        }],
        output="screen",
    )

    # --------- ros2_control ---------
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(pkg_spooder_hardware, "config", "hardware.yaml"),
            os.path.join(pkg_spooder_hardware, "config", "controllers.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    load_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["spooder_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --------- OAK-D ---------
    oakd = Node(
        package="depthai_ros_driver",
        executable="camera_node",
        name="oak_d",
        parameters=[oakd_yaml, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_oakd),
    )

    # --------- IMU republisher ---------
    imu_node = Node(
        package="spooder_robot",
        executable="imu_node",
        name="spooder_imu_node",
        parameters=[{
            "input_topic": "/oak_d/imu",
            "output_topic": "/imu/data",
            "output_frame_id": "imu_link",
            "use_sim_time": use_sim_time,
        }],
        output="screen",
        condition=IfCondition(use_oakd),
    )

    # --------- gait ---------
    gait_node = Node(
        package="spooder_robot",
        executable="gait_node",
        name="spooder_gait_node",
        parameters=[
            os.path.join(pkg_spooder_robot, "config", "gait_params.yaml"),
            {
                "calibration_yaml": calibration_yaml,
                "calibration_json_path": calibration_json_path,
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )

    # --------- coordinator ---------
    robot_node = Node(
        package="spooder_robot",
        executable="robot_node",
        name="spooder_robot_node",
        parameters=[{
            "calibration_yaml": calibration_yaml,
            "calibration_json_path": calibration_json_path,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_oakd", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("usb_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("calibration_yaml", default_value=os.path.join(
            pkg_spooder_robot, "config", "robot_calibration.yaml")),
        DeclareLaunchArgument("calibration_json_path", default_value=""),

        # Stage 1: TF + ros2_control as early as possible so they have
        # time to fully enumerate their interfaces before the gait
        # node starts publishing /spooder_controller/commands.
        TimerAction(period=0.0, actions=[rsp, ros2_control]),

        # Stage 2: controllers -- depends on ros2_control_node being
        # alive enough to load the plugin.
        TimerAction(period=2.0, actions=[load_jsb, load_ctrl]),

        # Stage 3: OAK-D camera (optional -- skip if you don't have one)
        TimerAction(period=4.0, actions=[oakd]),

        # Stage 4: IMU republisher requires the OAK-D driver to be up
        # and publishing /oak_d/imu, so it trails the camera.
        TimerAction(period=6.0, actions=[imu_node]),

        # Stage 5: gait + robot coordinator -- after everything else.
        TimerAction(period=7.0, actions=[gait_node, robot_node]),
    ])
