import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ur_moveit_config is installed under /opt/ros/jazzy/share/ur_moveit_config
    ur_moveit_share = get_package_share_directory("ur_moveit_config")
    ur_moveit_launch = os.path.join(ur_moveit_share, "launch", "ur_moveit.launch.py")
    kinematics_yaml = os.path.join(ur_moveit_share, "config", "kinematics.yaml")

    # your package share
    this_share = get_package_share_directory("ur_servo_demo")
    servo_yaml = os.path.join(this_share, "config", "ur_servo.yaml")

    # MoveIt (starts move_group + publishes robot_description)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
            "launch_servo": "false",  # we start our own servo_node below
        }.items(),
    )

    # Inject IK config into servo_node so TWIST/POSE works
    kin = load_yaml(kinematics_yaml) or {}
    robot_description_kinematics = kin.get("robot_description_kinematics", kin)

    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_yaml,  # <-- pass as STRING PATH, not ParameterFile(...)
            {"robot_description_kinematics": robot_description_kinematics},
        ],
    )

    return LaunchDescription([moveit, servo])
