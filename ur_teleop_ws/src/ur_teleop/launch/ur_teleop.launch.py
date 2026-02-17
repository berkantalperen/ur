import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(path: str) -> dict:
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except OSError:
        return None

def generate_launch_description():
    # --- ARGS ---
    sim_arg = DeclareLaunchArgument(
        "sim", 
        default_value="true", 
        description="Launch with simulated hand pose publisher (Lissajous)"
    )
    use_sim = LaunchConfiguration("sim")

    # --- PATHS ---
    ur_moveit_share = get_package_share_directory("ur_moveit_config")
    ur_description_share = get_package_share_directory("ur_description")
    ur_teleop_share = get_package_share_directory("ur_teleop")

    # --- ROBOT DESCRIPTION (URDF/SRDF) ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([ur_description_share, "urdf", "ur.urdf.xacro"]), " ",
            "name:=", "ur", " ", "ur_type:=", "ur5e", " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([ur_moveit_share, "srdf", "ur.srdf.xacro"]), " ",
            "name:=", "ur", " ", "ur_type:=", "ur5e", " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    kinematics_yaml = os.path.join(ur_moveit_share, "config", "kinematics.yaml")
    kin = load_yaml(kinematics_yaml) or {}
    robot_description_kinematics = {"robot_description_kinematics": kin.get("robot_description_kinematics", kin)}

    servo_yaml = os.path.join(ur_teleop_share, "config", "ur_servo.yaml")
    servo_params = load_yaml(servo_yaml)

    # --- NODES ---
    
    # 1. MoveIt
    ur_moveit_launch = os.path.join(ur_moveit_share, "launch", "ur_moveit.launch.py")
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": "true",
            "launch_servo": "false",
            "use_fake_hardware": "false",
        }.items(),
    )

    # 2. Servo
    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
    )

    # 3. Hand Simulation (Only if sim:=true)
    hand_sim = Node(
        package="ur_teleop",
        executable="hand_pose_sim",
        name="hand_pose_sim",
        output="screen",
        condition=IfCondition(use_sim)
    )

    # 4. Real IMU Driver (Only if sim:=false)
    # REPLACE "imu_real_driver" WITH YOUR ACTUAL DRIVER NODE NAME LATER
    real_imu = Node(
        package="ur_teleop", 
        executable="imu_real_driver", 
        name="imu_real_driver",
        output="screen",
        condition=UnlessCondition(use_sim)
    )

    return LaunchDescription([sim_arg, moveit, servo, hand_sim, real_imu])