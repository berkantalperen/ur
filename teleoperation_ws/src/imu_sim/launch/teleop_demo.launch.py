import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(f"DEBUG: Loading YAML from {absolute_file_path}")
    if not os.path.exists(absolute_file_path):
        raise FileNotFoundError(f"File not found: {absolute_file_path}")
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"DEBUG: Error loading YAML: {e}")
        raise e

def generate_launch_description():
    # 1. Get configuration
    servo_config = load_yaml('imu_sim', 'config/ur_servo_config.yaml')
    rviz_config_file = PathJoinSubstitution([get_package_share_directory('imu_sim'), 'config', 'teleop.rviz'])

    # 2. Define Nodes

    # A. imu_sim_server (Input)
    imu_sim_node = Node(
        package='imu_sim',
        executable='imu_sim',
        name='imu_sim_server',
        output='screen'
    )

    # B. safety_node (Clamping)
    safety_node = Node(
        package='imu_sim',
        executable='safety_node',
        name='safety_node',
        output='screen'
    )

    # Load URDF + SRDF using MoveItConfigsBuilder to keep descriptions aligned.
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(
            file_path="urdf/ur.urdf.xacro",
            mappings={"name": "ur5e", "ur_type": "ur5e"},
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro",
            mappings={"name": "ur5e", "ur_type": "ur5e"},
        )
        .to_moveit_configs()
    )

    # Path to Servo Config
    servo_config_path = os.path.join(
        get_package_share_directory('imu_sim'),
        'config',
        'ur_servo_config.yaml'
    )

    # C. MoveIt Servo
    # We pass the config file AND explicit overrides to ensure parameters are seen
    # regardless of namespace quirks.
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_config_path,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'moveit_servo.command_in_type': 'speed_units'}, 
            {'command_in_type': 'speed_units'}, # Try flat as well
            {'moveit_servo.move_group_name': 'ur_manipulator'},
            {'move_group_name': 'ur_manipulator'} 
        ],
        output='screen'
    )

    # D. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    return LaunchDescription([
        imu_sim_node,
        safety_node,
        servo_node,
        rviz_node
    ])
