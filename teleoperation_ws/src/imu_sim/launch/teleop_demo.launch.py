import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

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

    # Load URDF + SRDF
    # We need to find where ur_moveit_config is
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    urdf_path = os.path.join(ur_moveit_config_path, 'config', 'ur.urdf.xacro')
    srdf_path = os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_path,
            ' ',
            'name:=ur',
            ' ',
            'ur_type:=ur5e',
            ' ',
            'safety_limits:=true',
            ' ',
            'safety_pos_margin:=0.15',
            ' ',
            'safety_k_position:=20',
            ' ',
            'joint_limit_parameters_file:=',
            os.path.join(ur_moveit_config_path, 'config', 'joint_limits.yaml'),
            ' ',
            'kinematics_parameters_file:=',
            os.path.join(ur_moveit_config_path, 'config', 'kinematics.yaml'),
            ' ',
            'physical_parameters_file:=',
            os.path.join(ur_moveit_config_path, 'config', 'physical_parameters.yaml'),
            ' ',
            'visual_parameters_file:=',
            os.path.join(ur_moveit_config_path, 'config', 'visual_parameters.yaml'),
        ]
    )
    
    # Process xacro to get SRDF content
    robot_description_semantic_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', srdf_path, ' ', 'name:=ur', ' ', 'ur_type:=ur5e']
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
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            {'moveit_servo.command_in_type': 'speed_units'}, 
            {'command_in_type': 'speed_units'}, # Try flat as well
            {'moveit_servo.move_group_name': 'ur_manipulator'},
            {'move_group_name': 'ur_manipulator'} 
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},
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
        joint_state_publisher,
        robot_state_publisher,
        servo_node,
        rviz_node
    ])
