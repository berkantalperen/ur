import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load URDF + SRDF from the MoveIt config package to keep them aligned.
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    urdf_path = os.path.join(ur_moveit_config_path, 'urdf', 'ur.urdf.xacro')
    srdf_path = os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_path,
            ' ',
            'name:=ur5e',
            ' ',
            'ur_type:=ur5e',
        ]
    )
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            srdf_path,
            ' ',
            'name:=ur5e',
            ' ',
            'ur_type:=ur5e',
        ]
    )

    # Load Servo Config Manually
    servo_config_path = os.path.join(
        get_package_share_directory('imu_sim'),
        'config',
        'ur_servo_config.yaml'
    )
    
    with open(servo_config_path, 'r') as f:
        yaml_content = yaml.safe_load(f)

    # Extract params (servo_node -> ros__parameters -> moveit_servo) or fallback
    try:
        servo_params = yaml_content['servo_node']['ros__parameters']
    except (KeyError, TypeError):
        servo_params = yaml_content

    # Ensure sub-dictionaries exist
    if 'moveit_servo' not in servo_params:
        servo_params['moveit_servo'] = {}

    # OVERRIDES
    servo_params['moveit_servo']['command_in_type'] = 'speed_units'
    servo_params['moveit_servo']['move_group_name'] = 'ur_manipulator'
    servo_params['moveit_servo']['command_out_topic'] = '/servo_debug_out'
    servo_params['moveit_servo']['check_collisions'] = False
    
    # ROOT PARAMS
    servo_params['command_in_type'] = 'speed_units' # Flattened just in case
    servo_params['command_out_topic'] = '/servo_debug_out'
    servo_params['check_collisions'] = False
    servo_params['use_sim_time'] = True # CRITICAL

    # Isolated Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content}
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='imu_sim',
        executable='clock_bridge',
        name='clock_bridge',
        output='screen'
    )

    return LaunchDescription([
        clock_bridge,
        servo_node
    ])
