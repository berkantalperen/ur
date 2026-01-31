import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load SRDF
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    srdf_path = os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro')
    
    robot_description_semantic_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', srdf_path, ' ', 'name:=ur', ' ', 'ur_type:=ur5e']
    )

    # Load Servo Config Manually
    servo_config_path = os.path.join(
        get_package_share_directory('imu_sim'),
        'config',
        'ur_servo_config.yaml'
    )

    with open(servo_config_path, 'r') as f:
        servo_yaml = yaml.safe_load(f)

    servo_params = {
        'moveit_servo': {
            **servo_yaml,
            'command_in_type': 'speed_units',
            'move_group_name': 'ur_manipulator',
            'command_out_topic': '/servo_debug_out',
            'check_collisions': False,
            'use_sim_time': True
        }
    }

    # Isolated Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
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
