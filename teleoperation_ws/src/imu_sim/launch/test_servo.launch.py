import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("robot_name", default_value="ur5e", description="Name of the robot")
    )
    
    robot_name = LaunchConfiguration("robot_name")

    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    srdf_path = os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro')
    
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', srdf_path, ' ',
            'name:=', robot_name, ' ',
            'ur_type:=ur5e'
        ]
    )

    servo_config_path = os.path.join(
        get_package_share_directory('imu_sim'),
        'config',
        'ur_servo_config.yaml'
    )

    # 1. The Bridge Node
    qos_bridge_node = Node(
        package='imu_sim',
        executable='qos_bridge',
        name='qos_bridge',
        output='screen'
    )

    # 2. The Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_config_path,
            {
                'moveit_servo.command_in_type': 'speed_units',
                'moveit_servo.move_group_name': 'ur_manipulator',
                'moveit_servo.command_out_topic': '/scaled_joint_trajectory_controller/joint_trajectory', 
                # CRITICAL: Point Servo to the bridge topic
                'moveit_servo.joint_topic': '/joint_states_servo',
                'robot_description_semantic': robot_description_semantic_content,
                'use_sim_time': False
            }
        ],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [qos_bridge_node, servo_node])