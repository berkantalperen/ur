import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # REMOVED THE SUBPROCESS/PKILL LINE TO PREVENT SELF-TERMINATION

    # 1. PATHS
    ur_description_path = get_package_share_directory('ur_description')
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    imu_sim_path = get_package_share_directory('imu_sim')
    
    servo_config_path = os.path.join(imu_sim_path, 'config', 'ur_servo_config.yaml')
    rviz_config_file = os.path.join(imu_sim_path, 'config', 'teleop.rviz')

    # 2. ROBOT DESCRIPTION
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_description_path, 'urdf', 'ur.urdf.xacro'), 
                 ' name:=ur5e ur_type:=ur5e']), value_type=str)

    robot_description_semantic = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro'), 
                 ' name:=ur5e']), value_type=str)

    # 3. NODES
    # A. The Logic Nodes
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        namespace='servo_teleop',
        parameters=[
            servo_config_path,
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': os.path.join(ur_moveit_config_path, 'config', 'kinematics.yaml'),
                'robot_description_planning': os.path.join(ur_moveit_config_path, 'config', 'joint_limits.yaml'),
                'use_sim_time': False, 
                'command_in_type': 'speed_units', 
                'moveit_servo.move_group_name': 'ur_manipulator'
            }
        ],
        output='screen'
    )

    # B. The Visualization Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        Node(package='imu_sim', executable='imu_sim_server', name='imu_input'),
        Node(package='imu_sim', executable='safety_node', name='safety_node'),
        Node(package='imu_sim', executable='joint_state_relay', name='joint_relay'),
        servo_node,
        rviz_node
    ])
# colcon build --packages-select imu_sim && source install/setup.bash && ros2 launch imu_sim teleop_demo.launch.py
# ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=172.17.0.2 launch_rviz:=false