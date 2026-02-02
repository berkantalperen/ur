import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # 1. PATHS
    ur_description_path = get_package_share_directory('ur_description')
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    imu_sim_path = get_package_share_directory('imu_sim')
    
    servo_config_path = os.path.join(imu_sim_path, 'config', 'ur_servo_config.yaml')
    rviz_config_file = os.path.join(imu_sim_path, 'config', 'teleop.rviz')
    
    kinematics_yaml = load_yaml(os.path.join(ur_moveit_config_path, 'config', 'kinematics.yaml'))
    joint_limits_yaml = load_yaml(os.path.join(ur_moveit_config_path, 'config', 'joint_limits.yaml'))

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
    
    # A. RELAY (Required for timestamp fix)
    joint_relay = Node(
        package='imu_sim', 
        executable='joint_state_relay', 
        name='joint_relay'
    )

    # B. SERVO NODE
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        namespace='servo_teleop',
        parameters=[
            servo_config_path,
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
                'use_sim_time': False, 
                'command_in_type': 'speed_units', 
                'moveit_servo.move_group_name': 'ur_manipulator',
                'moveit_servo.joint_topic': '/joint_states_servo' # Listen to relay
            }
        ],
        output='screen'
    )

    # C. SYNCHRONIZED ROBOT STATE PUBLISHER (The Fix)
    # Listens to relay topic so TFs match Joint timestamps exactly
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 500.0 # Match driver rate
        }],
        remappings=[
            ('/joint_states', '/joint_states_servo') # Hijack input
        ]
    )

    # D. VISUALIZATION
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
        joint_relay,
        rsp_node, # Our new synchronized publisher
        servo_node,
        rviz_node
    ])