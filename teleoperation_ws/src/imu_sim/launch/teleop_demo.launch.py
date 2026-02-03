import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # --- 1. PATHS ---
    ur_description_path = get_package_share_directory('ur_description')
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    imu_sim_path = get_package_share_directory('imu_sim')
    
    servo_config_path = os.path.join(imu_sim_path, 'config', 'ur_servo_config.yaml')
    rviz_config_file = os.path.join(imu_sim_path, 'config', 'teleop.rviz')
    
    # --- 2. CONFIGS ---
    kinematics_yaml = load_yaml(os.path.join(ur_moveit_config_path, 'config', 'kinematics.yaml'))
    joint_limits_yaml = load_yaml(os.path.join(ur_moveit_config_path, 'config', 'joint_limits.yaml'))
    
    # --- 3. ROBOT DESCRIPTION ---
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_description_path, 'urdf', 'ur.urdf.xacro'), 
                 ' name:=ur5e ur_type:=ur5e']), value_type=str)

    robot_description_semantic = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro'), 
                 ' name:=ur5e']), value_type=str)

    # --- 4. NODES ---
    
    # The Universal Bridge (Handles Time + Order)
    bridge = Node(
        package='imu_sim', executable='universal_bridge', name='universal_bridge'
    )

    # Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        namespace='servo_teleop',
        parameters=[
            servo_config_path, 
            {
                'moveit_servo.move_group_name': 'ur_manipulator',
                'moveit_servo.check_collisions': False, 
                'moveit_servo.command_in_type': 'speed_units',
                'moveit_servo.cartesian_command_in_topic': '/servo_server/delta_twist_cmds',
                # Connect to Bridge, NOT driver directly
                'moveit_servo.joint_topic': '/joint_states_servo',
                'moveit_servo.command_out_topic': '/servo/trajectory_raw',
                
                'use_sim_time': False,
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
            }
        ],
        output='screen'
    )

    # Robot State Publisher (Reads Clean Data from Bridge)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False, 'publish_frequency': 500.0}],
        remappings=[('/joint_states', '/joint_states_servo')]
    )

    # Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'robot_description': robot_description, 'robot_description_semantic': robot_description_semantic, 'use_sim_time': False}]
    )

    # --- 5. ACTIONS ---

    # A. AUTO-KILLER: Wipes conflicting nodes from the Driver (or previous runs)
    kill_conflicts = ExecuteProcess(
        cmd=['pkill', '-f', 'robot_state_publisher'],
        shell=True,
        output='screen'
    )

    # B. Service Calls (Delayed to start AFTER the main system is up)
    unpause_servo = TimerAction(
        period=4.0, # Delayed 4s total (2s startup + 2s wait)
        actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/servo_teleop/servo_node/pause_servo', 'std_srvs/srv/SetBool', '{"data": false}'], output='screen')]
    )

    set_command_type = TimerAction(
        period=5.0, # Delayed 5s total
        actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/servo_teleop/servo_node/switch_command_type', 'moveit_msgs/srv/ServoCommandType', '{"command_type": 1}'], output='screen')]
    )

    # C. Main System (Delayed by 2.0s to allow 'kill_conflicts' to finish)
    main_system = TimerAction(
        period=2.0,
        actions=[
            Node(package='imu_sim', executable='imu_sim_server', name='imu_input'),
            Node(package='imu_sim', executable='safety_node', name='safety_node'),
            bridge,
            rsp_node,
            servo_node,
            unpause_servo,
            set_command_type,
            rviz_node
        ]
    )

    return LaunchDescription([
        kill_conflicts,
        main_system
    ])