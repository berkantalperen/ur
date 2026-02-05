import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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
    # We load this so Servo and RViz know what the robot looks like.
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_description_path, 'urdf', 'ur.urdf.xacro'), 
                 ' name:=ur5e ur_type:=ur5e']), value_type=str)

    robot_description_semantic = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', 
                 os.path.join(ur_moveit_config_path, 'srdf', 'ur.srdf.xacro'), 
                 ' name:=ur5e']), value_type=str)

    # --- 4. NODES ---
    
    # The Universal Bridge (Handles Command Timestamps + Ordering)
    bridge = Node(
        package='imu_sim', 
        executable='universal_bridge', 
        name='universal_bridge',
        output='screen'
    )

    # IMU Input Server (Your Teleop Logic)
    imu_server = Node(
        package='imu_sim', 
        executable='imu_sim_server', 
        name='imu_input',
        output='screen'
    )

    # Safety Node
    safety_node = Node(
        package='imu_sim', 
        executable='safety_node', 
        name='safety_node',
        output='screen'
    )

    # Servo Node (The Brain)
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
                
                # Read from bridge (which uses compatible QoS)
                'moveit_servo.joint_topic': '/joint_states_servo',
                
                # Output velocities to Bridge (So Bridge can reorder before sending to driver)
                'moveit_servo.command_out_topic': '/servo/velocity_raw',
                'moveit_servo.command_out_type': 'std_msgs/Float64MultiArray',
                'moveit_servo.publish_joint_positions': False,
                'moveit_servo.publish_joint_velocities': True,
                
                # DISABLE SINGULARITY CHECKING (Set VERY HIGH thresholds so never triggered)
                'moveit_servo.lower_singularity_threshold': 1e10,
                'moveit_servo.hard_stop_singularity_threshold': 1e12,
                
                # JOINT LIMIT MARGINS - Set to 0 to only halt at actual limits
                'moveit_servo.joint_limit_margins': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                
                # Don't halt all joints when one hits limit
                'moveit_servo.halt_all_joints_in_joint_mode': False,
                'moveit_servo.halt_all_joints_in_cartesian_mode': False,
                
                'use_sim_time': False,
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
            }
        ],
        output='screen'
    )

    # Visualization (Relies on Driver for TF, so RobotStatePublisher is NOT here)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'robot_description': robot_description, 
             'robot_description_semantic': robot_description_semantic, 
             'use_sim_time': False}
        ]
    )

    # --- 5. ACTIONS & SERVICE CALLS ---

    # Unpause Servo (Delayed slightly to ensure node is up)
    unpause_servo = TimerAction(
        period=2.0, 
        actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/servo_teleop/servo_node/pause_servo', 'std_srvs/srv/SetBool', '{"data": false}'], output='screen')]
    )

    # Switch Command Type (Delayed slightly more)
    set_command_type = TimerAction(
        period=3.0, 
        actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/servo_teleop/servo_node/switch_command_type', 'moveit_msgs/srv/ServoCommandType', '{"command_type": 1}'], output='screen')]
    )

    return LaunchDescription([
        bridge,
        imu_server,
        safety_node,
        servo_node,
        rviz_node,
        unpause_servo,
        set_command_type
    ])