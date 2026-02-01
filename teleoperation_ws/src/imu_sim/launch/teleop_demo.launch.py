import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    # Paths
    imu_sim_path = get_package_share_directory('imu_sim')
    ur_desc_path = get_package_share_directory('ur_description')
    ur_moveit_path = get_package_share_directory('ur_moveit_config')

    # Descriptions
    robot_description = {'robot_description': Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', os.path.join(ur_desc_path, 'urdf', 'ur.urdf.xacro'), ' name:=ur5e ur_type:=ur5e'])}
    robot_description_semantic = {'robot_description_semantic': Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', os.path.join(ur_moveit_path, 'srdf', 'ur.srdf.xacro'), ' name:=ur5e ur_type:=ur5e'])}
    
    with open(os.path.join(ur_moveit_path, 'config', 'kinematics.yaml'), 'r') as f:
        kinematics_yaml = {'robot_description_kinematics': yaml.safe_load(f)}

    # Flat Parameters
    servo_params = {
        "moveit_servo.use_gazebo": False,
        "moveit_servo.command_in_type": "speed_units",
        "moveit_servo.command_in_topic": "/servo_server/delta_twist_cmds",
        "moveit_servo.command_out_topic": "/servo_raw",
        "moveit_servo.publish_period": 0.01,
        "moveit_servo.move_group_name": "ur_manipulator",
        "moveit_servo.is_primary_planning_scene_monitor": True,
        "moveit_servo.check_collisions": False,
        "moveit_servo.robot_link_command_frame": "base_link",
        "moveit_servo.planning_frame": "base_link",
        "moveit_servo.ee_frame_name": "tool0",
        "moveit_servo.joint_topic": "/joint_states_fixed", # This matches the bridge
    }

    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher', name='stf', arguments=['0','0','0','0','0','0','world','base_link']),
        Node(package='imu_sim', executable='qos_bridge', name='bridge'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description]),
        Node(package='moveit_servo', executable='servo_node', name='servo_node', parameters=[servo_params, robot_description, robot_description_semantic, kinematics_yaml]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(imu_sim_path, 'config', 'teleop.rviz')], parameters=[robot_description, kinematics_yaml])
    ])