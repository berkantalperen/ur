from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Build the MoveIt configuration for UR5e
    # This automatically fetches the URDF and SRDF from the installed ur_moveit_config package
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur_moveit_config") \
        .robot_description(mappings={"ur_type": "ur5e"}) \
        .to_moveit_configs()

    return LaunchDescription([
        # 2. Run the Execute Node with the parameters loaded
        Node(
            package="ur_cpp_control",
            executable="execute_plan_node",
            name="execute_plan_service",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ]
        )
    ])