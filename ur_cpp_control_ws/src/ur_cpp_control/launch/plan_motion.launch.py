from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur_moveit_config") \
        .robot_description(mappings={"ur_type": "ur5e"}) \
        .to_moveit_configs()

    return LaunchDescription([
        Node(
            package="ur_cpp_control",
            executable="plan_request_node",
            name="plan_request_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ]
        )
    ])