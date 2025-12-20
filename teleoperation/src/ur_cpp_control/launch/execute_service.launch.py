from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur_cpp_control",
            executable="execute_plan_node",
            name="execute_plan_service",
            output="screen"
        )
    ])
