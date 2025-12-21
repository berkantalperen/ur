from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur_cpp_control",
            executable="plan_request_node",
            output="screen",
        )
    ])
