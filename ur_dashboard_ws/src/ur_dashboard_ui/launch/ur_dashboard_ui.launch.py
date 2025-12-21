from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="ur_dashboard_ui",
                executable="dashboard_ui",
                name="ur_dashboard_ui",
                output="screen",
            )
        ]
    )
