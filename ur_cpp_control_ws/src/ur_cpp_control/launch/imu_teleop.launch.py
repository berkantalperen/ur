from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur_cpp_control",
                executable="imu_sim_node",
                name="imu_simulator",
                output="screen",
            ),
            Node(
                package="ur_cpp_control",
                executable="imu_to_servo_node",
                name="imu_to_servo",
                output="screen",
                parameters=[
                    {
                        "command_topic": "/servo_node/pose_target_cmds",
                        "base_frame": "base_link",
                        "target_position": [0.3, 0.0, 0.3],
                        "max_angular_velocity": 1.5,
                    }
                ],
            ),
        ]
    )
