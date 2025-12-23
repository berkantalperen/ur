from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_simulator = Node(
        package="ur_cpp_control",
        executable="imu_simulator_node",
        name="imu_simulator_node",
        output="screen",
        parameters=[
            {
                "imu_topic": "imu/data",
                "frame_id": "imu_link",
                "publish_rate_hz": 50.0,
                "yaw_rate_rad_s": 0.6,
            }
        ],
    )

    imu_to_servo = Node(
        package="ur_cpp_control",
        executable="imu_to_servo_node",
        name="imu_to_servo_node",
        output="screen",
        parameters=[
            {
                "imu_topic": "imu/data",
                "command_topic": "/servo_node/delta_twist_cmds",
                "base_frame": "base_link",
                "wrist_frame": "wrist_3_link",
                "max_angular_velocity": 1.0,
                "control_rate_hz": 100.0,
                "orientation_gain": 2.0,
                "imu_timeout_s": 0.5,
            }
        ],
    )

    return LaunchDescription([
        imu_simulator,
        imu_to_servo,
    ])
