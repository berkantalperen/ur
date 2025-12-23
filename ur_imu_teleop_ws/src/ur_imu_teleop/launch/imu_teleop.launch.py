from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur_imu_teleop",
                executable="imu_sim_node",
                name="imu_sim_node",
                output="screen",
                parameters=[
                    {
                        "frame_id": "imu_link",
                        "publish_rate_hz": 50.0,
                        "roll_amplitude_rad": 0.6,
                        "pitch_amplitude_rad": 0.4,
                        "yaw_amplitude_rad": 0.8,
                        "angular_speed_rad_s": 0.6,
                    }
                ],
            ),
            Node(
                package="ur_imu_teleop",
                executable="imu_servo_node",
                name="imu_servo_node",
                output="screen",
                parameters=[
                    {
                        "imu_topic": "/imu/data",
                        "command_topic": "/servo_node/delta_twist_cmds",
                        "base_frame": "base_link",
                        "ee_frame": "tool0",
                        "publish_rate_hz": 50.0,
                        "kp": 2.0,
                        "max_angular_velocity": 1.0,
                        "imu_timeout_s": 0.5,
                        "tf_warn_throttle_s": 2.0,
                    }
                ],
            ),
        ]
    )
