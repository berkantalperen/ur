import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class ImuSimulator(Node):
    def __init__(self):
        super().__init__("imu_simulator")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("roll_amplitude_rad", 0.6)
        self.declare_parameter("pitch_amplitude_rad", 0.4)
        self.declare_parameter("yaw_amplitude_rad", 0.8)
        self.declare_parameter("angular_speed_rad_s", 0.6)

        self.publisher = self.create_publisher(Imu, "imu/data", 10)
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)
        self.start_time = self.get_clock().now()

    def publish_imu(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        roll_amp = self.get_parameter("roll_amplitude_rad").get_parameter_value().double_value
        pitch_amp = self.get_parameter("pitch_amplitude_rad").get_parameter_value().double_value
        yaw_amp = self.get_parameter("yaw_amplitude_rad").get_parameter_value().double_value
        omega = self.get_parameter("angular_speed_rad_s").get_parameter_value().double_value

        roll = roll_amp * math.sin(omega * t)
        pitch = pitch_amp * math.sin(omega * 0.7 * t)
        yaw = yaw_amp * math.sin(omega * 1.3 * t)
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ImuSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
