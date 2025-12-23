import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quaternion(q: Tuple[float, float, float, float]):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_conjugate(q: Tuple[float, float, float, float]):
    x, y, z, w = q
    return -x, -y, -z, w


def quaternion_multiply(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w


def quaternion_to_axis_angle(q: Tuple[float, float, float, float]):
    x, y, z, w = normalize_quaternion(q)
    w = max(min(w, 1.0), -1.0)
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-6:
        return 1.0, 0.0, 0.0, 0.0
    return x / s, y / s, z / s, angle


def clamp_vector(x: float, y: float, z: float, max_norm: float):
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= max_norm or norm == 0.0:
        return x, y, z
    scale = max_norm / norm
    return x * scale, y * scale, z * scale


class ImuServo(Node):
    def __init__(self):
        super().__init__("imu_servo")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("command_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("imu_timeout_s", 0.5)

        self.latest_orientation: Optional[Tuple[float, float, float, float]] = None
        self.latest_stamp = None

        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(TwistStamped, command_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.publish_twist)

    def imu_callback(self, msg: Imu):
        self.latest_orientation = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.latest_stamp = msg.header.stamp

    def publish_twist(self):
        if self.latest_orientation is None or self.latest_stamp is None:
            return

        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(self.latest_stamp)
        if (now - stamp).nanoseconds * 1e-9 > self.get_parameter("imu_timeout_s").get_parameter_value().double_value:
            return

        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, ee_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        current_q = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        target_q = normalize_quaternion(self.latest_orientation)
        error_q = quaternion_multiply(target_q, quaternion_conjugate(current_q))
        axis_x, axis_y, axis_z, angle = quaternion_to_axis_angle(error_q)

        kp = self.get_parameter("kp").get_parameter_value().double_value
        max_vel = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        ang_x = kp * angle * axis_x
        ang_y = kp * angle * axis_y
        ang_z = kp * angle * axis_z
        ang_x, ang_y, ang_z = clamp_vector(ang_x, ang_y, ang_z, max_vel)

        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = base_frame
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = ang_x
        twist.twist.angular.y = ang_y
        twist.twist.angular.z = ang_z
        self.publisher.publish(twist)


def main():
    rclpy.init()
    node = ImuServo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
