#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class IMURealDriver(Node):
    def __init__(self):
        super().__init__("imu_real_driver")
        self.pub = self.create_publisher(PoseStamped, "/arm/hand_pose", 10)
        self.get_logger().info("REAL IMU DRIVER STARTED. (Waiting for Xsens implementation...)")

    def publish_data(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = IMURealDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()