#!/usr/bin/env python3
import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


HELP = """
UR MoveIt Servo keyboard (TWIST):
  a/d : -X / +X
  w/s : +Y / -Y
  r/f : +Z / -Z
  q/e : +Yaw / -Yaw
  x   : stop (zero twist)
Ctrl+C quits.
"""


class ServoKeyboard(Node):
    def __init__(self):
        super().__init__("ur_servo_keyboard")

        self.pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")

        self.frame_id = "tool0"  # matches your servo yaml robot_link_command_frame
        self.lin = 0.1          # m/s
        self.ang = 0.3           # rad/s

        self.get_logger().info("Waiting for /servo_node/switch_command_type ...")
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available. Is servo_node running?")
            raise RuntimeError("switch_command_type service missing")

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST  # == 1
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.result() or not future.result().success:
            self.get_logger().warn("Failed to set command type to TWIST (continuing).")

        self.get_logger().info("Ready. Publish keys at ~50 Hz when pressed.")

    def publish_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = lx
        msg.twist.linear.y = ly
        msg.twist.linear.z = lz
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az
        self.pub.publish(msg)


def get_key():
    # non-blocking single-char read
    if select.select([sys.stdin], [], [], 0.02)[0]:
        return sys.stdin.read(1)
    return None


def main():
    # Hard guard: this node MUST be run in a real terminal
    if not sys.stdin.isatty():
        print("servo_keyboard must be run in an interactive TTY.\n"
              "Run it like:\n"
              "  ros2 run ur_servo_demo servo_keyboard\n")
        return 1

    rclpy.init()
    node = ServoKeyboard()

    print(HELP)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    # simple “press and hold” loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            k = get_key()
            if not k:
                continue

            if k == "w":
                node.publish_twist(ly=+node.lin)
            elif k == "s":
                node.publish_twist(ly=-node.lin)
            elif k == "a":
                node.publish_twist(lx=-node.lin)
            elif k == "d":
                node.publish_twist(lx=+node.lin)
            elif k == "r":
                node.publish_twist(lz=-node.lin)
            elif k == "f":
                node.publish_twist(lz=+node.lin)
            elif k == "q":
                node.publish_twist(az=+node.ang)
            elif k == "e":
                node.publish_twist(az=-node.ang)
            elif k == "x":
                node.publish_twist()  # zero
            time.sleep(0.02)  # ~50 Hz
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
