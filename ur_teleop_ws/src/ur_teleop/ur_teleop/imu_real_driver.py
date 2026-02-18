#!/usr/bin/env python3
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped


class IMURealDriver(Node):
    def __init__(self):
        super().__init__("imu_real_driver")

        self.udp_ip = "0.0.0.0"
        self.udp_port = 4030

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        self.pub = self.create_publisher(PoseStamped, "/arm/hand_pose", qos_profile_sensor_data)

        self._stop = threading.Event()
        self._last_log = 0.0
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"Listening UDP on {self.udp_ip}:{self.udp_port} -> publishing /arm/hand_pose"
        )

    def _parse(self, line: str):
        # HAND_POSE,t_ns,x,y,z,qx,qy,qz,qw,frame_id
        parts = line.split(",")
        if len(parts) < 9:
            return None
        if parts[0].strip() != "HAND_POSE":
            return None

        try:
            _t_ns = int(parts[1])  # optional; not used for ROS time here
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            qx = float(parts[5])
            qy = float(parts[6])
            qz = float(parts[7])
            qw = float(parts[8])
        except Exception:
            return None

        frame_id = parts[9].strip() if len(parts) >= 10 and parts[9].strip() else "world"
        return x, y, z, qx, qy, qz, qw, frame_id

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
                line = data.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                parsed = self._parse(line)
                if parsed is None:
                    continue

                x, y, z, qx, qy, qz, qw, frame_id = parsed

                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_id

                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = z

                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

                self.pub.publish(msg)

                now = time.time()
                if now - self._last_log > 1.0:
                    self._last_log = now
                    self.get_logger().info(
                        f"HAND_POSE p=({x:.3f},{y:.3f},{z:.3f}) "
                        f"q=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}) frame='{frame_id}' from {addr}"
                    )

            except OSError:
                break
            except Exception:
                pass

    def destroy_node(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = IMURealDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
