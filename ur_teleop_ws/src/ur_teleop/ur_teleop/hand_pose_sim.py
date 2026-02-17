#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class HandPoseSimPublisher(Node):
    """
    Simple simulator that publishes /arm/hand_pose (PoseStamped).
    Produces smooth, bounded positions consistent with arm lengths.
    """
    def __init__(self):
        super().__init__("hand_pose_sim_publisher")

        # Topic must match what pose_controller expects
        self.pub = self.create_publisher(PoseStamped, "/arm/hand_pose", qos_profile_sensor_data)

        # "Arm lengths" & Parameters
        self.reach = 2.0 + 1.5 + 0.8  # max reach
        self.scale = 0.02             # Scaling factor to keep it within safe robot workspace
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        
        # Motion parameters
        self.radius = 0.65 * self.reach
        self.z0 = 0.25 * self.reach
        self.z_amp = 0.10 * self.reach
        
        self.frame_id = "base_link" # Changed to base_link so it works with the controller logic easier
        self.t0 = time.time()

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"Publishing /arm/hand_pose at {self.rate_hz:.1f} Hz")

    def _tick(self):
        t = time.time() - self.t0

        # Lissajous-ish trajectory
        x = self.radius * math.cos(0.35 * t) + 0.10 * self.reach * math.cos(1.10 * t)
        y = self.radius * math.sin(0.35 * t) + 0.10 * self.reach * math.sin(0.90 * t)
        z = self.z0 + self.z_amp * math.sin(0.55 * t)

        x = max(x, 0.15 * self.reach) # Forward bias

        # Orientation Logic
        vx = -self.radius * 0.35 * math.sin(0.35 * t)
        vy =  self.radius * 0.35 * math.cos(0.35 * t)
        vz =  self.z_amp * 0.55 * math.cos(0.55 * t)
        
        vnorm = math.sqrt(vx*vx + vy*vy + vz*vz)
        if vnorm < 1e-6:
            fx, fy, fz = 1.0, 0.0, 0.0
        else:
            fx, fy, fz = vx / vnorm, vy / vnorm, vz / vnorm

        # Build Rotation Matrix
        up = (0.0, 0.0, 1.0)
        dot = fx*up[0] + fy*up[1] + fz*up[2]
        if abs(dot) > 0.95: up = (0.0, 1.0, 0.0)

        # y = up x x
        yx = up[1]*fz - up[2]*fy
        yy = up[2]*fx - up[0]*fz
        yz = up[0]*fy - up[1]*fx
        yn = math.sqrt(yx*yx + yy*yy + yz*yz)
        yx, yy, yz = yx/yn, yy/yn, yz/yn

        # z = x x y
        zx = fy*yz - fz*yy
        zy = fz*yx - fx*yz
        zz = fx*yy - fy*yx

        Rm = [[fx, yx, zx], [fy, yy, zy], [fz, yz, zz]]

        # Add roll
        roll = 0.35 * math.sin(0.8 * t)
        rot = R.from_matrix(Rm) * R.from_rotvec([roll, 0.0, 0.0])
        qx, qy, qz, qw = rot.as_quat()

        # Publish
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = self.scale * x
        msg.pose.position.y = self.scale * y
        msg.pose.position.z = self.scale * z
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HandPoseSimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()