#!/usr/bin/env python3
import sys
import threading
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from tf2_ros import Buffer, TransformListener

# --- MATH HELPERS ---
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# --- ROS NODE ---
class PoseTeleopNode(Node):
    def __init__(self):
        super().__init__("pose_teleop_node")
        
        # 1. Publisher to Servo
        self.pub = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        self.cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        
        # 2. Subscriber to Input (Sim or Real IMU)
        # We start with hand_pose = None. The robot CANNOT move until this updates.
        self.sub_hand = self.create_subscription(
            PoseStamped, 
            "/arm/hand_pose", 
            self.hand_pose_cb, 
            qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.robot_pose = None   # Current Real Robot Pose
        self.hand_pose = None    # Latest Input Pose (from Topic)
        self.clutch_active = False
        
        # Snapshots
        self.robot_start_pose = None
        self.hand_start_pose = None
        
        # Timers
        self.create_timer(0.02, self.control_loop) # 50Hz Control
        self.create_timer(0.1, self.update_tf)     # 10Hz TF

    def hand_pose_cb(self, msg):
        """
        This is the ONLY way self.hand_pose gets updated now.
        If no message comes in, self.hand_pose stays None.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        r, p, y_ang = euler_from_quaternion(qx, qy, qz, qw)
        self.hand_pose = [x, y, z, r, p, y_ang]

    def update_tf(self):
        try:
            t = self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            r, p, y = euler_from_quaternion(
                t.transform.rotation.x, t.transform.rotation.y, 
                t.transform.rotation.z, t.transform.rotation.w
            )
            self.robot_pose = [tx, ty, tz, r, p, y]
        except Exception:
            pass

    def set_mode_pose(self):
        if self.cli.wait_for_service(timeout_sec=1.0):
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.POSE
            self.cli.call_async(req)
            self.get_logger().info("Switched to POSE mode")

    def activate_clutch(self):
        if self.robot_pose is None:
            self.get_logger().warn("Cannot clutch: Robot state unknown!")
            return
        
        # CRITICAL CHECK: If topic is empty, we cannot clutch.
        if self.hand_pose is None:
            self.get_logger().warn("Cannot clutch: No Hand Pose data received! (Is sim:=true or driver running?)")
            return

        self.robot_start_pose = list(self.robot_pose)
        self.hand_start_pose = list(self.hand_pose)
        self.clutch_active = True
        self.get_logger().info("Clutch ENGAGED.")

    def deactivate_clutch(self):
        self.clutch_active = False
        self.get_logger().info("Clutch RELEASED.")

    def control_loop(self):
        if not self.clutch_active or self.hand_pose is None:
            return

        # Calculate Delta
        delta = [c - s for c, s in zip(self.hand_pose, self.hand_start_pose)]

        # Apply to Robot Start
        target = [r + d for r, d in zip(self.robot_start_pose, delta)]

        # Publish
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        
        q = quaternion_from_euler(target[3], target[4], target[5])
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        
        self.pub.publish(msg)

# --- GUI ---
class TeleopApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("UR Teleop Interface")
        self.root.geometry("400x350")

        # Status
        status_frame = ttk.LabelFrame(root, text="System Status", padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_robot = ttk.Label(status_frame, text="Robot: Waiting...")
        self.lbl_robot.pack(anchor="w")
        
        self.lbl_hand = ttk.Label(status_frame, text="Hand Input: NO DATA (Topic Empty)", foreground="red")
        self.lbl_hand.pack(anchor="w")

        # Controls
        ctrl_frame = ttk.LabelFrame(root, text="Controls", padding=10)
        ctrl_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(ctrl_frame, text="Set Mode: POSE", command=node.set_mode_pose).pack(fill="x", pady=2)
        
        self.btn_clutch = tk.Button(ctrl_frame, text="HOLD TO MOVE (Clutch)", bg="lightgray", height=3)
        self.btn_clutch.pack(fill="x", pady=10)
        
        self.btn_clutch.bind('<ButtonPress-1>', self.on_clutch_press)
        self.btn_clutch.bind('<ButtonRelease-1>', self.on_clutch_release)

        self.root.after(100, self.update_ui)

    def on_clutch_press(self, event):
        self.node.activate_clutch()
        if self.node.clutch_active:
            self.btn_clutch.config(bg="green", text="MOVING...")
        else:
            self.btn_clutch.config(bg="orange", text="NO DATA / ERROR")

    def on_clutch_release(self, event):
        self.node.deactivate_clutch()
        self.btn_clutch.config(bg="lightgray", text="HOLD TO MOVE (Clutch)")

    def update_ui(self):
        # Update Robot Status
        if self.node.robot_pose:
            p = self.node.robot_pose
            self.lbl_robot.config(text=f"Robot: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f}")
        
        # Update Hand Input Status
        if self.node.hand_pose:
            h = self.node.hand_pose
            self.lbl_hand.config(text=f"Hand Input: Active (X={h[0]:.2f})", foreground="green")
        else:
            self.lbl_hand.config(text="Hand Input: NO DATA (Topic Empty)", foreground="red")
            
        self.root.after(100, self.update_ui)

def main():
    rclpy.init()
    node = PoseTeleopNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    root = tk.Tk()
    app = TeleopApp(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()