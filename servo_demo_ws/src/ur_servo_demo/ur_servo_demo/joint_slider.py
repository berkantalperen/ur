#!/usr/bin/env python3
import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

class JointSliderNode(Node):
    def __init__(self):
        super().__init__("joint_slider_demo")
        
        # --- Configuration ---
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        # Range of motion for sliders (Radians)
        self.limits = {
            "shoulder_pan_joint": (-3.14, 3.14),
            "shoulder_lift_joint": (-3.14, 3.14),
            "elbow_joint": (-3.14, 3.14),
            "wrist_1_joint": (-3.14, 3.14),
            "wrist_2_joint": (-3.14, 3.14),
            "wrist_3_joint": (-3.14, 3.14),
        }
        self.Kp = 4.0  # Proportional Gain (Stiffness)

        # --- State ---
        self.current_joints = {}
        self.target_joints = {}
        self.robot_ready = False

        # --- ROS 2 Interfaces ---
        self.pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")

        self.get_logger().info("Waiting for initial robot state...")

    def joint_cb(self, msg):
        # Update current robot state
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joints[name] = msg.position[i]
        
        # One-time initialization of targets to match robot
        if not self.robot_ready and len(self.current_joints) >= 6:
            self.target_joints = self.current_joints.copy()
            self.robot_ready = True
            self.get_logger().info("Robot state received! Starting GUI...")

    def enable_servo(self):
        # Switch Servo to JOINT_JOG mode
        if self.cli.wait_for_service(timeout_sec=1.0):
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.JOINT_JOG
            self.cli.call_async(req)
            self.get_logger().info("Switched to JOINT_JOG mode.")

    def control_loop(self):
        if not self.robot_ready:
            return

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Calculate P-Control Velocity for each joint
        for name in self.joint_names:
            current = self.current_joints.get(name, 0.0)
            target = self.target_joints.get(name, current)
            
            error = target - current
            
            # Deadband to prevent jitter
            if abs(error) < 0.01:
                velocity = 0.0
            else:
                velocity = self.Kp * error

            msg.joint_names.append(name)
            msg.velocities.append(velocity)

        self.pub.publish(msg)

# --- GUI Class ---
class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("UR5e Joint Jog Demo")
        self.sliders = {}

        # Wait for ROS node to get robot state before building sliders
        self.check_ready()

    def check_ready(self):
        if self.node.robot_ready:
            self.build_gui()
            self.node.enable_servo()
        else:
            self.root.after(100, self.check_ready)

    def build_gui(self):
        row = 0
        for name in self.node.joint_names:
            # Label
            lbl = ttk.Label(self.root, text=name, font=("Arial", 10, "bold"))
            lbl.grid(row=row, column=0, padx=10, pady=5, sticky="w")
            
            # Value Label
            val_lbl = ttk.Label(self.root, text="0.00")
            val_lbl.grid(row=row, column=2, padx=10)

            # Slider
            min_val, max_val = self.node.limits[name]
            initial = self.node.target_joints.get(name, 0.0)
            
            scale = tk.Scale(
                self.root, from_=min_val, to=max_val, orient="horizontal", 
                resolution=0.01, length=300,
                command=lambda val, n=name, l=val_lbl: self.on_slider(val, n, l)
            )
            scale.set(initial) # Set slider to match REAL robot
            scale.grid(row=row, column=1, padx=10)
            
            self.sliders[name] = scale
            row += 1

        # Stop Button
        btn = ttk.Button(self.root, text="STOP / RESET", command=self.emergency_stop)
        btn.grid(row=row, column=0, columnspan=3, pady=20, sticky="ew")

    def on_slider(self, val, name, label_widget):
        float_val = float(val)
        label_widget.config(text=f"{float_val:.2f}")
        # Update ROS node target
        self.node.target_joints[name] = float_val

    def emergency_stop(self):
        # Reset sliders to current robot position (stops motion)
        for name, scale in self.sliders.items():
            current = self.node.current_joints.get(name, 0.0)
            scale.set(current)
            self.node.target_joints[name] = current

def main():
    rclpy.init()
    node = JointSliderNode()

    # Run ROS spinning in a separate thread so GUI doesn't freeze
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Start Control Loop Timer (runs in ROS thread)
    node.create_timer(0.02, node.control_loop) # 50 Hz

    # Start GUI (Must run in main thread)
    root = tk.Tk()
    app = App(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()