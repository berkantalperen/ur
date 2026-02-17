#!/usr/bin/env python3
import sys
import threading
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from tf2_ros import Buffer, TransformListener

# --- Math Helpers (Euler <-> Quaternion) ---
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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
    """
    Convert an Euler angle to a quaternion.
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# --- ROS Node ---
class PoseDemoNode(Node):
    def __init__(self):
        super().__init__("pose_demo_node")
        
        # 1. Communications
        # Note: Default pose topic for moveit_servo is ~/pose_target_cmds
        self.pub = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        self.cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        
        # 2. TF Listener (To get start position)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. State
        self.current_pose = None # [x, y, z, r, p, y]
        self.target_pose = None  # [x, y, z, r, p, y]
        self.robot_ready = False

        self.get_logger().info("Waiting for TF (base_link -> tool0)...")
        self.check_tf_timer = self.create_timer(1.0, self.check_initial_pose)

    def check_initial_pose(self):
        if self.robot_ready:
            return
            
        try:
            # Look up the transform
            t = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time())
            
            # Extract Position
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            
            # Extract Orientation (Quaternion -> Euler)
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            r, p, y = euler_from_quaternion(qx, qy, qz, qw)
            
            self.current_pose = [tx, ty, tz, r, p, y]
            self.target_pose = list(self.current_pose)
            self.robot_ready = True
            
            self.get_logger().info(f"Got Initial Pose: {self.current_pose}")
            self.check_tf_timer.cancel()
            
            # Start publishing loop
            self.create_timer(0.02, self.control_loop) # 50Hz

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    def enable_servo(self):
        if self.cli.wait_for_service(timeout_sec=1.0):
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.POSE # Mode 2
            self.cli.call_async(req)
            self.get_logger().info("Switched to POSE mode.")

    def control_loop(self):
        if not self.robot_ready:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Target [x, y, z, r, p, y]
        tx, ty, tz, tr, tp, ty_yaw = self.target_pose
        
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = tz
        
        q = quaternion_from_euler(tr, tp, ty_yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub.publish(msg)

# --- GUI Class ---
class PoseApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("UR5e Cartesian Pose Control")
        self.vars = []
        
        self.wait_for_robot()

    def wait_for_robot(self):
        if self.node.robot_ready:
            self.build_gui()
            self.node.enable_servo()
        else:
            lbl = ttk.Label(self.root, text="Waiting for TF (Robot State)...", font=("Arial", 12))
            lbl.pack(padx=20, pady=20)
            self.root.after(200, lambda: [lbl.pack_forget(), self.wait_for_robot()])

    def build_gui(self):
        # Configuration for sliders
        # We allow +/- 30cm for position and +/- 180 deg for rotation
        start_pose = self.node.current_pose
        
        params = [
            ("Position X (m)", start_pose[0], start_pose[0]-0.3, start_pose[0]+0.3),
            ("Position Y (m)", start_pose[1], start_pose[1]-0.3, start_pose[1]+0.3),
            ("Position Z (m)", start_pose[2], start_pose[2]-0.3, start_pose[2]+0.3),
            ("Roll (deg)",  math.degrees(start_pose[3]), -180, 180),
            ("Pitch (deg)", math.degrees(start_pose[4]), -180, 180),
            ("Yaw (deg)",   math.degrees(start_pose[5]), -180, 180),
        ]

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        for i, (name, val, min_v, max_v) in enumerate(params):
            # Label
            ttk.Label(main_frame, text=name, width=15).grid(row=i, column=0, sticky="w")
            
            # Value Var
            var = tk.DoubleVar(value=val)
            self.vars.append(var)
            
            # Value Label
            val_lbl = ttk.Label(main_frame, text=f"{val:.2f}")
            val_lbl.grid(row=i, column=2, padx=5)
            
            # Slider
            scale = ttk.Scale(
                main_frame, from_=min_v, to=max_v, orient="horizontal", length=300, variable=var,
                command=lambda v, idx=i, l=val_lbl: self.on_slider(v, idx, l)
            )
            scale.grid(row=i, column=1, padx=5, pady=5)

        ttk.Button(main_frame, text="STOP / RESET", command=self.reset).grid(row=6, column=0, columnspan=3, pady=20, sticky="ew")

    def on_slider(self, val, idx, lbl):
        v = float(val)
        lbl.config(text=f"{v:.2f}")
        
        # Update Node Target
        # If it's index 3,4,5 (RPY), convert deg->rad
        target = list(self.node.target_pose)
        if idx >= 3:
            target[idx] = math.radians(v)
        else:
            target[idx] = v
            
        self.node.target_pose = target

    def reset(self):
        # Hard reset to where we started the script
        # Note: This doesn't move the robot BACK, it just resets the target to the CURRENT slider values 
        # (or you can implement logic to pull the robot back to start)
        pass

def main():
    rclpy.init()
    node = PoseDemoNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = PoseApp(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()