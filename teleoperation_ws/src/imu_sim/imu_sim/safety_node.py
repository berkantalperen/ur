import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from visualization_msgs.msg import Marker
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # 1. Configuration
        self.x_lim = [0.2, 0.6]  # Forward range
        self.y_lim = [-0.4, 0.4] # Side range
        self.z_lim = [0.1, 0.6]  # Height range
        
        self.p_gain = 2.0 # Proportional gain for position error -> velocity
        self.max_lin_vel = 0.5 # m/s

        # 2. State
        self.latest_safe_pose = None
        self.robot_base_frame = 'base_link'
        self.robot_ee_frame = 'tool0'

        # 3. Communication
        # Input from IMU/Interactive Marker
        self.sub_raw = self.create_subscription(PoseStamped, '/teleop/target_raw', self.raw_callback, 10)
        
        # Debug topics
        self.pub_safe = self.create_publisher(PoseStamped, '/teleop/target_safe', 10)
        self.pub_viz  = self.create_publisher(Marker, '/teleop/safety_viz', 10)
        
        # CRITICAL: Publisher for MoveIt Servo (Absolute Topic)
        self.pub_twist = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

        # 4. TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. Control Loop (50Hz)
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Safety Node (Pose Following) Started.")

    def raw_callback(self, msg_raw):
        # --- CLAMPING LOGIC ---
        raw_x = msg_raw.pose.position.x
        raw_y = msg_raw.pose.position.y
        raw_z = msg_raw.pose.position.z

        # Clamp Pos
        safe_x = np.clip(raw_x, self.x_lim[0], self.x_lim[1])
        safe_y = np.clip(raw_y, self.y_lim[0], self.y_lim[1])
        safe_z = np.clip(raw_z, self.z_lim[0], self.z_lim[1])

        # Create Safe Pose
        msg_safe = PoseStamped()
        msg_safe.header = msg_raw.header # Keep timestamp/frame
        msg_safe.header.frame_id = self.robot_base_frame # Force base frame
        msg_safe.pose.position.x = float(safe_x)
        msg_safe.pose.position.y = float(safe_y)
        msg_safe.pose.position.z = float(safe_z)
        msg_safe.pose.orientation = msg_raw.pose.orientation 

        self.latest_safe_pose = msg_safe
        
        self.pub_safe.publish(msg_safe)
        self.publish_visualization(msg_safe.pose.position)

    def control_loop(self):
        if self.latest_safe_pose is None:
            return

        # 1. Get Current Robot Pose
        try:
            trans = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.robot_ee_frame,
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        current_pos = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])

        target_pos = np.array([
            self.latest_safe_pose.pose.position.x,
            self.latest_safe_pose.pose.position.y,
            self.latest_safe_pose.pose.position.z
        ])

        # 2. Compute Error
        error = target_pos - current_pos
        distance = np.linalg.norm(error)

        # 3. Compute Velocity Command (P-Controller)
        # Simple deadband
        if distance < 0.005: # 5mm
            vel = np.zeros(3)
        else:
            vel = error * self.p_gain
            
            # Clamp Max Velocity
            vel_mag = np.linalg.norm(vel)
            if vel_mag > self.max_lin_vel:
                vel = vel * (self.max_lin_vel / vel_mag)

        # 4. Publish Twist
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_base_frame
        msg.twist.linear.x = vel[0]
        msg.twist.linear.y = vel[1]
        msg.twist.linear.z = vel[2]
        
        # Keep angular zero for now (translation only demo)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub_twist.publish(msg)

    def publish_visualization(self, safe_pos):
        # 1. Draw the Safety Box (Red Wireframe)
        marker_box = Marker()
        marker_box.header.frame_id = self.robot_base_frame
        marker_box.id = 0
        marker_box.type = Marker.LINE_LIST
        marker_box.action = Marker.ADD
        marker_box.scale.x = 0.005 # Line width
        marker_box.color.r = 1.0; marker_box.color.a = 0.5

        # Define the 8 corners
        x_min, x_max = self.x_lim
        y_min, y_max = self.y_lim
        z_min, z_max = self.z_lim
        
        corners = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_min, y_max, z_min], [x_max, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_min, y_max, z_max], [x_max, y_max, z_max]
        ]
        
        # Define the 12 lines (pairs of points)
        lines = [
            # Bottom face
            corners[0], corners[1], corners[2], corners[3],
            corners[0], corners[2], corners[1], corners[3],
            # Top face
            corners[4], corners[5], corners[6], corners[7],
            corners[4], corners[6], corners[5], corners[7],
            # Vertical pillars
            corners[0], corners[4], corners[1], corners[5],
            corners[2], corners[6], corners[3], corners[7]
        ]
        
        for p in lines:
            pt = Point()
            pt.x = float(p[0]); pt.y = float(p[1]); pt.z = float(p[2])
            marker_box.points.append(pt)

        self.pub_viz.publish(marker_box)

        # 2. Draw the "Clamped" Goal (Blue Sphere)
        marker_pt = Marker()
        marker_pt.header.frame_id = self.robot_base_frame
        marker_pt.id = 1
        marker_pt.type = Marker.SPHERE
        marker_pt.action = Marker.ADD
        marker_pt.scale.x = 0.05; marker_pt.scale.y = 0.05; marker_pt.scale.z = 0.05
        marker_pt.color.b = 1.0; marker_pt.color.a = 1.0 # Blue
        
        marker_pt.pose.position = safe_pos
        
        self.pub_viz.publish(marker_pt)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()