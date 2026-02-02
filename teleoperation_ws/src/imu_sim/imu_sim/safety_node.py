import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import tf2_ros

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Listening to the Input Marker
        self.sub = self.create_subscription(PoseStamped, '/teleop/target_raw', self.target_cb, 10)
        
        # PUBLISHING TO THE SERVO SERVER (Absolute Topic)
        self.pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        
        self.target_pos = None
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info("--- Safety Node: Ready ---")

    def target_cb(self, msg):
        # Update target from marker
        self.target_pos = np.array([
            np.clip(msg.pose.position.x, 0.2, 0.8), # Expanded range
            np.clip(msg.pose.position.y, -0.6, 0.6),
            np.clip(msg.pose.position.z, 0.1, 0.8)
        ])

    def control_loop(self):
        if self.target_pos is None: return
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            current = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            error = self.target_pos - current
            dist = np.linalg.norm(error)
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            if dist > 0.01: # 1cm Deadband
                vel = error * 2.0
                msg.twist.linear.x = vel[0]
                msg.twist.linear.y = vel[1]
                msg.twist.linear.z = vel[2]
                self.pub.publish(msg)
                
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SafetyNode())
    rclpy.shutdown()