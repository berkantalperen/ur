import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.time import Time, Duration

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        
        # 1. Configuration
        # Add 50ms to timestamp to compensate for Docker/Processing lag
        self.time_offset_ns = 50 * 1_000_000 
        
        # 2. Hardcoded backup names for UR5e
        self.backup_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        self.sub = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_states_servo', 10)
        
        self.last_pub_nanos = 0
        self.get_logger().info("Relay Started: Adding 50ms future offset to fix lag.")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        now_nanos = now.nanoseconds
        
        # 1. Monotonic Fix (Prevent time going backward)
        if now_nanos <= self.last_pub_nanos:
            now_nanos = self.last_pub_nanos + 1
        self.last_pub_nanos = now_nanos
        
        # 2. Future Offset (The Fix for "Waiting for robot state")
        # We push the time forward so MoveIt thinks the data is brand new
        final_time_ns = now_nanos + self.time_offset_ns
        
        new_msg = JointState()
        new_msg.header.stamp = Time(nanoseconds=final_time_ns).to_msg()
        
        # 3. Frame ID Fix
        if not msg.header.frame_id:
            new_msg.header.frame_id = 'base_link'
        else:
            new_msg.header.frame_id = msg.header.frame_id

        # 4. Joint Names Check
        if not msg.name:
            if len(msg.position) == 6:
                new_msg.name = self.backup_names
            else:
                return # Invalid message
        else:
            new_msg.name = list(msg.name)

        # 5. Copy Data
        new_msg.position = msg.position
        new_msg.velocity = msg.velocity
        new_msg.effort = msg.effort
        
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateRelay())
    rclpy.shutdown()