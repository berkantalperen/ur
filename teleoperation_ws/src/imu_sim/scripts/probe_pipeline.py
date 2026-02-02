import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

class PipelineProbe(Node):
    def __init__(self):
        super().__init__('pipeline_probe')
        # 1. Check if Safety Node is actually sending data
        self.sub_twist = self.create_subscription(
            TwistStamped, '/servo_server/delta_twist_cmds', self.twist_cb, 10)
        # 2. Check if Joint Relay is actually sending data
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states_servo', self.joint_cb, 10)
        
        self.get_logger().info("--- PIPELINE PROBE STARTING ---")

    def twist_cb(self, msg):
        self.get_logger().info(f"[PIPELINE OK] Twist received! Frame: {msg.header.frame_id}", throttle_duration_sec=1.0)

    def joint_cb(self, msg):
        self.get_logger().info(f"[PIPELINE OK] Joint states received!", throttle_duration_sec=2.0)

def main():
    rclpy.init()
    rclpy.spin(PipelineProbe())
    rclpy.shutdown()

if __name__ == '__main__':
    main()