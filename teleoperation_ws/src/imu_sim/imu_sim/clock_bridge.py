import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

class ClockBridge(Node):
    def __init__(self):
        super().__init__('clock_bridge')
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info("Clock Bridge Started. Syncing /clock to /joint_states...")

    def joint_callback(self, msg):
        # Create Clock message from JointState timestamp
        clock_msg = Clock()
        clock_msg.clock = msg.header.stamp
        self.pub_clock.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClockBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
