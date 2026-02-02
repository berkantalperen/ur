import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointRelay(Node):
    def __init__(self):
        super().__init__('joint_relay')
        self.pub = self.create_publisher(JointState, '/joint_states_servo', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = ""
        new_msg.name, new_msg.position = msg.name, msg.position
        new_msg.velocity, new_msg.effort = msg.velocity, msg.effort
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointRelay())
    rclpy.shutdown()