import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SimpleListener(Node):
    def __init__(self):
        super().__init__('simple_listener')
        # Exact QoS from your config
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            qos)
        print("Listening for /joint_states...")

    def listener_callback(self, msg):
        print(f"RECEIVED! Timestamp: {msg.header.stamp.sec}")

rclpy.init()
node = SimpleListener()
rclpy.spin(node)