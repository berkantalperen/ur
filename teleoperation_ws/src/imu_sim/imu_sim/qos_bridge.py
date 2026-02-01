import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QosBridge(Node):
    def __init__(self):
        super().__init__('qos_bridge')
        
        # 1. FEEDBACK: Driver -> MoveIt
        # We listen to the driver and remove 'isolated_' so MoveIt is happy
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, sub_qos)
        self.pub_joints = self.create_publisher(JointState, '/joint_states_fixed', 10)

        # 2. COMMAND: MoveIt -> Driver
        # We listen to MoveIt and ADD 'isolated_' so the Driver is happy
        self.sub_traj = self.create_subscription(JointTrajectory, '/servo_raw', self.traj_callback, 10)
        self.pub_traj = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        self.get_logger().info("Bridge Active: Translating between Global and Isolated namespaces.")

    def joint_callback(self, msg):
        # Fix Time & Frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # Remove 'isolated_' from names
        msg.name = [n.replace('isolated_', '') for n in msg.name]
        self.pub_joints.publish(msg)

    def traj_callback(self, msg):
        # Add 'isolated_' to names
        msg.joint_names = ["isolated_" + name for name in msg.joint_names]
        self.pub_traj.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()