import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Test Publisher Started. Sending +Z velocity...")

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Move up slowly
        msg.twist.linear.z = 0.1 
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
