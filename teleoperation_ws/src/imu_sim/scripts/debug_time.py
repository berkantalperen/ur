import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class TimeDebugger(Node):
    def __init__(self):
        super().__init__('time_debugger')
        # Change this to '/joint_states' if you aren't using the relay
        self.sub = self.create_subscription(JointState, '/joint_states_servo', self.cb, 10)

    def cb(self, msg):
        # 1. Get current system time
        now = self.get_clock().now()
        
        # 2. Get message time
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        # 3. Calculate difference in seconds
        diff = (now - msg_time).nanoseconds / 1e9
        
        # 4. Print Diagnosis
        status = "OK"
        if diff < -0.01: status = "FUTURE TIMESTAMP (Clock Skew)"
        if diff > 0.1:   status = "OLD DATA (Lag)"
        
        print(f"Now: {now.nanoseconds} | Msg: {msg_time.nanoseconds} | Diff: {diff:.4f}s -> {status}")

rclpy.init()
rclpy.spin(TimeDebugger())