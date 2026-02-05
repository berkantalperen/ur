import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class UniversalBridge(Node):
    def __init__(self):
        super().__init__('universal_bridge')

        # --- CONFIGURATION ---
        # 1. Kinematic Order (What MoveIt Expects)
        self.kinematic_order = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # 2. Alphabetical Order (What Driver Sends/Expects)
        self.alpha_order = [
            "elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # Mappings
        # Input: alpha[source_i] -> kinematic[target_i]
        # kinematic[0]=alpha[2], kinematic[1]=alpha[1], kinematic[2]=alpha[0], ...
        self.input_map = [2, 1, 0, 3, 4, 5]
        
        # Output: kinematic[source_i] -> alpha[target_i]  
        # For reordering MoveIt output (kinematic) -> Driver input (alpha)
        # alpha[0]=kinematic[2], alpha[1]=kinematic[1], alpha[2]=kinematic[0]
        self.output_map = [2, 1, 0, 3, 4, 5]  # alpha_target_idx -> kinematic_source_idx

        # --- STATE ---
        self.latest_js_msg = None

        # --- QoS PROFILES ---
        # Match the joint_state_broadcaster's QoS (RELIABLE + TRANSIENT_LOCAL)
        js_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # For publishing to servo - MoveIt Servo uses BEST_EFFORT subscription
        # A BEST_EFFORT subscriber CANNOT receive from a RELIABLE publisher!
        servo_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- COMMUNICATIONS ---
        # 1. Input: Driver -> Bridge (use matching QoS)
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.js_callback, js_qos)
        
        # 2. Output: Bridge -> MoveIt (use BEST_EFFORT to match servo's subscription)
        self.pub_js = self.create_publisher(JointState, '/joint_states_servo', servo_qos)
        
        # QoS for receiving from servo (TRANSIENT_LOCAL publisher)
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 3. Input: MoveIt Servo -> Bridge (Float64MultiArray velocities in kinematic order)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, 
            '/servo/velocity_raw',  # Servo outputs here
            self.cmd_callback, 
            cmd_qos
        )
        
        # 4. Output: Bridge -> forward_velocity_controller (Float64MultiArray in alpha order)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

        # --- TIMER (The Master Clock) ---
        # We publish JointStates at 50Hz regardless of Driver Jitter
        self.create_timer(0.02, self.publish_clean_state)
        
        # Debug timer to show status even when not receiving messages
        self.create_timer(2.0, self.status_timer_callback)
        
        self.get_logger().info("Universal Bridge Started: Master Clock + Velocity Reordering Active")
        self.get_logger().info(f"Subscribed to: /joint_states (QoS: RELIABLE, TRANSIENT_LOCAL)")
        self.get_logger().info(f"Publishing to: /joint_states_servo (QoS: BEST_EFFORT, VOLATILE)")
        self.msg_count = 0  # Debug counter
        
    def status_timer_callback(self):
        """Report status every 2 seconds"""
        if self.latest_js_msg is None:
            self.get_logger().warn("No /joint_states messages received yet!")
        else:
            self.get_logger().info(f"Status: Received {self.msg_count} messages total")

    def js_callback(self, msg):
        # Just store the data, don't publish yet. Let the Timer handle timing.
        self.latest_js_msg = msg
        self.msg_count += 1
        if self.msg_count % 500 == 1:  # Log every 500 messages (~1 sec at 500Hz)
            self.get_logger().info(f"Received /joint_states msg #{self.msg_count}")

    def publish_clean_state(self):
        if self.latest_js_msg is None:
            return

        # 1. Create Clean Message
        clean_msg = JointState()
        clean_msg.header.stamp = self.get_clock().now().to_msg() # STRICTLY NOW
        clean_msg.header.frame_id = 'base_link'
        clean_msg.name = self.kinematic_order
        
        # 2. Initialize Lists
        clean_msg.position = [0.0] * 6
        clean_msg.velocity = [0.0] * 6  # Include velocities - some monitors need them
        clean_msg.effort = []    # Don't send effort

        # 3. Reorder Data (Alpha -> Kinematic)
        # If driver sends partial message, we skip to avoid crash
        if len(self.latest_js_msg.position) < 6:
            return

        try:
            for target_i, source_i in enumerate(self.input_map):
                clean_msg.position[target_i] = self.latest_js_msg.position[source_i]
                # Also reorder velocities if present
                if len(self.latest_js_msg.velocity) >= 6:
                    clean_msg.velocity[target_i] = self.latest_js_msg.velocity[source_i]
            
            self.pub_js.publish(clean_msg)
        except Exception:
            pass # Ignore glitches

    def cmd_callback(self, msg):
        """
        Receive velocity commands from MoveIt Servo in kinematic order,
        reorder to alphabetical order, and forward to the driver.
        """
        if len(msg.data) < 6:
            return

        # Reorder velocities: Kinematic -> Alpha
        new_msg = Float64MultiArray()
        new_msg.data = [0.0] * 6
        
        for alpha_idx, kinematic_idx in enumerate(self.output_map):
            new_msg.data[alpha_idx] = msg.data[kinematic_idx]
        
        self.pub_cmd.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UniversalBridge())
    rclpy.shutdown()