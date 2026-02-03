import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.time import Time

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
        # Input: Alpha(Source) -> Kinematic(Target)
        # Alpha: [Elbow(0), Lift(1), Pan(2), W1(3), W2(4), W3(5)]
        # Target Pan(0) comes from Source(2)
        self.input_map = [2, 1, 0, 3, 4, 5] 
        
        # Output: Kinematic(Source) -> Alpha(Target)
        # We perform the exact same swap in reverse (Symmetric swap)
        self.output_map = [2, 1, 0, 3, 4, 5]

        # --- STATE ---
        self.latest_js_msg = None

        # --- COMMUNICATIONS ---
        # 1. Input: Driver -> Bridge
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)
        
        # 2. Output: Bridge -> MoveIt (Fixed Rate Publisher)
        self.pub_js = self.create_publisher(JointState, '/joint_states_servo', 10)
        
        # 3. Input: MoveIt -> Bridge
        self.sub_cmd = self.create_subscription(JointTrajectory, '/servo/trajectory_raw', self.cmd_callback, 10)
        
        # 4. Output: Bridge -> Driver
        self.pub_cmd = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # --- TIMER (The Master Clock) ---
        # We publish JointStates at 50Hz regardless of Driver Jitter
        self.create_timer(0.02, self.publish_clean_state)
        
        self.get_logger().info("Universal Bridge Started: Master Clock + Reordering Active")

    def js_callback(self, msg):
        # Just store the data, don't publish yet. Let the Timer handle timing.
        self.latest_js_msg = msg

    def publish_clean_state(self):
        if self.latest_js_msg is None:
            return

        # 1. Create Clean Message
        clean_msg = JointState()
        clean_msg.header.stamp = self.get_clock().now().to_msg() # STRICTLY NOW
        clean_msg.header.frame_id = 'base_link'
        clean_msg.name = self.kinematic_order
        
        # 2. Initialize Empty Lists
        clean_msg.position = [0.0] * 6
        clean_msg.velocity = [0.0] * 6
        clean_msg.effort = [0.0] * 6

        # 3. Reorder Data (Alpha -> Kinematic)
        # If driver sends partial message, we skip to avoid crash
        if len(self.latest_js_msg.position) < 6:
            return

        try:
            for target_i, source_i in enumerate(self.input_map):
                clean_msg.position[target_i] = self.latest_js_msg.position[source_i]
                if self.latest_js_msg.velocity:
                    clean_msg.velocity[target_i] = self.latest_js_msg.velocity[source_i]
                if self.latest_js_msg.effort:
                    clean_msg.effort[target_i] = self.latest_js_msg.effort[source_i]
            
            self.pub_js.publish(clean_msg)
        except Exception:
            pass # Ignore glitches

    def cmd_callback(self, msg):
        # MoveIt sends Kinematic -> We convert to Alpha -> Send to Driver
        if not msg.points:
            return

        new_msg = JointTrajectory()
        new_msg.header = msg.header
        new_msg.joint_names = self.alpha_order # Force Alpha Names
        
        for point in msg.points:
            if len(point.positions) < 6:
                continue

            new_point = JointTrajectoryPoint()
            new_point.time_from_start = point.time_from_start
            new_point.positions = [0.0] * 6
            new_point.velocities = [0.0] * 6 if point.velocities else []
            new_point.accelerations = [0.0] * 6 if point.accelerations else []
            new_point.effort = [] 

            # Reorder (Kinematic -> Alpha)
            for target_i, source_i in enumerate(self.output_map):
                new_point.positions[target_i] = point.positions[source_i]
                
                if new_point.velocities:
                    new_point.velocities[target_i] = point.velocities[source_i]
                if new_point.accelerations:
                    new_point.accelerations[target_i] = point.accelerations[source_i]

            new_msg.points.append(new_point)
        
        self.pub_cmd.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UniversalBridge())
    rclpy.shutdown()