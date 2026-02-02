import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

class IMUServer(Node):
    def __init__(self):
        super().__init__('imu_sim_server')
        self.server = InteractiveMarkerServer(self, 'imu_input')
        self.pub = self.create_publisher(PoseStamped, '/teleop/target_raw', 10)
        self.pos = [0.4, 0.0, 0.4] 
        
        self.make_marker()
        self.create_timer(0.05, self.publish_pose)
        self.get_logger().info("--- Input Marker: Ready ---")

    def make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "user_target"
        int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z = self.pos
        int_marker.scale = 0.2

        # 1. The Green Sphere Visual
        visual = InteractiveMarkerControl(always_visible=True)
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.g = 1.0; sphere.color.a = 0.8
        visual.markers.append(sphere)
        int_marker.controls.append(visual)

        # 2. Add Directional Controls (Arrows)
        self.add_axis_control(int_marker, 1, 0, 0, "move_x") # X Axis
        self.add_axis_control(int_marker, 0, 1, 0, "move_y") # Y Axis
        self.add_axis_control(int_marker, 0, 0, 1, "move_z") # Z Axis

        # 3. Add 3D Free Move (Drag the sphere itself)
        move_3d = InteractiveMarkerControl()
        move_3d.name = "move_3d"
        move_3d.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(move_3d)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def add_axis_control(self, marker, x, y, z, name):
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = float(x)
        control.orientation.y = float(y)
        control.orientation.z = float(z)
        control.name = name
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

    def process_feedback(self, fb):
        p = fb.pose.position
        self.pos = [p.x, p.y, p.z]

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.pos
        # Default orientation pointing down
        msg.pose.orientation.x = 1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IMUServer())
    rclpy.shutdown()