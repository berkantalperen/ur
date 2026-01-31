import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import numpy as np

class IMUSimServer(Node):
    def __init__(self):
        super().__init__('imu_sim_server')
        
        self.server = InteractiveMarkerServer(self, 'imu_sim_input')
        
        # PUBLISH TO RAW TOPIC
        self.pub_raw = self.create_publisher(PoseStamped, '/teleop/target_raw', 10)
        
        # Initial Position (Target)
        self.target_pos = np.array([0.4, 0.0, 0.4]) 

        self.create_input_marker(self.target_pos)
        self.server.applyChanges()
        
        self.create_timer(0.02, self.publish_loop)
        self.get_logger().info("Input Node Ready. Publish to /teleop/target_raw")

    def create_input_marker(self, pos):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "user_input"
        int_marker.scale = 0.2
        int_marker.pose.position.x = pos[0]
        int_marker.pose.position.y = pos[1]
        int_marker.pose.position.z = pos[2]

        # Visual: GREEN Sphere (Means "Go!")
        control = InteractiveMarkerControl()
        control.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.08; sphere.scale.y = 0.08; sphere.scale.z = 0.08
        sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0; sphere.color.a = 0.8
        control.markers.append(sphere)
        int_marker.controls.append(control)

        # Controls: 6-DOF Movement
        self.add_axis_control(int_marker, 1, 1, 0, 0, "move_x")       # Rotation around X (X axis invariant)
        self.add_axis_control(int_marker, 1, 0, 1, 0, "move_z")       # Rotation around Y (X becomes Z)
        self.add_axis_control(int_marker, 1, 0, 0, 1, "move_y")       # Rotation around Z (X becomes Y)
        
        move_3d = InteractiveMarkerControl()
        move_3d.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(move_3d)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def add_axis_control(self, msg, w, x, y, z, name):
        control = InteractiveMarkerControl()
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        control.orientation.w = float(w / norm)
        control.orientation.x = float(x / norm)
        control.orientation.y = float(y / norm)
        control.orientation.z = float(z / norm)
        control.name = name
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        msg.controls.append(control)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.target_pos[0] = feedback.pose.position.x
            self.target_pos[1] = feedback.pose.position.y
            self.target_pos[2] = feedback.pose.position.z
            self.server.setPose(feedback.marker_name, feedback.pose)
            self.server.applyChanges()

    def publish_loop(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = self.target_pos[0]
        msg.pose.position.y = self.target_pos[1]
        msg.pose.position.z = self.target_pos[2]
        # Orientation is identity for now
        msg.pose.orientation.w = 1.0
        self.pub_raw.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = IMUSimServer()
    rclpy.spin(server)
    rclpy.shutdown()