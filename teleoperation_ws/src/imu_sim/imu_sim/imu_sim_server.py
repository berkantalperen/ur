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

        visual = InteractiveMarkerControl(always_visible=True)
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.g = 1.0; sphere.color.a = 0.8
        visual.markers.append(sphere)
        int_marker.controls.append(visual)

        move = InteractiveMarkerControl(name="move_3d", interaction_mode=InteractiveMarkerControl.MOVE_3D)
        int_marker.controls.append(move)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, fb):
        p = fb.pose.position
        self.pos = [p.x, p.y, p.z]

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.pos
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IMUServer())
    rclpy.shutdown()