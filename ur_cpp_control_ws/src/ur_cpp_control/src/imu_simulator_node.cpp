#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class ImuSimulatorNode : public rclcpp::Node
{
public:
  ImuSimulatorNode()
  : Node("imu_simulator_node"), start_time_(now())
  {
    topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    yaw_rate_rad_s_ = declare_parameter<double>("yaw_rate_rad_s", 0.6);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic_, 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ImuSimulatorNode::publishImu, this));
  }

private:
  void publishImu()
  {
    const auto current_time = now();
    const double t = (current_time - start_time_).seconds();
    const double yaw = yaw_rate_rad_s_ * t;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    quat.normalize();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = frame_id_;
    msg.orientation = tf2::toMsg(quat);
    msg.angular_velocity.z = yaw_rate_rad_s_;

    publisher_->publish(msg);
  }

  rclcpp::Time start_time_;
  std::string topic_;
  std::string frame_id_;
  double publish_rate_hz_{};
  double yaw_rate_rad_s_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
