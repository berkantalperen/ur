#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{
constexpr double kDefaultFrequency = 0.2;
constexpr double kDefaultMagnitude = 0.5;
constexpr double kDefaultRate = 50.0;
}  // namespace

class ImuSimulator : public rclcpp::Node
{
public:
  ImuSimulator() : Node("imu_simulator"), start_time_(now())
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", kDefaultRate);
    oscillation_frequency_hz_ = declare_parameter<double>("oscillation_frequency_hz", kDefaultFrequency);
    oscillation_magnitude_rad_ = declare_parameter<double>("oscillation_magnitude_rad", kDefaultMagnitude);
    axis_ = declare_parameter<std::vector<double>>("oscillation_axis", {0.0, 0.0, 1.0});

    if (axis_.size() != 3) {
      RCLCPP_WARN(get_logger(), "oscillation_axis must have 3 elements. Using default axis.");
      axis_ = {0.0, 0.0, 1.0};
    }

    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SystemDefaultsQoS());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&ImuSimulator::publish_imu, this));
  }

private:
  void publish_imu()
  {
    const auto now_time = now();
    const double t = (now_time - start_time_).seconds();
    const double angle = std::sin(2.0 * M_PI * oscillation_frequency_hz_ * t) *
                         oscillation_magnitude_rad_;

    tf2::Vector3 axis(axis_[0], axis_[1], axis_[2]);
    if (axis.length2() < 1e-6) {
      axis = tf2::Vector3(0.0, 0.0, 1.0);
    }
    axis.normalize();

    tf2::Quaternion q(axis, angle);
    q.normalize();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now_time;
    msg.header.frame_id = frame_id_;
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    publisher_->publish(msg);
  }

  rclcpp::Time start_time_;
  std::string frame_id_;
  double publish_rate_hz_;
  double oscillation_frequency_hz_;
  double oscillation_magnitude_rad_;
  std::vector<double> axis_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSimulator>());
  rclcpp::shutdown();
  return 0;
}
