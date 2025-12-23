#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{
constexpr double kDefaultMaxAngularVelocity = 1.5;
}  // namespace

class ImuToServoNode : public rclcpp::Node
{
public:
  ImuToServoNode() : Node("imu_to_servo"), has_last_orientation_(false)
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/servo_node/pose_target_cmds");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    position_ = declare_parameter<std::vector<double>>("target_position", {0.3, 0.0, 0.3});
    max_angular_velocity_ = declare_parameter<double>(
      "max_angular_velocity", kDefaultMaxAngularVelocity);
    orientation_offset_ = declare_parameter<std::vector<double>>(
      "orientation_offset_xyzw", {0.0, 0.0, 0.0, 1.0});

    if (position_.size() != 3) {
      RCLCPP_WARN(get_logger(), "target_position must have 3 elements. Using default position.");
      position_ = {0.3, 0.0, 0.3};
    }

    if (orientation_offset_.size() != 4) {
      RCLCPP_WARN(
        get_logger(), "orientation_offset_xyzw must have 4 elements. Using identity offset.");
      orientation_offset_ = {0.0, 0.0, 0.0, 1.0};
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      command_topic_, rclcpp::SystemDefaultsQoS());
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuToServoNode::handle_imu, this, std::placeholders::_1));
  }

private:
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion imu_q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    if (imu_q.length2() < 1e-9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU orientation is invalid.");
      return;
    }
    imu_q.normalize();

    tf2::Quaternion offset_q(
      orientation_offset_[0], orientation_offset_[1], orientation_offset_[2],
      orientation_offset_[3]);
    if (offset_q.length2() < 1e-9) {
      offset_q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    offset_q.normalize();

    tf2::Quaternion target_q = offset_q * imu_q;
    target_q.normalize();

    const auto now_time = now();
    tf2::Quaternion command_q = target_q;

    if (has_last_orientation_) {
      const double dt = (now_time - last_command_time_).seconds();
      if (dt > 0.0) {
        tf2::Quaternion relative_q = last_command_orientation_.inverse() * target_q;
        relative_q.normalize();
        const double angle = relative_q.getAngle();
        const double max_step = max_angular_velocity_ * dt;

        if (angle > max_step && max_step > 1e-6) {
          tf2::Vector3 axis = relative_q.getAxis();
          if (axis.length2() < 1e-9) {
            axis = tf2::Vector3(0.0, 0.0, 1.0);
          }
          axis.normalize();
          tf2::Quaternion step_q(axis, max_step);
          command_q = last_command_orientation_ * step_q;
          command_q.normalize();
        }
      }
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now_time;
    pose_msg.header.frame_id = base_frame_;
    pose_msg.pose.position.x = position_[0];
    pose_msg.pose.position.y = position_[1];
    pose_msg.pose.position.z = position_[2];
    pose_msg.pose.orientation.x = command_q.x();
    pose_msg.pose.orientation.y = command_q.y();
    pose_msg.pose.orientation.z = command_q.z();
    pose_msg.pose.orientation.w = command_q.w();

    command_publisher_->publish(pose_msg);

    last_command_orientation_ = command_q;
    last_command_time_ = now_time;
    has_last_orientation_ = true;
  }

  std::string imu_topic_;
  std::string command_topic_;
  std::string base_frame_;
  std::vector<double> position_;
  std::vector<double> orientation_offset_;
  double max_angular_velocity_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  bool has_last_orientation_;
  tf2::Quaternion last_command_orientation_;
  rclcpp::Time last_command_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToServoNode>());
  rclcpp::shutdown();
  return 0;
}
