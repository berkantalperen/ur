#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class ImuToServoNode : public rclcpp::Node
{
public:
  ImuToServoNode()
  : Node("imu_to_servo_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/servo_node/delta_twist_cmds");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    wrist_frame_ = declare_parameter<std::string>("wrist_frame", "wrist_3_link");
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
    orientation_gain_ = declare_parameter<double>("orientation_gain", 2.0);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.5);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&ImuToServoNode::handleImu, this, std::placeholders::_1));

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic_, 10);

    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ImuToServoNode::publishCommand, this));
  }

private:
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    last_imu_time_ = now();
    has_imu_ = true;
  }

  void publishCommand()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;

    if (!has_imu_ || (now() - last_imu_time_).seconds() > imu_timeout_s_) {
      command_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, wrist_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ex.what());
      command_pub_->publish(cmd);
      return;
    }

    tf2::Quaternion current_orientation;
    tf2::fromMsg(transform.transform.rotation, current_orientation);

    tf2::Quaternion target_orientation;
    tf2::fromMsg(last_imu_.orientation, target_orientation);
    target_orientation.normalize();

    tf2::Quaternion error = target_orientation * current_orientation.inverse();
    error.normalize();

    double angle = error.getAngle();
    tf2::Vector3 axis = error.getAxis();

    if (!std::isfinite(angle) || axis.length2() < 1e-12) {
      command_pub_->publish(cmd);
      return;
    }

    tf2::Vector3 angular_velocity = axis.normalized() * (orientation_gain_ * angle);

    const double speed = angular_velocity.length();
    if (speed > max_angular_velocity_) {
      angular_velocity *= (max_angular_velocity_ / speed);
    }

    cmd.twist.angular.x = angular_velocity.x();
    cmd.twist.angular.y = angular_velocity.y();
    cmd.twist.angular.z = angular_velocity.z();

    command_pub_->publish(cmd);
  }

  std::string imu_topic_;
  std::string command_topic_;
  std::string base_frame_;
  std::string wrist_frame_;
  double max_angular_velocity_{};
  double control_rate_hz_{};
  double orientation_gain_{};
  double imu_timeout_s_{};

  bool has_imu_{false};
  sensor_msgs::msg::Imu last_imu_;
  rclcpp::Time last_imu_time_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToServoNode>());
  rclcpp::shutdown();
  return 0;
}
