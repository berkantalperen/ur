#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_state.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plan_request_node");

  // ✅ Create MoveGroupInterface directly
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

  // ✅ Set robot's current state as the start
  move_group.setStartStateToCurrentState();

  // ✅ Define a joint target
  std::vector<double> target_joint_values = {
    -2.2123, -1.6084, -1.5006, -1.2337, 1.6022, -1.4457
  };
  move_group.setJointValueTarget(target_joint_values);

  // ✅ Optional tuning
  move_group.setPlanningTime(2.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // ✅ Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = move_group.plan(plan);

  if (success != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning successful. Trajectory published.");


  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
