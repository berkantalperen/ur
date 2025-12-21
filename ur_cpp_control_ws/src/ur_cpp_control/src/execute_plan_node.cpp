#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

class ExecutePlanNode : public rclcpp::Node
{
public:
  ExecutePlanNode()
  : Node("execute_plan_node"), has_plan_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for MoveGroupInterface to initialize...");
  }

  void init()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");

    sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10,
      std::bind(&ExecutePlanNode::planCallback, this, std::placeholders::_1));

    srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_saved_plan",
      std::bind(&ExecutePlanNode::executeCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface ready. Executor initialized.");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan latest_plan_;
  bool has_plan_;

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  void planCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
      return;
    }

    latest_plan_.trajectory = msg->trajectory.back();
    latest_plan_.start_state = msg->trajectory_start;
    has_plan_ = true;

    size_t point_count = latest_plan_.trajectory.joint_trajectory.points.size();
    RCLCPP_INFO(this->get_logger(), "Stored new plan with %zu points.", point_count);
  }

  void executeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!has_plan_) {
      res->success = false;
      res->message = "No plan available to execute.";
      return;
    }

    move_group_->setStartStateToCurrentState();

    // Optionally re-apply the goal (not required if trajectory is intact)
    if (!latest_plan_.trajectory.joint_trajectory.points.empty()) {
      const auto& joint_names = latest_plan_.trajectory.joint_trajectory.joint_names;
      const auto& last_point = latest_plan_.trajectory.joint_trajectory.points.back();

      std::map<std::string, double> target_state;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        target_state[joint_names[i]] = last_point.positions[i];
      }
      move_group_->setJointValueTarget(target_state);
    }

    auto result = move_group_->execute(latest_plan_);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      res->success = true;
      res->message = "Plan executed successfully.";
    } else {
      res->success = false;
      res->message = "Plan execution failed or timed out.";
      RCLCPP_ERROR(this->get_logger(), "Plan execution failed.");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecutePlanNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
