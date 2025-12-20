cmake_minimum_required(VERSION 3.10)
project(ur_cpp_control)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# Plan node
add_executable(plan_request_node
  src/plan_request_node.cpp
)
ament_target_dependencies(plan_request_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2_ros
  sensor_msgs
)

# Execute node
add_executable(execute_plan_node
  src/execute_plan_node.cpp
)
ament_target_dependencies(execute_plan_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2_ros
  sensor_msgs
)

# Install targets
install(TARGETS
  plan_request_node
  execute_plan_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files only
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
