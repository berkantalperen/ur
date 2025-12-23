cmake_minimum_required(VERSION 3.10)
project(ur_cpp_control)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
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

# IMU simulation node
add_executable(imu_sim_node
  src/imu_sim_node.cpp
)
ament_target_dependencies(imu_sim_node
  rclcpp
  sensor_msgs
  tf2
)

# IMU to MoveIt Servo node
add_executable(imu_to_servo_node
  src/imu_to_servo_node.cpp
)
ament_target_dependencies(imu_to_servo_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
)

# Install targets
install(TARGETS
  plan_request_node
  execute_plan_node
  imu_sim_node
  imu_to_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files only
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
