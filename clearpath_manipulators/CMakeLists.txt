cmake_minimum_required(VERSION 3.5)
project(clearpath_manipulators)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executables for your nodes
add_executable(gazebo_to_real_robot_node src/gazebo_to_real_robot_node.cpp)
add_executable(gazebo_to_real_gripper_node src/gazebo_to_real_gripper_node.cpp)

# Link against required libraries
ament_target_dependencies(gazebo_to_real_robot_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)

ament_target_dependencies(gazebo_to_real_gripper_node
  rclcpp
  sensor_msgs
  control_msgs
  rclcpp_action
)

# Install the executables
install(TARGETS 
  gazebo_to_real_robot_node
  gazebo_to_real_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
