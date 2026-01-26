cmake_minimum_required(VERSION 3.20)
project(clearpath_control)

message(WARNING "Before PYTHONPATH: $ENV{PYTHONPATH}")

SET(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()