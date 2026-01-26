cmake_minimum_required(VERSION 3.20)
project(clearpath_control)

message(STATUS "PYTHONPATH: $ENV{PYTHONPATH}")

find_package(ament_cmake REQUIRED)
find_package(ament_package REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()