cmake_minimum_required(VERSION 3.20)
project(clearpath_control)

message(WARNING "${CMAKE_PROJECT_NAME} BEFORE PYTHONPATH: [$ENV{PYTHONPATH}]")

SET(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

message(WARNING "${CMAKE_PROJECT_NAME} PYTHONPATH: [$ENV{PYTHONPATH}]")
message(WARNING "${CMAKE_PROJECT_NAME} CMAKE_PREFIX_PATH: [$ENV{CMAKE_PREFIX_PATH}]")
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()