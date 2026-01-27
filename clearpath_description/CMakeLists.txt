
cmake_minimum_required(VERSION 3.20)
project(clearpath_description)

message(WARNING "${CMAKE_PROJECT_NAME} PYTHONPATH: [$ENV{PYTHONPATH}]")
message(WARNING "${CMAKE_PROJECT_NAME} CMAKE_PREFIX_PATH: [$ENV{CMAKE_PREFIX_PATH}]")
find_package(ament_cmake REQUIRED)

ament_package()
