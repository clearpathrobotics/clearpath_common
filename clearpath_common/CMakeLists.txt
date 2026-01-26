cmake_minimum_required(VERSION 3.20)

project(clearpath_common NONE)
message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
