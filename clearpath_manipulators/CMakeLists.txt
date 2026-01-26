cmake_minimum_required(VERSION 3.20)
project(clearpath_manipulators)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
