cmake_minimum_required(VERSION 3.20)
project(clearpath_manipulators_description)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config srdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
