cmake_minimum_required(VERSION 3.20)
project(clearpath_manipulators_description)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
message(WARNING "CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config srdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
