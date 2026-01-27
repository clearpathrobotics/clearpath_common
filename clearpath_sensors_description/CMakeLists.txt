cmake_minimum_required(VERSION 3.20)
project(clearpath_sensors_description)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
message(WARNING "CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
