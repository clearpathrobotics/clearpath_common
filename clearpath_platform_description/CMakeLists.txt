cmake_minimum_required(VERSION 3.20)
project(clearpath_platform_description)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
message(WARNING "CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
