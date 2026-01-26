cmake_minimum_required(VERSION 3.20)
project(clearpath_platform_description)

message(WARNING "PYTHONPATH: $ENV{PYTHONPATH}")
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
