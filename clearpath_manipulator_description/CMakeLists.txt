cmake_minimum_required(VERSION 3.5)
project(clearpath_manipulator_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
