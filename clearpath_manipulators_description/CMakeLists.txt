cmake_minimum_required(VERSION 3.5)
project(clearpath_manipulators_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config srdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
