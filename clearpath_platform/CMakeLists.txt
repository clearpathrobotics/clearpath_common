cmake_minimum_required(VERSION 3.5)
project(clearpath_platform)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(clearpath_platform_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## COMPILE

# Lighting
set(LIGHTING_EXECUTABLE lighting_node)
set(LIGHTING_LIB clearpath_platform_lighting)
set(LIGHTING_DEPENDENCIES
    clearpath_platform_msgs
    sensor_msgs
    rclcpp
)

add_library(
  ${LIGHTING_LIB}
  SHARED
  src/lighting/lighting.cpp
  src/lighting/sequence.cpp
  src/lighting/color.cpp
)

ament_target_dependencies(${LIGHTING_LIB} ${LIGHTING_DEPENDENCIES})

target_include_directories(
  ${LIGHTING_LIB}
  PRIVATE
  include
)

add_executable(${LIGHTING_EXECUTABLE} src/lighting/main.cpp)
target_link_libraries(${LIGHTING_EXECUTABLE} ${LIGHTING_LIB})
ament_target_dependencies(${LIGHTING_EXECUTABLE} ${LIGHTING_DEPENDENCIES})

target_include_directories(
  ${LIGHTING_EXECUTABLE}
  PRIVATE
  include
)

# A200 Hardware
add_library(
  a200_hardware
  SHARED
  src/a200/hardware.cpp
  src/a200/status.cpp
  src/a200/horizon_legacy/horizon_legacy_wrapper.cpp
  src/a200/horizon_legacy/crc.cpp
  src/a200/horizon_legacy/Logger.cpp
  src/a200/horizon_legacy/Message.cpp
  src/a200/horizon_legacy/Message_data.cpp
  src/a200/horizon_legacy/Message_request.cpp
  src/a200/horizon_legacy/Message_cmd.cpp
  src/a200/horizon_legacy/Transport.cpp
  src/a200/horizon_legacy/Number.cpp
  src/a200/horizon_legacy/linux_serial.cpp
)

target_include_directories(
  a200_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  a200_hardware
  clearpath_platform_msgs
  hardware_interface
  pluginlib
  rclcpp
)


# J100 Hardware
add_library(
  j100_hardware
  SHARED
  src/j100/hardware.cpp
  src/j100/hardware_interface.cpp
  src/diff_drive/hardware.cpp
  src/diff_drive/hardware_interface.cpp
)

target_include_directories(
  j100_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  j100_hardware
  clearpath_platform_msgs
  hardware_interface
  pluginlib
  rclcpp
)

# W200 Hardware
add_library(
  w200_hardware
  SHARED
  src/w200/hardware.cpp
  src/w200/hardware_interface.cpp
)

target_include_directories(
  w200_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  w200_hardware
  clearpath_platform_msgs
  hardware_interface
  pluginlib
  rclcpp
)


pluginlib_export_plugin_description_file(hardware_interface src/a200/hardware.xml)
pluginlib_export_plugin_description_file(hardware_interface src/j100/hardware.xml)
pluginlib_export_plugin_description_file(hardware_interface src/w200/hardware.xml)


# INSTALL
install(
  TARGETS a200_hardware
          j100_hardware
          w200_hardware
          ${LIGHTING_EXECUTABLE}
          ${LIGHTING_LIB}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  a200_hardware
  j100_hardware
  w200_hardware
  ${LIGHTING_LIB}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
