cmake_minimum_required(VERSION 2.8.3)
project(clearpath_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy sensor_msgs std_msgs)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES clearpath_teleop
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
