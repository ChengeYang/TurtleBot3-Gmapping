cmake_minimum_required(VERSION 2.8.3)
project(my_gmapping)

#########################################################################
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

#########################################################################
# Search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  std_msgs
  geometry_msgs
  nav_msgs
)

#########################################################################
## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy tf2_ros std_msgs geometry_msgs nav_msgs
  DEPENDS
)

#########################################################################
# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#########################################################################
# Building
add_executable(my_gmapping
  src/main.cpp
)
target_link_libraries(my_gmapping
  ${catkin_LIBRARIES}
)