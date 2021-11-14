cmake_minimum_required(VERSION 3.10.1)
#cmake_minimum_required(VERSION 2.8.3)
project(odom_tf_calibrator)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE RelWithDebInfo) #None, Debug, Release, RelWithDebInfo, MinSizeRel

message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  geometry_msgs
  nav_msgs
)


catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# Set include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare info that other packages need to import library generated here
catkin_package()

