cmake_minimum_required(VERSION 3.0.2)
project(common)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Declare messages
add_message_files(
  FILES
  ScanMapStatus.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
