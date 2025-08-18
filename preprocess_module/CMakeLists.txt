cmake_minimum_required(VERSION 3.0.2)
project(preprocess_module)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)



# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs   
  sensor_msgs   
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sync_sensor
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs 
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS src/filter_imu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




