cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf
  message_generation
  utils
)


catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
#  LIBRARIES mpc
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} 
)