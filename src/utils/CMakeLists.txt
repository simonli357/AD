cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(TBB REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  persistence
)

add_message_files(
   FILES
   IMU.msg
   localisation.msg
   Sign.msg
   Lane.msg
   Lane2.msg
   Lane3.msg
   encoder.msg
   ImgInfo.msg
   Sensors.msg
   odometry.msg
   Point2D.msg
)
 
## Generate services in the 'srv' folder
add_service_files(
  FILES
  waypoints.srv
  go_to.srv
  go_to_multiple.srv
  goto_command.srv
  set_states.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils
  CATKIN_DEPENDS message_runtime persistence
)

include_directories(
  include
  include/utils
  ${catkin_INCLUDE_DIRS}
)

add_library(utils
  src/utils/constants.cpp
)

set_target_properties(utils
  PROPERTIES
    OUTPUT_NAME     utility
)

target_link_libraries(utils
  TBB::tbb
  persistence
  ${catkin_LIBRARIES}
)
