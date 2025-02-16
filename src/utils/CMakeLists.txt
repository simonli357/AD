cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   IMU.msg
   localisation.msg
   Sign.msg
   Lane.msg
   Lane2.msg
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
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
