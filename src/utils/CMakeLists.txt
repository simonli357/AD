cmake_minimum_required(VERSION 3.0.2)
project(utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)

add_library(sqlite3 STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sqlite/sqlite3.c
)

target_include_directories(sqlite3 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sqlite
)

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

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
  sqlite3
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
