cmake_minimum_required(VERSION 3.0.2)
project(simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  planning
  roscpp
  roslib
  rospy
  std_msgs
  utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simulation
  CATKIN_DEPENDS geometry_msgs planning roscpp roslib rospy std_msgs utils
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/control
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB CONTROL ${CMAKE_CURRENT_SOURCE_DIR}/src/control/*.cpp)
add_library(${PROJECT_NAME}
  ${SOURCES}
  ${CONTROL}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(traffic
  src/Main.cpp
)
target_link_libraries(traffic
  ${PROJECT_NAME}
)
