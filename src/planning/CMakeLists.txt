cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(${Eigen3_INCLUDE_DIRS})
message(STATUS "Found Eigen version: ${Eigen3_VERSION}")


find_package(nlohmann_json 3.2.0 REQUIRED)
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
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  include 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src
  ${CURSES_INCLUDE_DIR}
)

# file(GLOB_RECURSE ALGLIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src/*.cpp)
# add_library(alglib SHARED ${ALGLIB_SOURCES})

# add_executable(utility 
# src/utility_main.cpp
# src/utility.cpp
# )
# target_link_libraries(utility
# ${catkin_LIBRARIES}
# )
