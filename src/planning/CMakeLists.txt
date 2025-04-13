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

find_package(Boost REQUIRED COMPONENTS graph)
include_directories(${Boost_INCLUDE_DIRS})

include(FetchContent)

FetchContent_Declare(
    matplotplusplus
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
    GIT_TAG        v1.2.2
)

FetchContent_MakeAvailable(matplotplusplus)

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
  roslib
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime roslib
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
 DEPENDS system_lib
)

include_directories(
  include
  include/map
  include/utils
  ${catkin_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src
  ${CURSES_INCLUDE_DIR}
)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB MAP ${CMAKE_CURRENT_SOURCE_DIR}/src/map/*.cpp)
file(GLOB UTILS ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES} ${MAP} ${UTILS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  Boost::graph
  matplot
  alglib_control
)

add_executable(planner
  src/Main.cpp
)
target_link_libraries(planner
  ${PROJECT_NAME}
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

# add_executable(utility 
# src/utility_main.cpp
# src/utility.cpp
# )
# target_link_libraries(utility
# ${catkin_LIBRARIES}
# )
