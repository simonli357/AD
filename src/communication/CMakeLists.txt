cmake_minimum_required(VERSION 3.0.2)
project(communication)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")

find_package(OpenCV 4.5 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenSSL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  utils
  image_transport
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs utils image_transport cv_bridge
)

link_directories(/usr/lib/x86_64-linux-gnu)

include_directories(
  include
  include/service_calls
  include/msg
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB SERVICES ${CMAKE_CURRENT_SOURCE_DIR}/src/service_calls/*.cpp)
file(GLOB MSGS ${CMAKE_CURRENT_SOURCE_DIR}/src/msg/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES} ${SERVICES} ${MSGS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  OpenSSL::Crypto
  TBB::tbb
  hwloc
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

