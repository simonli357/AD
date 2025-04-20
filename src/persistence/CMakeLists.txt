cmake_minimum_required(VERSION 3.0.2)
project(persistence)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sqlite3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES persistence
  CATKIN_DEPENDS roscpp roslib
  DEPENDS system_lib
)

include_directories(
  include
  include/queries
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB QUERIES ${CMAKE_CURRENT_SOURCE_DIR}/src/queries/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES} ${QUERIES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  sqlite3
)

add_executable(db
  src/Main.cpp
)
target_link_libraries(db
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
