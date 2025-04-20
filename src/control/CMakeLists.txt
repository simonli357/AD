cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
# see if acados source dir found

if(DEFINED ENV{acados_DIR})
  find_package(acados REQUIRED)
  if(acados_FOUND)
    message(STATUS "-- ACADOS found")
  else()
    message(STATUS "-- ACADOS not found")
  endif()
else()
  if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(FATAL_ERROR "Please set the environment variable ACADOS_SOURCE_DIR to the acados source directory")
  else()
    message(STATUS "Found acados source directory: $ENV{ACADOS_SOURCE_DIR}")
  endif()
endif()

include_directories(${Eigen3_INCLUDE_DIRS})
message(STATUS "Found Eigen version: ${Eigen3_VERSION}")

if (DEFINED ENV{TRT_LIBPATH})
  message(STATUS "Found TensorRT")
else()
  include_directories(~/TensorRT-8.6.1.6/include)
  link_directories(~/TensorRT-8.6.1.6/lib)
endif()

find_package(CUDA REQUIRED)
if(CUDA_FOUND)
  message("-- CUDA found ${CUDA_VERSION_STRING}, ${CUDA_INCLUDE_DIRS}") 
else()
  message("-- CUDA not found")
endif()
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
include_directories(/usr/local/cuda-11.4/targets/aarch64-linux/include)
link_directories(/usr/local/cuda-11.4/targets/aarch64-linux/lib)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(VCPKG_TARGET_TRIPLET "x64-linux")
    message("system processor is x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(VCPKG_TARGET_TRIPLET "arm64-linux")
    message("system processor is aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../vcpkg/scripts/buildsystems/vcpkg.cmake)
set(realsense2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../vcpkg/installed/${VCPKG_TARGET_TRIPLET}/share/realsense2)
find_package(realsense2 CONFIG REQUIRED)
if (realsense2_FOUND)
  message("-- RealSense found")
else()
  message("-- RealSense not found")
endif()
include_directories(${realsense2_INCLUDE_DIRS})

include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
file(GLOB GENERATED_CODE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/c_generated_code*
)
include_directories(${GENERATED_CODE_DIRS})
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

file(GLOB_RECURSE ocp_solvers
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/acados_solver_mobile_robot_*.c
)
file(GLOB_RECURSE cascadi_fun
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/mobile_robot_*_expl_ode_fun.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/mobile_robot_*_expl_vde_forw.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/mobile_robot_*_expl_vde_adj.c
)
file(GLOB_RECURSE cost_fun
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/mobile_robot_*_cost/*.c
)
file(GLOB_RECURSE sim_solvers
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados/*/acados_sim_solver_mobile_robot_*.c
)

add_library(acados_shared_lib SHARED ${ocp_solvers} ${cascadi_fun} ${cost_fun} ${sim_solvers})
target_link_libraries(acados_shared_lib acados hpipm blasfeo)

find_package(Curses REQUIRED)
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
  alglib_pkg
  utils
  persistence
  perception
  planning
  communication
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime planning alglib_pkg persistence utils
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
 DEPENDS system_lib
)

include_directories(
  include 
  include/object-detection
  include/htn
  include/htn/actions
  include/htn/actions/park
  ${catkin_INCLUDE_DIRS} 
  ${realsense2_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)

# HTN Lib
# file(GLOB HTN ${CMAKE_CURRENT_SOURCE_DIR}/src/htn/*.cpp)
# file(GLOB OBJECT_DETECTION ${CMAKE_CURRENT_SOURCE_DIR}/src/object-detection/*.cpp)
# file(GLOB ACTIONS ${CMAKE_CURRENT_SOURCE_DIR}/src/htn/actions/*.cpp)
# add_library(htn ${HTN} ${OBJECT_DETECTION} ${ACTIONS})
# target_link_libraries(htn ocp_shared_lib sim_shared_lib realsense2::realsense2)
# file(GLOB HTN ${CMAKE_CURRENT_SOURCE_DIR}/src/htn/*.cpp)
# file(GLOB OBJECT_DETECTION ${CMAKE_CURRENT_SOURCE_DIR}/src/object-detection/*.cpp)
# file(GLOB ACTIONS ${CMAKE_CURRENT_SOURCE_DIR}/src/htn/actions/*.cpp)
# add_library(htn ${HTN} ${OBJECT_DETECTION} ${ACTIONS})
# target_link_libraries(htn ocp_shared_lib sim_shared_lib realsense2::realsense2)

# add_executable(opt
# src/optimizer_main.cpp
# src/optimizer.cpp
# )
# target_link_libraries(opt 
# ocp_shared_lib 
# sim_shared_lib
# alglib
# ${catkin_LIBRARIES}
# )

# add_executable(utility 
# src/utility_main.cpp
# src/Utility.cpp
# )
# target_link_libraries(utility
# ${catkin_LIBRARIES}
# communication
# )

add_executable(controller 
  src/Controller.cpp
  src/Utility.cpp
)
target_link_libraries(controller
  acados_shared_lib 
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
  object_detection_lib
  communication
  planning
  persistence
  utils
  # htn
)

# add_executable(controller2
# src/Controller2.cpp
# src/Utility.cpp
# )
# target_link_libraries(controller2
# ocp_shared_lib 
# sim_shared_lib
# ${catkin_LIBRARIES}
# ${CURSES_LIBRARIES}
# htn
# communication
# object_detection_lib
# )
# link_directories(~/TensorRT-8.6.1.6/include)
