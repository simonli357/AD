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
if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
  message(FATAL_ERROR "Please set the environment variable ACADOS_SOURCE_DIR to the acados source directory")
else()
  message(STATUS "Found acados source directory: $ENV{ACADOS_SOURCE_DIR}")
endif()
include_directories(${Eigen3_INCLUDE_DIRS})
message(STATUS "Found Eigen version: ${Eigen3_VERSION}")
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18)
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

file(GLOB ocp_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/acados_solver_mobile_robot.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/acados_solver_park.c
        )

file(GLOB casadi_fun
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_vde_adj.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_vde_adj.c)

file(GLOB sim_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/acados_sim_solver_mobile_robot.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/acados_sim_solver_park.c
        )


file(GLOB ocp_solver50
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50/acados_solver_mobile_robot_50.c
        )

file(GLOB casadi_fun50
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50/mobile_robot_50_model/mobile_robot_50_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50/mobile_robot_50_model/mobile_robot_50_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50/mobile_robot_50_model/mobile_robot_50_expl_vde_adj.c
        )

file(GLOB sim_solver50
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_50/acados_sim_solver_mobile_robot_50.c
        )

file(GLOB ocp_solver25
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/acados_solver_mobile_robot_25.c
        )

file(GLOB casadi_fun25
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/mobile_robot_25_model/mobile_robot_25_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/mobile_robot_25_model/mobile_robot_25_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/mobile_robot_25_model/mobile_robot_25_expl_vde_adj.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/mobile_robot_25_cost/*.c
        )

file(GLOB sim_solver25
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_25/acados_sim_solver_mobile_robot_25.c
        )
file(GLOB ocp_solver18
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18/acados_solver_mobile_robot_18.c
        )
file(GLOB casadi_fun18
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18/mobile_robot_18_model/mobile_robot_18_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18/mobile_robot_18_model/mobile_robot_18_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18/mobile_robot_18_model/mobile_robot_18_expl_vde_adj.c
        )
file(GLOB sim_solver18
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_18/acados_sim_solver_mobile_robot_18.c
        )

find_package(Curses REQUIRED)
find_package(communication REQUIRED)
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
  perception
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

add_library(ocp_shared_lib SHARED ${ocp_solver} ${casadi_fun} ${ocp_solver50} ${casadi_fun50} ${ocp_solver25} ${casadi_fun25} ${ocp_solver18} ${casadi_fun18})
target_link_libraries(ocp_shared_lib acados hpipm blasfeo)

add_library(sim_shared_lib SHARED ${sim_solver} ${casadi_fun} ${sim_solver50} ${casadi_fun50} ${sim_solver25} ${casadi_fun25} ${sim_solver18} ${casadi_fun18})
target_link_libraries(sim_shared_lib acados hpipm blasfeo)

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

add_executable(utility 
src/utility_main.cpp
src/Utility.cpp
src/RoadObject.cpp
)
target_link_libraries(utility
${catkin_LIBRARIES}
communication
)

add_executable(controller 
src/Controller.cpp
src/Utility.cpp
src/RoadObject.cpp
)
target_link_libraries(controller
ocp_shared_lib 
sim_shared_lib
${catkin_LIBRARIES}
${CURSES_LIBRARIES}
communication
)
