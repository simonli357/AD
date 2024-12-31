cmake_minimum_required(VERSION 3.0.2)
# cmake_minimum_required(VERSION 3.18)
project(perception)

set(CMAKE_CXX_STANDARD 17)

# Optimization flags for performance
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -ffast-math -funroll-loops -flto -fno-finite-math-only -funsafe-math-optimizations")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=fast -fassociative-math -fno-math-errno")
set(CMAKE_BUILD_TYPE Release)

# # Enable Link Time Optimization (LTO)
# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)

# # Enable CUDA optimizations
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3 --use_fast_math -lineinfo")

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

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


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  utils
)

find_package(CUDA REQUIRED)
if(CUDA_FOUND)
  message("-- CUDA found ${CUDA_VERSION_STRING}, ${CUDA_INCLUDE_DIRS}")
else()
  message("-- CUDA not found")
endif()
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
find_package(realsense2 CONFIG REQUIRED)
if (realsense2_FOUND)
  message("-- RealSense found")
else()
  message("-- RealSense not found")
endif()
find_package (Eigen3 REQUIRED )

# For finding FindTensorRT.cmake
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(ncnn_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/ncnn/lib/cmake/ncnn)
find_package(ncnn REQUIRED)
if (${ncnn_FOUND})
  message("-- NCNN found.")
  message("-- NCNN_VULKAN flag is ${NCNN_VULKAN}")
  if (${NCNN_VULKAN})
    message("-- AUTO-ENABLING GPU_SUPPORT")
  endif()
  set(GPU_SUPPORT ON)
  include_directories(${ncnn_INCLUDE})
  link_directories(${ncnn_LIB})
endif()

find_package(OpenCV 4.5 REQUIRED)
if (OpenCV_FOUND)
    message("OpenCV found ${OpenCV_VERSION}, ${OpenCV_INCLUDE_DIRS}")
else()
    message("OpenCV not found")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  utils
  message_runtime
  image_transport
  cv_bridge
  sensor_msgs
)

include_directories(
  include
  include/ncnn/include
  include/sort
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src
)
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(/TensorRT-8.6.1.6/include)
link_directories(/TensorRT-8.6.1.6/lib)

include_directories(/usr/local/cuda-11.4/targets/aarch64-linux/include)
link_directories(/usr/local/cuda-11.4/targets/aarch64-linux/lib)

file(GLOB_RECURSE ALGLIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src/*.cpp)
add_library(alglib SHARED ${ALGLIB_SOURCES})

add_executable(lane
  src/lane.cpp
)
target_link_libraries(lane
  ${catkin_LIBRARIES}
)

add_executable(signFastest
  src/signFastest.cpp
  src/yolo-fastestv2.cpp
)
target_link_libraries(signFastest
  ${catkin_LIBRARIES}
  ncnn ${OpenCV_LIBS}
)

add_executable(videoDetector
  src/videoDetector.cpp
  src/yolo-fastestv2.cpp
)
target_link_libraries(videoDetector
  ${catkin_LIBRARIES}
  ncnn ${OpenCV_LIBS}
)

add_executable(cameraNode
  src/cameraNode.cpp
  src/yolo-fastestv2.cpp
  src/engine.cpp
  src/yolov8.cpp
)
target_link_libraries(cameraNode
  ${catkin_LIBRARIES}
  ncnn
  ${OpenCV_LIBS}
  realsense2::realsense2
  ${CUDA_LIBRARIES}
  nvinfer
  nvparsers
  nvonnxparser
  alglib
  ${Python3_LIBRARIES}
)
add_executable(testNode
  src/testNode.cpp
  src/yolo-fastestv2.cpp
  src/engine.cpp
  src/yolov8.cpp
)
target_link_libraries(testNode
  ${catkin_LIBRARIES}
  ncnn
  ${OpenCV_LIBS}
  realsense2::realsense2
  ${CUDA_LIBRARIES}
  nvinfer
  nvparsers
  nvonnxparser
  alglib
  ${Python3_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)


