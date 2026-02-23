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

# # # Enable CUDA optimizations
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
  communication
  alglib_pkg
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

find_package(ncnn REQUIRED)
if (${ncnn_FOUND})
  message("-- NCNN found in system path")
  # NCNN exports 'ncnn' target and variable 'ncnn_INCLUDE_DIRS'
  include_directories(${ncnn_INCLUDE_DIRS})
  # Note: ncnn usually links automatically via target_link_libraries(target ncnn)
endif()

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

set(CURRENT_USER $ENV{USER})
if(CURRENT_USER STREQUAL "slsecret")
  set(TENSORRT_ROOT "$ENV{HOME}/TensorRT-8.6.1.6")
  set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include")
  set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib")
endif()

include_directories(
  include
  include/ncnn/include
  include/sort
  ${TENSORRT_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(${Eigen3_INCLUDE_DIRS})

set(_TRT8_FALLBACK_ROOT "$ENV{HOME}/TensorRT-8.6.1.6")
if (EXISTS "${_TRT8_FALLBACK_ROOT}/include/NvInfer.h")
  message(STATUS "TensorRT fallback detected at ${_TRT8_FALLBACK_ROOT}")
endif()

if (NOT TENSORRT_INCLUDE_DIR OR TENSORRT_INCLUDE_DIR STREQUAL "")
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS
      "${_TRT8_FALLBACK_ROOT}/include"
      "/usr/include"
      "/usr/include/x86_64-linux-gnu"
      "/usr/local/include"
  )
endif()

set(_TRT_LIB_HINTS "")
if (TENSORRT_LIB_DIR AND NOT TENSORRT_LIB_DIR STREQUAL "")
  list(APPEND _TRT_LIB_HINTS "${TENSORRT_LIB_DIR}")
endif()
if (TENSORRT_ROOT AND NOT TENSORRT_ROOT STREQUAL "")
  list(APPEND _TRT_LIB_HINTS "${TENSORRT_ROOT}/lib" "${TENSORRT_ROOT}/lib64")
endif()
if (EXISTS "${_TRT8_FALLBACK_ROOT}/lib")
  list(APPEND _TRT_LIB_HINTS "${_TRT8_FALLBACK_ROOT}/lib")
endif()
list(APPEND _TRT_LIB_HINTS
  "/usr/lib" "/usr/lib64"
  "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu"
  "/usr/local/lib" "/usr/local/lib64"
)

find_library(TENSORRT_NVINFER_LIB     nvinfer     HINTS ${_TRT_LIB_HINTS} REQUIRED)
find_library(TENSORRT_ONNX_PARSER_LIB nvonnxparser HINTS ${_TRT_LIB_HINTS})
find_library(TENSORRT_PARSERS_LIB     nvparsers    HINTS ${_TRT_LIB_HINTS})  # optional
find_library(TENSORRT_PLUGIN_LIB      nvinfer_plugin HINTS ${_TRT_LIB_HINTS}) # optional

set(TENSORRT_LIBS ${TENSORRT_NVINFER_LIB})
if (TENSORRT_ONNX_PARSER_LIB)
  list(APPEND TENSORRT_LIBS ${TENSORRT_ONNX_PARSER_LIB})
endif()
if (TENSORRT_PARSERS_LIB)
  list(APPEND TENSORRT_LIBS ${TENSORRT_PARSERS_LIB})
endif()
if (TENSORRT_PLUGIN_LIB)
  list(APPEND TENSORRT_LIBS ${TENSORRT_PLUGIN_LIB})
endif()

message(STATUS "TensorRT include dir: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT libs: ${TENSORRT_LIBS}")

if (TENSORRT_INCLUDE_DIR AND NOT TENSORRT_INCLUDE_DIR STREQUAL "")
  include_directories(${TENSORRT_INCLUDE_DIR})
endif()

if (EXISTS "${_TRT8_FALLBACK_ROOT}/include/NvInfer.h")
  include_directories(${_TRT8_FALLBACK_ROOT}/include)
endif()
if (EXISTS "${_TRT8_FALLBACK_ROOT}/lib")
  link_directories(${_TRT8_FALLBACK_ROOT}/lib)
endif()

if (TENSORRT_LIB_DIR AND NOT TENSORRT_LIB_DIR STREQUAL "")
  link_directories(${TENSORRT_LIB_DIR})
endif()

include_directories(/usr/local/cuda-11.4/targets/aarch64-linux/include)
link_directories(/usr/local/cuda-11.4/targets/aarch64-linux/lib)

add_library(object_detection_lib
  src/yolo-fastestv2.cpp
  src/engine.cpp
  src/yolov8.cpp
)
target_link_libraries(object_detection_lib PUBLIC
  ncnn
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBS}
  realsense2::realsense2
  ${OpenCV_LIBS}
)
target_include_directories(object_detection_lib PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${TENSORRT_INCLUDE_DIR}
  ${ncnn_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include ${ncnn_INC} ${TENSORRT_INCLUDE_DIR}
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
  communication
  LIBRARIES object_detection_lib
  alglib_pkg
  # CFG_EXTRAS perception-extras.cmake
)

add_executable(cameraNode
  src/CameraNode.cpp
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
  ${TENSORRT_LIBS}
  alglib_pkg
  ${Python3_LIBRARIES}
  communication
)