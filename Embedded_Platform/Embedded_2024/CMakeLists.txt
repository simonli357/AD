# Copyright (c) 2023 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET robot_car)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})

file(GLOB_RECURSE MY_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/brain/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/drivers/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/periodics/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/utils/*.cpp"
)

add_executable(${APP_TARGET} ${MY_SOURCE_FILES})

mbed_configure_app_target(${APP_TARGET})

set(includeDirs
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/brain
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/periodics
)

target_include_directories(${APP_TARGET}
    PUBLIC
        ${includeDirs}
)

target_link_libraries(${APP_TARGET}
    PRIVATE
        mbed-os
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()