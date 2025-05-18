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
    "${CMAKE_CURRENT_SOURCE_DIR}/source/hardware/drivers/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/hardware/encoders/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/signal/controllers/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/signal/filters/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/signal/systemmodels/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/periodics/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/utils/**/*.cpp"
)

add_executable(${APP_TARGET} ${MY_SOURCE_FILES})

mbed_configure_app_target(${APP_TARGET})

set(includeDirs
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/brain
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware/encoders
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/signal/controllers
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/signal/filters
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/signal/systemmodels
    ${CMAKE_CURRENT_SOURCE_DIR}/include/signal
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