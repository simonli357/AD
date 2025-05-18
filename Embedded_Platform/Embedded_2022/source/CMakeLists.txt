# ---------------------------------------------------------------------------
#  Collect every .cpp file under src/  (updates automatically when you add new files)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE APP_SOURCES
     CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# ---------------------------------------------------------------------------
#  Create the executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}  ${APP_SOURCES})

# Headers live in the sibling include/ folder
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Link against the full Mbed OS
target_link_libraries(${PROJECT_NAME}
    mbed-os
)
