cmake_minimum_required(VERSION 3.11)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

project(sqlite3 VERSION 3.49.0)

enable_language(C)

include(ExternalProject)

option(SQLITE3_WITH_SHELL "build sqlite3 executable" OFF)
option(BUILD_SHARED_LIBS "build sqlite3 shared or static library" OFF)
option(SQLITE3_USE_SHARED_CRT "use shared crt library" ON)
option(SQLITE3_ENABLE_MATH_FUNCTIONS "enable SQL math functions" ON)

if (MSVC)
if (NOT SQLITE3_USE_SHARED_CRT)
       foreach(flag_var
                CMAKE_CXX_FLAGS_DEBUG
                CMAKE_CXX_FLAGS_RELEASE
                CMAKE_CXX_FLAGS_MINSIZEREL
                CMAKE_CXX_FLAGS_RELWITHDEBINFO
                CMAKE_C_FLAGS_DEBUG
                CMAKE_C_FLAGS_RELEASE
                CMAKE_C_FLAGS_MINSIZEREL
                CMAKE_C_FLAGS_RELWITHDEBINFO
                )
            string(REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
        endforeach()
#else(SQLITE3_USE_SHARED_CRT)
endif(NOT SQLITE3_USE_SHARED_CRT)
endif(MSVC)

ExternalProject_Add(sqlite3-src
       URL https://sqlite.org/2025/sqlite-amalgamation-3490000.zip
       URL_HASH SHA3_256=138b6922eb274c953c9d8f3249c39706ad3e40f3eef5d81bbeb73509b41f52c9
       PREFIX 3rdparty
       SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3
       
       CONFIGURE_COMMAND ""
       BUILD_COMMAND ""
       INSTALL_COMMAND ""
)

set(SQLITE3_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3/sqlite3.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3/sqlite3.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3/sqlite3ext.h
)

set(SQLITE3_SHELL_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3/shell.c
  )

add_custom_command(
  OUTPUT ${SQLITE3_SOURCES} ${SQLITE3_SHELL_SOURCES}
  DEPENDS sqlite3-src
) 

add_library(sqlite3 ${SQLITE3_SOURCES})
target_include_directories(sqlite3 INTERFACE  ${CMAKE_CURRENT_SOURCE_DIR}/src/sqlite3)
set_property(TARGET sqlite3 PROPERTY POSITION_INDEPENDENT_CODE ON)
if (SQLITE3_ENABLE_MATH_FUNCTIONS)
  target_compile_definitions(sqlite3 PUBLIC SQLITE_ENABLE_MATH_FUNCTIONS)
endif (SQLITE3_ENABLE_MATH_FUNCTIONS)

if(SQLITE3_WITH_SHELL)
  add_executable(sqlite3-bin ${SQLITE3_SHELL_SOURCES})
  target_link_libraries(sqlite3-bin sqlite3)
  set_target_properties(sqlite3-bin
        PROPERTIES OUTPUT_NAME sqlite3)
endif(SQLITE3_WITH_SHELL)
     
install(TARGETS sqlite3
        EXPORT sqlite3
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )
export(TARGETS sqlite3 FILE sqlite3.cmake)
export(PACKAGE sqlite3)
