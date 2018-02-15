cmake_minimum_required(VERSION 3.2.2)
project(BICYCLE CXX)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${PROJECT_SOURCE_DIR}/cmake/modules)
include(cmake/functions.cmake)
include(cmake/boost.cmake) # set Boost_INCLUDE_DIR
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

add_definitions("-DASIO_STANDALONE")
include_directories(external/asio/asio/include)

add_subdirectory(external/flatbuffers)
include_directories(external/flatbuffers/include)
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATLIB ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATHASH OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_CODE_COVERAGE OFF CACHE BOOL "" FORCE)
mark_as_advanced(FORCE FLATBUFFERS_BUILD_TESTS FLATBUFFERS_INSTALL FLATBUFFERS_BUILD_FLATLIB
    FLATBUFFERS_BUILD_FLATC FLATBUFFERS_BUILD_FLATHASH FLATBUFFERS_CODE_COVERAGE)

# debug/release flags
set(CMAKE_C_FLAGS_DEBUG "-ggdb3 -O0" CACHE
    STRING "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0" CACHE
    STRING "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE
    STRING "Flags used by the compiler during release builds." FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE
    STRING "Flags used by the compiler during release builds." FORCE)

# compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers") # suppress warnings from googletest
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter") # suppress warnings from boost numeric


# default to release type if none provided
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build, options are: Debug, Release." FORCE)
    message("CMAKE_BUILD_TYPE not set. Using ${CMAKE_BUILD_TYPE}.")
endif()


# header files with template definitions are placed in the src directory with
# the extension .hh
include_directories(inc)
include_directories(src)
add_subdirectory(fbs) # set FLATBUFFER_SCHEMAS

flatbuffers_generate_c_headers(FLATBUFFER_GENERATED ${FLATBUFFER_SCHEMAS})
include_directories(${PROJECT_BINARY_DIR}/generated) # location of flatbuffer generated headers
add_custom_target(generate_flatbuffer_headers DEPENDS
    ${FLATBUFFER_GENERATED_OUTPUTS} ${FLATBUFFER_SCHEMAS})
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    ${PROJECT_BINARY_DIR}/generated)

set(BICYCLE_SOURCE
    ${BICYCLE_SOURCE_DIR}/src/bicycle/bicycle.cc
    ${BICYCLE_SOURCE_DIR}/src/bicycle/bicycle_solve_constraint_pitch.cc
    ${BICYCLE_SOURCE_DIR}/src/bicycle/arend.cc
    ${BICYCLE_SOURCE_DIR}/src/bicycle/kinematic.cc
    ${BICYCLE_SOURCE_DIR}/src/bicycle/whipple.cc
    ${BICYCLE_SOURCE_DIR}/src/parameters.cc
    ${BICYCLE_SOURCE_DIR}/src/network_server.cc
    ${BICYCLE_SOURCE_DIR}/src/serial.cc)
set_source_files_properties(${BICYCLE_SOURCE_DIR}/src/bicycle/bicycle_solve_constraint_pitch.cc
    PROPERTIES COMPILE_FLAGS "-fassociative-math")

option(BICYCLE_BUILD_TESTS "Build tests." ON)
option(BICYCLE_BUILD_TOOLS "Build tools." ON)
option(BICYCLE_BUILD_EXAMPLES "Build examples." ON)
option(BICYCLE_USE_DOUBLE_PRECISION_REAL "Use double precision for real types." ON)
option(BICYCLE_NO_DISCRETIZATION "Do not calculate state space discretization." OFF)

add_definitions("-DBICYCLE_USE_DOUBLE_PRECISION_REAL=${BICYCLE_USE_DOUBLE_PRECISION_REAL}")

if(BICYCLE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(BICYCLE_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BICYCLE_BUILD_TESTS)
    enable_testing()
    include_directories(external/googletest)
    include_directories(external/googletest/include)
    add_subdirectory(external/googletest)
    add_subdirectory(tests)
endif()

