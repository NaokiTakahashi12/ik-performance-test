
cmake_minimum_required(VERSION 3.13)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "Failed in source build\nPlease ```$ mkdir build/```")
endif()

if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++-9)
elseif(APPLE)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif()

project(MultipleIKBenchmark VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(qpOASES REQUIRED)
find_package(OsqpEigen REQUIRED)

add_subdirectory(src)

