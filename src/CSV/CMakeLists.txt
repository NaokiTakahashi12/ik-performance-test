
cmake_minimum_required(VERSION 3.13)

project(CSV VERSION 1.0.0 LANGUAGES CXX)

add_library(CSV SHARED)

target_sources(CSV
    PRIVATE
        Map.cpp
        Parser.cpp
)

target_compile_features(CSV PRIVATE cxx_std_17)
target_compile_options(CSV
    PUBLIC
        -Wall -Winline -Wextra -pedantic
)
target_include_directories(CSV
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)
set_target_properties(CSV
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        POSITION_INDEPENDENT_CODE ON
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

