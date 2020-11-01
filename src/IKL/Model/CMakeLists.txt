
cmake_minimum_required(VERSION 3.13)

project(Model VERSION 1.0.0 LANGUAGES CXX)

add_library(Model STATIC)

target_sources(Model
    PRIVATE
        RBDL.cpp
)

target_compile_features(Model PRIVATE cxx_std_17)
target_compile_options(Model
    PUBLIC
        -Wall -Winline -Wextra -pedantic
)
target_include_directories(Model
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(Model
    PRIVATE
        RBDL::RBDL
        RBDL::URDFReader
        Eigen3::Eigen
)
set_target_properties(Model
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        POSITION_INDEPENDENT_CODE ON
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

