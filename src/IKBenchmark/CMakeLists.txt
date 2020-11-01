
cmake_minimum_required(VERSION 3.13)

project(IKBenchmark VERSION 1.0.0 LANGUAGES CXX)

add_library(IKBenchmark STATIC)

target_sources(IKBenchmark
    PRIVATE
        BenchmarkFileIO.cpp
        Squat6D/Case1.cpp
        Squat6D/Case2.cpp
        Squat6D/Case3.cpp
        Squat6D/Case4.cpp
        Squat6D/Squat6DBenchmark.cpp
        Squat6D/SquatMotion.cpp
)

target_compile_features(IKBenchmark PRIVATE cxx_std_17)
target_compile_options(IKBenchmark
    PUBLIC
        -Wall -Winline -Wextra -pedantic
)
target_include_directories(IKBenchmark
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(IKBenchmark
    PRIVATE
        Eigen3::Eigen
        qpOASES::qpOASES
        RBDL::RBDL
        IKL
        Model
        CSV
)
set_target_properties(IKBenchmark
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        POSITION_INDEPENDENT_CODE ON
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
