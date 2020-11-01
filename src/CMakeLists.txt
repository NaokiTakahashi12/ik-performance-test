
cmake_minimum_required(VERSION 3.13)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CSV)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/IKL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/IKBenchmark)

add_executable(multiple_ik_benchmark ${CMAKE_CURRENT_SOURCE_DIR}/multiple_ik_benchmark.cpp)
target_compile_features(multiple_ik_benchmark PRIVATE cxx_std_17)
target_compile_options(multiple_ik_benchmark
    PUBLIC
        -Wall -Winline -Wextra -pedantic
)
target_link_libraries(multiple_ik_benchmark
    PRIVATE 
        Eigen3::Eigen
        qpOASES::qpOASES
        OsqpEigen::OsqpEigen
        osqp::osqp
        RBDL::RBDL
        IKBenchmark
)
set_target_properties(multiple_ik_benchmark
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

