
cmake_minimum_required(VERSION 3.13)

project(IKL VERSION 1.0.0 LANGUAGES CXX)

add_library(IKL SHARED)

target_sources(IKL
    PRIVATE
        Solver/GaussNewton.cpp
        Solver/GaussNewtonQR.cpp
        Solver/GaussNewtonSVD.cpp
        Solver/JacobianTranspose.cpp
        Solver/LeastSquares.cpp
        Solver/LeastSquaresQR.cpp
        Solver/LevenbergMarquardtED.cpp
        Solver/LevenbergMarquardtSRC.cpp
        Solver/LevenbergMarquardtSRM.cpp
        Solver/LevenbergMarquardtSu.cpp
        Solver/QProblemGNByqpOASES.cpp
        Solver/QProblemLMByqpOASES.cpp
        Solver/QProblemGNByOsqp.cpp
        Solver/QProblemLMByOsqp.cpp
)

target_compile_features(IKL PRIVATE cxx_std_17)
target_compile_options(IKL
    PUBLIC
        -Wall -Winline -Wextra -pedantic
)
target_include_directories(IKL
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(IKL
    PRIVATE
        Eigen3::Eigen
        qpOASES::qpOASES
        OsqpEigen::OsqpEigen
        osqp::osqp
        RBDL::RBDL
        Model
)
set_target_properties(IKL
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        POSITION_INDEPENDENT_CODE ON
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Model)

