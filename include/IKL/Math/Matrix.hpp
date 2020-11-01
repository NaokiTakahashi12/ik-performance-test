
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace IKL {
    namespace Math {
        constexpr auto DynamicSize = Eigen::Dynamic;
        constexpr auto RowMajor = Eigen::RowMajor;
        constexpr auto ColumnMajor = Eigen::ColMajor;

        template <typename T, int Row, int Column, int StorageOrder = ColumnMajor>
        using Matrix = Eigen::Matrix<T, Row, Column, StorageOrder>;

        template <typename T, int StorageOrder = ColumnMajor>
        using MatrixNxN = Matrix<T, DynamicSize, DynamicSize, StorageOrder>;

        template <typename T, int StorageOrder = ColumnMajor>
        using Matrix2x2 = Matrix<T, 2, 2, StorageOrder>;

        template <typename T, int StorageOrder = ColumnMajor>
        using Matrix3x3 = Matrix<T, 3, 3, StorageOrder>;

        template <typename T, int StorageOrder = ColumnMajor>
        using Matrix4x4 = Matrix<T, 4, 4, StorageOrder>;

        template <typename T, int StorageOrder = ColumnMajor>
        using Matrix6x6 = Matrix<T, 6, 6, StorageOrder>;
    }
}

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::MatrixNxN<int>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::MatrixNxN<float>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::MatrixNxN<double>);

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix2x2<int>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix2x2<float>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix2x2<double>);

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix3x3<int>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix3x3<float>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix3x3<double>);

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix4x4<int>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix4x4<float>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Matrix4x4<double>);

