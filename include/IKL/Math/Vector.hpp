
#pragma once

#include "Matrix.hpp"

namespace IKL {
    namespace Math {
        template <typename T, int Rows>
        using Vector = Matrix<T, Rows, 1>;

        template <typename T>
        using VectorN = Vector<T, DynamicSize>;

        template <typename T>
        using Vector2 = Vector<T, 2>;

        template <typename T>
        using Vector3 = Vector<T, 3>;

        template <typename T>
        using Vector4 = Vector<T, 4>;

        template <typename T>
        using Vector6 = Vector<T, 6>;
    }
}

    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::VectorN<int>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::VectorN<float>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::VectorN<double>);

    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector2<int>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector2<float>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector2<double>);

    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector3<int>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector3<float>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector3<double>);

    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector4<int>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector4<float>);
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IKL::Math::Vector4<double>);

