
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class GaussNewtonSVD : public NumericIK<Scaler> {
            public :
                GaussNewtonSVD(const Parameters<Scaler> &param);
                GaussNewtonSVD(const Parameters<Scaler> &param, const Scaler &singular_value_tolerance);

                virtual ~GaussNewtonSVD();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;

            private :
                Scaler singular_value_tolerance;
        };
    }
}

