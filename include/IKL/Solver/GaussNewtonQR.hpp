
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class GaussNewtonQR : public NumericIK<Scaler> {
            public :
                GaussNewtonQR(const Parameters<Scaler> &param);

                virtual ~GaussNewtonQR();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

