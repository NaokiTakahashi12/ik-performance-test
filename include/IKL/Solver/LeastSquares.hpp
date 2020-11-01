
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class LeastSquares : public NumericIK<Scaler> {
            public :
                LeastSquares(const Parameters<Scaler> &param);

                virtual ~LeastSquares();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

