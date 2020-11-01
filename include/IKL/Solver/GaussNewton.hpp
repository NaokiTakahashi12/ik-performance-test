
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class GaussNewton : public NumericIK<Scaler> {
            public :
                GaussNewton(const Parameters<Scaler> &param);

                virtual ~GaussNewton();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

