
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class JacobianTranspose : public NumericIK<Scaler> {
            public :
                JacobianTranspose(const Parameters<Scaler> &param);

                virtual ~JacobianTranspose();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}
