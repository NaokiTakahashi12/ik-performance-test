
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class LevenbergMarquardtSRC : public NumericIK<Scaler> {
            public :
                LevenbergMarquardtSRC(const Parameters<Scaler> &param);

                virtual ~LevenbergMarquardtSRC();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

