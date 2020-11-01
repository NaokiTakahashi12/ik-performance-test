
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class LevenbergMarquardtSRM : public NumericIK<Scaler> {
            public :
                LevenbergMarquardtSRM(const Parameters<Scaler> &param);

                virtual ~LevenbergMarquardtSRM();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

