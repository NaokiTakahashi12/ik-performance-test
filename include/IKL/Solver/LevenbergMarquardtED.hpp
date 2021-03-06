
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class LevenbergMarquardtED : public NumericIK<Scaler> {
            public :
                LevenbergMarquardtED(const Parameters<Scaler> &param);

                virtual ~LevenbergMarquardtED();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

