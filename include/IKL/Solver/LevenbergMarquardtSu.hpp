
#pragma once

#include "../NumericIK.hpp"

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class LevenbergMarquardtSu : public NumericIK<Scaler> {
            public :
                LevenbergMarquardtSu(const Parameters<Scaler> &param);

                virtual ~LevenbergMarquardtSu();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
        };
    }
}

