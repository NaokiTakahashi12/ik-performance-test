
#pragma once

#include "../NumericIK.hpp"

#include <memory>

#include <OsqpEigen/OsqpEigen.h>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class QProblemGNByOsqp : public NumericIK<Scaler> {
            public :
                QProblemGNByOsqp(const Parameters<Scaler> &param);

                virtual ~QProblemGNByOsqp();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
                void solve_setting(const Kinematic::JointState<Scaler> &) override;

            private :
                unsigned int solve_number_of_variables;

                std::unique_ptr<OsqpEigen::Solver> solver;
        };
    }
}

