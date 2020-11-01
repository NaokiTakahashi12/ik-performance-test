
#pragma once

#include "../NumericIK.hpp"

#include <memory>

#include <qpOASES/QProblemB.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        class QProblemLMByqpOASES : public NumericIK<Scaler> {
            public :
                QProblemLMByqpOASES(const Parameters<Scaler> &param);

                virtual ~QProblemLMByqpOASES();

            protected :
                Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) override;
                void solve_setting(const Kinematic::JointState<Scaler> &) override;

            private :
                unsigned int solve_number_of_variables;

                std::unique_ptr<qpOASES::QProblemB> qp_b;
        };
    }
}

