
#include <IKL/Solver/LeastSquares.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LeastSquares<Scaler>::LeastSquares(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LeastSquares";
        }

        template <typename Scaler>
        LeastSquares<Scaler>::~LeastSquares() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LeastSquares<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * jacobian;
            delta_step = inverse_order.inverse() * jacobian.transpose() * error;

            return delta_step;
        }

        template class LeastSquares<double>;
    }
}

