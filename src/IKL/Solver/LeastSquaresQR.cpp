
#include <IKL/Solver/LeastSquaresQR.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LeastSquaresQR<Scaler>::LeastSquaresQR(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LeastSquaresQR";
        }

        template <typename Scaler>
        LeastSquaresQR<Scaler>::~LeastSquaresQR() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LeastSquaresQR<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * jacobian;
            delta_step = inverse_order.colPivHouseholderQr().solve(jacobian.transpose() * error);

            return delta_step;
        }

        template class LeastSquaresQR<double>;
    }
}

