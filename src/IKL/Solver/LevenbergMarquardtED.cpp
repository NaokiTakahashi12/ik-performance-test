
#include <IKL/Solver/LevenbergMarquardtED.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LevenbergMarquardtED<Scaler>::LevenbergMarquardtED(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LevenbergMarquardtED";
        }

        template <typename Scaler>
        LevenbergMarquardtED<Scaler>::~LevenbergMarquardtED() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LevenbergMarquardtED<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e, weight_n;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            const Scaler E = 1/2 * error.transpose() * weight_e * error;
            weight_n = this->param.weight_n_scaler() * E * Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian + weight_n;
            delta_step = inverse_order.colPivHouseholderQr().solve(jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class LevenbergMarquardtED<double>;
    }
}

