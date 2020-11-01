
#include <IKL/Solver/LevenbergMarquardtSu.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LevenbergMarquardtSu<Scaler>::LevenbergMarquardtSu(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LevenbergMarquardtSu";
        }

        template <typename Scaler>
        LevenbergMarquardtSu<Scaler>::~LevenbergMarquardtSu() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LevenbergMarquardtSu<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e, weight_n;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            const Scaler E = 1/2 * error.transpose() * weight_e * error;
            weight_n = (E + this->param.minimum_bias()) * Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian + weight_n;
            delta_step = inverse_order.inverse() * (jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class LevenbergMarquardtSu<double>;
    }
}

