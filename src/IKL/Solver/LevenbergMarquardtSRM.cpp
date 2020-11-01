
#include <IKL/Solver/LevenbergMarquardtSRM.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LevenbergMarquardtSRM<Scaler>::LevenbergMarquardtSRM(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LevenbergMarquardtSRM";
        }

        template <typename Scaler>
        LevenbergMarquardtSRM<Scaler>::~LevenbergMarquardtSRM() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LevenbergMarquardtSRM<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> normalized_jacobian = jacobian;
            for(unsigned int i = 0; i < jacobian.cols(); i ++) {
                normalized_jacobian.col(i).normalize();
            }
            Math::MatrixNxN<Scaler> weight_e, weight_n;
            const Scaler omega = std::sqrt((normalized_jacobian * normalized_jacobian.transpose()).determinant());
            Scaler lambda;

            if(omega < this->param.manipulability_threshold()) {
                lambda = this->param.weight_n_scaler() * std::pow(1 - omega / this->param.manipulability_threshold(), 2);
            }
            else {
                lambda = 0;
            }

            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();
            weight_n = lambda * Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian + weight_n;
            delta_step = inverse_order.colPivHouseholderQr().solve(jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class LevenbergMarquardtSRM<double>;
    }
}

