
#include <IKL/Solver/LevenbergMarquardtSRC.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        LevenbergMarquardtSRC<Scaler>::LevenbergMarquardtSRC(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "LevenbergMarquardtSRC";
        }

        template <typename Scaler>
        LevenbergMarquardtSRC<Scaler>::~LevenbergMarquardtSRC() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &LevenbergMarquardtSRC<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e, weight_n;

            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();
            weight_n = std::pow(this->param.weight_n_scaler(), 2) * Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian + weight_n;
            delta_step = inverse_order.colPivHouseholderQr().solve(jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class LevenbergMarquardtSRC<double>;
    }
}

