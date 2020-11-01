
#include <IKL/Solver/GaussNewtonQR.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        GaussNewtonQR<Scaler>::GaussNewtonQR(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "GaussNewtonQR";
        }

        template <typename Scaler>
        GaussNewtonQR<Scaler>::~GaussNewtonQR() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &GaussNewtonQR<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian;
            delta_step = inverse_order.colPivHouseholderQr().solve(jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class GaussNewtonQR<double>;
    }
}

