
#include <IKL/Solver/GaussNewton.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        GaussNewton<Scaler>::GaussNewton(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "GaussNewton";
        }

        template <typename Scaler>
        GaussNewton<Scaler>::~GaussNewton() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &GaussNewton<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian;
            delta_step = inverse_order.inverse() * jacobian.transpose() * weight_e * error;

            return delta_step;
        }

        template class GaussNewton<double>;
    }
}

