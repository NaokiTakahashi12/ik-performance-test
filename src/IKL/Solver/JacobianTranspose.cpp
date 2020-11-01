
#include <IKL/Solver/JacobianTranspose.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        JacobianTranspose<Scaler>::JacobianTranspose(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "JacobianTranspose";
        }

        template <typename Scaler>
        JacobianTranspose<Scaler>::~JacobianTranspose() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &JacobianTranspose<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            delta_step.setZero(jacobian.cols());

            Math::MatrixNxN<Scaler> weight_e;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            const Math::VectorN<Scaler> jte = jacobian.transpose() * weight_e * error;
            const Math::VectorN<Scaler> jjte = jacobian * jte;
            const auto alpha = error.dot(jjte) / jjte.dot(jjte);

            if(alpha < 0) {
                return delta_step;
            }

            delta_step = alpha * jte;

            return delta_step;
        }

        template class JacobianTranspose<double>;
    }
}

