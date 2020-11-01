
#include <IKL/Solver/GaussNewtonSVD.hpp>

#include <Eigen/SVD>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        GaussNewtonSVD<Scaler>::GaussNewtonSVD(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "GaussNewtonSVD";
            this->singular_value_tolerance = 1e-7;
        }

        template <typename Scaler>
        GaussNewtonSVD<Scaler>::GaussNewtonSVD(const Parameters<Scaler> &param, const Scaler &singular_value_tolerance) : GaussNewtonSVD(param) {
            this->singular_value_tolerance = singular_value_tolerance;
        }

        template <typename Scaler>
        GaussNewtonSVD<Scaler>::~GaussNewtonSVD() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &GaussNewtonSVD<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) {
            static Math::VectorN<Scaler> delta_step;

            Math::MatrixNxN<Scaler> weight_e;
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            Math::MatrixNxN<Scaler> inverse_order = jacobian.transpose() * weight_e * jacobian;

            Eigen::JacobiSVD<Math::MatrixNxN<Scaler>> svd{inverse_order, Eigen::ComputeThinU | Eigen::ComputeThinV};
            auto singular_values = svd.singularValues();

            for(unsigned int i = 0; i < singular_values.size(); i ++) {
                if(singular_values(i) > singular_value_tolerance) {
                    singular_values(i) = 1 / singular_values(i);
                }
                else {
                    singular_values(i) = 0;
                }
            }

            Math::MatrixNxN<Scaler> pinv = svd.matrixV() * singular_values.asDiagonal() * svd.matrixU().transpose();

            delta_step = pinv * (jacobian.transpose() * weight_e * error);

            return delta_step;
        }

        template class GaussNewtonSVD<double>;
    }
}

