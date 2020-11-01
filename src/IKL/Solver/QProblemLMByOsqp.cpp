
#include <IKL/Solver/QProblemLMByOsqp.hpp>

#include <Eigen/Sparse>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        QProblemLMByOsqp<Scaler>::QProblemLMByOsqp(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "QProblemLMByOsqp";

            solve_number_of_variables = 0;
            solver = std::make_unique<OsqpEigen::Solver>();
        }

        template <typename Scaler>
        QProblemLMByOsqp<Scaler>::~QProblemLMByOsqp() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &QProblemLMByOsqp<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &joint_state) {
            static Math::VectorN<Scaler> delta_step;

            Math::VectorN<Scaler> u, l, q;
            Math::MatrixNxN<Scaler> weight_e, weight_n;
            Eigen::SparseMatrix<Scaler> P(jacobian.cols(), jacobian.cols());

            weight_n = Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            const Math::MatrixNxN<Scaler> mP = jacobian.transpose() * weight_e * jacobian + weight_n;

            q = -jacobian.transpose() * weight_e * error;

            for(unsigned int i = 0; i < P.cols(); i ++) {
                for(unsigned int j = 0; j < P.rows(); j ++) {
                    P.insert(j, i) = mP(j, i);
                }
            }

            //! @todo Throw an exception if the size is different.
            u = this->param.limit_q_max() - joint_state.q;
            l = this->param.limit_q_min() - joint_state.q;

            if(solver->isInitialized()) {
                solver->updateLowerBound(l);
                solver->updateUpperBound(u);
                solver->updateHessianMatrix(P);
                solver->updateGradient(q);
            }
            else {
                Eigen::SparseMatrix<Scaler> A(jacobian.cols(), jacobian.cols());
                A.setIdentity();

                solver->data()->setNumberOfVariables(jacobian.cols());
                solver->data()->setNumberOfConstraints(jacobian.cols());

                solver->data()->setLowerBound(l);
                solver->data()->setUpperBound(u);

                solver->data()->setHessianMatrix(P);
                solver->data()->setGradient(q);
                solver->data()->setLinearConstraintsMatrix(A);

                solver->initSolver();
            }

            solver->solve();
            delta_step = solver->getSolution();

            return delta_step;
        }

        template <typename Scaler>
        void QProblemLMByOsqp<Scaler>::solve_setting(const Kinematic::JointState<Scaler> &init_joint_state) {
            if(solve_number_of_variables == init_joint_state.size()) {
                return;
            }
            solve_number_of_variables = init_joint_state.size();

            if(!solver->isInitialized()) {
                solver->clearSolver();
            }

            solver->settings()->setWarmStart(true);
            solver->settings()->setVerbosity(false);
        }

        template class QProblemLMByOsqp<double>;
    }
}

