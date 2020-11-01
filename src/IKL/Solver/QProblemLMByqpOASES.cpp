
#include <IKL/Solver/QProblemLMByqpOASES.hpp>

#include <qpOASES/Options.hpp>

namespace IKL {
    namespace Solver {
        template <typename Scaler>
        QProblemLMByqpOASES<Scaler>::QProblemLMByqpOASES(const Parameters<Scaler> &param) : NumericIK<Scaler>(param) {
            this->solver_name = "QProblemLMByqpOASES";
            solve_number_of_variables = 0;
        }

        template <typename Scaler>
        QProblemLMByqpOASES<Scaler>::~QProblemLMByqpOASES() {
        }

        template <typename Scaler>
        Math::VectorN<Scaler> &QProblemLMByqpOASES<Scaler>::update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &joint_state) {
            static Math::VectorN<Scaler> delta_step;

            int maximum_number_of_iteration = 32;

            delta_step.setZero(jacobian.cols());

            Math::MatrixNxN<Scaler> weight_e, weight_n;
            weight_n = Math::VectorN<Scaler>::Ones(jacobian.cols()).asDiagonal();
            weight_e = this->param.weight_e_scaler() * Math::VectorN<Scaler>::Ones(jacobian.rows()).asDiagonal();

            Math::VectorN<Scaler> g, ub, lb;
            Math::MatrixNxN<Scaler, Math::RowMajor> H;

            g = -jacobian.transpose() * weight_e * error;
            H = jacobian.transpose() * weight_e * jacobian + weight_n;

            //! @todo Throw an exception if the size is different.
            ub = this->param.limit_q_max() - joint_state.q;
            lb = this->param.limit_q_min() - joint_state.q;

            qp_b->init(H.data(), g.data(), lb.data(), ub.data(), maximum_number_of_iteration);
            qp_b->getPrimalSolution(delta_step.data());

            return delta_step;
        }

        template <typename Scaler>
        void QProblemLMByqpOASES<Scaler>::solve_setting(const Kinematic::JointState<Scaler> &init_joint_state) {
            if(solve_number_of_variables == init_joint_state.size()) {
                return;
            }
            solve_number_of_variables = init_joint_state.size();

            qpOASES::Options qp_options;

            qp_options.setToMPC();
            qp_options.printLevel = qpOASES::PL_NONE;
            qp_options.enableRegularisation = qpOASES::BT_TRUE;

            if(qp_b != nullptr) {
                qp_b.reset();
            }
            qp_b = std::make_unique<qpOASES::QProblemB>(solve_number_of_variables);
            qp_b->setOptions(qp_options);
        }

        template class QProblemLMByqpOASES<double>;
    }
}

