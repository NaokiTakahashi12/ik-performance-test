
#include <IKBenchmark/Squat6D/Case4.hpp>

#include <stdexcept>

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        Case4<Scaler>::Case4() : SquatMotion<Scaler>::SquatMotion() {
        }

        template <typename Scaler>
        Case4<Scaler>::~Case4() {
        }

        template <typename Scaler>
        void Case4<Scaler>::collect_log(
                typename SquatMotion<Scaler>::Model &model,
                typename SquatMotion<Scaler>::IKSolverPtr &ik_solver,
                const typename SquatMotion<Scaler>::BodyNames &move_body_names) {
            if(this->init_joint_state.size() != model.dof()) {
                throw std::runtime_error("Failed squad6d benchmark case1 reason different joint size");
            }
            auto zero_init_joint_state = this->init_joint_state;
            zero_init_joint_state.set_epsilon();

            typename SquatMotion<Scaler>::BenchmarkConfig config;
            set_benchmark_param(config);

            config.move_body_names = move_body_names;

            this->do_benchmark(model, ik_solver, config, zero_init_joint_state);
        }

        template <typename Scaler>
        void Case4<Scaler>::collect_log(
                typename SquatMotion<Scaler>::Model &model,
                typename SquatMotion<Scaler>::IKSolverPtr &ik_solver,
                const typename SquatMotion<Scaler>::BodyNames &move_body_names,
                const typename SquatMotion<Scaler>::BodyNames &fixed_body_names) {
            if(this->init_joint_state.size() != model.dof()) {
                throw std::runtime_error("Failed squad6d benchmark case1 reason different joint size");
            }
            auto zero_init_joint_state = this->init_joint_state;
            zero_init_joint_state.set_epsilon();

            typename SquatMotion<Scaler>::BenchmarkConfig config;
            set_benchmark_param(config);

            config.move_body_names = move_body_names;
            config.fixed_body_names = fixed_body_names;

            this->do_benchmark(model, ik_solver, config, zero_init_joint_state);
        }

        template <typename Scaler>
        void Case4<Scaler>::set_benchmark_param(typename SquatMotion<Scaler>::BenchmarkConfig &config) {
            constexpr Scaler normalied_maximum_range = 1.00;
            constexpr Scaler normalied_minimum_range = 0.75;
            constexpr Scaler normalied_step_range = 0.005;

            config.step_resolution = normalied_step_range;
            config.maximum_step = normalied_maximum_range;
            config.minimum_step = normalied_minimum_range;
        }

        template class Case4<double>;
    }
}

