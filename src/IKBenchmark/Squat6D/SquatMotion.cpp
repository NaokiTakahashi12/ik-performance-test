
#include <IKBenchmark/Squat6D/SquatMotion.hpp>

#include <iostream>
#include <iomanip>

#include <IKL/Kinematic/EulerAngle.hpp>
#include <IKL/Constraint/Constraint.hpp>

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        SquatMotion<Scaler>::SquatMotion() {
        }

        template <typename Scaler>
        SquatMotion<Scaler>::~SquatMotion() {
        }

        template <typename Scaler>
        void SquatMotion<Scaler>::initial_status(const IKL::Kinematic::JointState<Scaler> &joint_state) {
            init_joint_state.resize(joint_state.size());
            init_joint_state.q = joint_state.q;
        }

        template <typename Scaler>
        const IKL::Kinematic::JointState<Scaler> &SquatMotion<Scaler>::initial_status() {
            return init_joint_state;
        }

        template <typename Scaler>
        void SquatMotion<Scaler>::do_benchmark(Model &model, IKSolverPtr &ik_solver, const BenchmarkConfig &config) {
            do_benchmark(model, ik_solver, config, init_joint_state);
        }

        template <typename Scaler>
        void SquatMotion<Scaler>::do_benchmark(Model &model, IKSolverPtr &ik_solver, const BenchmarkConfig &config, const IKL::Kinematic::JointState<Scaler> &init_joint_state) {
            if(config.move_body_names.size() == 0) {
                throw std::runtime_error("Failed move_body_names is zero");
            }
            //! @todo Support multiple id
            const auto move_body_id =
                body_id_excepter(model, config.move_body_names.front());
            const auto zero_joint_state = IKL::Kinematic::JointState<Scaler>(init_joint_state.size());

            model.update_kinematics(zero_joint_state);
            const IKL::Math::Vector3<Scaler> initial_move_leg_range =
                model.to_body_point(move_body_id, zero_joint_state);

            Logs logs;
            logs.result_joint_q.resize(model.dof());

            const Scaler maximum_point_z = initial_move_leg_range.z() * config.maximum_step;
            const Scaler minimum_point_z = initial_move_leg_range.z() * config.minimum_step;
            const Scaler step_z = initial_move_leg_range.z() * config.step_resolution;

            IKL::Kinematic::EulerAngle<Scaler> angle;
            IKL::Constraint::SetList<Scaler> csets;
            // Movement body constraint
            {
                IKL::Math::Vector6<Scaler> cvec = IKL::Math::Vector6<Scaler>::Zero();

                cvec.block(3, 0, 3, 1) = initial_move_leg_range;
                cvec.block(0, 0, 3, 1) = angle.pos(
                    model.to_body_rotation(move_body_id, zero_joint_state)
                );
                cvec(5) = minimum_point_z;

                csets.append(move_body_id, IKL::Constraint::UnitType::Twist6D, cvec);
            }
            // Fixed body constraint
            for(const auto &name : config.fixed_body_names) {
                const auto fixed_body_id = body_id_excepter(model, name);

                IKL::Math::Vector6<Scaler> cvec = IKL::Math::Vector6<Scaler>::Zero();
                cvec.block(3, 0, 3, 1) = model.to_body_point(fixed_body_id, zero_joint_state);
                cvec.block(0, 0, 3, 1) = angle.pos(
                    model.to_body_rotation(fixed_body_id, zero_joint_state)
                );

                csets.append(fixed_body_id, IKL::Constraint::UnitType::Twist6D, cvec);
            }

            Scaler normalized_step = config.minimum_step;
            for(Scaler step = minimum_point_z; step >= maximum_point_z; step += step_z) {
                model.update_kinematics(init_joint_state.size());
                const auto result = ik_solver->solve(model, init_joint_state, csets);

                logs.loop.push_back(result.loop);
                logs.time_ms.push_back(result.time_ms);
                logs.error_norm.push_back(result.error_norm);
                logs.step_norm.push_back(result.delta_step_norm);
                logs.below_pose_tole.push_back(result.is_below_pose_tole());
                logs.below_step_tole.push_back(result.is_below_step_tole());
                logs.count_over.push_back(result.is_count_over());
                logs.normalized_range.push_back(normalized_step);

                for(unsigned int i = 0; i < result.object.q.size(); i ++) {
                    logs.result_joint_q.at(i).push_back(result.object.q(i));
                }

                std::cout
                    << std::setprecision(8)
                    << ik_solver->get_name()
                    << ": { Step: "
                    << normalized_step
                    << ", Target: "
                    << step
                    << ", Loop: "
                    << result.loop
                    << ", Time: "
                    << result.time_ms
                    << ", ErrorNorm: "
                    << result.error_norm
                    << ", DeltaNorm: "
                    << result.delta_step_norm
                    << " }"
                    << std::endl;

                normalized_step += config.step_resolution;
                csets.at(0).vector()(5) = step;
            }
            append_logs(model, logs);
        }

        template <typename Scaler>
        unsigned int SquatMotion<Scaler>::body_id_excepter(Model &model, const std::string &body_name) {
            const unsigned int body_id = model.body_name_to_id(body_name);

            if("" == model.body_id_to_name(body_id)) {
                throw std::runtime_error("Not match body_id " + body_name);
            }
            return body_id;
        }

        template <typename Scaler>
        void SquatMotion<Scaler>::append_logs(Model &model, const Logs &logs) {
            csv_parser.append("Loop", logs.loop);
            csv_parser.append("Time_ms", logs.time_ms);
            csv_parser.append("Error_norm", logs.error_norm);
            csv_parser.append("Step_norm", logs.step_norm);
            csv_parser.append("Below_pose_tolerance", logs.below_pose_tole);
            csv_parser.append("Below_step_tolerance", logs.below_step_tole);
            csv_parser.append("Count_over", logs.count_over);
            csv_parser.append("Normalized_range", logs.normalized_range);

            for(unsigned int i = 0; i < logs.result_joint_q.size(); i ++) {
                csv_parser.append(model().GetBodyName(i + 1), logs.result_joint_q.at(i));
            }
        }

        template class SquatMotion<double>;
    }
}

