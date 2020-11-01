
#pragma once

#include <chrono>

#include "Model/Model.hpp"
#include "Kinematic/Kinematic.hpp"
#include "Parameters.hpp"
#include "Constraint/Constraint.hpp"
#include "ResultStatus.hpp"

namespace IKL {
    template <typename Scaler>
    class NumericIK {
        public :
            using IKResultStatus = ResultStatus<Kinematic::JointState<Scaler>, Scaler>;

            NumericIK(const Parameters<Scaler> &param) {
                this->solver_name = "NumericIK";
                this->param = param;
            }

            virtual ~NumericIK() {
            }

            const std::string &get_name() const {
                return solver_name;
            }

            inline IKResultStatus solve(Model::RBDL &model, const Kinematic::JointState<Scaler> &init_joint_state, const Constraint::SetList<Scaler> &csets) {
                IKResultStatus result;
                auto joint_state = init_joint_state;
                unsigned int loop_counter = 0;

                solve_setting(init_joint_state);

                const auto start_time_point = std::chrono::high_resolution_clock::now();
                while(true) {
                    model.update_kinematics(joint_state);
                    decltype(auto) error = update_error(model, joint_state, csets);

                    if(error.norm() < param.min_pose_tol()) {
                        const auto end_time_point = std::chrono::high_resolution_clock::now();
                        result.error_norm = error.norm();
                        result.delta_step_norm = joint_state.qdot.norm();
                        result.loop = loop_counter;
                        result.time(start_time_point, end_time_point);
                        result.object.q = joint_state.q;
                        result.below_pose_tolerance();
                        break;
                    }

                    decltype(auto) jacobian = update_jacobian(model, joint_state, csets);
                    decltype(auto) delta_step = update_delta_step(jacobian, error, joint_state);

                    joint_state.qdot = delta_step;
                    joint_state.q += joint_state.qdot;

                    loop_counter ++;

                    if(loop_counter >= param.max_itr_reached()) {
                        const auto end_time_point = std::chrono::high_resolution_clock::now();
                        result.error_norm = error.norm();
                        result.delta_step_norm = joint_state.qdot.norm();
                        result.loop = loop_counter;
                        result.time(start_time_point, end_time_point);
                        result.object.q = joint_state.q;
                        result.count_over();
                        break;
                    }
                    if(delta_step.norm() < param.min_step_tol()) {
                        const auto end_time_point = std::chrono::high_resolution_clock::now();
                        result.error_norm = error.norm();
                        result.delta_step_norm = joint_state.qdot.norm();
                        result.loop = loop_counter;
                        result.time(start_time_point, end_time_point);
                        result.object.q = joint_state.q;
                        result.below_step_tolerance();
                        break;
                    }
                }
                return result;
            }

            NumericIK &operator = (const NumericIK &numeric_ik) {
                if(this != &numeric_ik) {
                    if(this->solver_name == numeric_ik.solver_name) {
                        this->param = numeric_ik.param;
                    }
                }
                return *this;
            }

        protected :
            std::string solver_name;

            Parameters<Scaler> param;

            virtual void solve_setting(const Kinematic::JointState<Scaler> &) {
            }

            virtual Math::VectorN<Scaler> &update_delta_step(const Math::MatrixNxN<Scaler> &jacobian, const Math::VectorN<Scaler> &error, const Kinematic::JointState<Scaler> &) = 0;

        private :
            const Math::VectorN<Scaler> &update_error(Model::RBDL &model, const Kinematic::JointState<Scaler> &joint_state, const Constraint::SetList<Scaler> &csets) {
                static Math::VectorN<Scaler> e;
                static Kinematic::CartesianPoint<Scaler> point;
                static Kinematic::EulerAngle<Scaler> angle;

                unsigned int c_size = 0;
                for(const auto &c : csets) {
                    c_size += c.unit_size();
                }

                if(e.size() != c_size) {
                    e.setZero(c_size);
                }

                unsigned int e_index = 0;
                for(const auto &c : csets) {
                    if(Constraint::UnitType::Twist6D == c.unit_type()) {
                        point.r = c.vector().block(3, 0, 3, 1);
                        angle.a = c.vector().block(0, 0, 3, 1);

                        e.block(e_index + 3, 0, 3, 1) = point.diff_pos(
                                model.to_body_point(c.body_id(), joint_state)
                        );
                        e.block(e_index, 0, 3, 1) = angle.diff_pos(
                                model.to_body_rotation(c.body_id(), joint_state)
                        );
                    }
                    else {
                        throw std::runtime_error("Not support");
                    }
                    e_index += c.unit_size();
                }
                return e;
            }

            const Math::MatrixNxN<Scaler> &update_jacobian(Model::RBDL &model, const Kinematic::JointState<Scaler> &joint_state, const Constraint::SetList<Scaler> &csets) {
                static Math::MatrixNxN<Scaler> J;

                unsigned int c_size = 0;
                for(const auto &c : csets) {
                    c_size += c.unit_size();
                }
                
                if(J.cols() != joint_state.size() || J.rows() != c_size) {
                    J.setZero(c_size, joint_state.size());
                }
                else {
                    J.setZero();
                }

                unsigned int J_index = 0;
                for(const auto &c : csets) {
                    if(Constraint::UnitType::Twist6D == c.unit_type()) {
                        J.block(J_index, 0, c.unit_size(), joint_state.size()) =
                            model.twist_basic_jacobian(c.body_id(), joint_state);
                    }
                    else {
                        throw std::runtime_error("Not support");
                    }
                    J_index += c.unit_size();
                }

                return J;
            }
    };
}

