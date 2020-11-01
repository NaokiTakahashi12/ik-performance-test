
#include <IKBenchmark/Squat6D/Squat6DBenchmark.hpp>

#include <iostream>
#include <stdexcept>

#include <IKBenchmark/Squat6D/Case1.hpp>
#include <IKBenchmark/Squat6D/Case2.hpp>
#include <IKBenchmark/Squat6D/Case3.hpp>
#include <IKBenchmark/Squat6D/Case4.hpp>

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        Squat6DBenchmark<Scaler>::Squat6DBenchmark() {
        }

        template <typename Scaler>
        Squat6DBenchmark<Scaler>::~Squat6DBenchmark() {
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::output_directory(const std::string &output_log_directory) {
            this->output_log_directory = output_log_directory;
        }

        template <typename Scaler>
        std::string Squat6DBenchmark<Scaler>::output_directory() const {
            return output_log_directory;
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::move_foot_name(const std::string &body_name) {
            for(const auto &name : move_body_names) {
                if(body_name == name) {
                    throw std::runtime_error("Failed move_foot_name reason same key");
                }
            }
            move_body_names.push_back(body_name);
        }

        template <typename Scaler>
        const typename SquatMotion<Scaler>::BodyNames &Squat6DBenchmark<Scaler>::move_foot_name() const {
            return move_body_names;
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::fixed_body_name(const std::string &body_name) {
            for(const auto &name : fixed_body_names) {
                if(body_name == name) {
                    throw std::runtime_error("Faild fixed_body_name reason same key");
                }
            }
            fixed_body_names.push_back(body_name);
        }

        template <typename Scaler>
        const typename SquatMotion<Scaler>::BodyNames &Squat6DBenchmark<Scaler>::fixed_body_name() const {
            return fixed_body_names;
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::init_joint_state(const IKL::Kinematic::JointState<Scaler> &new_initial_joint_state) {
            initial_joint_state.resize(new_initial_joint_state.size());
            initial_joint_state.q = new_initial_joint_state.q;
        }

        template <typename Scaler>
        IKL::Kinematic::JointState<Scaler> Squat6DBenchmark<Scaler>::init_joint_state() const {
            return initial_joint_state;
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::move_ik_solvers(IKSolvers &ik_solvers) {
            iks = std::move(ik_solvers);
        }

        template <typename Scaler>
        void Squat6DBenchmark<Scaler>::operator () (typename SquatMotion<Scaler>::Model &model) {
            //! @todo range based for case
            for(auto &&ik : iks) {
                std::cout << "Start benchmark squat case 1 " << std::endl;
                Case1<Scaler> case_1;
                case_1.initial_status(initial_joint_state);
                if(fixed_body_names.size() == 0) {
                    case_1.collect_log(model, ik, move_body_names);
                }
                else {
                    case_1.collect_log(model, ik, move_body_names, fixed_body_names);
                }
                case_1.save_log(output_log_directory + "/squat_case_1_" + ik->get_name());
                std::cout << "Finish benchmark squat case 1" << std::endl;
            }
            for(auto &&ik : iks) {
                std::cout << "Start benchmark squat case 2" << std::endl;
                Case2<Scaler> case_2;
                case_2.initial_status(initial_joint_state);
                if(fixed_body_names.size() == 0) {
                    case_2.collect_log(model, ik, move_body_names);
                }
                else {
                    case_2.collect_log(model, ik, move_body_names, fixed_body_names);
                }
                case_2.save_log(output_log_directory + "/squat_case_2_" + ik->get_name());
                std::cout << "Finish benchmark squat case 2" << std::endl;
            }
            for(auto &&ik : iks) {
                std::cout << "Start benchmark squat case 3" << std::endl;
                Case3<Scaler> case_3;
                case_3.initial_status(initial_joint_state);
                if(fixed_body_names.size() == 0) {
                    case_3.collect_log(model, ik, move_body_names);
                }
                else {
                    case_3.collect_log(model, ik, move_body_names, fixed_body_names);
                }
                case_3.save_log(output_log_directory + "/squat_case_3_" + ik->get_name());
                std::cout << "Finish benchmark squat case 3" << std::endl;
            }
            for(auto &&ik : iks) {
                std::cout << "Start benchmark squat case 4" << std::endl;
                Case4<Scaler> case_4;
                case_4.initial_status(initial_joint_state);
                if(fixed_body_names.size() == 0) {
                    case_4.collect_log(model, ik, move_body_names);
                }
                else {
                    case_4.collect_log(model, ik, move_body_names, fixed_body_names);
                }
                case_4.save_log(output_log_directory + "/squat_case_4_" + ik->get_name());
                std::cout << "Finish benchmark squat case 4" << std::endl;
            }
        }

        template class Squat6DBenchmark<double>;
    }
}

