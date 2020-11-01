
#pragma once

#include "../BenchmarkFileIO.hpp"

#include <memory>
#include <string>
#include <vector>

#include <IKL/NumericIK.hpp>
#include <IKL/Kinematic/Kinematic.hpp>

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        class SquatMotion : public BenchmarkFileIO {
            public :
                using Model = IKL::Model::RBDL;
                using IKSolver = IKL::NumericIK<Scaler>;
                using IKSolverPtr = std::unique_ptr<IKSolver>;
                using BodyNames = std::vector<std::string>;

                SquatMotion();
                virtual ~SquatMotion();

                void initial_status(const IKL::Kinematic::JointState<Scaler> &);
                const IKL::Kinematic::JointState<Scaler> &initial_status();

                virtual void collect_log(Model &, IKSolverPtr &, const BodyNames &move_body_names) = 0;
                virtual void collect_log(Model &, IKSolverPtr &, const BodyNames &move_body_names, const BodyNames &fixed_body_names) = 0;

            protected :
                struct BenchmarkConfig {
                    Scaler step_resolution,
                           maximum_step,
                           minimum_step;

                    BodyNames move_body_names,
                              fixed_body_names;
                };

                struct Logs {
                    std::vector<Scaler> loop,
                                        time_ms,
                                        error_norm,
                                        step_norm,
                                        below_pose_tole,
                                        below_step_tole,
                                        count_over,
                                        normalized_range;

                    std::vector<std::vector<Scaler>> result_joint_q;
                };

                IKL::Kinematic::JointState<Scaler> init_joint_state;

                void do_benchmark(Model &, IKSolverPtr &, const BenchmarkConfig &);
                void do_benchmark(Model &, IKSolverPtr &, const BenchmarkConfig &, const IKL::Kinematic::JointState<Scaler> &);

            private :
                unsigned int body_id_excepter(Model &, const std::string &body_name);
                void append_logs(Model &, const Logs &);
        };
    }
}

