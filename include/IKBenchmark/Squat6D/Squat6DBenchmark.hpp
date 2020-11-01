
#pragma once

#include <vector>

#include <IKL/Kinematic/Kinematic.hpp>

#include "SquatMotion.hpp"

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        class Squat6DBenchmark {
            public :
                using IKSolvers = std::vector<typename SquatMotion<Scaler>::IKSolverPtr>;

                Squat6DBenchmark();
                ~Squat6DBenchmark();

                void output_directory(const std::string &);
                std::string output_directory() const;

                void move_foot_name(const std::string &);
                const typename SquatMotion<Scaler>::BodyNames &move_foot_name() const;

                void fixed_body_name(const std::string &);
                const typename SquatMotion<Scaler>::BodyNames &fixed_body_name() const;

                void init_joint_state(const IKL::Kinematic::JointState<Scaler> &);
                IKL::Kinematic::JointState<Scaler> init_joint_state() const;

                void move_ik_solvers(IKSolvers &);

                void operator () (typename SquatMotion<Scaler>::Model &);

            private :
                std::string output_log_directory;

                typename SquatMotion<Scaler>::BodyNames  move_body_names,
                                                         fixed_body_names;

                IKL::Kinematic::JointState<Scaler> initial_joint_state;

                IKSolvers iks;
        };
    }
}

