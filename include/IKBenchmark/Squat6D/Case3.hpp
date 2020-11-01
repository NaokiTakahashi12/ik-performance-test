
#pragma once

#include "SquatMotion.hpp"

namespace IKBenchmark {
    namespace Squat6D {
        template <typename Scaler>
        class Case3 : public SquatMotion<Scaler> {
            public :
                Case3();
                virtual ~Case3();

                virtual void collect_log(
                        typename SquatMotion<Scaler>::Model &,
                        typename SquatMotion<Scaler>::IKSolverPtr &,
                        const typename SquatMotion<Scaler>::BodyNames &move_body_names) override;

                virtual void collect_log(
                        typename SquatMotion<Scaler>::Model &,
                        typename SquatMotion<Scaler>::IKSolverPtr &,
                        const typename SquatMotion<Scaler>::BodyNames &move_body_names,
                        const typename SquatMotion<Scaler>::BodyNames &fixed_body_names) override;

            private :
                void set_benchmark_param(typename SquatMotion<Scaler>::BenchmarkConfig &);
        };
    }
}
