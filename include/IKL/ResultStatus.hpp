
#pragma once

#include <chrono>

#include "Math/Math.hpp"

namespace IKL {
    template <class ResultObject, typename Scaler = double>
    class ResultStatus {
        public :
            enum class ExitStatus : unsigned short {
                Unknown = 0,
                BelowStepTolerance,
                BelowPoseTolerance,
                CountOver,
                TimeOver
            };

            ResultStatus() {
                reset();
            }

            virtual ~ResultStatus() {
            }

            unsigned int loop;
            Scaler error_norm;
            Scaler delta_step_norm;
            Scaler time_ms;

            ResultObject object;

            void reset() {
                exit_status = ExitStatus::Unknown;
                loop = 0;
                error_norm = 0;
                delta_step_norm = 0;
                time_ms = 0;
            }

            void time(
                    const std::chrono::high_resolution_clock::time_point &start, 
                    const std::chrono::high_resolution_clock::time_point &end
                    ) {
                time_ms = std::chrono::duration<Scaler, std::milli>(end - start).count();
            }

            bool is_success() const {
                if(exit_status == ExitStatus::BelowPoseTolerance || exit_status == ExitStatus::BelowStepTolerance) {
                    return true;
                }
                else {
                    return false;
                }
            }

            bool is_below_step_tole() const {
                if(exit_status == ExitStatus::BelowStepTolerance) {
                    return true;
                }
                else {
                    return false;
                }
            }

            bool is_below_pose_tole() const {
                if(exit_status == ExitStatus::BelowPoseTolerance) {
                    return true;
                }
                else {
                    return false;
                }
            }

            bool is_count_over() const {
                if(exit_status == ExitStatus::CountOver) {
                    return true;
                }
                else {
                    return false;
                }
            }

            bool is_time_over() const {
                if(exit_status == ExitStatus::TimeOver) {
                    return true;
                }
                else {
                    return false;
                }
            }

            void below_step_tolerance() {
                exit_status = ExitStatus::BelowStepTolerance;
            }

            void below_pose_tolerance() {
                exit_status = ExitStatus::BelowPoseTolerance;
            }

            void count_over() {
                exit_status = ExitStatus::CountOver;
            }

            void time_over() {
                exit_status = ExitStatus::TimeOver;
            }

        protected :
            ExitStatus exit_status;

    };
}

