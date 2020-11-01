
#pragma once

#include "Math/Math.hpp"

namespace IKL {
    template <typename Scaler>
    class Parameters {
        public :
            Parameters() {
                max_iterations_reached = 1024;
                min_step_tolerance = 1e-9;
                min_pose_tolerance = 1e-5;

                weight_e_scaler_value = 1 - 1e-4;
                weight_n_scaler_value = 2e-5;

                manipulability_singuler_threshold = 1e-1;
                minimum_bias_for_damping_factor = 1e-3;
            }

            virtual ~Parameters() {
            }

            const unsigned int &max_itr_reached() const {
                return max_iterations_reached;
            }

            void max_itr_reached(const unsigned int &max_iterations_reached) {
                this->max_iterations_reached = max_iterations_reached;
            }

            const Scaler &min_step_tol() const {
                return min_step_tolerance;
            }

            void min_step_tol(const Scaler &min_step_tolerance) {
                this->min_step_tolerance = min_step_tolerance;
            }

            const Scaler &min_pose_tol() const {
                return min_pose_tolerance;
            }

            void min_pose_tol(const Scaler &min_pose_tolerance) {
                this->min_pose_tolerance = min_pose_tolerance;
            }

            const Scaler &weight_e_scaler() const {
                return weight_e_scaler_value;
            }

            void weight_e_scaler(const Scaler &weight_e_scaler_value) {
                this->weight_e_scaler_value = weight_e_scaler_value;
            }

            const Scaler &weight_n_scaler() const {
                return weight_n_scaler_value;
            }

            void weight_n_scaler(const Scaler &weight_n_scaler_value) {
                this->weight_n_scaler_value = weight_n_scaler_value;
            }

            const Scaler &manipulability_threshold() const {
                return manipulability_singuler_threshold;
            }

            void manipulability_threshold(const Scaler &manipulability_singuler_threshold) {
                this->manipulability_singuler_threshold = manipulability_singuler_threshold;
            }

            const Scaler &minimum_bias() const {
                return minimum_bias_for_damping_factor;
            }

            void minimum_bias(const Scaler &minimum_bias_for_damping_factor) {
                this->minimum_bias_for_damping_factor = minimum_bias_for_damping_factor;
            }

            const Math::VectorN<Scaler> &limit_q_min() const {
                return limit_of_q_min;
            }

            void limit_q_min(const Math::VectorN<Scaler> &q) {
                if(this->limit_of_q_min.size() != q.size()) {
                    limit_of_q_min.setZero(q.size());
                }
                this->limit_of_q_min = q;
            }

            const Math::VectorN<Scaler> &limit_q_max() const {
                return limit_of_q_max;
            }

            void limit_q_max(const Math::VectorN<Scaler> &q) {
                if(this->limit_of_q_max.size() != q.size()) {
                    limit_of_q_max.setZero(q.size());
                }
                this->limit_of_q_max = q;
            }

            Parameters &operator = (const Parameters &parameters) {
                if(this != &parameters) {
                    this->max_iterations_reached = parameters.max_iterations_reached;
                    this->min_step_tolerance = parameters.min_step_tolerance;
                    this->min_pose_tolerance = parameters.min_pose_tolerance;

                    this->weight_e_scaler_value = parameters.weight_e_scaler_value;
                    this->weight_n_scaler_value = parameters.weight_n_scaler_value;

                    this->manipulability_singuler_threshold = parameters.manipulability_singuler_threshold;
                    this->minimum_bias_for_damping_factor = parameters.minimum_bias_for_damping_factor;

                    this->limit_of_q_min = parameters.limit_of_q_min;
                    this->limit_of_q_max = parameters.limit_of_q_max;
                }
                return *this;
            }

        private :
            unsigned int max_iterations_reached;
            Scaler min_step_tolerance;
            Scaler min_pose_tolerance;

            Scaler weight_e_scaler_value;
            Scaler weight_n_scaler_value;

            Scaler manipulability_singuler_threshold;
            Scaler minimum_bias_for_damping_factor;

            Math::VectorN<Scaler> limit_of_q_min,
                                  limit_of_q_max;
    };
}

