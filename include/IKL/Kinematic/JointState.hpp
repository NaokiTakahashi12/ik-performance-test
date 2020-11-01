
#pragma once

#include "KinematicUnitBase.hpp"

#include "../Math/Vector.hpp"

namespace IKL {
    namespace Kinematic {
        template <typename Scaler>
        class JointState : public KinematicUnitBase<Scaler> {
            public :
                Math::VectorN<Scaler> q, qdot, qddot, tau;

                JointState() {
                }

                JointState(const unsigned int &rank) : KinematicUnitBase<Scaler>() {
                    resize(rank);
                    set_zero();
                }

                virtual ~JointState() {
                }

                void resize(const unsigned int &rank) {
                    q = Math::VectorN<Scaler>::Zero(rank);
                    qdot = Math::VectorN<Scaler>::Zero(rank);
                    qddot = Math::VectorN<Scaler>::Zero(rank);
                    tau = Math::VectorN<Scaler>::Zero(rank);
                }

                void set_epsilon() override {
                    q = this->epsilon * Math::VectorN<Scaler>::Ones(q.size());
                    qdot = q;
                    qddot = q;
                    tau = q;
                }

                void set_zero() override {
                    q.setZero();
                    qdot.setZero();
                    qddot.setZero();
                    tau.setZero();
                };

                unsigned int size() const override {
                    return q.size();
                }

                Math::VectorN<Scaler> diff_pos(const Math::VectorN<Scaler> &pos) {
                    return q - pos;
                }

                JointState &operator = (const JointState &joint_state) {
                    if(this != &joint_state) {
                        this->q = joint_state.q;
                        this->qdot = joint_state.qdot;
                        this->qddot = joint_state.qddot;
                        this->tau = joint_state.tau;
                    }
                    return *this;
                }
        };
    }
}

