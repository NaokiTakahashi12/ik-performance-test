
#pragma once

#include "KinematicUnitBase.hpp"

#include "../Math/Vector.hpp"

namespace IKL {
    namespace Kinematic {
        template <typename Scaler>
        class CartesianPoint : public KinematicUnitBase<Scaler> {
            public :
                Math::Vector3<Scaler> r, rdot, rddot;

                CartesianPoint() : KinematicUnitBase<Scaler>() {
                    set_zero();
                }

                virtual ~CartesianPoint() {
                }

                void set_epsilon() override {
                    r = this->epsilon * Math::Vector3<Scaler>::Ones();
                    rdot = r;
                    rddot = r;
                }

                void set_zero() override {
                    r.setZero();
                    rdot.setZero();
                    rddot.setZero();
                }

                unsigned int size() const override {
                    return r.size();
                }

                Math::Vector3<Scaler> diff_pos(const Math::Vector3<Scaler> &pos) {
                    return r - pos;
                }

                CartesianPoint &operator = (const CartesianPoint &point) {
                    if(this != &point) {
                        this->r = point.r;
                        this->rdot = point.rdot;
                        this->rddot = point.rddot;
                    }
                    return *this;
                }
        };
    }
}

