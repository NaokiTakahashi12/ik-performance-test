
#pragma once

#include "KinematicUnitBase.hpp"

#include "../Math/Vector.hpp"
#include "../Math/Matrix.hpp"

namespace IKL {
    namespace Kinematic {
        template <typename Scaler>
        class EulerAngle : public KinematicUnitBase<Scaler> {
            public :
                Math::Vector3<Scaler> a, adot, addot;

                EulerAngle() : KinematicUnitBase<Scaler>() {
                    set_zero();
                }

                virtual ~EulerAngle() {
                }

                void set_epsilon() override {
                    a = this->epsilon * Math::Vector3<Scaler>::Ones();
                    adot = a;
                    addot = a;
                }

                void set_zero() override {
                    a.setZero();
                    adot.setZero();
                    addot.setZero();
                }

                unsigned int size() const override {
                    return a.size();
                }

                EulerAngle &operator = (const EulerAngle &euler_angle) {
                    if(this != &euler_angle) {
                        this->a = euler_angle.a;
                        this->adot = euler_angle.adot;
                        this->addot = euler_angle.addot;
                    }
                    return *this;
                }

            private :
                virtual Math::Vector3<Scaler> angular_from_rotation(const Math::Matrix3x3<Scaler> &rotation) {
                    static Math::Vector3<Scaler> angl, l;

                    unsigned short count_ones = 0,
                                   count_negative_ones = 0;

                    for(unsigned int i = 0; i < 3; i ++) {
                        if(rotation(i, i) == -1) {
                            count_negative_ones ++;
                        }
                        if(std::abs(rotation(i, i)) == 1) {
                            count_ones ++;
                        }
                    }
                    if(count_negative_ones == 2) {
                        l << 1 + rotation(0, 0),
                             1 + rotation(1, 1),
                             1 + rotation(2, 2);
                        angl = l * M_PI / 2;
                    }
                    else if(count_ones == 3) {
                        angl.setZero();
                    }
                    else {
                        l << rotation(2, 1) - rotation(1, 2),
                             rotation(0, 2) - rotation(2, 0),
                             rotation(1, 0) - rotation(0, 1);
                        const Scaler norm = l.norm();
                        angl = l * std::atan2(
                            norm,
                            rotation(0, 0) + rotation(1, 1) + rotation(2, 2) - 1
                        ) / norm;
                    }

                    return angl;
                }

                virtual Math::Matrix3x3<Scaler> rotation_from_anguler(const Math::Vector3<Scaler> &anguler) {
                    static Math::Matrix3x3<Scaler> rotation;

                    rotation = Eigen::AngleAxis<Scaler>(anguler.z(), Math::Vector3<Scaler>::UnitZ());
                    rotation = rotation * Eigen::AngleAxis<Scaler>(anguler.y(), Math::Vector3<Scaler>::UnitY());
                    rotation = rotation * Eigen::AngleAxis<Scaler>(anguler.x(), Math::Vector3<Scaler>::UnitX());

                    return rotation;
                }

            public :
                Math::Vector3<Scaler> diff_pos(const Math::Vector3<Scaler> &pos) {
                    static Math::Matrix3x3<Scaler> a_rot, pos_rot;

                    a_rot = rotation_from_anguler(a);
                    pos_rot = rotation_from_anguler(pos);

                    return angular_from_rotation(a_rot.transpose() * pos_rot);
                }

                Math::Vector3<Scaler> diff_pos(const Math::Matrix3x3<Scaler> &pos_rot) {
                    static Math::Matrix3x3<Scaler> a_rot;

                    a_rot = rotation_from_anguler(a);

                    return angular_from_rotation(a_rot.transpose() * pos_rot);
                }

                Math::Vector3<Scaler> pos(const Math::Matrix3x3<Scaler> &rotation) {
                    return angular_from_rotation(rotation);
                }
        };
    }
}

