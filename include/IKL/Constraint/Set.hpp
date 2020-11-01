
#pragma once

#include <stdexcept>

#include "../Math/Math.hpp"

#include "UnitType.hpp"

namespace IKL {
    namespace Constraint {
        template <typename Scaler>
        class Set {
            public :
                Set(const unsigned int &body_id, const UnitType &unit_type) : constraint_body_id(body_id), constraint_unit_type(unit_type) {
                    if(constraint_unit_type == UnitType::Twist6D) {
                        constraint_unit_size = 6;
                        constraint_vector.resize(constraint_unit_size);
                    }
                    else {
                        throw std::runtime_error("Unknown constraint unit type");
                    }
                    constraint_vector.setZero();
                }

                Set(const Set &set) : Set(set.constraint_body_id, set.constraint_unit_type) {
                    if(this != &set) {
                        this->constraint_vector = set.constraint_vector;
                    }
                }

                virtual ~Set() {
                }

                void vector(const Math::VectorN<Scaler> &vec) {
                    if(vec.size() != constraint_vector.size()) {
                        throw std::runtime_error("Different constraint vector size");
                    }
                    constraint_vector = vec;
                }

                const Math::VectorN<Scaler> &vector() const {
                    return constraint_vector;
                }

                Math::VectorN<Scaler> &vector() {
                    return constraint_vector;
                }

                unsigned int body_id() const {
                    return constraint_body_id;
                }

                UnitType unit_type() const {
                    return constraint_unit_type;
                }

                unsigned int unit_size() const {
                    return constraint_unit_size;
                }

                Set &operator = (const Set &set) const {
                    if(this != &set) {
                        set_checker(set);
                        this->constraint_vector = set.constraint_vector;
                    }
                    return *this;
                }

                Set &operator += (const Set &set) const {
                    if(this != &set) {
                        set_checker(set);
                        this->constraint_vector += set.constraint_vector;
                    }
                    return *this;
                }

                Set &operator -= (const Set &set) const {
                    if(this != &set) {
                        set_checker(set);
                        this->constraint_vector -= set.constraint_vector;
                    }
                    return *this;
                }

            protected :
                const unsigned int constraint_body_id;
                const UnitType constraint_unit_type;

                unsigned int constraint_unit_size;

                Math::VectorN<Scaler> constraint_vector;

                void set_checker(const Set &set) {
                    if(this->constraint_body_id != set.constraint_body_id) {
                        throw std::runtime_error("Different constraint body id");
                    }
                    if(this->constraint_unit_type != set.constraint_unit_type) {
                        throw std::runtime_error("Different constraint unit type");
                    }
                }

        };
    }
}

