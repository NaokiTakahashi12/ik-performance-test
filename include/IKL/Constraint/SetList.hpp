
#pragma once

#include <vector>

#include "Set.hpp"

namespace IKL {
    namespace Constraint {
        template <typename Scaler>
        class SetList {
            public :
                SetList() {
                }

                virtual ~SetList() {
                }

                void append(const unsigned int &body_id, const UnitType &unit_type, const Math::VectorN<Scaler> &vec) {
                    append(Set<Scaler>(body_id, unit_type), vec);
                }

                void append(const Set<Scaler> &set, const Math::VectorN<Scaler> &vec) {
                    for(const auto &s : list) {
                        if(s.body_id() == set.body_id()) {
                            throw std::runtime_error("Not support append some body_id");
                        }
                    }
                    list.push_back(set);
                    list.back().vector(vec);
                }

                void reset() {
                    list.reset();
                }

                const Set<Scaler> &at(const unsigned int &i) const {
                    return list.at(i);
                }

                Set<Scaler> &at(const unsigned int &i) {
                    return list.at(i);
                }

                unsigned int size() const {
                    return list.size();
                }

                const std::vector<Set<Scaler>> &stl_container() const {
                    return list;
                }

                decltype(auto) begin() const {
                    return list.begin();
                }

                decltype(auto) begin() {
                    return list.begin();
                }

                decltype(auto) end() const {
                    return list.end();
                }

                decltype(auto) end() {
                    return list.end();
                }

                std::vector<Set<Scaler>> &stl_container() {
                    return list;
                }

                const Set<Scaler> &operator [] (const unsigned int &i) const {
                    return list[i];
                }

                Set<Scaler> &operator [] (const unsigned int &i) {
                    return list[i];
                }

                SetList &operator = (const SetList &set_list) const {
                    if(this != &set_list) {
                        this->list.reset();
                        this->list.resize(set_list.size());
                        std::copy(set_list.list.begin(), set_list.list.end(), this->list.begin());
                    }
                    return *this;
                }

            private :
                std::vector<Set<Scaler>> list;

        };
    }
}

