
#pragma once

#include <limits>

namespace IKL {
    namespace Kinematic {
        template <typename Scaler>
        class KinematicUnitBase {
            public :
                using ScalerType = Scaler;

                KinematicUnitBase() : epsilon(std::numeric_limits<Scaler>::epsilon()) {
                }

                virtual ~KinematicUnitBase() {
                }

                virtual void set_zero() = 0;
                virtual void set_epsilon() = 0;
                virtual unsigned int size() const = 0;

            protected :
                const Scaler epsilon;

        };
    }
}

