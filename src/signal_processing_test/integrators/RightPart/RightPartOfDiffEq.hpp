//
// Created by Арсений Плахотнюк on 09.03.2023.
//

#ifndef BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
#define BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
#include "../../../signal_processing_test/Control/Controllers.hpp"
#include "../../../signal_processing_test/Utils/Types.hpp"

namespace Integrators {

    struct RightPartFSM{
        // thetaDot = p
        // pDot = B * (u(t, theta) - theta) - A * p
        constexpr static uint size = 4;
        using Time = scalar;

        struct Params{
            const Matrix2s A;
            const Matrix2s B;
            const Vector2s control;
            const scalar timeStep;

        };

        struct State {
            Vector<scalar, size> u;
            scalar t;
        };

        [[nodiscard]] static inline Vector<scalar, size> calc(const State &stateVector, Params &params) {

            const Vector2s u1 = stateVector.u.segment<2>(0);

            const Vector2s u1Dot = stateVector.u.segment<2>(2);

            const Vector2s controlU = params.control;

            const Vector2s force = params.B *(controlU - u1) - params.A * u1Dot;

            return Vector<scalar, size>{stateVector.u[2], stateVector.u[3], force[0], force[1]};

        }

    };

}

#endif //BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
