//
// Created by Арсений Плахотнюк on 09.03.2023.
//

#ifndef BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
#define BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
#include "/Utils/Utils.hpp"
#include "Laser-P/GimbalControl/GuidanceDevice.hpp"

namespace Integrators {

    struct RightPartFSM{
        // thetaDot = p
        // pDot = A * (u(t, theta) - p) - B * theta
        constexpr static uint size = 4;
        using Time = double;

        struct Params{
            const Matrix2d A;
            const Matrix2d B;
            const Vector2d control;
            const scalar timeStep;

        };

        struct State {
            Vector<scalar, size> u;
            scalar t;
        };

        [[nodiscard]] static inline Vector<scalar, size> calc(const State &stateVector, Params &params) {

            const Vector2d u1 = stateVector.u.segment<2>(0);

            const Vector2d u1Dot = stateVector.u.segment<2>(2);

            const Vector2d controlU = params.control;

            const Vector2d force = params.B *(controlU - u1) - params.A * u1Dot;

            return Vector<scalar, size>{stateVector.u[2], stateVector.u[3], force[0], force[1]};

        }

    };

}

#endif //BALLISTICS_RIGHTPARTOFDIFFEQ_HPP
