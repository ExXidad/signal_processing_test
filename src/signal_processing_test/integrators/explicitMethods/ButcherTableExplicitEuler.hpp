//
// Created by Арсений Плахотнюк on 07.08.2023.
//

#ifndef LASER_P_BUTCHERTABLEEXPLICITEULER_HPP
#define LASER_P_BUTCHERTABLEEXPLICITEULER_HPP

#include "Laser-P/Utils/Utils.hpp"

namespace Integrators::ExplicitRK {

    // Forward Euler method
    class ButcherTableForwardEuler {
    public:
        constexpr static uint size = 1;
        constexpr static uint numOfSubDiagEl = 1;
        constexpr static std::array<scalar, size> column = {
                static_cast<scalar>(0)
        };
        constexpr static std::array<scalar, size> row = {
                static_cast<scalar>(1)
        };
        constexpr static std::array<scalar, numOfSubDiagEl> matrix = {
                static_cast<scalar>(0)
        };
    };
}


#endif //LASER_P_BUTCHERTABLEEXPLICITEULER_HPP
