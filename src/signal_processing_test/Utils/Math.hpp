//
// Created by Ivan Kalesnikau on 07.08.2023.
//

#ifndef SIGNAL_PROCESSING_TEST_MATH_HPP
#define SIGNAL_PROCESSING_TEST_MATH_HPP

#include "Types.hpp"

namespace math
{

constexpr scalar pi = M_PI;

template <typename T>
T sin(const T val)
{
    return sinf(val);
}

template <typename T>
T cos(const T val)
{
    return cosf(val);
}

template <typename T>
T sqrt(const T val)
{
    return sqrtf(val);
}

template <typename T>
T abs(const T val)
{
    return fabs(val);
}

}


#endif//SIGNAL_PROCESSING_TEST_MATH_HPP
