//
// Created by Арсений Плахотнюк on 10.08.2023.
//

#ifndef SIGNAL_PROCESSING_TEST_UTILS_HPP
#define SIGNAL_PROCESSING_TEST_UTILS_HPP
#include "Types.hpp"

template<typename T, uint size>
static bool isEqualByRelativeError(const Vector<T, size> &l, const Vector<T, size> &r, const scalar tolerance)
{
    return (l - r).norm() <= tolerance * r.norm();
}
#endif  // SIGNAL_PROCESSING_TEST_UTILS_HPP
