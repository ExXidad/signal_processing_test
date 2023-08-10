//
// Created by Арсений Плахотнюк on 10.08.2023.
//

#ifndef SIGNAL_PROCESSING_TEST_UTILS_HPP
#define SIGNAL_PROCESSING_TEST_UTILS_HPP
#include "Types.hpp"
namespace SignalProcessingUtils {

template <typename T, uint size>
static bool isEqualByRelativeError(const Vector<T, size> &l, const Vector<T, size> &r, const scalar tolerance) {
    return (l - r).norm() <= tolerance * r.norm();
}

template <typename T>
T bound(const T value, const T lower, const T upper) {
    return (value > upper) ? upper : ((value < lower) ? lower : value);
}
}
#endif  // SIGNAL_PROCESSING_TEST_UTILS_HPP
