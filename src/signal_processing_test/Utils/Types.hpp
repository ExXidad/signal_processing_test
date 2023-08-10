//
// Created by Ivan Kalesnikau on 29.06.2023.
//

#ifndef QPD_TYPES_HPP
#define QPD_TYPES_HPP

//#include "../../third_party/MCU_Libs/eigen/Eigen/Dense"
#include <Eigen/Dense>

using indexType = uint32_t;
using sizeType = uint32_t;
using scalar = float;
using ADC_ReadingType = uint16_t;
using byte = uint8_t;

using Matrix2s = Eigen::Matrix<scalar, 2, 2>;
using Vector2s = Eigen::Vector<scalar, 2>;

#ifdef MCU
#include <etl/array.h>
template<typename T, sizeType size>
using array = etl::array<T, size>;
#endif
#ifdef PC
#include <array>
template<typename T, sizeType size>
using array = std::array<T, size>;
#endif



//template<typename T>
//using ArrayType = etl::array<T, size>;

#endif// QPD_TYPES_HPP
