//
// Created by Арсений Плахотнюк on 27.07.2023.
//

#ifndef SIGNAL_PROCESSING_TEST_CONTROLLERS_HPP
#define SIGNAL_PROCESSING_TEST_CONTROLLERS_HPP

#include "../../signal_processing_test/Utils/Types.hpp"


namespace Control {

    template<typename T, uint Size>
    struct ControlPID{
        /**
         * ПИД-регулятор
         */
        scalar Kp_;
        scalar Ki_;
        scalar Kd_;
        Vector<T, Size> integralError_; // накопленная ошибка
        Vector<T, Size> prevError_;

        ControlPID(scalar Kp, scalar Ki, scalar Kd): Kp_(Kp), Ki_(Ki), Kd_(Kd),
                                                      integralError_(Vector<T, Size>::Zero()),
                                                      prevError_(Vector<T, Size>::Zero()) {};

        Vector<T, Size> calculate(const Vector<T, Size>& currentState,
                                  const Vector<T, Size>& goalState,
                                  const T timeStep){
            /**
             * Функция считает управляющее воздействие ПИД-регулятора
             */

            const Vector<T, Size> stateError = goalState - currentState;
            integralError_ += stateError * timeStep;
            const Vector<T, Size> derivativeError = (stateError - prevError_) / timeStep;
            prevError_ = stateError;

            return Kp_ * stateError + Ki_ * integralError_ + Kd_ * derivativeError;;
        }

    };
}

#endif //SIGNAL_PROCESSING_TEST_CONTROLLERS_HPP
