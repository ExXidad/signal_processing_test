//
// Created by Арсений Плахотнюк on 27.07.2023.
//

#ifndef LASER_P_GUIDANCEDEVICE_HPP
#define LASER_P_GUIDANCEDEVICE_HPP

#include "Laser-P/Utils/Utils.hpp"


namespace GimbalControl{


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

        ControlPID(scalar Kp, scalar Ki, scalar Kd){
            Kp_ = Kp;
            Ki_ = Ki;
            Kd_ = Kd;
            integralError_ = Vector<T, Size>::Zero();
            prevError_ = Vector<T, Size>::Zero();
        }

        Vector<T, Size> calculate(const Vector<T, Size>& currentState,
                                  const Vector<T, Size>& goalState, const T timeStep){
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


//    struct TwoAxisGimbal{
//        /**
//         * Структура двух осевого поворотного устройства.
//         */
//
//         Vector2d m; // [mI, mO] - inner gimbal and outer gimbal mass
//         Vector2d q; // [epsilon, eta] - angular position of the inner gimbal and outer gimbal
//         Vector2d qDot; // [epsilonDot, etaDot] - angular velocity of the inner gimbal and outer gimbal
//         Vector3d r_PGI; // vector from pivot point to mass center of inner gimbal
//         Vector3d r_PGO; // vector from pivot point to mass center of outer gimbal
//         Vector3d OmegaO; // angular velocity of the outer gimbal in base coordinate system
//         Vector3d OmegaI; // angular velocity of the inner gimbal in base coordinate system
//         Matrix3d Ji; // tensor of inertia of the inner gimbal calculated about the pivot point
//         Matrix3d Jo; // tensor of inertia of the outer gimbal calculated about the pivot point
//    };
//
//
//    void controlStepTwoAxisGimbal(TwoAxisGimbal& twoAxisGimbal, const Vector2d& goalState, const Vector2d& goalStateDot,
//                                     ControlPID<double, 2>& controlPid, const double dt){
//        /**
//         * Функция обновляет состояние поворотки и контроллера
//         * @param twoAxisGimbal - двух осевое поворотное устройство
//         * @param controlGoalPoint - целевое состояние осей поворотки
//         * @param controlPid - контроллер
//         * @param dt - шаг по времени
//         */
//
//        //Euler’s equation of the inner gimbal about the pivot point:
//        // I_i * alpha + OmegaI x I_i * OmegaI = M + D_I
//        // D = mI * r_PGI x g_I
//        // M_I = [0, T_mI + T_frI, 0]
//        // alpha = I_i^(-1) * (M + D_I - Omega x I_i * Omega)
//
//
//        const Vector2d Tmotor = controlPid.calculate(twoAxisGimbal.q, twoAxisGimbal.qDot, goalState, goalStateDot, dt); // [T_mI, T_mO]
//        const Vector2d T_fr = {0., 0.};
//        const Vector3d M_I = {0, Tmotor[0] + T_fr[0], 0};
//        const Vector3d gI = Vector3d::Zero();
//        const Vector3d D_I = twoAxisGimbal.r_PGI.cross(gI) * twoAxisGimbal.m[0];
//
//
//        const Vector3d alpha_i =
//                twoAxisGimbal.Ji.inverse() * (M_I + D_I - twoAxisGimbal.OmegaI.cross(twoAxisGimbal.Ji * twoAxisGimbal.OmegaI));
//
//        twoAxisGimbal.OmegaI = twoAxisGimbal.OmegaI + alpha_i * dt;
//
//        twoAxisGimbal.qDot[0] = twoAxisGimbal.OmegaI[1];
//        twoAxisGimbal.q[0] = twoAxisGimbal.q[0] + twoAxisGimbal.qDot[0] * dt;
//
//        const double epsilon = twoAxisGimbal.q[0];
//        const double epsilonDot = twoAxisGimbal.qDot[0];
//        const Matrix3d Tio {
//            {std::cos(epsilon), 0, std::sin(epsilon)},
//            {0, 1, 0},
//            {-std::cos(epsilon), 0, std::sin(epsilon)}
//        };
//
//        const Matrix3d TioDot {
//                {std::sin(epsilon)*epsilonDot, 0, std::cos(epsilon)*epsilonDot},
//                {0, 0, 0},
//                {-std::sin(epsilon)*epsilonDot, 0, std::cos(epsilon)*epsilonDot}
//        };
//
//        // M_bo = [0, 0, Tmo + T_fr_bo]
//        // M_io = [0, 0, T_fr_io]
//        // D_o = (mI + mO) * r_PGO x g
//
//        // I_o * alpha_o + TioDot * I_i * OmegaI + Tio * I_i * alpha_i + Omega x (I_o * OmegaO + Tio*I_o*OmegaI) =
//        // = M_bo + M_io + D_o
//
//
//        const Vector3d M_bo = {0, 0, Tmotor[1] + T_fr[1]};
//        const Vector3d M_io = {0, 0, T_fr[0]};
//        const Vector3d gO = Vector3d::Zero();
//        const Vector3d D_o = (twoAxisGimbal.m[0] + twoAxisGimbal.m[0]) * twoAxisGimbal.r_PGO.cross(gO);
//
//        const Vector3d alpha_o =
//                twoAxisGimbal.Jo.inverse() * (M_bo + M_io + D_o - (TioDot * twoAxisGimbal.Ji * twoAxisGimbal.OmegaI + Tio * twoAxisGimbal.Ji * alpha_i + twoAxisGimbal.OmegaO.cross((twoAxisGimbal.Jo * twoAxisGimbal.OmegaO + Tio*twoAxisGimbal.Jo*twoAxisGimbal.OmegaI))));
//
//        twoAxisGimbal.OmegaO = twoAxisGimbal.OmegaO + alpha_o * dt;
//
//        twoAxisGimbal.qDot[1] =  twoAxisGimbal.OmegaO[2];
//
//         twoAxisGimbal.q[1] = twoAxisGimbal.q[1] + twoAxisGimbal.qDot[1] * dt;
//
//    }

}

#endif //LASER_P_GUIDANCEDEVICE_HPP
