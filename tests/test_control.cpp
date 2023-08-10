//
// Created by Арсений Плахотнюк on 10.08.2023.
//

#include <iostream>
#include "gtest/gtest.h"
#include <fstream>
#include "signal_processing_test/integrators/RightPart/RightPartOfDiffEq.hpp"
#include "signal_processing_test/integrators/implicitRungeKutta/ButcherTableIRK5.hpp"
#include "signal_processing_test/integrators/implicitRungeKutta/implicitRungeKutta.hpp"
#include "signal_processing_test/Control/Controllers.hpp"


const std::string FILE_PATH = __FILE__;
const std::string DIR_PATH = FILE_PATH.substr(0, FILE_PATH.size() - 16);

TEST(PID2, Control2){
    constexpr scalar Omega01 = 278;
    constexpr scalar Q1 = 27.4;
    constexpr scalar Q2 = 28.3;
    constexpr scalar Omega02 = 278;
    Vector<scalar, 4> currentState = {1, -0.5, 0, 0};
    const Vector<scalar, 4> goalState = {0, 0, 0, 0};

    const scalar Kp = 0.0;
    const scalar Ki = 0.3;
    const scalar Kd = 0.0;

    Control::ControlPID<scalar, 2> controlPid(Kp, Ki, Kd);

    const scalar step = 0.001;


    const Matrix2s A{{Omega01 / Q1, 0.},
                     {0., Omega02 / Q2}};

    const Matrix2s B{{Omega01*Omega01, 0.},
                     {0., Omega02*Omega02}};

    Vector2s u = {0., 0.};


    const scalar tolerance = 1.e-3;
    const Integrators::ImplicitRK::IntegrationParameters<Integrators::RightPartFSM> intParams = {step, step, tolerance};

    std::fstream file;
    file.open(DIR_PATH + "data_files/control_data/integratorImRk5.txt", std::ios::out);

    int n = 100;
    for (int i = 0; i < n; ++i)
    {
        Integrators::RightPartFSM::State initState = {currentState, 0.};
        u = controlPid.calculate(currentState.segment<2>(0), goalState.segment<2>(0), step);
        Integrators::RightPartFSM::Params params = {A, B, u, step};

        currentState =
            Integrators::ImplicitRK::ImplicitRungeKutta<Integrators::ImplicitRK::ButcherTableIRK5>::
                calc<Integrators::RightPartFSM>(initState, params, intParams)[1].u;

        file << i*step << " " << currentState(0) << " " << currentState(1) <<  "\n";
    }
    file.close();
}
