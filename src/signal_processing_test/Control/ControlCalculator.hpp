//
// Created by Арсений Плахотнюк on 10.08.2023.
//

#ifndef SIGNAL_PROCESSING_TEST_CONTROLCALCULATOR_HPP
#define SIGNAL_PROCESSING_TEST_CONTROLCALCULATOR_HPP
#include "Controllers.hpp"
#include "../Utils/Utils.hpp"
#include "../integrators/implicitRungeKutta/implicitRungeKutta.hpp"
#include "../integrators/RightPart/RightPartOfDiffEq.hpp"

namespace Control {

    template<typename T, uint Size>
    Vector2s calculateControlU(const Vector<T, Size>& currentState,
                                  const Vector<T, Size>& goalState,
                                  ControlPID<T, Size>& controlPid,
                                  const T timeStep, const T uMax){

        Vector2s u = controlPid.calculate(currentState, goalState, timeStep);
        u(0) = SignalProcessingUtils::bound(u(0), -uMax, uMax);
        u(1) = SignalProcessingUtils::bound(u(1), -uMax, uMax);
        return u;
    }

    template<typename T, uint Size, typename stateTemplate, typename paramsTemplate, typename intParamsTemplate>
    Vector<T, Size> stateUpdate(const stateTemplate& initState,
                                const paramsTemplate& params,
                                const intParamsTemplate& intParams){
        return Integrators::ImplicitRK::ImplicitRungeKutta<Integrators::ImplicitRK::ButcherTableIRK5>::
            calc<Integrators::RightPartFSM>(initState, params, intParams)[1].u;
    }

}

#endif  // SIGNAL_PROCESSING_TEST_CONTROLCALCULATOR_HPP
