//
// Created by Арсений Плахотнюк on 09.03.2023.
//

#ifndef BALLISTICS_EXPLICITRUNGEKUTTA_HPP
#define BALLISTICS_EXPLICITRUNGEKUTTA_HPP

#include "Laser-P/Utils/Utils.hpp"

namespace Integrators::ExplicitRK {

    template<typename RightPart>
    struct IntegrationParameters {
        const scalar step;
        const scalar tFinal;
    };

    template<typename ButcherTable>
    class ExplicitRungeKutta {
    public:
        template<typename RightPart>
        [[nodiscard]] static inline std::vector<typename RightPart::State> calc(const typename RightPart::State& initState,
                                                                                typename RightPart::Params& params,
                                                                                const IntegrationParameters<RightPart>& intParams) {
            const uint numOfIter = static_cast<uint>(std::abs(initState.t - intParams.tFinal) / intParams.step);
            Vector<scalar, RightPart::size> uTmp = initState.u;
            double tTmp = initState.t;
            std::vector<typename RightPart::State> solution = {initState};

            for (integer j = 0; j < numOfIter; ++j) {
                // Calculating k[i]
                std::vector<Vector<scalar, RightPart::size>> kMas = {RightPart::calc({uTmp, tTmp}, params)};
                uint n = 0;
                for (integer i = 1; i < ButcherTable::size; ++i) {
                    Vector<scalar, RightPart::size> sum = Vector<scalar, RightPart::size>::Zero();
                    for (integer k = 0; k < i; ++k)
                    {
                        sum += ButcherTable::matrix[n + k] * kMas[k];
                    }
                    n += i;

                    kMas.push_back(RightPart::calc({uTmp + sum * intParams.step, tTmp + ButcherTable::column[i] * intParams.step},
                                                   params));
                }
                // Calculating sum k[i] * b[i];
                Vector<scalar, RightPart::size> sum = Vector<scalar, RightPart::size>::Zero();
                for (integer i = 0; i < ButcherTable::size; ++i) {
                    sum += kMas[i] * ButcherTable::row[i];
                }

                uTmp = uTmp + intParams.step * sum;
                tTmp = tTmp + intParams.step;

                solution.push_back({uTmp, tTmp});
            }

            return solution;
        }
    };
}



#endif //BALLISTICS_EXPLICITRUNGEKUTTA_HPP
