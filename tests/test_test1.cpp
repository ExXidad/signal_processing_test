//
// Created by Ivan Kalesnikau on 16.07.2023.
//

#include "signal_processing_test/Utils/Types.hpp"
#include "third_party/MCU_Libs/SignalProcessors/GoertzelFrequencyDetectionSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/ParallelSignalProcessing.hpp"
#include "third_party/MCU_Libs/SignalProcessors/QPD_PositionSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/SignalProcessingPipeline.hpp"
#include "third_party/MCU_Libs/SignalProcessors/TestProcessors/DoubleSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/TestProcessors/FromVecSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/TestProcessors/IncrementSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/TestProcessors/ToVecSignalProcessor.hpp"
#include "third_party/MCU_Libs/SignalProcessors/UnitySignalProcessor.hpp"
#include <MathUtils.hpp>
#include <cmath>
#include <etl/array.h>
#include <gtest/gtest.h>
#include <iostream>

//TEST(TEST1, TEST1)
//{
//    using IncrementSignalProcessorID0 = IncrementSignalProcessor<scalar, 0>;
//    using IncrementSignalProcessorID1 = IncrementSignalProcessor<scalar, 1>;
//    using DoubleSignalProcessorID0 = DoubleSignalProcessor<scalar, 0>;
//    using ToVecSignalProcessorID0 = ToVecSignalProcessor<0>;
//    using FromVecSignalProcessorID0 = FromVecSignalProcessor<0>;
//    using UnitySignalProcessorID0 = UnitySignalProcessor<Vector2s, 0>;
//
//    using MySignalProcessingPipeline = SignalProcessingPipeline<ToVecSignalProcessorID0, UnitySignalProcessorID0, FromVecSignalProcessorID0>;
//    auto MySignalProcessingPipelineParameters = MySignalProcessingPipeline::Parameters{};
//    std::cout << MySignalProcessingPipeline::process(1., MySignalProcessingPipelineParameters) << std::endl;
//}
//
//TEST(TEST1, TEST2)
//{
//    //region Create test data
//    constexpr static sizeType bufferSize = 1024;
//    constexpr static sizeType numberOfChannels = 4;
//
//    scalar samplingRate = 100000.;
//    scalar modulationFrequency = 10000.;
//    etl::array<etl::array<scalar, bufferSize>, numberOfChannels> buffers;
//    for (indexType j = 0; j < numberOfChannels; ++j) {
//        for (indexType i = 0; i < bufferSize; ++i) {
//            buffers[j][i] = std::sin(2 * M_PI * modulationFrequency / samplingRate * i);
//        }
//    }
//
//    etl::array<scalar *, numberOfChannels> data;
//    for (indexType i = 0; i < numberOfChannels; ++i) {
//        data[i] = buffers[i].data();
//    }
//    //endregion
//
//    //region Parallel signal processing section initialization
//    using GoertzelFrequencyDetectionSignalProcessor_ = GoertzelFrequencyDetectionSignalProcessor<scalar, bufferSize, 0>;
//    GoertzelFrequencyDetectionSignalProcessor_::Parameters GoertzelFrequencyDetectionSignalProcessorParameters_{modulationFrequency, samplingRate};
//
//    using UnitySignalProcessor_ = UnitySignalProcessor<scalar>;
//    UnitySignalProcessor_::Parameters UnitySignalProcessorParameters_{};
//
//    using SignalProcessingPipelineForParallelProcessing_ = SignalProcessingPipeline<GoertzelFrequencyDetectionSignalProcessor_, UnitySignalProcessor_>;
//    SignalProcessingPipelineForParallelProcessing_::Parameters SignalProcessingPipelineForParallelProcessingParameters_{GoertzelFrequencyDetectionSignalProcessorParameters_, UnitySignalProcessorParameters_};
//
//    using ParallelSignalProcessing_ = ParallelSignalProcessing<SignalProcessingPipelineForParallelProcessing_, numberOfChannels>;
//    ParallelSignalProcessing_::Parameters ParallelSignalProcessingParameters_{};
//    for (indexType i = 0; i < numberOfChannels; ++i) {
//        ParallelSignalProcessingParameters_[i] = SignalProcessingPipelineForParallelProcessingParameters_;
//    }
//    //endregion
//
//    //region Serial signal processing initialization
//    using QPD_PositionSignalProcessor_ = QPD_PositionSignalProcessor<>;
//    QPD_PositionSignalProcessor_::Parameters QPD_PositionSignalProcessorParameters_{};
//    //endregion
//
//    //region Collecting everything to the single signal processing pipeline
//    using TotalSignalProcessingPipeline_ = SignalProcessingPipeline<ParallelSignalProcessing_, QPD_PositionSignalProcessor_>;
//    TotalSignalProcessingPipeline_::Parameters TotalSignalProcessingPipelineParameters_{ParallelSignalProcessingParameters_, QPD_PositionSignalProcessorParameters_};
//    //endregion
//
//    std::cout << TotalSignalProcessingPipeline_::process(data, TotalSignalProcessingPipelineParameters_)
//              << std::endl;
//}

TEST(TEST1, TEST3)
{
    constexpr sizeType N = 4096;
    constexpr scalar samplingRate = 200000;
    constexpr scalar modulationFrequency = 10000;
    constexpr scalar amp = 100;
    constexpr scalar offset = 200;

    etl::array<scalar, N> data{};
    for (indexType i = 0; i < N; ++i) {
        //data[i] = amp * std::sin(2 * M_PI * modulationFrequency * i / samplingRate) + offset;
        data[i] = 4095;
    }

    //    for (indexType i = 0; i < N; ++i) {
    //        std::cout << data[i] << " ";
    //    }
    //    std::cout << std::endl;

    std::cout << GoertzelAlgorithm(N, modulationFrequency, samplingRate, data.data());
}

TEST(TEST1, TEST4)
{
    constexpr sizeType N = 16;
    scalar buffer[N];
    for (indexType i = 0; i < N; ++i) {
        buffer[i] = i;
    }
    for (indexType i = 0; i < N / 2; ++i) {
        scalar re = buffer[2 * i];
        scalar im = buffer[2 * i + 1];
        buffer[i] = sqrtf(re * re + im * im);
    }
    for (indexType i = 0; i < N; ++i) {
        std::cout << buffer[i] << "\t";
    }
}

#include <IrregularMeshLinearInterpolation.hpp>
#include <algorithm>

TEST(TEST1, TEST5)
{
    etl::array<scalar, 5> arr{10, 11, 12, 13, 14};
    std::cout << std::distance(arr.begin(), std::upper_bound(arr.begin(), arr.end(), 100500)) << std::endl;
}

TEST(TEST1, TEST6)
{
    constexpr sizeType size = 5;
    etl::array<scalar, size> dataX{0, 1, 2, 3, 4};
    etl::array<scalar, size> dataY{0, 1, 4, 9, 16};
    IrregularMeshLinearInterpolation<scalar, scalar, size> interpolation(dataX, dataY);
    std::cout << interpolation.interpolate(1600) << std::endl;

    //    std::cout << std::distance(arr.begin(), std::upper_bound(arr.begin(), arr.end(), 100500)) << std::endl;
}

#include <ByteMergeArraySignalProcessor.hpp>
#include <ByteMergeSignalProcessor.hpp>
#include <ByteSplitArraySignalProcessor.hpp>
#include <ByteSplitSignalProcessor.hpp>

TEST(TEST1, TEST7)
{
    constexpr sizeType size = 5;
    array<uint32_t, size> floats{1, 2, 3, 4, 5};

    auto bytes = ByteSplitArraySignalProcessor<uint32_t, size>::process(floats, {});
    for (auto byte : bytes) {
        printf("%x ", byte);
    }
    std::cout << std::endl;
    auto revFloats = ByteMergeArraySignalProcessor<uint32_t, size>::process(bytes, {});

    for (indexType i = 0; i < size; ++i) {
        std::cout << revFloats[i] << std::endl;
    }
}

#include <ControlAlgorithms.hpp>
#include <PID_Controller.hpp>

TEST(TEST1, TEST8)
{
    constexpr uint32_t size = 5;
    Eigen::Vector<scalar, size> vec{1, 2, 3, 4, 5};
    for (const scalar el : vec) {
        std::cout << el;
    }
}