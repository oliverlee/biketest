#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include "bicycle/whipple.h"
#include "parameters.h"

#include "flatbuffers/flatbuffers.h"
#include "sample_log_generated.h"
#include "sample_util.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model::BicycleWhipple::state_t, N> system_state;
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::FlatBufferBuilder log_builder;
    flatbuffers::Offset<fbs::SampleBuffer> sample_locations[N];
} // namespace


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::BicycleWhipple bicycle(v0);

    model::BicycleWhipple::state_t x; // yaw angle, roll angle, steer angle, roll rate, steer rate
    x << 0, 0, 10, 10, 0; // define x0 in degrees
    x *= constants::as_radians; // convert to radians
    system_state[0] = x;
    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout << "states are: [yaw angle, roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    // flatbuffer objects must be serialized in depth first pre-order traversal
    size_t current_sample = 0;
    auto bicycle_location = fbs::create_bicycle(builder, bicycle);
    auto fbs_state = fbs::state(x);
    builder.Finish(fbs::CreateSample(builder, current_sample, 0,
                bicycle_location, 0, 0, &fbs_state, 0, 0, 0));

    auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
    sample_locations[current_sample++] = fbs::CreateSampleBuffer(log_builder, data);

    std::cout << "simulating discrete time system at constant speed (" <<
        N << " steps at " << fs << " Hz) ..." << std::endl;
    auto disc_start = std::chrono::system_clock::now();
    for (; current_sample < N; ++current_sample) {
        x = bicycle.integrate_state(dt, x);
        system_state[current_sample] = x;
        system_state[current_sample] = x;

        builder.Clear();
        fbs_state = fbs::state(x);
        builder.Finish(fbs::CreateSample(builder,
                    current_sample, 0, 0, 0, 0, &fbs_state, 0, 0, 0));
        // sample is serialized

        unsigned char* p = nullptr;
        auto data = log_builder.CreateUninitializedVector<unsigned char>(builder.GetSize(), &p);
        std::memcpy(p, builder.GetBufferPointer(), builder.GetSize());

        fbs::SampleBufferBuilder log_sample_builder(log_builder);
        log_sample_builder.add_data(data);
        sample_locations[current_sample] = log_sample_builder.Finish();
    }
    auto disc_stop = std::chrono::system_clock::now();
    auto disc_time = disc_stop - disc_start;

    std::cout << "final state: " << system_state.back().transpose() << std::endl;
    std::cout << "simulation duration: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;

    auto samples_vector = log_builder.CreateVector(sample_locations, N);
    auto log_location = fbs::CreateSampleLog(log_builder, samples_vector);
    log_builder.Finish(log_location, fbs::SampleLogIdentifier());

    {
        disc_start = std::chrono::system_clock::now();
        auto f = std::fopen("samples.bin", "wb");
        std::fwrite(log_builder.GetBufferPointer(), sizeof(char), log_builder.GetSize(), f);
        std::fclose(f);
        disc_stop = std::chrono::system_clock::now();
        disc_time = disc_stop - disc_start;
        std::cout << "write to file took: " <<
            std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
            " us" << std::endl;
    }

    return EXIT_SUCCESS;
}
