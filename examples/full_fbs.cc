#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>

#include "bicycle/whipple.h"
#include "kalman.h"
#include "lqr.h"
#include "parameters.h"

#include "flatbuffers/flatbuffers.h"
#include "sample_log_generated.h"
#include "sample_util.h"


namespace {
    using bicycle_t = model::BicycleWhipple;
    using kalman_t = observer::Kalman<bicycle_t>;
    using lqr_t = controller::Lqr<bicycle_t>;

    constexpr double fs = 200; // sample rate [Hz]
    constexpr double dt = 1.0/fs; // sample time [s]
    constexpr double v0 = 5.0; // forward speed [m/s]
    constexpr size_t N = 2000; // length of simulation in samples
    constexpr size_t n = 100; // length of horizon in samples

    constexpr double q0 = 0.0; // yaw angle cost weight
    constexpr double q1 = 0.01; // roll angle cost weight
    constexpr double q2 = 0.001; // steer angle cost weight
    constexpr double q3 = q1/10; // roll rate cost weight
    constexpr double q4 = q2/10; // steer rate cost weight

    constexpr double qi0 = 100.0; // error integral yaw angle cost weight
    constexpr double qi1 = 0.0; // error integral roll angle cost weight
    constexpr double qi2 = 0.0; // error integral steer angle cost weight
    constexpr double qi3 = 0.0; // error integral roll rate cost weight
    constexpr double qi4 = 0.0; // error integral steer rate cost weight

    constexpr double rho = 0.01; // input cost weight scaling constant
    constexpr double r0 = 0.0; // roll torque cost weight (this is disabled)
    constexpr double r1 = 1.0; // steer torque cost weight

    const double sigma0 = 1 * constants::as_radians; // yaw angle measurement noise variance
    const double sigma1 = 0.008 * constants::as_radians; // steer angle measurement noise variance

    bicycle_t::state_t x; // yaw angle, roll angle, steer angle, roll rate, steer rate
    bicycle_t::auxiliary_state_t aux; // x, y, rear wheel angle, pitch angle

    /* used for serializing/logging */
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::FlatBufferBuilder log_builder;
    flatbuffers::Offset<fbs::SampleBuffer> sample_locations[N];

    /* reference trajectory (yaw angle) */
    auto reference = [](double t) {
        const double f = 0.2; // 0.2 Hz sine wave (5 second period)
        const bicycle_t::state_t r((bicycle_t::state_t() <<
            5 * std::sin(constants::two_pi*f*t),
            0,
            0,
            0,
            0).finished() * constants::as_radians);
        return r;
    };

    /* used for noise generation */
    std::random_device rd; // used only to seed rng
} // namespace


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0, sigma0);
    std::normal_distribution<> rn1(0, sigma1);

    bicycle_t bicycle(v0);
    x << 0, 3, 5, 0, 0; // define x0 in degrees
    x *= constants::as_radians; // convert to radians
    aux.setZero();
    aux[static_cast<uint8_t>(bicycle_t::auxiliary_state_index_t::pitch_angle)]
        = bicycle.solve_constraint_pitch(
                x[static_cast<uint8_t>(bicycle_t::state_index_t::roll_angle)],
                x[static_cast<uint8_t>(bicycle_t::state_index_t::steer_angle)],
                0);

    kalman_t kalman(bicycle,
            bicycle_t::state_t::Zero(), // starts at zero state
            parameters::defaultvalue::kalman::Q(dt),
            (kalman_t::measurement_noise_covariance_t() <<
             sigma0,      0,
                  0, sigma1).finished(),
            std::pow(x[1]/2, 2) * bicycle_t::state_matrix_t::Identity());

    lqr_t lqr(bicycle,
            (bicycle_t::state_t() <<
             q0, q1, q2, q3, q4).finished().asDiagonal() * constants::as_radians,
            (bicycle_t::input_t() <<
             r0, r1).finished().asDiagonal() * rho,
            bicycle_t::state_t::Zero(),
            n,
            (bicycle_t::state_t() <<
             qi0, qi1, qi2, qi3, qi4).finished().asDiagonal() * constants::as_radians);

    std::cout << "initial state: [" << x.transpose() << "]' rad" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "states are: [yaw, roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    std::cout << "simulating discrete time system at constant speed (" <<
        N << " steps at " << fs << " Hz)" << std::endl;
    std::cout << "with kalman filter and lqr controller..." << std::endl;

    /* flatbuffer objects must be serialized in depth first pre-order traversal
     * bicycle, kalman, and lqr objects must be serialized first
     */

    size_t current_sample = 0;
    auto bicycle_location = fbs::create_bicycle(builder, bicycle);
    auto kalman_location = fbs::create_kalman(builder, kalman);
    auto lqr_location = fbs::create_lqr(builder, lqr);
    auto fbs_state = fbs::state(x);
    auto fbs_input = fbs::input(bicycle_t::input_t::Zero());
    auto fbs_measurement = fbs::output(bicycle_t::output_t::Zero());
    auto fbs_auxiliary_state = fbs::auxiliary_state(aux);
    builder.Finish(fbs::CreateSample(builder, current_sample, 0,
                bicycle_location, kalman_location, lqr_location,
                &fbs_state, 0, 0, &fbs_measurement, &fbs_auxiliary_state));

    auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
    sample_locations[current_sample++] = fbs::CreateSampleBuffer(log_builder, data);

    auto disc_start = std::chrono::system_clock::now();
    for (; current_sample < N; ++current_sample) {
        auto comp_start = std::chrono::high_resolution_clock::now();
        /* compute control law */
        auto u = lqr.control_calculate(kalman.x(), reference(dt*current_sample));

        /* system simulate */
        x = bicycle.integrate_state(dt, x, u);

        /* update auxiliary states */
        aux = bicycle.integrate_auxiliary_state(x, aux, bicycle.dt());

        /* measure output with noise */
        auto z = bicycle.calculate_output(x);
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        /* observer time/measurement update */
        kalman.time_update(u);
        kalman.measurement_update(z);

        builder.Clear();

        kalman_location = fbs::create_kalman(builder, kalman,
                true, true, false, false, true); // x, P, Q, R, K
        lqr_location = fbs::create_lqr(builder, lqr,
                false, true, true, false, false, true, // n, r, P, Q, R, K
                false, true); // Qi, q

        /* skip output as we can get it from state */
        fbs_state = fbs::state(x);
        fbs_input = fbs::input(u);
        fbs_measurement = fbs::output(z);
        fbs_auxiliary_state = fbs::auxiliary_state(aux);

        auto comp_stop = std::chrono::high_resolution_clock::now();
        auto comp_time = std::chrono::duration<double>(comp_stop - comp_start);
        builder.Finish(fbs::CreateSample(builder, current_sample, comp_time.count(),
                    0, kalman_location, lqr_location,
                    &fbs_state, &fbs_input, 0, &fbs_measurement,
                    &fbs_auxiliary_state));
        /* sample is serialized */

        //uint8_t* p = nullptr;
        //auto data = log_builder.CreateUninitializedVector<uint8_t>(builder.GetSize(), &p);
        //std::memcpy(p, builder.GetBufferPointer(), builder.GetSize());
        // Using CreateUninitializedVector occasionally causes the sample to be
        // written incorrectly.
        auto data = log_builder.CreateVector(builder.GetBufferPointer(), builder.GetSize());
        sample_locations[current_sample] = fbs::CreateSampleBuffer(log_builder, data);
    }

    auto disc_stop = std::chrono::system_clock::now();
    auto disc_time = disc_stop - disc_start;

    std::cout << "final state: " << x.transpose() << std::endl;
    std::cout << "final state estimate: " << kalman.x().transpose() << std::endl;
    std::cout << "simulation duration: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;

    auto samples_vector = log_builder.CreateVector(sample_locations, N);
    auto log_location = fbs::CreateSampleLog(log_builder, samples_vector);
    log_builder.Finish(log_location, fbs::SampleLogIdentifier());

    {
        disc_start = std::chrono::system_clock::now();
        auto f = std::fopen("samples_full.bin", "wb");
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
