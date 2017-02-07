#include <array>
#include <iostream>
#include <random>
#include "bicycle/whipple.h"
#include "luenberger.h"
#include "parameters.h"

namespace {
    constexpr double fs = 200; // sample rate [Hz]
    constexpr double dt = 1.0/fs; // sample time [s]
    constexpr double v0 = 4.0; // forward speed [m/s]
    constexpr size_t N = 1000; // length of simulation in samples
    constexpr double observer_bandwidth = 10; // Hz

    using bicycle_t = model::BicycleWhipple;
    bicycle_t::state_t x;

    std::array<bicycle_t::state_t, N> system_state;
    std::array<bicycle_t::state_t, N> system_state_estimate;
    std::array<bicycle_t::output_t, N> system_output;
    std::array<bicycle_t::output_t, N> system_measurement;

    std::random_device rd; // used only to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    // kalman default values are used for noise generation
    std::normal_distribution<> r0(0, parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> r1(0, parameters::defaultvalue::kalman::R(1, 1));

    bicycle_t bicycle(v0, dt);
    x << 0, 0, 10, 10, 0; // define x in degrees
    x *= constants::as_radians; // convert degrees to radians

    observer::Luenberger<bicycle_t> luenberger(bicycle,
            bicycle_t::state_t::Zero(),
            observer_bandwidth);

    std::cout << "simulating bicycle model with measurement noise (equal to R)" << std::endl;
    std::cout << "initial state:          [" << x.transpose() << "] deg" << std::endl;
    std::cout << "initial state estimate: [" << luenberger.state().transpose() << "] deg" << std::endl;
    std::cout << "observer desired bandhwidth: " << luenberger.bandwidth() << std::endl;

    auto it_x = system_state.begin();
    auto it_y = system_output.begin();
    auto it_z = system_measurement.begin();
    auto it_xh = system_state_estimate.begin();

    *it_x++ = x;
    *it_y++ = bicycle.calculate_output(x);
    *it_z++ = bicycle.calculate_output(x); // first measurement isn't used
    *it_xh++ = luenberger.state();

    std::cout << std::endl << "simulating..." << std::endl;
    for (; it_x != system_state.end(); ++it_x, ++it_y, ++it_z, ++it_xh) {
        // simulate bicycle system
        x = bicycle.update_state(x);
        *it_x = x;
        *it_y = bicycle.calculate_output(x);

        // add measurement noise
        *it_z = *it_y;
        (*it_z)(0) += r0(gen);
        (*it_z)(1) += r1(gen);

        // update observer
        luenberger.update_state(bicycle_t::input_t::Zero(), *it_z);
        *it_xh = luenberger.state();
    }

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "true:      [" << x.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "estimated: [" << luenberger.state().transpose() * constants::as_degrees << "]' deg" << std::endl;

    return EXIT_SUCCESS;
}
