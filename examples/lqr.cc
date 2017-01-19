#include <array>
#include <chrono>
#include <iostream>
#include "bicycle/whipple.h"
#include "lqr.h"
#include "parameters.h"

namespace {
    using bicycle_t = model::BicycleWhipple;
    using lqr_t = controller::Lqr<bicycle_t>;

    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.1; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples
    const size_t n = 100;  // length of horizon in samples

    const bicycle_t::state_t x0(
            (bicycle_t::state_t() <<
                0, 0, 10, 10, 0).finished() * constants::as_radians);
    bicycle_t::state_t x;

    //constexpr double r0 = 10; // roll torque cost weight
    constexpr double r0 = 0.0; // roll torque cost weight
    constexpr double r1 = 1.0; // steer torque cost weight

    std::array<bicycle_t::state_t, N> system_state;
    std::array<bicycle_t::input_t, N> system_control_input;

    std::chrono::time_point<std::chrono::system_clock> start, stop;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    bicycle_t bicycle(v0);
    lqr_t lqr(bicycle,
            lqr_t::state_cost_t::Identity(),
            (lqr_t::input_cost_t() <<
             r0,  0,
              0, r1).finished(),
            bicycle_t::state_t::Zero(), n);

    x = x0;
    std::cout << "initial state: [" << x.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << std::endl << "simulating without controller..." << std::endl;
    auto it_x = system_state.begin();
    start = std::chrono::system_clock::now();
    for (; it_x != system_state.end(); ++it_x) {
        x = bicycle.integrate_state(dt, x);
        *it_x = x;
    }
    stop = std::chrono::system_clock::now();
    auto duration = stop - start;
    std::cout << "duration for simulation without controller: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count() <<
        " us" << std::endl;

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state: [" << system_state.back().transpose() * constants::as_degrees << "]' deg" << std::endl;

    x = x0;
    std::cout << std::endl << "simulating with controller..." << std::endl;
    it_x = system_state.begin();
    auto it_u = system_control_input.begin();
    start = std::chrono::system_clock::now();
    for (; it_x != system_state.end(); ++it_x, ++it_u) {
        *it_u = lqr.control_calculate(x);
        x = bicycle.integrate_state(dt, x, *it_u);
        *it_x = x;
    }
    stop = std::chrono::system_clock::now();
    duration = stop - start;
    std::cout << "duration for simulation with controller: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count() <<
        " us" << std::endl;

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state: [" << system_state.back().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "final control input: [" << system_control_input.back().transpose() << "]'" << std::endl;

    return EXIT_SUCCESS;
}
