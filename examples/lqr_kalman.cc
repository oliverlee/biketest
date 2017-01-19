#include <array>
#include <chrono>
#include <iostream>
#include <random>
#include "bicycle/whipple.h"
#include "lqr.h"
#include "kalman.h"
#include "parameters.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples
    const size_t n = 100;  // length of horizon in samples

    model::BicycleWhipple::state_t x(
            (model::BicycleWhipple::state_t() <<
                0, 5, 5, 0, 0).finished() * constants::as_radians);

    std::array<model::BicycleWhipple::state_t, N> system_state;
    std::array<model::BicycleWhipple::state_t, N> system_state_estimate;

    std::random_device rd; // used only to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> rn0(0,
            parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> rn1(0,
            parameters::defaultvalue::kalman::R(1, 1));

    model::BicycleWhipple bicycle(v0);

    controller::Lqr<model::BicycleWhipple> lqr(bicycle,
            controller::Lqr<model::BicycleWhipple>::state_cost_t::Identity(),
            0.1 * controller::Lqr<model::BicycleWhipple>::input_cost_t::Identity(),
            model::BicycleWhipple::state_t::Zero(), n);
    observer::Kalman<model::BicycleWhipple> kalman(bicycle,
            model::BicycleWhipple::state_t::Zero(), // starts at zero state
            parameters::defaultvalue::kalman::Q(dt),
            parameters::defaultvalue::kalman::R,
            std::pow(x[1]/2, 2) * model::BicycleWhipple::state_matrix_t::Identity());

    std::cout << "initial state:          [" << x.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;

    std::cout << std::endl << "simulating with observer and controller..." << std::endl;

    auto it_x = system_state.begin();
    auto it_xh =  system_state_estimate.begin();
    *it_x++ = x;
    *it_xh++ = kalman.x();

    auto start = std::chrono::system_clock::now();
    for (; it_x != system_state.end(); ++it_x, ++it_xh) {
        // compute control law
        auto u = lqr.control_calculate(kalman.x());

        // system simulate
        x = bicycle.integrate_state(dt, x, u);

        // measure output with noise
        auto y = bicycle.calculate_output(x);
        auto z = y;
        z(0) += rn0(gen);
        z(1) += rn1(gen);

        // observer time/measurement update
        kalman.time_update(u);
        kalman.measurement_update(z);

        //auto error = x - kalman.x();
        //std::cout << error.transpose() * constants::as_degrees << std::endl;
        *it_x = x;
        *it_xh = kalman.x();
    }
    auto stop = std::chrono::system_clock::now();
    auto duration = stop - start;
    std::cout << "duration for simulation: " <<
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() <<
        " ms" << std::endl;
    std::cout << "time per iteration: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(duration/N).count() <<
        " us" << std::endl << std::endl;

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "final state:          [" << system_state.back().transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "final state estimate: [" << system_state_estimate.back().transpose() * constants::as_degrees << "]'" << std::endl;

    return EXIT_SUCCESS;
}
