#include <array>
#include <chrono>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include "bicycle/whipple.h"
#include "parameters.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    std::array<model::BicycleWhipple::state_t, N> continuous_time_system_state_n;
    std::array<model::BicycleWhipple::state_t, N> continuous_time_system_state_0;
    std::array<model::BicycleWhipple::state_t, N> discrete_time_system_state_n;
    std::array<model::BicycleWhipple::state_t, N> discrete_time_system_state_0;
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::BicycleWhipple bicycle(v0);

    std::chrono::time_point<std::chrono::system_clock> cont_start, cont_stop;
    std::chrono::time_point<std::chrono::system_clock> disc_start, disc_stop;

    cont_start = std::chrono::system_clock::now();
    bicycle.set_v_dt(v0, 0);
    cont_stop = std::chrono::system_clock::now();

    disc_start = std::chrono::system_clock::now();
    bicycle.set_v_dt(v0, dt);
    disc_stop = std::chrono::system_clock::now();

    std::chrono::duration<double> cont_time = cont_stop - cont_start;
    std::chrono::duration<double> disc_time = disc_stop - disc_start;
    disc_time -= cont_time;
    std::cout << "time for continuous state space computation: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(cont_time).count() <<
        " us" << std::endl;
    std::cout << "(additional) time for discrete state space computation: " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;

    std::cout << "M: " << std::endl << bicycle.M() << std::endl;
    std::cout << "C1: " << std::endl << bicycle.C1() << std::endl;
    std::cout << "K0: " << std::endl << bicycle.K0() << std::endl;
    std::cout << "K2: " << std::endl << bicycle.K2() << std::endl << std::endl;

    std::cout << "for v = " << bicycle.v() << " m/s" << std::endl;
    std::cout << "A: " << std::endl << bicycle.A() << std::endl;
    std::cout << "B: " << std::endl << bicycle.B() << std::endl << std::endl;

    std::cout << "for fs = "  << fs << "  Hz" << std::endl;
    std::cout << "Ad: " << std::endl << bicycle.Ad() << std::endl;
    std::cout << "Bd: " << std::endl << bicycle.Bd() << std::endl << std::endl;

    model::BicycleWhipple::state_t x, x0;
    x0 << 0, 0, 10, 10, 0; // define in degrees
    x0 *= constants::as_radians;

    std::cout << "initial state: [" << x0.transpose() << "]' rad" << std::endl;
    std::cout << "states are: [yaw angle, roll angle, steer angle, roll rate, steer rate]'" << std::endl << std::endl;

    std::cout << "simulating (no input) continuous time system at constant speed..." << std::endl;
    x = x0;
    for (auto& state: continuous_time_system_state_n) {
        state = bicycle.integrate_state(x, model::BicycleWhipple::input_t::Zero(), dt);
        x = state;
    }

    std::cout << "simulating (zero input) continuous time system at constant speed..." << std::endl;
    cont_start = std::chrono::system_clock::now();
    x = x0;
    for (auto& state: continuous_time_system_state_0) {
        state = bicycle.integrate_state(x, model::BicycleWhipple::input_t::Zero(), dt);
        x = state;
    }
    cont_stop = std::chrono::system_clock::now();

    std::cout << "simulating (no input) discrete time system at constant speed..." << std::endl;
    x = x0;
    for (auto& state: discrete_time_system_state_n) {
        state = bicycle.integrate_state(dt, x);
        x = state;
    }

    std::cout << "simulating (zero input) discrete time system at constant speed..." << std::endl;
    disc_start = std::chrono::system_clock::now();
    x = x0;
    for (auto& state: discrete_time_system_state_0) {
        state = bicycle.integrate_state(dt, x, model::BicycleWhipple::input_t::Zero());
        x = state;
    }
    disc_stop = std::chrono::system_clock::now();

    std::cout << std::endl;
    std::cout << "state at end of simulation (" << N << " steps)" << std::endl;
    std::cout << "continuous time (no input):   " << continuous_time_system_state_n.back().transpose() << std::endl;
    std::cout << "continuous time (zero input): " << continuous_time_system_state_0.back().transpose() << std::endl;
    std::cout << "discrete time (no input):     " << discrete_time_system_state_n.back().transpose() << std::endl;
    std::cout << "discrete time (zero input):   " << discrete_time_system_state_0.back().transpose() << std::endl;
    std::cout << std::endl;

    cont_time = cont_stop - cont_start;
    disc_time = disc_stop - disc_start;
    std::cout << "simulation time (zero input form)" << std::endl;
    std::cout << "continuous: Tc = " <<
        std::chrono::duration_cast<std::chrono::microseconds>(cont_time).count() <<
        " us" << std::endl;
    std::cout << "discrete: Td = " <<
        std::chrono::duration_cast<std::chrono::microseconds>(disc_time).count() <<
        " us" << std::endl;
    std::cout << "Tc - Td = " <<
        std::chrono::duration_cast<std::chrono::microseconds>(cont_time - disc_time).count() <<
        " us" << std::endl;

    return EXIT_SUCCESS;
}
