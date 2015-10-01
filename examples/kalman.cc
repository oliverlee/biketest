#include <array>
#include <iostream>
#include <random>
#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples

    model::Bicycle::state_t x;

    std::array<model::Bicycle::state_t, N> system_state;
    std::array<model::Bicycle::state_t, N> system_state_estimate;
    std::array<model::Bicycle::output_t, N> system_output;
    std::array<model::Bicycle::output_t, N> system_measurement;

    std::random_device rd; // used only to seed rng
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 gen(rd());
    std::normal_distribution<> r0(0, parameters::defaultvalue::kalman::R(0, 0));
    std::normal_distribution<> r1(0, parameters::defaultvalue::kalman::R(1, 1));

    model::Bicycle bicycle(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v0, dt);
    bicycle.set_C(parameters::defaultvalue::bicycle::C);
    x << 0, 0, 10, 10, 0; // define x in degrees
    x *= constants::as_radians; // convert degrees to radians

    observer::Kalman<model::Bicycle> kalman(bicycle,
            parameters::defaultvalue::kalman::Q(dt),
            parameters::defaultvalue::kalman::R,
            model::Bicycle::state_t::Zero(),
            std::pow(x[0]/2, 2) * model::Bicycle::state_matrix_t::Identity());

    std::cout << "simulating bicycle model with measurement noise (equal to R)" << std::endl;
    std::cout << "initial state:          [" << x.transpose() << "] deg" << std::endl;
    std::cout << "initial state estimate: [" << kalman.x().transpose() << "] deg" << std::endl;
    std::cout << "initial error covariance" << std::endl << kalman.P() << std::endl;
    std::cout << "process noise covariance" << std::endl << kalman.Q() << std::endl;
    std::cout << "measurement noise covariance" << std::endl << kalman.R() << std::endl;

    auto it_x = system_state.begin();
    auto it_y = system_output.begin();
    auto it_z = system_measurement.begin();
    auto it_xh = system_state_estimate.begin();

    *it_x++ = x;
    *it_y++ = bicycle.y(x);
    *it_z++ = bicycle.y(x); // first measurement isn't used
    *it_xh++ = kalman.x();

    std::cout << std::endl << "simulating..." << std::endl;
    for (; it_x != system_state.end(); ++it_x, ++it_y, ++it_z, ++it_xh) {
        // simulate bicycle system
        x = bicycle.x_next(x);
        *it_x = x;
        *it_y = bicycle.y(x);

        // add measurement noise
        *it_z = *it_y;
        (*it_z)(0) += r0(gen);
        (*it_z)(1) += r1(gen);

        // update observer
        kalman.time_update();
        kalman.measurement_update(*it_z);
        *it_xh = kalman.x();

//        std::cout << it_x->transpose() << std::endl;
//        std::cout << it_y->transpose() << std::endl;
//        std::cout << it_z->transpose() << std::endl;
//        std::cout << it_xh->transpose() << std::endl;
//        std::cout << std::endl;
    }

    std::cout << "state at end of simulation (" << N << " steps @ " << fs << " Hz)" << std::endl;
    std::cout << "true:      [" << x.transpose() * constants::as_degrees << "]' deg" << std::endl;
    std::cout << "estimated: [" << kalman.x().transpose() * constants::as_degrees << "]' deg" << std::endl;

    return EXIT_SUCCESS;
}
