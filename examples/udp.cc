#include <array>
#include <iostream>
#include <thread>
#include "bicycle/whipple.h"
#include "parameters.h"
#include "network_server.h"

namespace {
    const double fs = 200; // sample rate [Hz]
    const double dt = 1.0/fs; // sample time [s]
    const double v0 = 4.0; // forward speed [m/s]
    const size_t N = 1000; // length of simulation in samples
    const auto simulation_period = std::chrono::microseconds(100);
    const auto transmission_period = std::chrono::milliseconds(1);

    std::array<model::BicycleWhipple::state_t, N> discrete_time_system_state_n;
    std::array<int32_t, N> simulation_loop_period;
    std::array<int32_t, N> transmit_loop_period;

    void update(const asio::error_code& error, asio::high_resolution_timer* timer,
            size_t* count, model::BicycleWhipple* bicycle, model::BicycleWhipple::state_t* x,
            std::chrono::high_resolution_clock::time_point time) {
        if (!error) {
            if (*count < N) {
                *x = bicycle->integrate_state(dt, *x);

                auto now = std::chrono::high_resolution_clock::now();
                auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - time).count();

                simulation_loop_period[*count] = dt; // store calculated loop period
                discrete_time_system_state_n[(*count)++] = *x; // store system state

                timer->expires_at(timer->expires_at() + simulation_period);
                timer->async_wait(std::bind(update,
                            std::placeholders::_1,
                            timer,
                            count,
                            bicycle,
                            x,
                            now));

            }
        } else {
            std::cerr << error.message() << "\n";
        }
    }
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::BicycleWhipple bicycle(v0);
    model::BicycleWhipple::state_t x;
    x << 0, 0, 10, 10, 0; // define in degrees
    x *= constants::as_radians;

    using clock = std::chrono::high_resolution_clock;
    auto start = clock::now();
    auto i = transmit_loop_period.begin();

    // periodic transmit buffer function
    auto f = [&x, &i, &start]() -> asio::const_buffer {
        auto now = clock::now();
        *i++ = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
        start = now;
        return asio::const_buffer(static_cast<void*>(x.data()), x.size()*sizeof(double));
    };

    network::udp::PeriodicTransmitServer<decltype(f)> server(
            network::udp::default_server_port,
            network::udp::default_remote_port,
            transmission_period,
            std::cref(f));

    asio::io_service io_service;
    asio::high_resolution_timer simulation_timer(io_service, simulation_period);
    size_t count = 0;

    simulation_timer.async_wait(std::bind(update,
                std::placeholders::_1,
                &simulation_timer,
                &count,
                &bicycle,
                &x,
                start));

    io_service.run();

    std::cout << "simulation loop periods (us): ";
    for (auto dt: simulation_loop_period) {
        std::cout << dt << " ";
    }

    std::cout << "\n";
    std::cout << "transmission loop periods (us): ";
    for (auto dt: transmit_loop_period) {
        if (dt == 0) {
            break;
        }
        std::cout << dt << " ";
    }
    std::cout << "\n";
}
