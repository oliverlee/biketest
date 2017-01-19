#include <array>
#include <chrono>
#include <iostream>
#include <asio.hpp>
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
} // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    model::BicycleWhipple bicycle(v0);
    model::BicycleWhipple::state_t x;
    x << 0, 0, 10, 10, 0; // define in degrees
    x *= constants::as_radians;

    network::udp::Server server;

    auto start = std::chrono::high_resolution_clock::now();

    for (auto& state: discrete_time_system_state_n) {
        state = bicycle.integrate_state(dt, x);
        x = state;
        server.wait_for_send_complete(); // wait for previous message to be sent
        server.async_send(asio::buffer(reinterpret_cast<uint8_t*>(x.data()),
                    x.size()*sizeof(double)));
    }
    server.wait_for_send_complete();
    server.wait_for_receive_complete();

    std::cout << "simulation of 1000 iterations completed in "
        << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start).count()
        << "ms\n";
}
