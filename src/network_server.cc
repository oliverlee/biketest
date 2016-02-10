#include <functional>
#include <iostream>
#include "network_server.h"

namespace network {
namespace udp {

Server::Server(uint16_t port) :
    m_remote_endpoint(asio::ip::udp::v4(), port),
    m_socket(m_io_service, m_remote_endpoint),
    m_service_thread(std::bind(&Server::run_service, this)) {
        std::cout << "Starting UDP server on port " << port << "\n";
}

Server::~Server() {
    m_io_service.stop();
    m_service_thread.join();
}

asio::ip::udp::endpoint Server::remote_endpoint() const {
    return m_remote_endpoint;

}

void Server::start_receive() {
    m_socket.async_receive_from(asio::buffer(m_receive_buffer), m_remote_endpoint,
            std::bind(&Server::handle_receive,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void Server::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::cout << "received " << bytes_transferred << " bytes\n";
    } else {
        std::cerr << error.message() << "\n";
    }
    start_receive();
}

void Server::handle_send(const asio::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::cout << "sent " << bytes_transferred << " bytes\n";
    } else {
        std::cerr << error.message() << "\n";
    }
}

void Server::async_send(uint8_t* buffer, size_t length) {
    // TODO: ensure that buffer data is not changed if transmission queued
    m_socket.async_send_to(asio::buffer(buffer, length), m_remote_endpoint,
            std::bind(&Server::handle_send,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void Server::run_service() {
    start_receive();
    try {
        m_io_service.run();
    } catch (std::exception& e) {
        std::cerr << e.what() << "\n";
    }
}

} // namespace udp
} // namespace network
