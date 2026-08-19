#pragma once

#include "tx/LidarController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class TransceiverApplet {
public:
    struct Host {
        std::string initial_port;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<void(const std::function<void(LidarController*)>&)> with_controller;
        std::function<uint64_t()> control_count;
        std::function<uint64_t()> status_count;
    };

    explicit TransceiverApplet(Host host);
    ~TransceiverApplet();

    TransceiverApplet(const TransceiverApplet&) = delete;
    TransceiverApplet& operator=(const TransceiverApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
