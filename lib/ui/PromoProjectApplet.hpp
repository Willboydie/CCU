#pragma once

#include "tx/ProjectorController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Bench host for promo projection. Nodes come from a file. The projector
// Teensy is the only device on the link.
class PromoProjectApplet {
public:
    struct Host {
        std::string initial_port;
        std::string initial_nodes;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<void(const std::function<void(ProjectorController*)>&)> with_controller;
        std::function<bool(std::string& path)> browse_nodes;
    };

    explicit PromoProjectApplet(Host host);
    ~PromoProjectApplet();

    PromoProjectApplet(const PromoProjectApplet&) = delete;
    PromoProjectApplet& operator=(const PromoProjectApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
