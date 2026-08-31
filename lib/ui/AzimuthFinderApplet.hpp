#pragma once

#include "tx/LidarController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class AzimuthFinderApplet {
public:
    struct Host {
        std::string initial_port;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<std::shared_ptr<LidarController>()> controller;
    };

    explicit AzimuthFinderApplet(Host host);
    ~AzimuthFinderApplet();

    AzimuthFinderApplet(const AzimuthFinderApplet&) = delete;
    AzimuthFinderApplet& operator=(const AzimuthFinderApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
