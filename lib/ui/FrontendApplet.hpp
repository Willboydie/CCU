#pragma once

#include "tx/FrontendController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class FrontendApplet {
public:
    struct Host {
        std::string initial_port;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<void(const std::function<void(FrontendController*)>&)> with_controller;
    };

    explicit FrontendApplet(Host host);
    ~FrontendApplet();

    FrontendApplet(const FrontendApplet&) = delete;
    FrontendApplet& operator=(const FrontendApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
