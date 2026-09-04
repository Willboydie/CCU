#pragma once

#include "tx/ProjectorController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class ProjectorCalibrationApplet {
public:
    struct Host {
        std::string initial_port;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<void(const std::function<void(ProjectorController*)>&)> with_controller;
    };

    explicit ProjectorCalibrationApplet(Host host);
    ~ProjectorCalibrationApplet();

    ProjectorCalibrationApplet(const ProjectorCalibrationApplet&) = delete;
    ProjectorCalibrationApplet& operator=(const ProjectorCalibrationApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
