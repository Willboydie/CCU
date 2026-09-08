#pragma once

#include "tx/ProjectorController.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class ProjectorEspApplet {
public:
    struct Host {
        std::string initial_port;
        std::string initial_esp_port;
        int initial_esp_baud = 921600;
        std::function<std::vector<std::string>()> list_ports;
        std::function<bool(const std::string& port, std::string& err)> connect;
        std::function<void()> disconnect;
        std::function<bool()> connected;
        std::function<std::string()> port_name;
        std::function<void(const std::function<void(ProjectorController*)>&)> with_controller;
        std::function<bool(const std::string& port, int baud, std::string& err)> connect_esp;
        std::function<void()> disconnect_esp;
        std::function<bool()> esp_connected;
        std::function<std::string()> esp_port_name;
        std::function<std::string()> take_esp_text;
        std::function<bool(const std::string& text)> send_esp;
    };

    explicit ProjectorEspApplet(Host host);
    ~ProjectorEspApplet();

    ProjectorEspApplet(const ProjectorEspApplet&) = delete;
    ProjectorEspApplet& operator=(const ProjectorEspApplet&) = delete;

    int run(std::atomic<bool>* keep_running = nullptr);

private:
    struct Impl;
    Host host_;
    std::unique_ptr<Impl> impl_;
};
