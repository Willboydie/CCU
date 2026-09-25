#include "angle.hpp"
#include "device_config.hpp"
#include "dongle.hpp"
#include "lidar.hpp"
#include "projector.hpp"
#include "session.hpp"

#include <cctype>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

volatile std::sig_atomic_t quit = 0;

void on_signal(int) { quit = 1; }

std::string trim(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

struct Ports {
    std::string angle;
    std::string dongle;
    std::string projector;
    std::string lidar;
};

bool load_ports(const char* path, Ports& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot read %s\n", path);
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        if (key == "angle") out.angle = value;
        else if (key == "dongle") out.dongle = value;
        else if (key == "projector") out.projector = value;
        else if (key == "lidar") out.lidar = value;
    }
    if (out.angle.empty() || out.dongle.empty() || out.projector.empty() || out.lidar.empty()) {
        std::fprintf(stderr, "%s needs angle, dongle, projector, and lidar\n", path);
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const char* cfg = argc > 1 ? argv[1] : "ccu_ports.cfg";
    Ports ports;
    if (!load_ports(cfg, ports)) return 1;

    const ccu::DeviceConfigStatus survey_cfg = ccu::device_config_load("ccu_survey.cfg");
    if (!survey_cfg.opened) {
        std::printf("ccu_survey.cfg missing; defaults in use\n");
    } else if (!survey_cfg.height_set) {
        std::printf("ccu_survey.cfg DEVICE_HEIGHT missing or invalid; default in use\n");
    }
    std::printf("DEVICE_HEIGHT %.3f m D_MAX %.3g ANGLE_MAX %.2f\n",
                static_cast<double>(ccu::device_height_m()),
                static_cast<double>(ccu::d_max_m()),
                static_cast<double>(ccu::angle_max_rad()));
    std::fflush(stdout);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (!ccu::angle_open(ports.angle.c_str())) return 1;
    if (!ccu::projector_open(ports.projector.c_str())) return 1;
    if (!ccu::lidar_open(ports.lidar.c_str())) {
        std::fprintf(stderr, "lidar not opened\n");
    }
    if (!ccu::dongle_open(ports.dongle.c_str())) return 1;

    std::printf("ccu running\n");
    std::fflush(stdout);
    while (!quit) ccu::session_poll();

    ccu::dongle_close();
    ccu::lidar_close();
    ccu::projector_close();
    ccu::angle_close();
    return 0;
}
