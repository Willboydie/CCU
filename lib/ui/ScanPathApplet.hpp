#pragma once

#include <atomic>

class ScanPathApplet {
public:
    int run(std::atomic<bool>* keep_running = nullptr);
};
