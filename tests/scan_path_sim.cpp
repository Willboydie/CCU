#include "ui/ScanPathApplet.hpp"

#include <atomic>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

std::atomic<bool> g_run{true};

#ifdef _WIN32
BOOL WINAPI on_console_ctrl(DWORD) {
    g_run = false;
    return TRUE;
}
#endif

}  // namespace

int main() {
#ifdef _WIN32
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif
    ScanPathApplet applet;
    return applet.run(&g_run);
}
