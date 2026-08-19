#pragma once

#include <atomic>
#include <cstdint>

struct SlotRelease {
    std::atomic<bool>* flag = nullptr;

    SlotRelease() = default;
    explicit SlotRelease(std::atomic<bool>* f) : flag(f) {}

    SlotRelease(const SlotRelease&) = delete;
    SlotRelease& operator=(const SlotRelease&) = delete;

    SlotRelease(SlotRelease&& other) noexcept : flag(other.flag) {
        other.flag = nullptr;
    }

    SlotRelease& operator=(SlotRelease&& other) noexcept {
        if (this != &other) {
            reset();
            flag = other.flag;
            other.flag = nullptr;
        }
        return *this;
    }

    ~SlotRelease() { reset(); }

    void reset() {
        if (flag) {
            flag->store(false, std::memory_order_release);
            flag = nullptr;
        }
    }
};

struct RxMessage {
    uint8_t cmd = 0;
    uint16_t len = 0;
    const uint8_t* payload = nullptr;
    SlotRelease lease;
};
