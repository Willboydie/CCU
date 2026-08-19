#pragma once

#include "msg_queue/MessageQueue.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <thread>
#include <unordered_map>

class MessageConsumer {
public:
    using Handler = std::function<void(uint8_t cmd, std::span<const uint8_t> payload)>;

    explicit MessageConsumer(MessageQueue& queue);
    ~MessageConsumer();

    MessageConsumer(const MessageConsumer&) = delete;
    MessageConsumer& operator=(const MessageConsumer&) = delete;

    void set_handler(uint8_t cmd, Handler handler);
    void set_default_handler(Handler handler);

    void start();
    void stop();

private:
    void run();

    MessageQueue& queue_;
    std::mutex handlers_mu_;
    std::unordered_map<uint8_t, Handler> handlers_;
    Handler default_handler_;
    std::thread thread_;
    std::atomic<bool> running_{false};
};
