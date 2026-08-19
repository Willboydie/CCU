#pragma once

#include "RxMessage.hpp"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

class MessageQueue {
public:
    MessageQueue() = default;
    ~MessageQueue() { shutdown(); }

    MessageQueue(const MessageQueue&) = delete;
    MessageQueue& operator=(const MessageQueue&) = delete;

    void push(RxMessage msg);
    std::optional<RxMessage> pop();
    void shutdown();

private:
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<RxMessage> q_;
    bool shutdown_ = false;
};
