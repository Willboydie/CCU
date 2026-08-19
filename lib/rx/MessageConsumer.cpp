#include "MessageConsumer.hpp"

#include <optional>

MessageConsumer::MessageConsumer(MessageQueue& queue)
    : queue_(queue) {}

MessageConsumer::~MessageConsumer() {
    stop();
}

void MessageConsumer::set_handler(uint8_t cmd, Handler handler) {
    std::lock_guard<std::mutex> lock(handlers_mu_);
    handlers_[cmd] = std::move(handler);
}

void MessageConsumer::set_default_handler(Handler handler) {
    std::lock_guard<std::mutex> lock(handlers_mu_);
    default_handler_ = std::move(handler);
}

void MessageConsumer::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;
    thread_ = std::thread([this] { run(); });
}

void MessageConsumer::stop() {
    if (!running_.exchange(false)) return;
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
}

void MessageConsumer::run() {
    while (running_) {
        std::optional<RxMessage> msg = queue_.pop();
        if (!msg) break;

        const std::span<const uint8_t> payload{msg->payload, msg->len};

        Handler handler;
        {
            std::lock_guard<std::mutex> lock(handlers_mu_);
            auto it = handlers_.find(msg->cmd);
            if (it != handlers_.end())
                handler = it->second;
            else
                handler = default_handler_;
        }
        if (handler)
            handler(msg->cmd, payload);
    }
}
