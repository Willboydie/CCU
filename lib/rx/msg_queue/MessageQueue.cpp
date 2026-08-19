#include "MessageQueue.hpp"

void MessageQueue::push(RxMessage msg) {
    std::lock_guard<std::mutex> lock(mu_);
    if (shutdown_) return;
    q_.push_back(std::move(msg));
    cv_.notify_one();
}

std::optional<RxMessage> MessageQueue::pop() {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [this] { return shutdown_ || !q_.empty(); });
    if (q_.empty()) return std::nullopt;
    RxMessage msg = std::move(q_.front());
    q_.pop_front();
    return msg;
}

void MessageQueue::shutdown() {
    std::lock_guard<std::mutex> lock(mu_);
    if (shutdown_) return;
    shutdown_ = true;
    q_.clear();
    cv_.notify_all();
}
