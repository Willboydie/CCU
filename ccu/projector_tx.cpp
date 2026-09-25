#include "projector.hpp"

namespace ccu {
namespace {

std::vector<std::vector<uint8_t>> tx_;

}  // namespace

void projector_enqueue(std::vector<uint8_t> frame) { tx_.push_back(std::move(frame)); }

std::vector<std::vector<uint8_t>> projector_take_frames() {
    std::vector<std::vector<uint8_t>> out;
    out.swap(tx_);
    return out;
}

void projector_test_reset() { tx_.clear(); }

}  // namespace ccu
