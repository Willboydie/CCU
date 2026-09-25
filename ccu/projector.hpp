#pragma once

#include <cstdint>
#include <vector>

namespace ccu {

bool projector_open(const char* path);
void projector_close();
void projector_poll();

void projector_enqueue(std::vector<uint8_t> frame);
std::vector<std::vector<uint8_t>> projector_take_frames();
void projector_test_reset();

}  // namespace ccu
