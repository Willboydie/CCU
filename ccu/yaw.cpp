#include "yaw.hpp"

#include <cmath>
#include <numbers>

namespace ccu {

float wrap_2pi(float radians) {
    constexpr float kTwoPi = 2.f * std::numbers::pi_v<float>;
    float wrapped = std::fmod(radians, kTwoPi);
    if (wrapped < 0.f) wrapped += kTwoPi;
    if (wrapped >= kTwoPi) wrapped = 0.f;
    return wrapped;
}

float effective_yaw(float north_offset, float stage) {
    return wrap_2pi(north_offset - stage);
}

}  // namespace ccu
