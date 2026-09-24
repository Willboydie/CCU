#include "lidar_model.hpp"

#include "scan/Geometry.hpp"

#include <cmath>

namespace ccu {

LidarAngle lidar_motor_to_angle(float mech_x, float mech_y) {
    LidarAngle angle;
    mechanical_to_boresight(mech_x, mech_y, angle.azimuth, angle.elevation);
    return angle;
}

float lidar_up_mm(float range_mm, float elevation) {
    return range_mm * std::sin(elevation);
}

LidarOrigin lidar_origin_mm() {
    return {};
}

}  // namespace ccu
