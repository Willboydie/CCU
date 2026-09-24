#include "path_engine.hpp"

#include <cstdio>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_eq_f(float got, float want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void check_point(const ccu::PathPoint& point, float east, float north, float up, bool laser, uint8_t duty,
                 const char* what) {
    check_eq_f(point.east_mm, east, what);
    check_eq_f(point.north_mm, north, what);
    check_eq_f(point.up_mm, up, what);
    check(point.laser_on == laser, what);
    check(point.duty == duty, what);
}

void test_empty() {
    const std::vector<ccu::PathPoint> path = ccu::path_engine({}, 30);
    check(path.empty(), "empty input emits an empty path");
}

void test_two_nodes() {
    const std::vector<ccu::AimPoint> aims = {
        {100.f, 200.f, -150.f},
        {500.f, 800.f, -80.f},
    };
    const std::vector<ccu::PathPoint> path = ccu::path_engine(aims, 30);
    check(path.size() == 10, "two nodes emit ten points");
    if (path.size() != 10) return;

    check_point(path[0], 100.f, 200.f, -150.f, false, 0, "first connector is laser off");
    check_point(path[1], 140.f, 200.f, -150.f, true, 30, "first corner");
    check_point(path[2], 140.f, 240.f, -150.f, true, 30, "second corner");
    check_point(path[3], 100.f, 240.f, -150.f, true, 30, "third corner");
    check_point(path[4], 100.f, 200.f, -150.f, true, 30, "back to the first aim");

    check_point(path[5], 100.f, 200.f, -150.f, false, 0, "second connector leaves the previous aim");
    check_point(path[6], 540.f, 800.f, -80.f, true, 30, "second node first corner");
    check_point(path[7], 540.f, 840.f, -80.f, true, 30, "second node second corner");
    check_point(path[8], 500.f, 840.f, -80.f, true, 30, "second node third corner");
    check_point(path[9], 500.f, 800.f, -80.f, true, 30, "back to the second aim");

    check(ccu::kCornerSizeMm == 40.f, "corner is 40 mm");
}

}  // namespace

int main() {
    test_empty();
    test_two_nodes();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_path_engine_test passed\n");
    return 0;
}
