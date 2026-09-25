#include "device_config.hpp"
#include "galvo_model.hpp"
#include "path_engine.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-4f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

float hypot2(float east, float north) { return std::hypot(east, north); }

void check_arms(const ccu::PathNode& node, const char* what) {
    const std::vector<ccu::CartPoint> points = ccu::path_corner_points(node);
    const float length = ccu::corner_len_m();
    const int count = static_cast<int>(std::lround(
        static_cast<double>(length) * (ccu::corner_density() + ccu::base_path_density()) / 2.0));
    check(static_cast<int>(points.size()) == count * 2 - 1, what);
    if (points.empty()) return;
    const std::size_t mid = points.size() / 2;
    check_near(points[mid].east_m, node.east_m, what);
    check_near(points[mid].north_m, node.north_m, what);
    check(points[mid].east_m == node.east_m && points[mid].north_m == node.north_m, "vertex is present once");
    int vertex_hits = 0;
    for (const ccu::CartPoint& point : points) {
        check_near(point.up_m, node.up_m, what);
        if (point.east_m == node.east_m && point.north_m == node.north_m) ++vertex_hits;
    }
    check(vertex_hits == 1, "vertex is the single middle point");
    check_near(hypot2(points.front().east_m - node.east_m, points.front().north_m - node.north_m), length, what);
    check_near(hypot2(points.back().east_m - node.east_m, points.back().north_m - node.north_m), length, what);
}

bool laser_runs(const std::vector<ccu::DacPoint>& path, int want_on_runs) {
    if (path.empty()) return false;
    int runs = 0;
    bool on = path.front().laser_on;
    if (!on) return false;
    ++runs;
    bool saw_off = false;
    for (const ccu::DacPoint& point : path) {
        if (point.laser_on == on) continue;
        on = point.laser_on;
        if (on) {
            if (!saw_off) return false;
            ++runs;
            saw_off = false;
        } else {
            saw_off = true;
        }
    }
    return runs == want_on_runs && !path.back().laser_on;
}

void test_empty() {
    const std::vector<ccu::DacPoint> path = ccu::path_engine({}, 0.f);
    check(path.empty(), "empty input emits an empty path");
}

void test_one_node() {
    ccu::device_config_reset();
    const ccu::PathNode node{0.f, 2.f, 1.10f, std::numbers::pi_v<float> / 2.f, 0.f};
    check_arms(node, "one node arms are CORNER_LEN on the stored up");
    const std::vector<ccu::CartPoint> cart = ccu::path_corner_points(node);
    const std::vector<ccu::DacPoint> path = ccu::path_engine({node}, 0.f);
    int laser_on = 0;
    for (const ccu::DacPoint& point : path)
        if (point.laser_on) ++laser_on;
    check(laser_on == static_cast<int>(cart.size()), "laser-on points are the arms");
    check(laser_runs(path, 1), "one node closes with a laser-off join");
    if (cart.empty() || path.size() < cart.size()) return;
    const ccu::GalvoDac vertex = ccu::galvo_plan_to_dac(node.east_m, node.north_m, node.up_m, 0.f);
    const std::size_t mid = cart.size() / 2;
    check(path[mid].laser_on, "middle arm point is laser on");
    check(path[mid].x == vertex.x && path[mid].y == vertex.y, "vertex DAC is the single middle point");
}

void test_two_nodes() {
    ccu::device_config_reset();
    const ccu::PathNode a{0.f, 2.f, 1.10f, std::numbers::pi_v<float> / 2.f, 0.f};
    const ccu::PathNode b{0.4f, 3.f, 1.40f, 1.2f, 0.3f};
    check_arms(a, "first node arms stay on its stored up");
    check_arms(b, "second node arms stay on its stored up");
    const std::vector<ccu::DacPoint> path = ccu::path_engine({a, b}, 0.f);
    check(laser_runs(path, 2), "two nodes have a laser-off join between them and a laser-off close");
}

}  // namespace

int main() {
    test_empty();
    test_one_node();
    test_two_nodes();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_path_engine_test passed\n");
    return 0;
}
