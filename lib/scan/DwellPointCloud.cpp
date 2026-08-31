#include "scan/DwellPointCloud.hpp"

#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

namespace {

constexpr uint16_t kDwellFileVersion = 1;
constexpr uint16_t kDwellRecordSize = 12;
constexpr char kDwellMagic[4] = {'W', 'D', 'L', '1'};

#pragma pack(push, 1)
struct DwellFileHeader {
    char magic[4];
    uint16_t version;
    uint16_t rec_size;
    uint32_t count;
    float target_x;
    float target_y;
    float centre_az;
    float centre_el;
    float centre_mx;
    float centre_my;
    float amp_x;
    float amp_y;
    float freq_x;
    float freq_y;
    float phase_x;
    float phase_y;
    float duration_s;
};

struct DwellFileRecord {
    float angle_x;
    float angle_y;
    uint16_t distance_cm;
    uint16_t flags;
};
#pragma pack(pop)

static_assert(sizeof(DwellFileHeader) == 64);
static_assert(sizeof(DwellFileRecord) == kDwellRecordSize);

}  // namespace

void DwellPointCloud::set_centre(float azimuth, float elevation) {
    centre_az_ = azimuth;
    centre_el_ = elevation;
    have_centre_ = true;
}

void DwellPointCloud::set_centre_mechanical(float mx, float my) {
    centre_mx_ = mx;
    centre_my_ = my;
}

void DwellPointCloud::set_target_xy(float x, float y) {
    target_x_ = x;
    target_y_ = y;
}

void DwellPointCloud::set_params(const DwellParams& p) { params_ = p; }

std::optional<float> DwellPointCloud::estimate_z() const {
    return std::nullopt;
}

bool DwellPointCloud::save_binary(const std::string& path, std::string& err) const {
    err.clear();
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        err = "Failed to open " + path + " for write";
        return false;
    }

    DwellFileHeader hdr{};
    std::memcpy(hdr.magic, kDwellMagic, 4);
    hdr.version = kDwellFileVersion;
    hdr.rec_size = kDwellRecordSize;
    hdr.count = static_cast<uint32_t>(points_.size());
    hdr.target_x = target_x_;
    hdr.target_y = target_y_;
    hdr.centre_az = centre_az_;
    hdr.centre_el = centre_el_;
    hdr.centre_mx = centre_mx_;
    hdr.centre_my = centre_my_;
    hdr.amp_x = params_.amp_x;
    hdr.amp_y = params_.amp_y;
    hdr.freq_x = params_.freq_x;
    hdr.freq_y = params_.freq_y;
    hdr.phase_x = params_.phase_x;
    hdr.phase_y = params_.phase_y;
    hdr.duration_s = params_.duration_s;

    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (!out) {
        err = "Failed to write dwell header";
        return false;
    }

    for (const auto& p : points_) {
        DwellFileRecord rec{};
        rec.angle_x = p.mechanical_x;
        rec.angle_y = p.mechanical_y;
        rec.distance_cm = p.distance_cm;
        rec.flags = p.valid ? 1 : 0;
        out.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
        if (!out) {
            err = "Failed to write dwell records";
            return false;
        }
    }
    return true;
}

bool DwellPointCloud::load_binary(const std::string& path, DwellPointCloud& out,
                                  std::string& err) {
    err.clear();
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        err = "Failed to open " + path;
        return false;
    }

    DwellFileHeader hdr{};
    in.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!in || std::memcmp(hdr.magic, kDwellMagic, 4) != 0) {
        err = "Not a WDL1 dwell file";
        return false;
    }
    if (hdr.version != kDwellFileVersion || hdr.rec_size != kDwellRecordSize) {
        err = "Unsupported dwell file version";
        return false;
    }

    DwellPointCloud cloud;
    cloud.set_target_xy(hdr.target_x, hdr.target_y);
    cloud.set_centre(hdr.centre_az, hdr.centre_el);
    cloud.set_centre_mechanical(hdr.centre_mx, hdr.centre_my);
    DwellParams p;
    p.amp_x = hdr.amp_x;
    p.amp_y = hdr.amp_y;
    p.freq_x = hdr.freq_x;
    p.freq_y = hdr.freq_y;
    p.phase_x = hdr.phase_x;
    p.phase_y = hdr.phase_y;
    p.duration_s = hdr.duration_s;
    cloud.set_params(p);
    cloud.reserve(hdr.count);

    for (uint32_t i = 0; i < hdr.count; ++i) {
        DwellFileRecord rec{};
        in.read(reinterpret_cast<char*>(&rec), sizeof(rec));
        if (!in) {
            err = "Truncated dwell file";
            return false;
        }
        ControlPayload ctrl{};
        ctrl.associated_angle_x = rec.angle_x;
        ctrl.associated_angle_y = rec.angle_y;
        ctrl.distance = rec.distance_cm;
        cloud.add_from_control(ctrl);
    }

    out = std::move(cloud);
    return true;
}
