#pragma once

#include <cctype>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

// ESP-NOW_Galvo_Detector_Protocol.md — USB serial responses from the master.
enum class EspLineKind {
    Unknown,
    Enrolled,
    Ready,
    Slave,
    Reading,
    Active,
    SampleResult,
    Rssi,
    RssiDone,
    Timeout,
    Deselected,
    Error,
};

struct EspLine {
    EspLineKind kind = EspLineKind::Unknown;
    int slave = 0;
    int count = 0;
    int sample = 0;
    int intensity = 0;
    int battery = 0;
    int rssi = 0;
    bool have_intensity = false;
    bool have_rssi = false;
    bool rssi_no_reply = false;
    std::string mac;
    std::string target;
    std::string quality;
    std::string state;
    std::string error;
};

inline std::string_view trim_sv(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
        s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
        s.remove_suffix(1);
    return s;
}

inline std::vector<std::string> split_csv(std::string_view line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            out.push_back(std::string(trim_sv(cur)));
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(std::string(trim_sv(cur)));
    return out;
}

inline bool ieq(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i])))
            return false;
    }
    return true;
}

inline int to_int(std::string_view s) {
    return std::atoi(std::string(s).c_str());
}

inline EspLine parse_esp_line(std::string_view raw) {
    EspLine msg;
    const auto fields = split_csv(trim_sv(raw));
    if (fields.empty() || fields[0].empty())
        return msg;

    const auto& key = fields[0];
    auto at = [&](std::size_t i) -> const std::string& {
        static const std::string empty;
        return i < fields.size() ? fields[i] : empty;
    };

    if (ieq(key, "ENROLLED")) {
        msg.kind = EspLineKind::Enrolled;
        msg.slave = to_int(at(1));
        msg.mac = at(2);
    } else if (ieq(key, "READY")) {
        msg.kind = EspLineKind::Ready;
        msg.count = to_int(at(1));
    } else if (ieq(key, "SLAVE")) {
        msg.kind = EspLineKind::Slave;
        msg.slave = to_int(at(1));
        msg.mac = at(2);
        msg.intensity = to_int(at(3));
        msg.battery = to_int(at(4));
        msg.rssi = to_int(at(5));
        msg.quality = at(6);
        msg.state = at(7);
        msg.have_intensity = !at(3).empty();
        msg.have_rssi = !at(5).empty();
    } else if (ieq(key, "READING")) {
        msg.kind = EspLineKind::Reading;
        msg.target = at(1);
        msg.sample = to_int(at(2));
        msg.slave = to_int(at(3));
        msg.intensity = to_int(at(4));
        msg.battery = to_int(at(5));
        msg.rssi = to_int(at(6));
        msg.have_intensity = !at(4).empty();
        msg.have_rssi = !at(6).empty();
    } else if (ieq(key, "ACTIVE")) {
        msg.kind = EspLineKind::Active;
        msg.target = at(1);
        msg.slave = to_int(at(2));
        msg.intensity = to_int(at(3));
        msg.battery = to_int(at(4));
        msg.rssi = to_int(at(5));
        msg.have_intensity = !at(3).empty();
        msg.have_rssi = !at(5).empty();
    } else if (ieq(key, "SAMPLE_RESULT")) {
        msg.kind = EspLineKind::SampleResult;
        msg.target = at(1);
        msg.sample = to_int(at(2));
        msg.slave = to_int(at(3));
        msg.intensity = to_int(at(4));
        msg.battery = to_int(at(5));
        msg.rssi = to_int(at(6));
        msg.have_intensity = !at(4).empty();
        msg.have_rssi = !at(6).empty();
    } else if (ieq(key, "RSSI_DONE")) {
        msg.kind = EspLineKind::RssiDone;
        msg.count = to_int(at(1));
    } else if (ieq(key, "RSSI")) {
        msg.kind = EspLineKind::Rssi;
        msg.slave = to_int(at(1));
        if (ieq(at(2), "NO_REPLY")) {
            msg.rssi_no_reply = true;
            msg.quality = at(3).empty() ? "POOR" : at(3);
        } else {
            msg.rssi = to_int(at(2));
            msg.have_rssi = !at(2).empty();
            msg.quality = at(3);
        }
    } else if (ieq(key, "TIMEOUT")) {
        msg.kind = EspLineKind::Timeout;
        msg.target = at(1);
        msg.sample = to_int(at(2));
        msg.slave = to_int(at(3));
    } else if (ieq(key, "DESELECTED")) {
        msg.kind = EspLineKind::Deselected;
    } else if (ieq(key, "ERROR")) {
        msg.kind = EspLineKind::Error;
        const auto comma = raw.find(',');
        msg.error = (comma == std::string_view::npos)
            ? std::string{}
            : std::string(trim_sv(raw.substr(comma + 1)));
    }
    return msg;
}
