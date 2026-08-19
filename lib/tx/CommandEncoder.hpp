#pragma once

#include "../utils/cmd_refs.hpp"

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <optional>
#include <span>
#include <vector>

class CommandEncoder {
public:
    static std::optional<std::vector<uint8_t>> pack(
        uint8_t cmd, std::span<const uint8_t> payload) {
        if (payload.size() > kCmdPayloadMax)
            return std::nullopt;

        std::vector<uint8_t> out(3 + payload.size());
        out[0] = cmd;
        const auto len = static_cast<uint16_t>(payload.size());
        out[1] = static_cast<uint8_t>(len & 0xFF);
        out[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
        if (!payload.empty())
            std::memcpy(out.data() + 3, payload.data(), payload.size());
        return out;
    }

    static std::optional<std::vector<uint8_t>> pack_f32(
        uint8_t cmd, std::initializer_list<float> args) {
        const std::size_t nbytes = args.size() * sizeof(float);
        if (nbytes > kCmdPayloadMax)
            return std::nullopt;

        uint8_t raw[kCmdPayloadMax];
        std::size_t i = 0;
        for (float v : args) {
            std::memcpy(raw + i, &v, sizeof(float));
            i += sizeof(float);
        }
        return pack(cmd, {raw, nbytes});
    }
};
