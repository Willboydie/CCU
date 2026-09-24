#pragma once

#include <cstdint>
#include <span>

namespace ccu {

// Bytes the host writes to the dongle. Return the count written, or -1.
using DongleWriteFn = int (*)(void* ctx, const uint8_t* data, int n);

// Attach a writer and clear paired. No serial port.
void dongle_link_attach(DongleWriteFn write, void* ctx);
void dongle_link_detach();
void dongle_link_feed(std::span<const uint8_t> bytes);
bool dongle_link_paired();
void dongle_link_set_paired(bool paired);
bool dongle_link_write(const uint8_t* data, int n);
void dongle_send_device_id_if_due();

}  // namespace ccu
