#pragma once

namespace ccu {

bool dongle_open(const char* path);
void dongle_close();
void dongle_poll();
void dongle_send_device_id_if_due();

}  // namespace ccu
